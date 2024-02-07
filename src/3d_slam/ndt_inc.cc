#include "3d_slam/ndt_inc.hh"

#include <glog/logging.h>
#include <execution>
#include <set>
#include "common/lidar_utils.h"
#include "common/math_utils.h"
#include "common/timer/timer.h"

namespace lh {
/**
 * @param 点云进来,将点云映射到体素当中
 */
void IncNdt3d::AddCloud(CloudPtr cloud_world) {
    // 用一个set集合代表那些体素需要更新
    std::set<VOXEL_LOC, less_vec<3>> active_voxels;
    for (const auto& point : cloud_world->points) {
        auto pt = ToVec3d(point);
        const Vec3i key = CastToInt(Vec3d(pt * options_.inv_voxel_size_));
        auto iter = grids_.find(key);
        if (iter == grids_.end()) {
            // 调用voxel的构造函数
            data_.push_front({key, {pt}});
            grids_.insert({key, data_.begin()});
            // 大于最大的容量
            if (data_.size() >= options_.capacity_) {
                grids_.erase(data_.back().first);
                data_.pop_back();
            }
        } else {
            // 体素中增加点
            iter->second->second.AddPoint(pt);
            data_.splice(data_.begin(), data_, iter->second);
            iter->second = data_.begin();
        }
        active_voxels.emplace(key);
    }
    // 更新体素
    std::for_each(std::execution::par_unseq, active_voxels.begin(), active_voxels.end(),
                  [this](const auto& key) { UpdateVoxel(grids_[key]->second); });
    flag_first_scan_ = false;
}

void IncNdt3d::GenerateNearbyGrids() {
    if (options_.nearby_type_ == NearbyType::CENTER) {
        nearby_grids_.emplace_back(VOXEL_LOC::Zero());
    } else if (options_.nearby_type_ == NearbyType::NEARBY6) {
        nearby_grids_ = {VOXEL_LOC(0, 0, 0),  VOXEL_LOC(-1, 0, 0), VOXEL_LOC(1, 0, 0), VOXEL_LOC(0, 1, 0),
                         VOXEL_LOC(0, -1, 0), VOXEL_LOC(0, 0, -1), VOXEL_LOC(0, 0, 1)};
    }
}

void IncNdt3d::UpdateVoxel(VoxelData& v) {
    // 第一帧雷达数据
    if (flag_first_scan_) {
        if (v.pts_.size() > 1) {
            math::ComputeMeanAndCov(v.pts_, v.mu_, v.sigma_, [this](const Vec3d& p) { return p; });
            v.info_ = (v.sigma_ + Mat3d::Identity() * 1e-3).inverse();  // 避免nan
        } else {
            v.mu_ = v.pts_[0];
            v.info_ = Mat3d::Identity() * 1e2;
        }

        v.ndt_estimated_ = true;

        // 为什么要clear?
        v.pts_.clear();
        return;
    }
    // 体素已经估计过协方差切> 体素的点云阈值
    if (v.ndt_estimated_ && v.num_pts_ > options_.max_pts_in_voxel_) {
        return;
    }
    // 对于新建的体素,那么肯定还没有估计协方差,但是要等到点云>某个阈值在开始估计
    if (!v.ndt_estimated_ && v.pts_.size() > options_.min_pts_in_voxel_) {
        math::ComputeMeanAndCov(v.pts_, v.mu_, v.sigma_, [this](const Vec3d& pt) { return pt; });
        v.info_ = (v.sigma_ + Mat3d::Identity() * 1e-3).inverse();  // 避免nan
        v.ndt_estimated_ = true;
        v.pts_.clear();
    } else if (v.ndt_estimated_ && v.pts_.size() > options_.min_effective_pts_) {
        // 已估计的点素中有新的点,更新协方差和均值
        // 这里是老的均值和协防+新的均值和协方差,所有pts需要clear();
        Vec3d curr_mu, new_mu;
        Mat3d curr_var, new_var;
        // 计算增加的n个点的均值和协方差
        math::ComputeMeanAndCov(v.pts_, curr_mu, curr_var, [this](const Vec3d& pt) { return pt; });
        math::UpdateMeanAndCov(v.num_pts_, v.pts_.size(), v.mu_, v.sigma_, curr_mu, curr_var, new_mu, new_var);
        v.mu_ = new_mu;
        v.sigma_ = new_var;
        v.num_pts_ += v.pts_.size();
        v.pts_.clear();
        // check info, 将协方差矩阵svd分解获得特征值
        Eigen::JacobiSVD svd(v.sigma_, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Vec3d lambda = svd.singularValues();
        if (lambda[1] < lambda[0] * 1e-3) {
            lambda[1] = lambda[0] * 1e-3;
        }
        if (lambda[2] < lambda[0] * 1e-3) {
            lambda[2] = lambda[0] * 1e-3;
        }
        Mat3d inv_lambda = Vec3d(1.0 / lambda[0], 1.0 / lambda[1], 1.0 / lambda[2]).asDiagonal();
        v.info_ = svd.matrixV() * inv_lambda * svd.matrixU().transpose();
    }
}

bool IncNdt3d::AlignNdt(SE3& init_pose) {
    LOG(INFO) << "aligning with inc ndt, pts: " << source_->size() << ", grids:" << grids_.size();
    assert(grids_.empty() == false);

    SE3 pose = init_pose;
    int num_residual_per_point = 1;
    if (options_.nearby_type_ == NearbyType::NEARBY6) {
        num_residual_per_point = 7;
    }

    std::vector<int> index(source_->points.size());
    for (int i = 0; i < index.size(); ++i) {
        index[i] = i;
    }

    int total_size = index.size() * num_residual_per_point;
    for (int iter = 0; iter < options_.max_iteration_; ++iter) {
        std::vector<bool> effect_pts(total_size, false);
        // de / dR, de / dt;
        std::vector<Eigen::Matrix<double, 3, 6>> jacobians(total_size);
        std::vector<Vec3d> errors(total_size);
        std::vector<Mat3d> infos(total_size);

        std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](int idx) {
            auto q = ToVec3d(source_->points[idx]);
            auto trans_q = pose * q;

            Vec3i key = CastToInt(Vec3d(trans_q * options_.inv_voxel_size_));
            for (int i = 0; i < nearby_grids_.size(); ++i) {
                Vec3i real_key = key + nearby_grids_[i];
                auto it = grids_.find(real_key);
                int real_idx = idx * num_residual_per_point + i;
                if (it != grids_.end() && it->second->second.ndt_estimated_) {
                    // voxel
                    auto& v = it->second->second;
                    Vec3d e = trans_q - v.mu_;
                    double res = e.transpose() * v.info_ * e;
                    if (std::isnan(res) || res > options_.res_outlier_th_) {
                        effect_pts[real_idx] = false;
                        continue;
                    }
                    // 构建残差
                    Eigen::Matrix<double, 3, 6> J;
                    // 和ndt的jacobian求导是一样的
                    J.block<3, 3>(0, 0) = -pose.so3().matrix() * SO3::hat(q);
                    J.block<3, 3>(0, 3) = Mat3d::Identity();

                    jacobians[real_idx] = J;
                    errors[real_idx] = e;
                    infos[real_idx] = v.info_;
                    effect_pts[real_idx] = true;
                } else {
                    effect_pts[real_idx] = false;
                }
            }
        });

        // 累加Hessian和error 计算dx
        double total_res = 0;
        int effective_num = 0;
        Mat6d H = Mat6d::Zero();
        Vec6d err = Vec6d::Zero();

        for (int idx = 0; idx < effect_pts.size(); ++idx) {
            if (!effect_pts[idx]) {
                continue;
            }
            total_res += errors[idx].transpose() * infos[idx] * errors[idx];
            effective_num++;
            H += jacobians[idx].transpose() * infos[idx] * jacobians[idx];
            err += -jacobians[idx].transpose() * infos[idx] * errors[idx];
        }

        if (effective_num < options_.min_effective_pts_) {
            LOG(WARNING) << "effective num too small: " << effective_num;
            init_pose = pose;
            return false;
        }

        Vec6d dx = H.inverse() * err;
        pose.so3() = pose.so3() * SO3::exp(dx.head<3>());
        pose.translation() += dx.tail<3>();

        // 更新
        LOG(INFO) << "iter " << iter << " total res: " << total_res << ", eff: " << effective_num << ", mean res: " << total_res / effective_num
                  << ", dxn: " << dx.norm() << ", dx: " << dx.transpose();

        if (dx.norm() < options_.eps_) {
            LOG(INFO) << "converged, dx = " << dx.transpose();
            break;
        }
    }

    init_pose = pose;
    return true;
}

void IncNdt3d::ComputeResidualAndJacobians(const SE3& input_pose, Mat18d& HTVH, Vec18d& HTVr) {}

void IncNdt3d::BuildNDTEdges(lh::VertexPose* v, std::vector<EdgeNDT*>& edges) {}

}  // namespace lh