#include "ndt_3d.h"
#include <glog/logging.h>
#include <Eigen/SVD>
#include <execution>
#include "common/math_utils.h"

namespace lh {
void Ndt3d::BuildVoxels() {
    assert(target_ != nullptr);
    assert(target_->empty() == false);
    grid_.clear();

    // 分配体素
    std::vector<size_t> index(target_->size());
    // 给index赋值
    std::for_each(index.begin(), index.end(), [idx = 0](size_t& i) mutable { i = idx++; });

    std::for_each(index.begin(), index.end(), [this](const size_t& idx) {
        auto pt = ToVec3d(target_->points[idx]);
        // 有点像栅格地图的分辨率
        auto key = (pt * options_.inv_voxel_size_).cast<int>();
        if (grid_.find(key) == grid_.end()) {
            // 构建key 和 voxeldata
            grid_.insert({key, {idx}});  // map 插入
        } else {
            grid_[key].idx_.emplace_back(idx);  // map 更新
        }
    });

    // 计算体素的均值和协方差
    std::for_each(std::execution::par_unseq, grid_.begin(), grid_.end(), [this](auto& v) {
        // v 是std::pair类型,每个体素中最少包含3个点
        if (v.second.idx_.size() > options_.min_pts_in_voxel_) {
            math::ComputeMeanAndCov(v.second.idx_, v.second.mu_, v.second.sigma_,
                                    [this](const size_t& idx) { return ToVec3d(target_->points[idx]); });

            // SVD 检查最大与最小奇异值
            Eigen::JacobiSVD svd(v.second.sigma_, Eigen::ComputeFullU | Eigen::ComputeFullV);
            Vec3d lambda = svd.singularValues();
            if (lambda[1] < lambda[0] * 1e-3) {
                lambda[1] = lambda[0] * 1e-3;
            }

            if (lambda[2] < lambda[0] * 1e-3) {
                lambda[2] = lambda[0] * 1e-3;
            }
            // 构建对角矩阵
            Mat3d inv_lambda = Vec3d(1.0 / lambda[0], 1.0 / lambda[1], 1.0 / lambda[2]).asDiagonal();

            v.second.info_ = svd.matrixV() * inv_lambda * svd.matrixU().transpose();
        }
    });

    // 删除体素中点数不够的
    for (auto iter = grid_.begin(); iter != grid_.end();) {
        if (iter->second.idx_.size() > options_.min_pts_in_voxel_) {
            iter++;
        } else {
            iter = grid_.erase(iter);
        }
    }
}

/**
 * @brief 高斯牛顿
 */
bool Ndt3d::AlignNdt(SE3& init_pose) {
    LOG(INFO) << "aligning with ndt";
    assert(grid_.empty() == false);

    SE3 pose = init_pose;
    // 设置平移初始值
    if (options_.remove_centroid_) {
        pose.translation() = target_center_ - source_center_;
        LOG(INFO) << "init trans set to " << pose.translation().transpose();
    }

    int num_residual_per_point = 1;
    if (options_.nearby_type_ == NearbyType::NEARBY6) {
        num_residual_per_point = 7;  // 6个周围点+1个中心点
    }

    std::vector<int> index(source_->points.size());
    for (int i = 0; i < index.size(); ++i) {
        index[i] = i;
    }

    int total_size = index.size() * num_residual_per_point;

    for (int iter = 0; iter < options_.max_iteration_; ++iter) {
        std::vector<bool> effect_pts(total_size, false);
        std::vector<Eigen::Matrix<double, 3, 6>> jacobians(total_size);
        std::vector<Vec3d> errors(total_size);
        std::vector<Mat3d> infos(total_size);

        std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](int idx) {
            auto q = ToVec3d(source_->points[idx]);
            Vec3d qs = pose * q;

            // 计算qs 所在的栅格
            Vec3i key = (qs * options_.inv_voxel_size_).cast<int>();

            for (int i = 0; i < nearby_grids_.size(); ++i) {
                auto key_offset = key + nearby_grids_[i];
                auto it = grid_.find(key_offset);
                int real_idx = idx * num_residual_per_point + i;
                if (it != grid_.end()) {
                    auto& v = it->second;
                    // error
                    Vec3d e = qs - v.mu_;

                    double res = e.transpose() * v.info_ * e;
                    if (std::isnan(res) || res > options_.res_outlier_th_) {
                        effect_pts[real_idx] = false;
                        continue;
                    }

                    // residual
                    Eigen::Matrix<double, 3, 6> J;
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

        // 累加Hessian矩阵和error
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
            return false;
        }

        Vec6d dx = H.inverse() * err;
        pose.so3() = pose.so3() * SO3::exp(dx.head<3>());
        pose.translation() += dx.tail<3>();

        // 更新
        LOG(INFO) << "iter " << iter << " total res: " << total_res << ", eff: " << effective_num << ", mean res: " << total_res / effective_num
                  << ", dxn: " << dx.norm() << ", dx: " << dx.transpose();

        // std::sort(chi2.begin(), chi2.end());
        // LOG(INFO) << "chi2 med: " << chi2[chi2.size() / 2] << ", .7: " << chi2[chi2.size() * 0.7]
        //           << ", .9: " << chi2[chi2.size() * 0.9] << ", max: " << chi2.back();

        if (gt_set_) {
            double pose_error = (gt_pose_.inverse() * pose).log().norm();
            LOG(INFO) << "iter " << iter << " pose error: " << pose_error;
        }

        if (dx.norm() < options_.eps_) {
            LOG(INFO) << "converged, dx = " << dx.transpose();
            break;
        }
    }
    init_pose = pose;
    return true;
}

void Ndt3d::GenerateNearByGrids() {
    if (options_.nearby_type_ == NearbyType::CENTER) {
        nearby_grids_.emplace_back(KeyType::Zero());
    } else if (options_.nearby_type_ == NearbyType::NEARBY6) {
        nearby_grids_ = {KeyType(0, 0, 0),  KeyType(-1, 0, 0), KeyType(1, 0, 0), KeyType(0, 1, 0),
                         KeyType(0, -1, 0), KeyType(0, 0, -1), KeyType(0, 0, 1)};
    }
}
}  // namespace lh
