#include "ndt.hh"
#include <glog/logging.h>
#include <Eigen/SVD>
#include <execution>
#include "common/lidar_utils.h"
#include "common/math_utils.h"

namespace lh {
void Ndt3d::BuildVoxels() {
    assert(target_cloud_ != nullptr);
    assert(target_cloud_->empty() == false);
    voxel_grid_.clear();

    // 分配体素
    std::vector<size_t> index(target_cloud_->size());
    std::for_each(index.begin(), index.end(), [idx = 0](size_t& i) mutable { i = idx++; });
    std::for_each(index.begin(), index.end(), [this](const size_t& idx) {
        Vec3d pt = ToVec3d(target_cloud_->points[idx]) * options_.inv_voxel_size_;
        auto grid_coord = CastToInt(pt);
        //  没有对应的体素坐标
        if (voxel_grid_.find(grid_coord) == voxel_grid_.end()) {
            // 存放体素的坐标和在体素内的点云索引
            voxel_grid_.insert({grid_coord, {idx}});
        } else {
            voxel_grid_[grid_coord].idx_.emplace_back(idx);
        }
    });
    // 计算每个体素中的均值和协方差
    std::for_each(std::execution::par_unseq, voxel_grid_.begin(), voxel_grid_.end(), [this](auto& v) {
        // 判断体素中的点云个数大于3,才计算均值和方差
        if (v.second.idx_.size() > options_.min_pts_in_voxel_) {
            math::ComputeMeanAndCov(v.second.idx_, v.second.mu_, v.second.sigma_,
                                    [this](const size_t& idx) { return ToVec3d(target_cloud_->points[idx]); });
            // svd 检查最大和最小奇异值
            Eigen::JacobiSVD svd(v.second.sigma_, Eigen::ComputeFullU | Eigen::ComputeFullV);
            Vec3d lambda = svd.singularValues();

            if (lambda[1] < lambda[0] * 1e-3) {
                lambda[1] = lambda[0] * 1e-3;
            }

            if (lambda[2] < lambda[0] * 1e-3) {
                lambda[2] = lambda[0] * 1e-3;
            }
            // 创建对角矩阵
            Mat3d inv_lambda = Vec3d(1.0 / lambda[0], 1.0 / lambda[1], 1.0 / lambda[2]).asDiagonal();

            v.second.info_ = svd.matrixV() * inv_lambda * svd.matrixU().transpose();
        }
    });
    // 这里要非常注意map删除元素
    for (auto iter = voxel_grid_.begin(); iter != voxel_grid_.end();) {
        if (iter->second.idx_.size() > options_.min_pts_in_voxel_) {
            iter++;
        } else {
            iter = voxel_grid_.erase(iter);
        }
    }
}

void Ndt3d::GenerateNearbyGrids() {
    if (options_.nearby_type_ == NearbyType::CENTER) {
        nearby_grids_.emplace_back(KeyType::Zero());
    } else if (options_.nearby_type_ == NearbyType::NEARBY6) {
        nearby_grids_ = {KeyType(0, 0, 0),  KeyType(-1, 0, 0), KeyType(1, 0, 0), KeyType(0, 1, 0),
                         KeyType(0, -1, 0), KeyType(0, 0, -1), KeyType(0, 0, 1)};
    }
}

bool Ndt3d::AlignNdt(SE3& init_pose) {
    LOG(INFO) << "aligning with ndt";
    assert(voxel_grid_.empty() == false);

    SE3 pose = init_pose;
    if (options_.remove_centroid_) {
        pose.translation() = target_center_ - source_center_;
        LOG(INFO) << "init trans set to" << pose.translation().transpose();
    }

    int num_residual_per_point = 1;
    if (options_.nearby_type_ == NearbyType::NEARBY6) {
        num_residual_per_point = 7;
    }

    std::vector<int> index(source_cloud_->size());
    std::for_each(index.begin(), index.end(), [idx = 0](int& i) mutable { i = idx++; });

    int total_size = index.size() * num_residual_per_point;

    for (int iter = 0; iter < options_.max_iteration_; ++iter) {
        std::vector<bool> effect_pts(total_size, false);
        std::vector<Eigen::Matrix<double, 3, 6>> jacobians(total_size);
        std::vector<Vec3d> errors(total_size);
        std::vector<Mat3d> infos(total_size);

        std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](int idx) {
            auto vec_q = Vec3d(source_cloud_->points[idx]);
            Vec3d trans_q = pose * vec_q;

            Vec3i key = CastToInt(Vec3d(trans_q * options_.inv_voxel_size_));
            // 计算trans_q所在的栅格以及周围最邻近
            for (int i = 0; i < nearby_grids_.size(); ++i) {
                Vec3i key_offset = key + nearby_grids_[i];
                auto iter = voxel_grid_.find(key_offset);
                int real_idx = idx * num_residual_per_point + i;
                if (iter != voxel_grid_.end()) {
                    auto& v = iter->second;
                    Vec3d e = trans_q - v.mu_;  // 误差等于 点-体素的均值
                    // 7.13 最后的r,t又最小二乘问题决定,构建残差
                    double res = e.transpose() * v.info_ * e;
                    // 检验残差
                    if (std::isnan(res) || res > options_.res_outlier_th_) {
                        effect_pts[real_idx] = false;
                        continue;
                    }
                    // 7.15 残差的jacobian delta_e / dR, delta_e / dt;
                    Eigen::Matrix<double, 3, 6> J;
                    J.block<3, 3>(0, 0) = -pose.so3().matrix() * SO3::hat(vec_q);
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
        // 累加Hessian和error,计算dx
        // 原则上可以用reduce并发，写起来比较麻烦，这里写成accumulate
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
            // 7.15
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
}  // namespace lh