#include "3d_icp.hh"
#include "common/math_utils.h"

#include <execution>
namespace lh {
bool ICP3D::AlignP2P(SE3& init_pose) {
    LOG(INFO) << "Aligning with point to point";
    assert(target_ != nullptr && source_ != nullptr);

    SE3 pose = init_pose;
    // 不使用外部设置的initial_pose
    if (!options_.use_initial_translation_) {
        LOG(INFO) << " not use_initial_translation_";
        pose.translation() = target_center_ - source_center_;
    }
    std::vector<int> source_index(source_->size());
    for (int i = 0; i < source_index.size(); ++i) {
        source_index[i] = i;
    }
    // 统计所有的有效点
    std::vector<bool> effect_pts(source_index.size(), false);
    // e = p_i - R*q_i - t 是一个3x1的vector
    // de / dr , de / dt 构成3 * 6的jabobian矩阵
    std::vector<Eigen::Matrix<double, 3, 6>> jacobians(source_index.size());
    std::vector<Vec3d> errors(source_index.size());

    for (int iter = 0; iter < options_.max_iteration_; ++iter) {
        //  gauss-newton  迭代
        std::for_each(source_index.begin(), source_index.end(), [&](int idx) {
            auto vec_q = ToVec3d(source_->points[idx]);
            Vec3d trans_q = pose * vec_q;

            std::vector<int> nn;
            std::vector<float> dists;
            // kdtree查找
            kdtree_->nearestKSearch(ToPointType(trans_q), 1, nn, dists);
            // 不为空
            if (!nn.empty()) {
                Vec3d target_p = ToVec3d(target_->points[nn[0]]);
                double dis2 = (target_p - trans_q).squaredNorm();
                // 距离 > 1.0,无效点
                if (dis2 > options_.max_nn_distance_) {
                    effect_pts[idx] = false;
                    return;
                }
                effect_pts[idx] = true;

                // 误差
                Vec3d e = target_p - trans_q;
                Eigen::Matrix<double, 3, 6> J_i;
                J_i.block<3, 3>(0, 0) = pose.so3().matrix() * SO3::hat(vec_q);
                J_i.block<3, 3>(0, 3) = -Mat3d::Identity();

                jacobians[idx] = J_i;
                errors[idx] = e;
            } else {
                effect_pts[idx] = false;
            }
        });
        // 计算dx
        double total_res = 0;
        int effective_num = 0;
        auto H_and_err = std::accumulate(
            source_index.begin(), source_index.end(), std::pair<Mat6d, Vec6d>(Mat6d::Zero(), Vec6d::Zero()),
            [&jacobians, &errors, &effect_pts, &total_res, &effective_num](const std::pair<Mat6d, Vec6d>& pre, int idx) -> std::pair<Mat6d, Vec6d> {
                if (!effect_pts[idx]) {
                    return pre;
                } else {
                    total_res += errors[idx].dot(errors[idx]);
                    effective_num++;
                    return std::pair<Mat6d, Vec6d>(pre.first + jacobians[idx].transpose() * jacobians[idx],
                                                   pre.second - jacobians[idx].transpose() * errors[idx]);
                }
            });
        if (effective_num < options_.min_effective_pts_) {
            LOG(WARNING) << "effective num too small: " << effective_num;
            return false;
        }

        Mat6d H = H_and_err.first;
        Vec6d err = H_and_err.second;

        Vec6d dx = H.inverse() * err;
        pose.so3() = pose.so3() * SO3::exp(dx.head<3>());
        pose.translation() += dx.tail<3>();

        // 更新
        LOG(INFO) << "iter " << iter << " total res: " << total_res << ", eff: " << effective_num << ", mean res: " << total_res / effective_num
                  << ", dxn: " << dx.norm();
        if (gt_set_) {
            LOG(INFO) << "iter pose: " << pose.so3().unit_quaternion().coeffs().transpose() << ", " << pose.translation().transpose();
            LOG(INFO) << "gt pose: " << gt_pose_.so3().unit_quaternion().coeffs().transpose() << ", " << gt_pose_.translation().transpose();

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
bool ICP3D::AlignP2Line(SE3& init_pose) {
    LOG(INFO) << "Aligning with point to Line";
    assert(target_ != nullptr && source_ != nullptr);

    SE3 pose = init_pose;
    // 不使用外部设置的initial_pose
    if (!options_.use_initial_translation_) {
        LOG(INFO) << " not use_initial_translation_";
        pose.translation() = target_center_ - source_center_;
    }
    std::vector<int> source_index(source_->size());
    for (int i = 0; i < source_index.size(); ++i) {
        source_index[i] = i;
    }
    // 统计所有的有效点
    std::vector<bool> effect_pts(source_index.size(), false);
    // e = p_i - R*q_i - t 是一个3x1的vector
    // de / dr , de / dt 构成3 * 6的jabobian矩阵
    std::vector<Eigen::Matrix<double, 3, 6>> jacobians(source_index.size());
    std::vector<Vec3d> errors(source_index.size());

    for (int iter = 0; iter < options_.max_iteration_; ++iter) {
        //  gauss-newton  迭代
        std::for_each(source_index.begin(), source_index.end(), [&](int idx) {
            auto vec_q = ToVec3d(source_->points[idx]);
            Vec3d trans_q = pose * vec_q;

            std::vector<int> nn;
            std::vector<float> dists;
            // kdtree查找
            kdtree_->nearestKSearch(ToPointType(trans_q), 5, nn, dists);
            // 不为空
            if (nn.size() == 5) {
                std::vector<Vec3d> nn_eigen;
                for (size_t i = 0; i < nn.size(); ++i) {
                    nn_eigen.emplace_back(ToVec3d(target_->points[nn[i]]));
                }

                Vec3d p0, d;
                if (!math::FitLine(nn_eigen, p0, d, options_.max_line_distance_)) {
                    effect_pts[idx] = false;
                    return;
                }

                effect_pts[idx] = true;

                // 误差
                Vec3d e = SO3::hat(d) * (trans_q - p0);
                Eigen::Matrix<double, 3, 6> J_i;
                J_i.block<3, 3>(0, 0) = -pose.so3().matrix() * SO3::hat(vec_q);
                J_i.block<3, 3>(0, 3) = SO3::hat(vec_q);

                jacobians[idx] = J_i;
                errors[idx] = e;
            } else {
                effect_pts[idx] = false;
            }
        });
        // 计算dx
        double total_res = 0;
        int effective_num = 0;
        auto H_and_err = std::accumulate(
            source_index.begin(), source_index.end(), std::pair<Mat6d, Vec6d>(Mat6d::Zero(), Vec6d::Zero()),
            [&jacobians, &errors, &effect_pts, &total_res, &effective_num](const std::pair<Mat6d, Vec6d>& pre, int idx) -> std::pair<Mat6d, Vec6d> {
                if (!effect_pts[idx]) {
                    return pre;
                } else {
                    total_res += errors[idx].dot(errors[idx]);
                    effective_num++;
                    return std::pair<Mat6d, Vec6d>(pre.first + jacobians[idx].transpose() * jacobians[idx],
                                                   pre.second - jacobians[idx].transpose() * errors[idx]);
                }
            });
        if (effective_num < options_.min_effective_pts_) {
            LOG(WARNING) << "effective num too small: " << effective_num;
            return false;
        }

        Mat6d H = H_and_err.first;
        Vec6d err = H_and_err.second;

        Vec6d dx = H.inverse() * err;
        pose.so3() = pose.so3() * SO3::exp(dx.head<3>());
        pose.translation() += dx.tail<3>();

        // 更新
        LOG(INFO) << "iter " << iter << " total res: " << total_res << ", eff: " << effective_num << ", mean res: " << total_res / effective_num
                  << ", dxn: " << dx.norm();
        if (gt_set_) {
            LOG(INFO) << "iter pose: " << pose.so3().unit_quaternion().coeffs().transpose() << ", " << pose.translation().transpose();
            LOG(INFO) << "gt pose: " << gt_pose_.so3().unit_quaternion().coeffs().transpose() << ", " << gt_pose_.translation().transpose();

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
bool ICP3D::AlignP2Plane(SE3& init_pose) {
    LOG(INFO) << "Aligning with point to point";
    assert(target_ != nullptr && source_ != nullptr);

    SE3 pose = init_pose;
    // 不使用外部设置的initial_pose
    if (!options_.use_initial_translation_) {
        LOG(INFO) << " not use_initial_translation_";
        pose.translation() = target_center_ - source_center_;
    }
    std::vector<int> source_index(source_->size());
    for (int i = 0; i < source_index.size(); ++i) {
        source_index[i] = i;
    }
    // 统计所有的有效点
    std::vector<bool> effect_pts(source_index.size(), false);
    // e = p_i - R*q_i - t 是一个3x1的vector
    // de / dr , de / dt 构成3 * 6的jabobian矩阵
    std::vector<Eigen::Matrix<double, 3, 6>> jacobians(source_index.size());
    std::vector<double> errors(source_index.size());

    for (int iter = 0; iter < options_.max_iteration_; ++iter) {
        //  gauss-newton  迭代
        std::for_each(source_index.begin(), source_index.end(), [&](int idx) {
            auto vec_q = ToVec3d(source_->points[idx]);
            Vec3d trans_q = pose * vec_q;

            std::vector<int> nn;
            std::vector<float> dists;
            // kdtree查找
            kdtree_->nearestKSearch(ToPointType(trans_q), 5, nn, dists);
            // 不为空
            if (nn.size() == 5) {
                std::vector<Vec3d> nn_eigen;
                for (size_t i = 0; i < nn.size(); ++i) {
                    nn_eigen.emplace_back(ToVec3d(target_->points[nn[i]]));
                }

                Vec4d plane_coeffs;

                if (!math::FitPlane(nn_eigen, plane_coeffs)) {
                    effect_pts[idx] = false;
                    return;
                }

                double error = plane_coeffs.head<3>().dot(trans_q) + plane_coeffs[3];
                if (fabs(error) > options_.max_plane_distance_) {
                    effect_pts[idx] = false;
                    return;
                }

                // 误差
                Eigen::Matrix<double, 1, 6> J_i;
                J_i.block<1, 3>(0, 0) = -plane_coeffs.head<3>().transpose() * pose.so3().matrix() * SO3::hat(vec_q);
                J_i.block<1, 3>(0, 3) = plane_coeffs.head<3>().transpose();

                jacobians[idx] = J_i;
                errors[idx] = error;
            } else {
                effect_pts[idx] = false;
            }
        });
        // 计算dx
        double total_res = 0;
        int effective_num = 0;
        auto H_and_err = std::accumulate(
            source_index.begin(), source_index.end(), std::pair<Mat6d, Vec6d>(Mat6d::Zero(), Vec6d::Zero()),
            [&jacobians, &errors, &effect_pts, &total_res, &effective_num](const std::pair<Mat6d, Vec6d>& pre, int idx) -> std::pair<Mat6d, Vec6d> {
                if (!effect_pts[idx]) {
                    return pre;
                } else {
                    total_res += errors[idx] * (errors[idx]);
                    effective_num++;
                    return std::pair<Mat6d, Vec6d>(pre.first + jacobians[idx].transpose() * jacobians[idx],
                                                   pre.second - jacobians[idx].transpose() * errors[idx]);
                }
            });
        if (effective_num < options_.min_effective_pts_) {
            LOG(WARNING) << "effective num too small: " << effective_num;
            return false;
        }

        Mat6d H = H_and_err.first;
        Vec6d err = H_and_err.second;

        Vec6d dx = H.inverse() * err;
        pose.so3() = pose.so3() * SO3::exp(dx.head<3>());
        pose.translation() += dx.tail<3>();

        // 更新
        LOG(INFO) << "iter " << iter << " total res: " << total_res << ", eff: " << effective_num << ", mean res: " << total_res / effective_num
                  << ", dxn: " << dx.norm();
        if (gt_set_) {
            LOG(INFO) << "iter pose: " << pose.so3().unit_quaternion().coeffs().transpose() << ", " << pose.translation().transpose();
            LOG(INFO) << "gt pose: " << gt_pose_.so3().unit_quaternion().coeffs().transpose() << ", " << gt_pose_.translation().transpose();

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