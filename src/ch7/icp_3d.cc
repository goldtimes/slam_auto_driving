#include "icp_3d.h"
#include "common/math_utils.h"

#include <execution>

namespace lh {

void Icp3D::BuildTargetKdTree() {
    kdtree_ = std::make_shared<KdTree>();
    kdtree_->BuildTree(target_);
    kdtree_->SetEnableANN();
}

bool Icp3D::AlignP2P(SE3& init_pose) {
    LOG(INFO) << "aligning with point to point";
    assert(target_ != nullptr && source_ != nullptr);

    SE3 pose = init_pose;
    if (!options_.use_initial_translation_) {
        pose.translation() = target_center_ - source_center_;  // 设置平移初值
    }

    // 对点的索引
    std::vector<int> index(source_->points.size());
    for (int i = 0; i < index.size(); ++i) {
        index[i] = i;
    }
    // 并发代码
    std::vector<bool> effect_pts(index.size(), false);                 // 有效点
    std::vector<Eigen::Matrix<double, 3, 6>> jacobians(index.size());  // jacobians
    std::vector<Vec3d> errors(index.size());                           // 误差
    // gauss-newton 迭代次数
    for (int iter = 0; iter < options_.max_interation_; ++iter) {
        std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](int idx) {
            auto q = ToVec3d(source_->points[idx]);
            // 将source点转换到target坐标系中
            Vec3d qs = pose * q;
            std::vector<int> nn;
            // 获得最近的点
            kdtree_->GetClosestPoint(ToPointType(qs), nn, 1);

            if (!nn.empty()) {
                // 获得在target匹配的点
                Vec3d p = ToVec3d(target_->points[nn[0]]);
                double dis2 = (p - qs).squaredNorm();
                if (dis2 > options_.max_nn_distance_) {
                    // 点太远了
                    LOG(ERROR) << "dis2 < options_.max_nn_distance_";
                    effect_pts[idx] = false;
                    return;
                }
                effect_pts[idx] = true;

                // 误差
                Vec3d e = p - qs;
                Eigen::Matrix<double, 3, 6> J;
                // 7.3 公式
                J.block<3, 3>(0, 0) = pose.so3().matrix() * SO3::hat(q);
                J.block<3, 3>(0, 3) = -Mat3d::Identity();
                jacobians[idx] = J;
                errors[idx] = e;
            } else {
                effect_pts[idx] = false;
            }
        });

        // 上面完成被匹配点云的遍历
        // 累加Hessian矩阵和error, 计算dx
        double total_res = 0;
        int effective_num = 0;
        // 可以看到这里计算和之前的计算是一样的
        // H += J * J.transpose() b += -J.transpose() * e;
        auto H_and_err = std::accumulate(
            index.begin(), index.end(), std::pair<Mat6d, Vec6d>(Mat6d::Zero(), Vec6d::Zero()),
            [&jacobians, &errors, &effect_pts, &total_res, &effective_num](const std::pair<Mat6d, Vec6d> pre, int idx) -> std::pair<Mat6d, Vec6d> {
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
        // 更新位姿
        pose.so3() = pose.so3() * SO3::exp(dx.head<3>());
        pose.translation() += dx.tail<3>();

        // 更新
        LOG(INFO) << "iter:" << iter << "total res: " << total_res << ",eff:" << effective_num << ",mean res:" << total_res / effective_num
                  << ",dx norm:" << dx.norm();
        // 计算 真实的姿态和匹配后的姿态的误差
        if (gt_set_) {
            double pose_error = (gt_pose_.inverse() * pose).log().norm();
            LOG(INFO) << "iter: " << iter << ",pose error: " << pose_error;
        }
        // 收敛
        if (dx.norm() < options_.eps_) {
            LOG(INFO) << ",converged, dx = " << dx.transpose();
            break;
        }
    }
    init_pose = pose;
    return true;
}

bool Icp3D::AlignP2Plane(SE3& init_pose) {
    LOG(INFO) << "Align with point to plane";
    assert(target_ != nullptr && source_ != nullptr);

    SE3 pose = init_pose;
    if (!options_.use_initial_translation_) {
        pose.translation() = target_center_ - source_center_;  // 设置平移初始值
    }

    std::vector<int> index(source_->points.size());
    for (int i = 0; i < index.size(); ++i) {
        index[i] = i;
    }

    std::vector<bool> effect_pts(index.size(), false);
    // 注意这里的jacobian矩阵是1x6
    std::vector<Eigen::Matrix<double, 1, 6>> jacobians(index.size());
    std::vector<double> errors(index.size());
    // 迭代次数
    for (int iter = 0; iter < options_.max_interation_; ++iter) {
        std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](int idx) {
            auto q = ToVec3d(source_->points[idx]);
            Vec3d qs = pose * q;  // 转换之后的q
            std::vector<int> nn;
            kdtree_->GetClosestPoint(ToPointType(qs), nn, 5);  // 这里取5个最近邻
            // 将点云的点转换成vector3d
            if (nn.size() > 3) {
                std::vector<Vec3d> nn_eigen;
                for (int i = 0; i < nn.size(); ++i) {
                    nn_eigen.emplace_back(ToVec3d(target_->points[nn[i]]));
                }
                // 获取平面参数
                Vec4d n;
                if (!math::FitPlane(nn_eigen, n)) {
                    effect_pts[idx] = false;
                    return;
                }
                // 点距离拟合平面的距离7.5
                double dis = n.head<3>().dot(qs) + n[3];
                if (fabs(dis) > options_.max_plane_distance_) {
                    effect_pts[idx] = false;
                    return;
                }
                // 平面拟合成功 且距离平面的距离小于阈值
                effect_pts[idx] = true;

                // 构建残差
                Eigen::Matrix<double, 1, 6> J;
                // 7.11 公式
                J.block<1, 3>(0, 0) = -n.head<3>().transpose() * pose.so3().matrix() * SO3::hat(q);
                J.block<1, 3>(0, 3) = n.head<3>().transpose();

                jacobians[idx] = J;
                errors[idx] = dis;
            } else {
                effect_pts[idx] = false;
            }
        });

        // 计算dx
        double total_res = 0;
        int effective_num = 0;
        // accumulate 第三个参数是最后累加出来的值
        auto H_and_err = std::accumulate(
            index.begin(), index.end(), std::pair<Mat6d, Vec6d>(Mat6d::Zero(), Vec6d::Zero()),
            [&jacobians, &errors, &effect_pts, &total_res, &effective_num](const std::pair<Mat6d, Vec6d>& pre, int idx) -> std::pair<Mat6d, Vec6d> {
                if (!effect_pts[idx]) {
                    return pre;
                } else {
                    total_res += errors[idx] * errors[idx];
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

bool Icp3D::AlignP2Line(SE3& init_pose) {
    LOG(INFO) << "aligning with point to line";
    assert(target_ != nullptr && source_ != nullptr);
    SE3 pose = init_pose;
    if (options_.use_initial_translation_) {
        pose.translation() = target_center_ - source_center_;  // 设置平移初始值
        LOG(INFO) << "init trans set to " << pose.translation().transpose();
    }

    std::vector<int> index(source_->points.size());
    for (int i = 0; i < index.size(); ++i) {
        index[i] = i;
    }
    std::vector<bool> effect_pts(index.size(), false);
    std::vector<Eigen::Matrix<double, 3, 6>> jacobians(index.size());
    std::vector<Vec3d> errors(index.size());

    for (int iter = 0; iter < options_.max_interation_; ++iter) {
        // gauss-newton 迭代
        // 最近邻，可以并发
        std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](int idx) {
            auto q = ToVec3d(source_->points[idx]);
            Vec3d qs = pose * q;  // 转换之后的q
            std::vector<int> nn;
            kdtree_->GetClosestPoint(ToPointType(qs), nn, 5);  // 这里取5个最近邻
            if (nn.size() == 5) {
                // convert to eigen
                std::vector<Vec3d> nn_eigen;
                for (int i = 0; i < 5; ++i) {
                    nn_eigen.emplace_back(ToVec3d(target_->points[nn[i]]));
                }
                // 直线的方向向量，p0为直线上的一点
                Vec3d d, p0;
                if (!math::FitLine(nn_eigen, p0, d, options_.max_line_distance_)) {
                    // 失败的不要
                    effect_pts[idx] = false;
                    return;
                }

                // 方向向量 * pip0 向量 = 垂直距离
                Vec3d err = SO3::hat(d) * (qs - p0);

                if (err.norm() > options_.max_interation_) {
                    effect_pts[idx] = false;
                    return;
                }
                effect_pts[idx] = true;

                // 残差
                Eigen::Matrix<double, 3, 6> J;
                J.block<3, 3>(0, 0) = -SO3::hat(d) * pose.so3().matrix() * SO3::hat(q);
                J.block<3, 3>(0, 3) = SO3::hat(d);

                jacobians[idx] = J;
                errors[idx] = err;
            } else {
                effect_pts[idx] = false;
            }
        });

        // 累加Hessian和error,计算dx
        // 原则上可以用reduce并发，写起来比较麻烦，这里写成accumulate
        double total_res = 0;
        int effective_num = 0;
        auto H_and_err = std::accumulate(
            index.begin(), index.end(), std::pair<Mat6d, Vec6d>(Mat6d::Zero(), Vec6d::Zero()),
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

        if (gt_set_) {
            double pose_error = (gt_pose_.inverse() * pose).log().norm();
            LOG(INFO) << "iter " << iter << " pose error: " << pose_error;
        }

        // 更新
        LOG(INFO) << "iter " << iter << " total res: " << total_res << ", eff: " << effective_num << ", mean res: " << total_res / effective_num
                  << ", dxn: " << dx.norm();

        if (dx.norm() < options_.eps_) {
            LOG(INFO) << "converged, dx = " << dx.transpose();
            break;
        }
    }
    init_pose = pose;
    return true;
}

}  // namespace lh
