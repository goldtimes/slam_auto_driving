#include "icp_2d.h"
#include <glog/logging.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/impl/kdtree.hpp>
#include "common/math_utils.h"

namespace lh {
/**
 * 点到点的icp
 */
bool Icp2d::AlignGaussNewton(SE2& init_pose) {
    // 迭代次数
    int iterations = 10;
    double cost = 0;
    double last_cost = 0;
    SE2 current_pose = init_pose;
    const float max_dist2 = 0.01;   // 最近邻的最远距离平方
    const int min_effect_pts = 20;  // 最小有效点数

    for (int iter = 0; iter < iterations; ++iter) {
        Mat3d H = Mat3d::Zero();
        Vec3d b = Vec3d::Zero();
        cost = 0;

        int effective_num = 0;  // 有效点数
        // 遍历source
        for (size_t i = 0; i < source_scan_->ranges.size(); ++i) {
            float r = source_scan_->ranges[i];
            // 去除异常点
            if (r < source_scan_->range_min || r > source_scan_->range_max) {
                continue;
            }
            float angle = source_scan_->angle_min + i * source_scan_->angle_increment;
            float theta = current_pose.so2().log();
            // p^w = Twb * p^b
            Vec2d pw = current_pose * Vec2d(r * std::cos(angle), r * std::sin(angle));
            Point2d pt;
            pt.x = pw.x();
            pt.y = pw.y();

            // 最近邻
            std::vector<int> nn_idx;
            std::vector<float> dis;
            kdtree_.nearestKSearch(pt, 1, nn_idx, dis);
            if (nn_idx.size() > 0 && dis[0] < max_dist2) {
                effective_num++;
                Mat32d J;
                // clang-format off
                // 雅可比矩阵
                J << 1, 0, 
                     0, 1, 
                    -r * std::sin(angle + theta), r * std::cos(angle + theta);
                // 累积每一次的雅可比矩阵和
                H += J * J.transpose();
                // 误差
                Vec2d e(pt.x - target_cloud_->points[nn_idx[0]].x, pt.y - target_cloud_->points[nn_idx[0]].y);
                // 雅可比矩阵 * 误差
                b += -J * e;
                // 代价值
                cost += e.dot(e);
                // clang-format on
            }
        }
        // 找到的点小于20个，则认为不满足
        if (effective_num < min_effect_pts) {
            return false;
        }
        // solve for dx
        Vec3d dx = H.ldlt().solve(b);
        if (isnan(dx[0])) {
            break;
        }

        cost /= effective_num;
        // 确保了cost 是往下递减的，等于last_cost说明以及经迭代完了
        if (iter > 0 && cost >= last_cost) {
            break;
        }

        LOG(INFO) << "iter" << iter << " cost = " << cost << ", effect num: " << effective_num;
        current_pose.translation() += dx.head<2>();
        current_pose.so2() = current_pose.so2() * SO2::exp(dx[2]);
        last_cost = cost;
    }

    init_pose = current_pose;
    LOG(INFO) << "estimated pose: " << current_pose.translation().transpose() << ", theta: " << current_pose.so2().log();

    return true;
}

/**
 * 点到线的icp PL-ICP point-to-line
 */
bool Icp2d::AlignGaussNewtonPoint2Plane(SE2& init_pose) {
    int iterations = 10;
    double cost = 0;
    double last_cost = 0;

    SE2 current_pose = init_pose;
    const float max_dist = 0.3;       // 最近邻时的最远距离
    const float min_effect_pts = 20;  // 有效点数

    for (int iter = 0; iter < iterations; ++iter) {
        Mat3d H = Mat3d::Zero();
        Vec3d b = Vec3d::Zero();
        cost = 0;

        int effective_num = 0;
        // 遍历source
        for (size_t i = 0; i < source_scan_->ranges.size(); ++i) {
            float r = source_scan_->ranges[i];
            if (r < source_scan_->range_min || r > source_scan_->range_max) {
                continue;
            }

            float angle = source_scan_->angle_min + i * source_scan_->angle_increment;
            float theta = current_pose.so2().log();
            // point in world
            Vec2d pw = current_pose * Vec2d(r * std::cos(angle), r * std::sin(angle));
            Point2d pt;
            pt.x = pw.x();
            pt.y = pw.y();

            // 查找5个最邻近
            std::vector<int> nn_idx;
            std::vector<float> dis;

            kdtree_.nearestKSearch(pt, 5, nn_idx, dis);

            std::vector<Vec2d> effective_pts;
            for (int j = 0; j < nn_idx.size(); ++j) {
                if (dis[j] < max_dist) {
                    effective_pts.emplace_back(target_cloud_->points[nn_idx[j]].x, target_cloud_->points[nn_idx[j]].y);
                }
            }

            if (effective_pts.size() < 3) {
                continue;
            }
            // 拟合直线 a,b,c参数
            Vec3d line_coeffs;
            if (math::FitLine2D(effective_pts, line_coeffs)) {
                effective_num++;
                Vec3d J;
                J << line_coeffs[0], line_coeffs[1], line_coeffs[0] * r * std::sin(angle + theta) + line_coeffs[1] * r * std::sin(angle + theta);
                H += J * J.transpose();

                double e = line_coeffs[0] * pw[0] + line_coeffs[1] * pw[1] + line_coeffs[2];

                b += -J * e;

                cost += e * e;
            }
        }
        if (effective_num < min_effect_pts) {
            return false;
        }

        // solve for dx
        Vec3d dx = H.ldlt().solve(b);
        if (isnan(dx[0])) {
            break;
        }
        cost /= effective_num;
        if (iter > 0 && cost >= last_cost) {
            break;
        }

        LOG(INFO) << "iter " << iter << " cost = " << cost << ", effect num: " << effective_num;

        current_pose.translation() += dx.head<2>();
        current_pose.so2() = current_pose.so2() * SO2::exp(dx[2]);
        last_cost = cost;
    }
    init_pose = current_pose;
    LOG(INFO) << "estimated pose: " << current_pose.translation().transpose() << ", theta: " << current_pose.so2().log();

    return true;
}

void Icp2d::BuildTargetKdTree() {
    if (target_scan_ == nullptr) {
        LOG(ERROR) << "target is not set";
        return;
    }

    target_cloud_.reset(new Cloud2d);
    for (size_t i = 0; i < target_scan_->ranges.size(); ++i) {
        if (target_scan_->ranges[i] < target_scan_->range_min || target_scan_->ranges[i] > target_scan_->range_max) {
            continue;
        }
        double real_angle = target_scan_->angle_min + i * target_scan_->angle_increment;

        Point2d p;
        p.x = target_scan_->ranges[i] * std::cos(real_angle);
        p.y = target_scan_->ranges[i] * std::sin(real_angle);

        target_cloud_->points.push_back(p);
    }

    target_cloud_->width = target_cloud_->points.size();
    target_cloud_->is_dense = false;

    kdtree_.setInputCloud(target_cloud_);
}
}  // namespace lh