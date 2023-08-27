#pragma once

#include "common/eigen_sophus.h"
#include "common/lidar_utils.h"

#include <pcl/search/kdtree.h>

namespace lh {
/**
 * icp代码
 * 1. setTarget将点云构建kd数，setSource 设置想要匹配的点云 Align方法获得输出
 */
class Icp2d {
   public:
    using Point2d = pcl::PointXY;
    using Cloud2d = pcl::PointCloud<Point2d>;
    Icp2d() {}

    // 设置目标的scan
    void SetTarget(Scan2d::Ptr target) {
        target_scan_ = target;
        BuildTargetKdTree();
    }
    // 设置被配准的scan
    void setSource(Scan2d::Ptr source) { source_scan_ = source; }
    /**
     * @brief  使用高斯牛顿进行匹配
     *
     * 1/2||f(x)||_2^2  f(x)为误差函数
     * H * deltaX = b
     * H = J(x) * J(x).transpose()
     * b = -J(x) * f(x)
     * deltaX = H.ldlt().slove(b)
     *
     */
    bool AlignGaussNewton(SE2& init_pose);
    // 高斯牛顿 point-to-plane
    bool AlignGaussNewtonPoint2Plane(SE2& init_pose);

   private:
    /**
     * 手动将scan->pointcloud
     */
    void BuildTargetKdTree();

    pcl::search::KdTree<Point2d> kdtree_;
    Cloud2d::Ptr target_cloud_;
    Scan2d::Ptr target_scan_ = nullptr;
    Scan2d::Ptr source_scan_ = nullptr;
};
}  // namespace lh