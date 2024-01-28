#pragma once
#include <glog/logging.h>
#include <pcl/search/kdtree.h>
#include "common/eigen_sophus.h"
#include "common/point_types.h"

namespace lh {
class ICP3D {
   public:
    struct Options {
        int max_iteration_ = 20;                //最大迭代次数
        double max_nn_distance_ = 1.0;          // 点-点的最大距离
        double max_plane_distance_ = 0.05;      // 点到平面的最大距离阈值
        double max_line_distance_ = 0.5;        // 点到线的最大距离阈值
        int min_effective_pts_ = 10;            // 有效的点个数阈值
        double eps_ = 1e-2;                     // 收敛阈值
        bool use_initial_translation_ = false;  // 是否使用初始姿态中的平移估计
        bool use_ceres_ = false;
    };

    ICP3D() {}
    ICP3D(Options options) : options_(options) {}

    void SetTarget(CloudPtr target) {
        target_ = target;
        kdtree_.reset(new pcl::search::KdTree<PointType>());
        kdtree_->setInputCloud(target_);

        // 计算点云中心
        target_center_ = std::accumulate(target_->points.begin(), target_->points.end(), Vec3d::Zero().eval(),
                                         [](const Vec3d& c, const PointType& pt) -> Vec3d { return c + ToVec3d(pt); }) /
                         target_->size();
        LOG(INFO) << "target center: " << target_center_.transpose();
    }
    void SetSource(CloudPtr source) {
        source_ = source;
        source_center_ = std::accumulate(source_->points.begin(), source_->points.end(), Vec3d::Zero().eval(),
                                         [](const Vec3d& c, const PointType& pt) -> Vec3d { return c + ToVec3d(pt); }) /
                         source_->size();
        LOG(INFO) << "source center: " << source_center_.transpose();
    }

    void SetGroundTruth(const SE3& gt_pose) {
        gt_set_ = true;
        gt_pose_ = gt_pose;
    }

    bool AlignP2P(SE3& init_pose);
    bool AlignP2Line(SE3& init_pose);
    bool AlignP2Plane(SE3& init_pose);

   private:
    PointType VecPointToPcl(const Vec3d point) {
        PointType pcl_point;
        pcl_point.x = point.x();
        pcl_point.y = point.y();
        pcl_point.z = point.z();
        return pcl_point;
    }

   private:
    Options options_;

    CloudPtr target_;
    CloudPtr source_;
    pcl::search::KdTree<PointType>::Ptr kdtree_;
    Vec3d target_center_;
    Vec3d source_center_;
    bool gt_set_ = false;  // 真值
    SE3 gt_pose_;
};
}  // namespace lh