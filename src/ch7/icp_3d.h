#ifndef SLAM_IN_AUTO_DRIVING_ICP_3D_H
#define SLAM_IN_AUTO_DRIVING_ICP_3D_H

#include "ch5/kdtree.h"

namespace lh {
/**
 * 3D 的ICP
 * 使用第5章的kdtree来求解最近邻
 */
class Icp3D {
   public:
    struct Options {
        int max_interation_ = 20;               // 迭代次数
        double max_nn_distance_ = 1.0;          // 点到点最邻近查找时的阈值
        double max_plane_distance_ = 0.05;      // 点到面的最邻近查找阈值
        double max_line_distance_ = 0.5;        // 点到线的最邻近查找阈值
        int min_effective_pts_ = 10;            // 最邻近点数阈值
        double eps_ = 1e-2;                     // 收敛的判定条件
        bool use_initial_translation_ = false;  // 是否使用初始化姿态中的平移估计
    };

    Icp3D() {}
    Icp3D(Options options) : options_(options) {}
    /**
     * @brief 设置目标点云，并计算目标点云的中心
     */
    void SetTarget(CloudPtr target) {
        target_ = target;
        BuildTargetKdTree();
        // 计算点云中心
        target_center_ = std::accumulate(target_->points.begin(), target_->points.end(), Vec3d::Zero().eval(),
                                         [](const Vec3d& c, const PointType& pt) -> Vec3d { return c + ToVec3d(pt); }) /
                         target_->points.size();
        LOG(INFO) << "target center: " << target_center_.transpose();
    }
    /**
     * @brief 设置被配准的点云，并计算source点云的中心
     */
    void SetSource(CloudPtr source) {
        source_ = source;
        source_center_ = std::accumulate(source_->points.begin(), source_->points.end(), Vec3d::Zero().eval(),
                                         [](const Vec3d& c, const PointType& pt) -> Vec3d { return c + ToVec3d(pt); }) /
                         source_->size();
        LOG(INFO) << "source center: " << source_center_.transpose();
    }
    /**
     * @brief 设置真实的姿态
     */
    void SetGroundTruth(const SE3& gt_pose) {
        gt_pose_ = gt_pose;
        gt_set_ = true;
    }

    /**
     * @brief 点到点的ICP
     */
    bool AlignP2P(SE3& init_pose);
    /**
     * @brief 点到线的ICP
     */
    bool AlignP2Line(SE3& init_pose);
    /**
     * @brief 点到面的ICP
     */
    bool AlignP2Plane(SE3& init_pose);

   private:
    void BuildTargetKdTree();

   private:
    std::shared_ptr<KdTree> kdtree_ = nullptr;
    CloudPtr target_ = nullptr;
    CloudPtr source_ = nullptr;

    Vec3d target_center_ = Vec3d::Zero();
    Vec3d source_center_ = Vec3d::Zero();

    bool gt_set_ = false;  // 真值是否设置
    SE3 gt_pose_;

    Options options_;
};

}  // namespace lh

#endif