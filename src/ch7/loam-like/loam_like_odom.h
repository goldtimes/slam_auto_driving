
#ifndef SLAM_IN_AUTO_DRIVING_LOAM_LIKE_ODOM_H
#define SLAM_IN_AUTO_DRIVING_LOAM_LIKE_ODOM_H

#include "ch5/kdtree.h"
#include "ch7/icp_3d.h"
#include "common/eigen_sophus.h"
#include "common/point_types.h"
#include "tools/pcl_map_viewer.h"

#include <deque>

namespace lh {
class FeatureExtraction;

/**
 * @brief 类loam的里程计
 * 使用feature extraction 提出点云中的边缘点和平面点
 * 分别对角点和平面点用不同方法的icp
 */
class LoamLikeOdom {
   public:
    struct Options {
        Options() {}

        int min_edge_pts_ = 20;
        int min_surf_pts_ = 20;
        double kf_distance_ = 1.0;       // 关键帧距离
        double kf_angle_deg_ = 15;       // 关键帧旋转角度
        int num_kfs_in_local_map_ = 30;  //关键帧个数
        bool display_realtime_cloud_ = true;

        // icp参数
        int max_iteration_ = 5;
        double max_plane_distance_ = 0.05;  // 平面最近邻阈值
        double max_line_distance_ = 0.5;
        int min_effective_pts_ = 10;
        double eps_ = 1e3;  // 收敛条件

        bool use_edge_points_ = true;
        bool use_surf_points_ = true;
    };
    explicit LoamLikeOdom(Options options = Options());

    /**
     * @brief 往里程计添加一个点云
     */
    void ProcessPointCloud(FullCloudPtr full_cloud);

    void SaveMap(const std::string& path);

   private:
    SE3 AlignWithLocalMap(CloudPtr edge, CloudPtr surf);
    bool IsKeyFrame(const SE3& current_pose);

   private:
    Options options_;
    int cnt_frame_ = 0;
    int last_kf_id_ = 0;
    CloudPtr local_map_edge_ = nullptr;  // 局部地图
    CloudPtr local_map_surf_ = nullptr;

    std::vector<SE3> estimated_poses_;  // 所有估计出来的poses, 用于记录轨迹和预测下一帧

    SE3 last_kf_pose_;  // 上一关键帧的位姿

    std::deque<CloudPtr> edges_, surfs_;  // 缓存的角点和平面点
    CloudPtr global_map_ = nullptr;

    std::shared_ptr<FeatureExtraction> feature_extraction_ = nullptr;
    std::shared_ptr<PCLMapViewer> viewer_ = nullptr;
    KdTree kdtree_edge_;
    KdTree kdtree_surf_;
};

}  // namespace lh

#endif