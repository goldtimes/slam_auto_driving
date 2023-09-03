#ifndef SLAM_IN_AUTO_DRIVING_DIRECT_NDT_LO_H
#define SLAM_IN_AUTO_DRIVING_DIRECT_NDT_LO_H

#include <pcl/registration/ndt.h>
#include <deque>

#include "common/eigen_sophus.h"
#include "common/point_types.h"
#include "ndt_3d.h"
#include "tools/pcl_map_viewer.h"

namespace lh {
/**
 * 使用直接NDT方法进行递推的LO
 * 使用历史几个关键帧作为local_map，进行NDT定位
 */
class DirectNDTLO {
   public:
    struct Options {
        Options() {}
        double kf_distance_ = 0.5;       // 关键帧距离
        double kf_angle_deg_ = 30;       // 旋转角度
        int num_kfs_in_local_map_ = 30;  // 局部地图包含多少关键帧
        bool use_pcl_ndt_ = true;
        bool display_realtime_cloud_ = true;

        Ndt3d::Options ndt3d_options_;  // NDT3D的配置
    };

    DirectNDTLO(Options options = Options()) : options_(options) {}

    void AddCloud(CloudPtr scan, SE3& pose);
    void SaveMap(const std::string& map_path);

   private:
    /**
     * @brief 于local map进行配准
     */
    SE3 AlignWithLocalMap(CloudPtr scan);
    /**
     * @brief 判断是否为关键帧
     */
    bool IsKeyframe(const SE3& current_pose);

   private:
    Options options_;
    CloudPtr local_map_ = nullptr;
    std::deque<CloudPtr> scans_in_local_map_;
    std::vector<SE3> estimated_pose_;  // 所有估计出来的pose, 用于记录轨迹和预测下一帧
    SE3 last_kf_pose_;                 // 上一帧关键帧的位姿
    // pcl的ndt
    pcl::NormalDistributionsTransform<PointType, PointType> ndt_pcl_;
    Ndt3d ndt_;

    std::shared_ptr<PCLMapViewer> viewer_ = nullptr;
};
}  // namespace lh

#endif