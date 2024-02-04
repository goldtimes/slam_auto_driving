#pragma once

#include <nav_msgs/Odometry.h>
#include <pcl/registration/ndt.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <deque>
#include "3d_slam/ndt.hh"
#include "common/eigen_sophus.h"
#include "common/point_types.h"
#include "tools/pcl_map_viewer.h"

namespace lh {
struct Options {
    Options() {}
    double kf_distance_ = 0.5;
    double kf_angle_deg_ = 30;
    double num_kfs_in_local_map_ = 30;
    bool use_pcl_ndt_ = true;
    bool display_realtime_cloud_ = true;

    Ndt3d::Options ndt3d_options_;  // 自己写的ndt3d配置
};

class DirectNDTLO {
   public:
    DirectNDTLO(Options options = Options()) : options_(options) {
        if (options_.display_realtime_cloud_) {
            // 0.5为滤波的大小啊
            viewer_ = std::make_shared<PCLMapViewer>(0.5);
        }

        ndt3d_ = Ndt3d(options_.ndt3d_options_);
        pcl_ndt_.setResolution(1.0);
        pcl_ndt_.setStepSize(0.1);
        pcl_ndt_.setTransformationEpsilon(0.01);
    }

    void AddCloud(CloudPtr cloud, SE3& pose);

    void SaveMap(const std::string& map_path);

   private:
    SE3 AlignWithLocalMap(CloudPtr cloud);
    bool IsKeyframe(const SE3& current_pose);

   private:
    Options options_;

    CloudPtr local_map = nullptr;

    std::deque<CloudPtr> localmaps_queue_;
    std::vector<SE3> estimated_pose_;
    SE3 last_kf_pose_;

    pcl::NormalDistributionsTransform<PointType, PointType> pcl_ndt_;
    Ndt3d ndt3d_;

    std::shared_ptr<PCLMapViewer> viewer_ = nullptr;
};
}  // namespace lh