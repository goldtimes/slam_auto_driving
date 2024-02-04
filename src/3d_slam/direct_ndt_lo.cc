#include "3d_slam/direct_ndt_lo.h"
#include "common/math_utils.h"
#include "pcl/common/transforms.h"
#include "tools/pcl_map_viewer.h"

namespace lh {
void DirectNDTLO::AddCloud(CloudPtr cloud, SE3& pose) {
    // 如果是第一帧
    if (local_map == nullptr) {
        local_map.reset(new PointCloudType());
        local_map = cloud;
        pose = SE3();
        last_kf_pose_ = pose;
        if (options_.use_pcl_ndt_) {
            pcl_ndt_.setInputTarget(local_map);
        } else {
            ndt3d_.SetTarget(local_map);
        }
        // 第一帧放到localmaps_queue_.emplace_back(cloud_world);
        // localmaps_queue_.emplace_back(cloud);
        return;
    }

    auto aligned_pose = AlignWithLocalMap(cloud);
    CloudPtr cloud_world(new PointCloudType());
    pcl::transformPointCloud(*cloud, *cloud_world, aligned_pose.matrix().cast<float>());
    bool is_keyframe = IsKeyframe(aligned_pose);
    if (is_keyframe) {
        // 记录pose
        last_kf_pose_ = aligned_pose;
        // 重构 localmaps
        localmaps_queue_.emplace_back(cloud_world);
        if (localmaps_queue_.size() > options_.num_kfs_in_local_map_) {
            localmaps_queue_.pop_front();
        }
        local_map.reset(new PointCloudType());
        for (const auto& cloud : localmaps_queue_) {
            *local_map += *cloud;
        }
        // 更新target点云
        if (options_.use_pcl_ndt_) {
            pcl_ndt_.setInputTarget(local_map);
        } else {
            ndt3d_.SetTarget(local_map);
        }
    }

    if (viewer_ != nullptr) {
        viewer_->SetPoseAndCloud(aligned_pose, cloud_world);
    }
}

SE3 DirectNDTLO::AlignWithLocalMap(CloudPtr cloud) {}

bool DirectNDTLO::IsKeyframe(const SE3& current_pose) {}

void DirectNDTLO::SaveMap(const std::string& map_path) {}
}  // namespace lh