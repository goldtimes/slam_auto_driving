#include "3d_slam/incremental_ndt_lo.hh"
#include "common/math_utils.h"
#include "common/timer/timer.h"

namespace lh {
void INcrementalNDTLO::AddCloud(CloudPtr scan, SE3& pose, bool use_guess) {
    // 第一帧
    if (first_frame) {
        pose = SE3();
        last_kf_pose_ = pose;
        inc_ndt_.AddCloud(scan);
        first_frame = false;
        return;
    }

    SE3 guess_pose;
    inc_ndt_.SetSource(scan);
    if (estimated_poses_.size() < 2) {
        // 第二帧
        inc_ndt_.AlignNdt(guess_pose);
    } else {
        // 不使用外部传入的guess pose
        if (!use_guess) {
            SE3 T1 = estimated_poses_[estimated_poses_.size() - 1];
            SE3 T2 = estimated_poses_[estimated_poses_.size() - 1];
            auto last_delta = T2.inverse() * T1;
            guess_pose = T1 * last_delta;
        } else {
            guess_pose = pose;
        }
        inc_ndt_.AlignNdt(guess_pose);
    }
    // ndt 配准后更新了guess_pose
    pose = guess_pose;
    estimated_poses_.emplace_back(pose);
    // 判断是否为keyframe 来更新localmap
    CloudPtr scan_world(new PointCloudType);
    pcl::transformPointCloud(*scan, *scan_world, pose.matrix().cast<float>());

    if (IsKeyframe(pose)) {
        last_kf_pose_ = pose;
        cnt_frame_ = 0;  // keyframe置为0
        // 调用addcloud,增量更新体素
        inc_ndt_.AddCloud(scan_world);
    }
    if (viewer_ != nullptr) {
        viewer_->SetPoseAndCloud(pose, scan_world);
    }
    cnt_frame_++;
}

void INcrementalNDTLO::SaveMap(const std::string& map_path) {
    if (viewer_ != nullptr) {
        viewer_->SaveMap(map_path);
    }
}

bool INcrementalNDTLO::IsKeyframe(const SE3& current_pose) {
    if (cnt_frame_ > 10) {
        return true;
    }
    SE3 delta = last_kf_pose_.inverse() * current_pose;
    return delta.translation().norm() > options_.kf_distance || delta.so3().log().norm() > options_.kf_angle * math::kDEG2RAD;
}

}  // namespace lh