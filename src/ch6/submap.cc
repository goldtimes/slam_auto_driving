#include "submap.h"
#include <glog/logging.h>

namespace lh {
void Submap::SetOccuFromOtherSubmap(std::shared_ptr<Submap> other) {
    auto frames_in_other = other->GetFrames();
    // 取vector中最新的10个帧
    for (size_t i = frames_in_other.size() - 10; i < frames_in_other.size(); ++i) {
        if (i > 0) {
            occu_map_.AddLidarFrame(frames_in_other[i]);
        }
    }
    field_.SetFieldImageFromOccuMap(occu_map_.GetOccupancyGrid());
}

bool Submap::MatchScan(std::shared_ptr<Frame> frame) {
    field_.SetSourceScan(frame->scan_);
    field_.AlignG2O(frame->pose_submap_);
    // T_wc = Tws * Tsc
    frame->pose_ = pose_ * frame->pose_submap_;
    return true;
}

void Submap::AddScanInOccupancyMap(std::shared_ptr<Frame> frame) {
    // 更新栅格地图
    occu_map_.AddLidarFrame(frame, OccupancyMap::GridMethod::BRESENHAM);
    // 更新场函数
    field_.SetFieldImageFromOccuMap(occu_map_.GetOccupancyGrid());
}

bool Submap::HasOutsidePoint() const { return occu_map_.HasOutSidePoints(); }

void Submap::SetPose(const SE2& pose) {
    pose_ = pose;
    occu_map_.SetPose(pose);
    field_.SetPose(pose);
}

void Submap::UpdateFramePoseWorld() {
    for (auto& frame : frames_) {
        // Twc = Tws * Tsc
        frame->pose_ = pose_ * frame->pose_submap_;
    }
}
}  // namespace lh