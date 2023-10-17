#ifndef SLAM_IN_AUTO_DRIVING_KEYFRAME_H
#define SLAM_IN_AUTO_DRIVING_KEYFRAME_H

#include <map>
#include "common/eigen_sophus.h"
#include "common/point_types.h"

namespace lh {
struct Keyframe {
    Keyframe() {}
    Keyframe(double time, IdType id, const SE3& lidar_pose, CloudPtr cloud) : timestamp_(time), id_(id), lidar_pose_(lidar_pose), cloud_(cloud) {}
    // dump点云
    void SaveAndUnloadScan(const std::string& path);

    void LoadScan(const std::string& path);

    void Save(std::ostream& os);

    void Load(std::istream& is);

    double timestamp_ = 0;  // 时间戳
    IdType id_ = 0;         // 关键帧id
    SE3 lidar_pose_;        // 雷达pose
    SE3 rtk_pose_;
    SE3 opti_pose_1_;                 // 第一阶段优化pose
    SE3 opti_pose_2_;                 // 第二阶段优化pose
    bool rtk_heading_valid_ = false;  // rtk旋转
    bool rtk_valid_ = true;           // rtk 状态
    bool rtk_inlier_ = true;          // rtk优化过程是否为正常值
    CloudPtr cloud_ = nullptr;
};
bool LoadKeyFrames(const std::string& path, std::map<IdType, std::shared_ptr<Keyframe>>& keyframes);
}  // namespace lh

using KFPtr = std::shared_ptr<lh::Keyframe>;

#endif