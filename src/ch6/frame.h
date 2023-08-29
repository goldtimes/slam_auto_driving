#ifndef SLAM_IN_AUTO_DRIVING_FRAME_H
#define SLAM_IN_AUTO_DRIVING_FRAME_H

#include "common/eigen_sophus.h"
#include "common/lidar_utils.h"

namespace lh {
/**
 * 2d scan
 */
struct Frame {
    Frame() {}
    Frame(Scan2d::Ptr scan) : scan_(scan) {}

    /**
     * @brief 当前帧存储为文本，离线调用
     */
    void Dump(const std::string& filename);

    /**
     * @brief 文件中读取frame数据
     */
    void Load(const std::string& filename);

    size_t id_ = 0;           // scan id
    size_t keyframe_id_ = 0;  // 关键帧id
    double timestamp_ = 0;    // 时间戳，可能用不上
    Scan2d::Ptr scan_ = nullptr;
    SE2 pose_;         // T_scanToworld
    SE2 pose_submap_;  // T_submapToscan
};

}  // namespace lh

#endif