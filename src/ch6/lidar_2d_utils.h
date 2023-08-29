#pragma once

#include "common/eigen_sophus.h"
#include "common/lidar_utils.h"

#include <opencv2/core/core.hpp>

namespace lh {
/**
 * @brief 在image上绘制一个2D scan
 * @param scan
 * @param pose 机器人在世界坐标下的姿态 Twb
 * @param image
 * @param image_size
 * @param resolution 分辨率
 * @param pose_submap 子地图的位姿 Tws
 */
void Visualize2DScan(Scan2d::Ptr scan, const SE2& pose, cv::Mat& image, const Vec3b& color, int image_size = 800, float resolution = 20.0,
                     const SE2& pose_submap = SE2());
}  // namespace lh