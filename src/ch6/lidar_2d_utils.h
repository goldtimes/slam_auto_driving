#pragma once

#include "common/eigen_sophus.h"
#include "common/lidar_utils.h"

#include <opencv2/core/core.hpp>

namespace lh {
void Visualize2DScan(Scan2d::Ptr scan, const SE2& pose, cv::Mat& image, const Vec3b& color, int image_size = 800, float resolution = 20.0,
                     const SE2& pose_submap = SE2());
}