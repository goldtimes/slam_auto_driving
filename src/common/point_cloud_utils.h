#ifndef SLAM_IN_AUTO_DRIVING_POINT_CLOUD_UTILS_H
#define SLAM_IN_AUTO_DRIVING_POINT_CLOUD_UTILS_H

#include "common/point_types.h"

namespace lh {
// 滤波
void VoxelGrid(CloudPtr cloud, float voxel_size = 0.05);

/// 写点云文件
template <typename CloudType>
void SaveCloudToFile(const std::string &filePath, CloudType &cloud);

}  // namespace lh

#endif