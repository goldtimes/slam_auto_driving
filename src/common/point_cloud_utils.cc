#include "common/point_cloud_utils.h"
#include "common/point_types.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

namespace lh {
void VoxelGrid(CloudPtr cloud, float voxel_size) {
    pcl::VoxelGrid<PointType> voxel;
    voxel.setLeafSize(voxel_size, voxel_size, voxel_size);
    voxel.setInputCloud(cloud);

    CloudPtr output(new PointCloudType);
    voxel.filter(*output);
    cloud->swap(*output);
}
// 这里采用了模板函数的头文件分离，会出现问题
template <typename CloudType>
void SaveCloudToFile(const std::string &filePath, CloudType &cloud) {
    cloud.height = 1;
    cloud.width = cloud.size();
    pcl::io::savePCDFileASCII(filePath, cloud);
}
// 需要实例化模板函数
template void SaveCloudToFile<PointCloudType>(const std::string &filePath, PointCloudType &cloud);

template void SaveCloudToFile<FullPointCloudType>(const std::string &filePath, FullPointCloudType &cloud);
}  // namespace lh