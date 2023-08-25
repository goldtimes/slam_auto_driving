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
}  // namespace lh