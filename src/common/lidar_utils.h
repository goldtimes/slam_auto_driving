#pragma once
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/MultiEchoLaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include "point_types.h"
#include "velodyne_msgs/VelodyneScan.h"

using Scan2d = sensor_msgs::LaserScan;
using MutilScan2d = sensor_msgs::MultiEchoLaserScan;
using PacketsMsg = velodyne_msgs::VelodyneScan;
using PacketsMsgPtr = boost::shared_ptr<PacketsMsg>;

namespace lh {
inline Scan2d::Ptr MultiToScan2d(MutilScan2d::Ptr mscan) {
    Scan2d::Ptr scan(new Scan2d);
    scan->header = mscan->header;
    scan->range_max = mscan->range_max;
    scan->range_min = mscan->range_min;
    scan->angle_increment = mscan->angle_increment;
    scan->angle_max = mscan->angle_max;
    scan->angle_min = mscan->angle_min;
    for (auto r : mscan->ranges) {
        if (r.echoes.empty()) {
            scan->ranges.emplace_back(scan->range_max + 1.0);
        } else {
            scan->ranges.emplace_back(r.echoes[0]);
        }
    }
    for (auto i : mscan->intensities) {
        if (i.echoes.empty()) {
            scan->intensities.emplace_back(0);
        } else {
            scan->intensities.emplace_back(i.echoes[0]);
        }
    }
    scan->scan_time = mscan->scan_time;
    scan->time_increment = mscan->time_increment;

    scan->range_max = 20.0;
    return scan;
}

/**
 * @brief ros point cloud to pcl point cloud
 */
inline CloudPtr PointCloud2ToCloudPtr(sensor_msgs::PointCloud2::Ptr msgs) {
    CloudPtr cloud(new PointCloudType);
    pcl::fromROSMsg(*msgs, *cloud);
    return cloud;
}

/**
 * 其他类型点云转到PointType点云
 * 将全量点云类型转到XYZI点云
 */
template <typename PointT = FullPointType>
CloudPtr ConvertToCloud(typename pcl::PointCloud<PointT>::Ptr input) {
    CloudPtr cloud(new PointCloudType);
    for (auto& pt : input->points) {
        PointType point;
        point.x = pt.x;
        point.y = pt.y;
        point.z = pt.z;
        point.intensity = pt.intensity;
        cloud->points.template emplace_back(point);
    }
    cloud->width = input->width;
    return cloud;
}

/**
 * @brief 体素滤波
 */
inline CloudPtr VoxelCloud(CloudPtr cloud, float voxel_size = 0.1) {
    pcl::VoxelGrid<PointType> voxel;
    voxel.setLeafSize(voxel_size, voxel_size, voxel_size);
    voxel.setInputCloud(cloud);

    CloudPtr output(new PointCloudType);
    voxel.filter(*output);
    return output;
}

template <typename S, int n>
inline Eigen::Matrix<int, n, 1> CastToInt(const Eigen::Matrix<S, n, 1>& value) {
    return value.array().template round().template cast<int>();
}

}  // namespace lh
