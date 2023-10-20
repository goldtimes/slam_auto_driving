#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using PointType = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<PointType>;
using CloudPtr = PointCloud::Ptr;