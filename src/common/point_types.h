#ifndef SLAM_IN_AUTO_DRIVING_POINT_TYPES_H
#define SLAM_IN_AUTO_DRIVING_POINT_TYPES_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/impl/pcl_base.hpp>

#include "common/eigen_sophus.h"

namespace lh {
// 定义系统中用到的点和点云类型
using PointType = pcl::PointXYZI;
using PointCloudType = pcl::PointCloud<PointType>;
using CloudPtr = PointCloudType::Ptr;
using IndexVec = std::vector<int>;

// 点云到Eigen的转换函数
inline Vec3f ToVec3f(const PointType& pt) { return pt.getVector3fMap(); }
inline Vec3d ToVec3d(const PointType& pt) { return pt.getVector3fMap().cast<double>(); }

// 模板类型转换函数 声明
template <typename T, int dim>
inline Eigen::Matrix<T, dim, 1> ToEigen(const PointType& pt);
// 模板函数实现

template <>
inline Eigen::Matrix<float, 2, 1> ToEigen(const PointType& pt) {
    return Vec2f(pt.x, pt.y);
}

template <>
inline Eigen::Matrix<float, 3, 1> ToEigen(const PointType& pt) {
    return Vec3f(pt.x, pt.y, pt.z);
}

// 带ring,range等其他信息的全量信息点云
struct FullPointType {
    PCL_ADD_POINT4D;
    float range = 0;
    float radius = 0;
    uint8_t intensity = 0;
    uint8_t ring = 0;
    uint8_t angle = 0;
    double time = 0;
    float height = 0;

    inline FullPointType() {}
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using FullPointCloudType = pcl::PointCloud<FullPointType>;
using FullCloudPtr = FullPointCloudType::Ptr;

}  // namespace lh

#endif