#include <pcl/common/transforms.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <iostream>

#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <yaml-cpp/yaml.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <sophus/se3.hpp>
#include <string>
#include <thread>
#include "ceres_icp.h"
#include "cloud_data.h"

int main(int argc, char** argv) {
    std::string config_file = "/home/slam_auto_driving/config/config.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file);
    std::string target_path = config_node["target_file"].as<std::string>();
    std::string source_path = config_node["source_file"].as<std::string>();
    std::cout << "load target pcd:" << target_path << ",source path:" << source_path << std::endl;

    double ds_size = config_node["ds_size"].as<double>();
    CloudPtr cloud_source(new PointCloud());
    CloudPtr cloud_target(new PointCloud());
    CloudPtr transformed_cloud(new PointCloud());
    // load pcd
    if (pcl::io::loadPCDFile<PointType>(target_path, *cloud_target) == -1) {
        return -1;
    }
    if (pcl::io::loadPCDFile<PointType>(source_path, *cloud_source) == -1) {
        return -1;
    }

    std::shared_ptr<CeresIcp> icp = std::make_shared<CeresIcp>(config_node);

    // 滤波
    pcl::VoxelGrid<PointType> filter;
    filter.setLeafSize(ds_size, ds_size, ds_size);
    filter.setInputCloud(cloud_source);
    filter.filter(*cloud_source);
    filter.setInputCloud(cloud_target);
    filter.filter(*cloud_target);

    Eigen::AngleAxisd r_z(M_PI / 30, Eigen::Vector3d(0, 0, 1));
    Eigen::AngleAxisd r_x(M_PI / 130, Eigen::Vector3d(1, 0, 0));
    Eigen::Matrix4d Tr = Eigen::Matrix4d::Identity();
    Tr.block<3, 3>(0, 0) = (r_x * r_z).matrix();
    Tr.block<3, 1>(0, 3) = Eigen::Vector3d(0.1, -0.3, 0.2);
    pcl::transformPointCloud(*cloud_source, *cloud_target, Tr);
    // pcl::io::savePCDFile("/home/slam_auto_driving/data/icp_cloud/transformed_cloud.pcd", *transformed_cloud);
    std::cout << "origi T:\n" << Tr << std::endl;

    CloudPtr transformed_source(new PointCloud);
    icp->setTargetCloud(cloud_target);
    Eigen::Matrix4f T;
    icp->P2P(cloud_source, Eigen::Matrix4f::Identity(), transformed_source, T);
    std::cout << "after p2p T: \n" << T << std::endl;

    Eigen::Matrix4f T_p;
    icp->P2P(cloud_source, Eigen::Matrix4f::Identity(), transformed_source, T_p);
    std::cout << "after p2plan T: \n" << T_p << std::endl;

    return 0;
}
