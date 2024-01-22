#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Dense>
#include <iostream>

using PointType = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<PointType>;

PointCloud::Ptr current_scan;

Eigen::Matrix4f init_pose = Eigen::Matrix4f::Identity();
PointCloud::Ptr global_map;

struct SearchResult {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double score_;
    Eigen::Matrix4f init_pose_;
    Eigen::Matrix4f result_pose_;
};

PointCloud::Ptr ds_cloud(const PointCloud::Ptr& cloud_in, double resolution) {
    PointCloud::Ptr cloud_out(new PointCloud);
    pcl::VoxelGrid<PointType> vox;
    vox.setLeafSize(resolution, resolution, resolution);
    vox.setInputCloud(cloud_in);
    vox.filter(*cloud_out);
    return cloud_out;
}

void point_cloud_callback(const sensor_msgs::PointCloud2& cloud_msg) {
    current_scan.reset(new PointCloud());
    pcl::fromROSMsg(cloud_msg, *current_scan);
}

void align_for_map(SearchResult& sr) {
    pcl::NormalDistributionsTransform<PointType, PointType> ndt;
    ndt.setTransformationEpsilon(0.05);
    ndt.setStepSize(0.7);
    ndt.setMaximumIterations(40);
    ndt.setInputSource(current_scan);
    PointCloud::Ptr output_cloud(new PointCloud());
    Eigen::Matrix4f T;
    T = init_pose;
    std::vector<double> res{10.0, 5.0, 4.0, 3.0};
    for (const auto& r : res) {
        double resolution = r * 0.1;
        auto ds_global_map = ds_cloud(global_map, resolution);
        ndt.setInputTarget(ds_global_map);
        ndt.setResolution(resolution);
        ndt.align(*output_cloud, T);
        T = ndt.getFinalTransformation();
    }
    sr.score_ = ndt.getTransformationProbability();
    sr.result_pose_ = ndt.getFinalTransformation();
}

void initalpose_callback(const geometry_msgs::PoseWithCovarianceStamped& pose_msgs) {
    Eigen::Quaternionf quad = (Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitY()) *
                               Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitX()));
    quad.normalize();
    quad.w() = pose_msgs.pose.pose.orientation.w;
    quad.x() = pose_msgs.pose.pose.orientation.x;
    quad.y() = pose_msgs.pose.pose.orientation.y;
    quad.z() = pose_msgs.pose.pose.orientation.z;

    init_pose.block<3, 3>(0, 0) = quad.toRotationMatrix();
    init_pose.block<3, 1>(0, 3) << pose_msgs.pose.pose.position.x, pose_msgs.pose.pose.position.y, pose_msgs.pose.pose.position.z;
    std::vector<SearchResult> search_results;
    Eigen::Vector3f t(pose_msgs.pose.pose.position.x, pose_msgs.pose.pose.position.y, pose_msgs.pose.pose.position.z);
    for (int angle = 0; angle <= 360; angle += 10) {
        auto rotaion = Eigen::AngleAxisf(static_cast<float>(angle), Eigen::Vector3f::UnitZ()).toRotationMatrix();
        Eigen::Matrix4f guess_pose = Eigen::Matrix4f::Identity();
        guess_pose.block<3, 3>(0, 0) = rotaion;
        guess_pose.block<3, 1>(0, 3) = t;
        SearchResult sr;
        sr.init_pose_ = guess_pose;
        search_results.push_back(sr);
    }
    std::cout << "search pose: %ld" << search_results.size() << std::endl;

    // 搜索
    for (int i = 0; i < search_results.size(); ++i) {
        align_for_map(search_results[i]);
    }
    auto max_element = std::max_element(search_results.begin(), search_results.end(),
                                        [](const SearchResult& result_1, const SearchResult& result_2) { return result_1.score_ > result_2.score_; });
    // 打印pose和score
    std::cout << "best score:%d" << max_element->score_;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ndt_test");
    ros::NodeHandle nh;
    std::string global_map_path = "/home/kilox/global_map.pcd";
    global_map.reset(new PointCloud());
    pcl::io::loadPCDFile(global_map_path, *global_map);
    ros::Subscriber point_cloud_sub = nh.subscribe("/lslidar_point_cloud", 1, point_cloud_callback);
    ros::Subscriber initial_pose_sub = nh.subscribe("/initialpose", 1, initalpose_callback);

    ros::spin();
    return 0;
}