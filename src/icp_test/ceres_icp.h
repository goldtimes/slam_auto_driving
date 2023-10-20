#pragma once
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <yaml-cpp/yaml.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "cloud_data.h"

class CeresIcp {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    CeresIcp(const YAML::Node &node);
    ~CeresIcp();
    bool setTargetCloud(const CloudPtr &target);
    bool P2P(const CloudPtr &source, const Eigen::Matrix4f &predict_pose, CloudPtr &transformed_source_ptr, Eigen::Matrix4f &result_pose);
    bool P2Plane(const CloudPtr &source, const Eigen::Matrix4f &predict_pose, CloudPtr &transformed_source_ptr, Eigen::Matrix4f &result_pose);

    float getFitnessScore();

   private:
    CloudPtr target_ptr, source_ptr;
    Eigen::Matrix4f final_pose;
    int max_iterations;
    float max_coresspoind_dis;
    float trans_eps;
    float euc_fitness_eps;

    double parameters[7] = {0, 0, 0, 1, 0, 0, 0};
    Eigen::Map<Eigen::Quaterniond> q_w_curr = Eigen::Map<Eigen::Quaterniond>(parameters);
    Eigen::Map<Eigen::Vector3d> t_w_curr = Eigen::Map<Eigen::Vector3d>(parameters + 4);

    pcl::KdTreeFLANN<PointType>::Ptr kdtree_flann;
};