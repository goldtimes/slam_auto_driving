#include <gflags/gflags.h>
#include <glog/logging.h>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/search/kdtree.h>

#include "ch7/icp_3d.h"
#include "ch7/ndt_3d.h"
#include "common/point_cloud_utils.h"
#include "common/sys_utils.h"

DEFINE_string(source, "/sad/data/ch7/EPFL/kneeling_lady_source.pcd", "第1个点云路径");
DEFINE_string(target, "/sad/data/ch7/EPFL/kneeling_lady_target.pcd", "第2个点云路径");
DEFINE_string(ground_truth_file, "/sad/data/ch7/EPFL/kneeling_lady_pose.txt", "真值Pose");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    SE3 gt_pose;
    lh::CloudPtr source(new lh::PointCloudType), target(new lh::PointCloudType);
    pcl::io::loadPCDFile(fLS::FLAGS_source, *source);
    pcl::io::loadPCDFile(fLS::FLAGS_target, *target);
    /// PCL ICP 作为备选
    evaluate_and_call(
        [&]() {
            pcl::IterativeClosestPoint<lh::PointType, lh::PointType> icp_pcl;
            icp_pcl.setInputSource(source);
            icp_pcl.setInputTarget(target);
            lh::CloudPtr output_pcl(new lh::PointCloudType);
            icp_pcl.align(*output_pcl);
            SE3f T = SE3f(icp_pcl.getFinalTransformation());
            LOG(INFO) << "pose from icp pcl: " << T.so3().unit_quaternion().coeffs().transpose() << ", " << T.translation().transpose();
            lh::SaveCloudToFile("/home/slam_auto_driving/data/pcl_icp_trans.pcd", *output_pcl);

            // 计算GT pose差异
            double pose_error = (gt_pose.inverse() * T.cast<double>()).log().norm();
            LOG(INFO) << "ICP PCL pose error: " << pose_error;
        },
        "ICP PCL", 1);
    // pcl GICP

    evaluate_and_call(
        [&]() {
            // pcl::IterativeClosestPoint<lh::PointType, lh::PointType> icp_pcl;
            pcl::GeneralizedIterativeClosestPoint<lh::PointType, lh::PointType>::Ptr gicp;
            gicp.reset(new pcl::GeneralizedIterativeClosestPoint<lh::PointType, lh::PointType>());
            gicp->setMaxCorrespondenceDistance(5);
            gicp->setMaximumIterations(50);
            gicp->setTransformationEpsilon(1e-6);
            gicp->setEuclideanFitnessEpsilon(1e-6);
            gicp->setRANSACIterations(1);

            pcl::search::KdTree<lh::PointType>::Ptr source_tree(new pcl::search::KdTree<lh::PointType>);
            pcl::search::KdTree<lh::PointType>::Ptr target_tree(new pcl::search::KdTree<lh::PointType>);
            source_tree->setInputCloud(source);
            target_tree->setInputCloud(target);
            gicp->setSearchMethodSource(source_tree);
            gicp->setSearchMethodTarget(target_tree);
            gicp->setInputTarget(target);

            gicp->setInputSource(source);
            lh::CloudPtr output_pcl(new lh::PointCloudType);
            gicp->align(*output_pcl);
            SE3f T = SE3f(gicp->getFinalTransformation());
            LOG(INFO) << "pose from gicp pcl: " << T.so3().unit_quaternion().coeffs().transpose() << ", " << T.translation().transpose();
            lh::SaveCloudToFile("/home/slam_auto_driving/data/pcl_gicp_trans.pcd", *output_pcl);

            // 计算GT pose差异
            double pose_error = (gt_pose.inverse() * T.cast<double>()).log().norm();
            LOG(INFO) << "gICP PCL pose error: " << pose_error;

            if (gicp->hasConverged()) {
                LOG(INFO) << "gICP score: " << gicp->getFitnessScore();
            }
        },
        "ICP PCL", 1);
}