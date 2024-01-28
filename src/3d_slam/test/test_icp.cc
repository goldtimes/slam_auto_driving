
#include <gflags/gflags.h>
#include <glog/logging.h>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include "3d_slam/3d_icp.hh"
#include "common/point_cloud_utils.h"
#include "common/sys_utils.h"

DEFINE_string(source, "/home/slam_auto_driving/data/ch7/EPFL/kneeling_lady_source.pcd", "第1个点云路径");
DEFINE_string(target, "/home/slam_auto_driving/data/ch7/EPFL/kneeling_lady_target.pcd", "第2个点云路径");
DEFINE_string(ground_truth_file, "/home/slam_auto_driving/data/ch7/EPFL/kneeling_lady_pose.txt", "真值Pose");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::INFO;
    google::ParseCommandLineFlags(&argc, &argv, true);

    std::ifstream fin(FLAGS_ground_truth_file);
    SE3 gt_pose;
    if (fin) {
        double tx, ty, tz, qw, qx, qy, qz;
        fin >> tx >> ty >> tz >> qw >> qx >> qy >> qz;
        fin.close();
        gt_pose = SE3(Quatd(qw, qx, qy, qz), Vec3d(tx, ty, tz));
    }
    lh::CloudPtr source_trans_gt(new lh::PointCloudType);

    lh::CloudPtr source(new lh::PointCloudType), target(new lh::PointCloudType);
    pcl::io::loadPCDFile(fLS::FLAGS_source, *source);
    pcl::io::loadPCDFile(fLS::FLAGS_target, *target);

    pcl::transformPointCloud(*source, *source_trans_gt, gt_pose.matrix().cast<float>());
    lh::SaveCloudToFile("/home/slam_auto_driving/data/ch7/icp_trans_gt.pcd", *source_trans_gt);

    bool success;

    evaluate_and_call(
        [&]() {
            lh::ICP3D icp;
            icp.SetSource(source);
            icp.SetTarget(target);
            icp.SetGroundTruth(gt_pose);
            SE3 init_pose;
            success = icp.AlignP2P(init_pose);
            SE3 result_pose = init_pose;
            if (success) {
                LOG(INFO) << "icp p2p align success, pose: " << result_pose.so3().unit_quaternion().coeffs().transpose() << ", "
                          << result_pose.translation().transpose();
                lh::CloudPtr source_trans(new lh::PointCloudType);
                pcl::transformPointCloud(*source, *source_trans, result_pose.matrix().cast<float>());
                lh::SaveCloudToFile("/home/slam_auto_driving/data/ch7/icp_trans.pcd", *source_trans);
            } else {
                LOG(ERROR) << "align failed.";
            }
        },
        "ICP P2P", 1);

    return 0;
}
