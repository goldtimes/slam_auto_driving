#include <gflags/gflags.h>
#include <glog/logging.h>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>

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
    // EPFL 雕像数据集：./ch7/EPFL/aquarius_{sourcd.pcd, target.pcd}，真值在对应目录的_pose.txt中
    // EPFL 模型比较精细，配准时应该采用较小的栅格

    std::ifstream fin(FLAGS_ground_truth_file);
    SE3 gt_pose;
    if (fin) {
        double tx, ty, tz, qw, qx, qy, qz;
        fin >> tx >> ty >> tz >> qw >> qx >> qy >> qz;
        fin.close();
        gt_pose = SE3(Quatd(qw, qx, qy, qz), Vec3d(tx, ty, tz));
    }

    lh::CloudPtr source(new lh::PointCloudType);
    lh::CloudPtr target(new lh::PointCloudType);

    pcl::io::loadPCDFile(fLS::FLAGS_source, *source);
    pcl::io::loadPCDFile(fLS::FLAGS_target, *target);

    bool success;

    // 利用pcl_viewer target icp_trans 可以查看匹配的效果
    evaluate_and_call(
        [&]() {
            lh::Icp3D icp;
            icp.SetSource(source);
            icp.SetTarget(target);
            SE3 pose;
            success = icp.AlignP2P(pose);
            if (success) {
                LOG(INFO) << "icp p2p align success, pose: " << pose.so3().unit_quaternion().coeffs().transpose() << ", "
                          << pose.translation().transpose();
                // 转换后的点云
                lh::CloudPtr source_trans(new lh::PointCloudType);
                pcl::transformPointCloud(*source, *source_trans, pose.matrix().cast<float>());
                lh::SaveCloudToFile("/home/slam_auto_driving/data/ch7/icp_trans.pcd", *source_trans);
            } else {
                LOG(ERROR) << "align failed.";
            }
        },
        "ICP p2p", 1);

    // 点到线
    evaluate_and_call(
        [&]() {
            lh::Icp3D icp;
            icp.SetSource(source);
            icp.SetTarget(target);
            icp.SetGroundTruth(gt_pose);
            SE3 pose;
            success = icp.AlignP2Line(pose);
            if (success) {
                LOG(INFO) << "icp p2line align success, pose: " << pose.so3().unit_quaternion().coeffs().transpose() << ", "
                          << pose.translation().transpose();
                lh::CloudPtr source_trans(new lh::PointCloudType);
                pcl::transformPointCloud(*source, *source_trans, pose.matrix().cast<float>());
                lh::SaveCloudToFile("/home/slam_auto_driving/data/ch7/icp_line_trans.pcd", *source_trans);
            } else {
                LOG(ERROR) << "align failed.";
            }
        },
        "ICP P2Line", 1);
    // 点到面
    // 测试下来 点到面的消耗的时间最短，并且匹配最好
    evaluate_and_call(
        [&]() {
            lh::Icp3D icp;
            icp.SetSource(source);
            icp.SetTarget(target);
            icp.SetGroundTruth(gt_pose);
            SE3 pose;
            success = icp.AlignP2Plane(pose);
            if (success) {
                LOG(INFO) << "icp p2plane align success, pose: " << pose.so3().unit_quaternion().coeffs().transpose() << ", "
                          << pose.translation().transpose();
                lh::CloudPtr source_trans(new lh::PointCloudType);
                pcl::transformPointCloud(*source, *source_trans, pose.matrix().cast<float>());
                lh::SaveCloudToFile("/home/slam_auto_driving/data/ch7/icp_plane_trans.pcd", *source_trans);
            } else {
                LOG(ERROR) << "align failed.";
            }
        },
        "ICP P2Plane", 1);

    /// 第７章的NDT
    evaluate_and_call(
        [&]() {
            lh::Ndt3d::Options options;
            options.voxel_size_ = 0.5;
            options.remove_centroid_ = true;
            options.nearby_type_ = lh::Ndt3d::NearbyType::CENTER;
            lh::Ndt3d ndt(options);
            ndt.SetSource(source);
            ndt.SetTarget(target);
            ndt.SetGtPose(gt_pose);
            SE3 pose;
            success = ndt.AlignNdt(pose);
            if (success) {
                LOG(INFO) << "ndt align success, pose: " << pose.so3().unit_quaternion().coeffs().transpose() << ", "
                          << pose.translation().transpose();
                lh::CloudPtr source_trans(new lh::PointCloudType);
                pcl::transformPointCloud(*source, *source_trans, pose.matrix().cast<float>());
                lh::SaveCloudToFile("/home/slam_auto_driving/data/ch7/ndt_trans.pcd", *source_trans);
            } else {
                LOG(ERROR) << "align failed.";
            }
        },
        "NDT", 1);
}