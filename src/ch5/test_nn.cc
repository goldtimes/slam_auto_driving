#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include "bfnn.h"
#include "common/point_types.h"

DEFINE_string(first_scan_path, "/home/slam_auto_driving/data/ch5/first.pcd", "第一个点云路径");
DEFINE_string(second_scan_path, "/home/slam_auto_driving/data/ch5/second.pcd", "第二个点云路径");

TEST(CH5_TEST, BFNN) {
    lh::CloudPtr first(new lh::PointCloudType);
    lh::CloudPtr second(new lh::PointCloudType);
    pcl::io::loadPCDFile(FLAGS_first_scan_path, *first);
    pcl::io::loadPCDFile(FLAGS_second_scan_path, *second);

    if (first->empty() || second->empty()) {
        LOG(ERROR) << "cannot load cloud";
        FAIL();
    }

    // 体素滤波
}

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;

    testing::InitGoogleTest(&argc, argv);
    google::ParseCommandLineFlags(&argc, &argv, true);
    return RUN_ALL_TESTS();
}