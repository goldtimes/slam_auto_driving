#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/highgui.hpp>

#include "common/io_utils.h"
#include "lidar_2d_utils.h"

DEFINE_string(bag_path, "./dataset/sad/2dmapping/test_2d_lidar.bag", "数据包路径");

/**
 * 测试rosbag中读取2d scan数据
 */
int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);
    lh::RosbagIO rosbag_io(fLS::FLAGS_bag_path);
    rosbag_io
        .AddScan2DHandler("/pavo_scan_bottom",
                          [](Scan2d::Ptr scan) {
                              cv::Mat img;
                              lh::Visualize2DScan(scan, SE2(), img, Vec3b(255, 0, 0));
                              cv::imshow("scan", img);
                              cv::waitKey(20);
                              return true;
                          })
        .Go();
}