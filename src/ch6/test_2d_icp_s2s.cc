#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/highgui.hpp>
#include "common/io_utils.h"
#include "icp_2d.h"
#include "lidar_2d_utils.h"

DEFINE_string(bag_path, "/sad/rosbags/2dmapping/floor1.bag", "数据包路径");
DEFINE_string(method, "point2point", "2d icp方法：point2point/point2plane");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;

    google::ParseCommandLineFlags(&argc, &argv, true);

    lh::RosbagIO rosbag_io(fLS::FLAGS_bag_path);
    Scan2d::Ptr last_scan = nullptr;
    Scan2d::Ptr current_scan = nullptr;

    rosbag_io
        .AddScan2DHandler("/pavo_scan_bottom",
                          [&](Scan2d::Ptr scan) -> bool {
                              current_scan = scan;
                              // 第二帧开始
                              if (last_scan == nullptr) {
                                  last_scan = current_scan;
                                  return true;
                              }

                              lh::Icp2d icp;
                              icp.SetTarget(last_scan);
                              icp.setSource(current_scan);

                              SE2 pose;
                              if (fLS::FLAGS_method == "point2point") {
                                  icp.AlignGaussNewton(pose);
                              } else if (fLS::FLAGS_method == "point2plane") {
                                  icp.AlignGaussNewtonPoint2Plane(pose);
                              }
                              cv::Mat image;
                              lh::Visualize2DScan(last_scan, SE2(), image, Vec3b(255, 0, 0));
                              lh::Visualize2DScan(current_scan, pose, image, Vec3b(0, 0, 255));
                              cv::imshow("scan", image);
                              cv::waitKey(20);

                              last_scan = current_scan;
                              return true;
                          })
        .Go();

    return 0;
}
