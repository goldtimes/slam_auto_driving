#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/highgui.hpp>

#include "ch6/lidar_2d_utils.h"
#include "ch6/likelihood_field.h"
#include "common/io_utils.h"

DEFINE_string(bag_path, "/sad/rosbags/2dmapping/floor1.bag", "数据包路径");
DEFINE_string(method, "gauss-newton", "gauss-newton/g2o");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;

    google::ParseCommandLineFlags(&argc, &argv, true);

    lh::RosbagIO rosbag_io(fLS::FLAGS_bag_path);

    Scan2d::Ptr current_scan = nullptr;
    Scan2d::Ptr last_scan = nullptr;

    rosbag_io
        .AddScan2DHandler("/pavo_scan_bottom",
                          [&](Scan2d::Ptr scan) -> bool {
                              lh::LikelihoodField lf;
                              current_scan = scan;
                              SE2 pose;
                              if (last_scan == nullptr) {
                                  last_scan = current_scan;
                                  return true;
                              }

                              lf.SetTargetScan(last_scan);
                              lf.SetSourceScan(current_scan);

                              if (FLAGS_method == "gauss-newton") {
                                  lf.AlignGaussNewton(pose);
                              } else if (FLAGS_method == "g2o") {
                                  lf.AlignG2O(pose);
                              }

                              LOG(INFO) << "aligned pose: " << pose.translation().transpose() << ", " << pose.so2().log();
                              cv::Mat image;
                              lh::Visualize2DScan(last_scan, SE2(), image, Vec3b(255, 0, 0));
                              lh::Visualize2DScan(current_scan, pose, image, Vec3b(0, 0, 255));

                              cv::imshow("scan", image);
                              // 画出target 和它的场函数
                              cv::Mat field_image = lf.GetFieldImage();
                              lh::Visualize2DScan(last_scan, SE2(), field_image, Vec3b(255, 0, 0), 1000, 20.0);
                              cv::imshow("field", field_image);
                              cv::waitKey(10);

                              last_scan = current_scan;
                              return true;
                          })
        .Go();
}