#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/highgui.hpp>

#include "ch6/lidar_2d_utils.h"
#include "ch6/occupancy_map.h"
#include "common/io_utils.h"
#include "common/sys_utils.h"

DEFINE_string(bag_path, "/sad/rosbags/2dmapping/floor1.bag", "数据包路径");
DEFINE_string(method, "bresenham", "填充算法:model/bresenham");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    lh::RosbagIO rosbag_io(fLS::FLAGS_bag_path);
    rosbag_io
        .AddScan2DHandler(
            "/pavo_scan_bottom",
            [&](Scan2d::Ptr scan) -> bool {
                lh::OccupancyMap oc_map;
                if (FLAGS_method == "model") {
                    evaluate_and_call([&]() { oc_map.AddLidarFrame(std::make_shared<lh::Frame>(scan), lh::OccupancyMap::GridMethod::MODEL_POINTS); },
                                      "Occupancy with model points");
                } else {
                    evaluate_and_call([&]() { oc_map.AddLidarFrame(std::make_shared<lh::Frame>(scan), lh::OccupancyMap::GridMethod::BRESENHAM); },
                                      "Occupancy with bresenham");
                }
                cv::imshow("occupancy map", oc_map.GetOccupancyGridBlackWhite());
                cv::waitKey(10);
                return true;
            })
        .Go();
    return 0;
}