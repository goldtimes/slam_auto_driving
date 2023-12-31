#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/highgui.hpp>

#include "ch6/lidar_2d_utils.h"
#include "ch6/mapping_2d.h"
#include "common/io_utils.h"

DEFINE_string(bag_path, "/sad/rosbags/2dmapping/floor1.bag", "数据包路径");
DEFINE_bool(with_loop_closing, true, "是否使用回环检测");

/// 测试2D lidar SLAM

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    lh::RosbagIO rosbag_io(fLS::FLAGS_bag_path);
    lh::Mapping2D mapping;

    std::system("rm -rf /home/slam_auto_driving/data/ch6/*");

    if (mapping.Init(FLAGS_with_loop_closing) == false) {
        return -1;
    }

    rosbag_io.AddScan2DHandler("/pavo_scan_bottom", [&](Scan2d::Ptr scan) { return mapping.ProcessScan(scan); }).Go();
    cv::imwrite("/home/slam_auto_driving/data/ch6/global_map.png", mapping.ShowGlobalMap(2000));
    return 0;
}