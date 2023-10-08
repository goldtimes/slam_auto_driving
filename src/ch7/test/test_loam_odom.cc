//
// Created by xiang on 2022/7/18.
//

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "ch7/loam-like/loam_like_odom.h"
#include "common/io_utils.h"
#include "common/timer/timer.h"

DEFINE_string(bag_path, "/media/kilox/PS2000/sad/wxb/test1.bag", "path to wxb bag");
DEFINE_string(topic, "/velodyne_packets_1", "topic of lidar packets");
DEFINE_bool(display_map, true, "display map?");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    // 测试loam-like odometry的表现
    lh::LoamLikeOdom::Options options;
    options.display_realtime_cloud_ = FLAGS_display_map;
    lh::LoamLikeOdom lo(options);

    LOG(INFO) << "using topic: " << FLAGS_topic;
    lh::RosbagIO bag_io(fLS::FLAGS_bag_path);
    bag_io
        .AddVelodyneHandle(FLAGS_topic,
                           [&](lh::FullCloudPtr cloud) -> bool {
                               lh::Timer::Evaluate([&]() { lo.ProcessPointCloud(cloud); }, "Loam-like odom");
                               return true;
                           })
        .Go();  // 在Go函数中调用了lambda传入的函数

    lo.SaveMap("/home/slam_auto_driving/data/ch7/loam_map.pcd");

    lh::Timer::PrintAll();
    LOG(INFO) << "done.";

    return 0;
}
