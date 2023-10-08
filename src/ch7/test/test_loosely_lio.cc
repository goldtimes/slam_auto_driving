//
// Created by xiang on 22-8-15.
//
#include <gflags/gflags.h>
#include <glog/logging.h>

#include "ch7/loosely_coupled_lio/loosely_lio.h"
#include "common/io_utils.h"
#include "common/sys_utils.h"
#include "common/timer/timer.h"

DEFINE_string(bag_path, "/media/kilox/PS2000/sad/ulhk/test3.bag", "path to rosbag");
DEFINE_string(dataset_type, "ULHK", "NCLT/ULHK/UTBM/AVIA");                                         // 数据集类型
DEFINE_string(config, "/home/slam_auto_driving/config/velodyne_ulhk.yaml", "path of config yaml");  // 配置文件类型
DEFINE_bool(display_map, true, "display map?");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    lh::RosbagIO rosbag_io(fLS::FLAGS_bag_path, lh::Str2DatasetType(FLAGS_dataset_type));

    lh::LooselyLIO::Options options;
    options.with_ui_ = FLAGS_display_map;
    lh::LooselyLIO lm(options);
    lm.Init(FLAGS_config);

    rosbag_io
        .AddAutoPointCloudHandler([&](sensor_msgs::PointCloud2::Ptr cloud) -> bool {
            lh::Timer::Evaluate([&]() { lm.PCLCallBack(cloud); }, "loosely lio");
            return true;
        })
        .AddLivoxHandle([&](const livox_ros_driver::CustomMsg::ConstPtr& msg) -> bool {
            lh::Timer::Evaluate([&]() { lm.LivoxPCLCallBack(msg); }, "loosely lio");
            return true;
        })
        .AddImuHandler([&](IMUPtr imu) {
            lm.IMUCallBack(imu);
            return true;
        })
        .Go();

    lm.Finish();
    lh::Timer::PrintAll();
    LOG(INFO) << "done.";

    return 0;
}