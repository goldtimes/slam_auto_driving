//
// Created by xiang on 22-11-10.
//

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "ch8/lio-iekf/lio_iekf.h"
#include "common/io_utils.h"
#include "common/sys_utils.h"
#include "common/timer/timer.h"

DEFINE_string(bag_path, "/media/kilox/PS2000/sad/nclt/20120511.bag", "path to rosbag");
DEFINE_string(dataset_type, "NCLT", "NCLT/ULHK/UTBM/AVIA");                                         // 数据集类型
DEFINE_string(config, "/home/slam_auto_driving/config/velodyne_nclt.yaml", "path of config yaml");  // 配置文件类型
DEFINE_bool(display_map, true, "display map?");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    lh::RosbagIO rosbag_io(fLS::FLAGS_bag_path, lh::Str2DatasetType(FLAGS_dataset_type));

    lh::LioIEKF lio;
    lio.Init(FLAGS_config);

    rosbag_io
        .AddAutoPointCloudHandler([&](sensor_msgs::PointCloud2::Ptr cloud) -> bool {
            lh::Timer::Evaluate([&]() { lio.PCLCallBack(cloud); }, "IEKF lio");
            return true;
        })
        .AddLivoxHandle([&](const livox_ros_driver::CustomMsg::ConstPtr& msg) -> bool {
            lh::Timer::Evaluate([&]() { lio.LivoxPCLCallBack(msg); }, "IEKF lio");
            return true;
        })
        .AddImuHandler([&](IMUPtr imu) {
            lio.IMUCallBack(imu);
            return true;
        })
        .Go();

    lio.Finish();
    lh::Timer::PrintAll();
    LOG(INFO) << "done.";

    return 0;
}