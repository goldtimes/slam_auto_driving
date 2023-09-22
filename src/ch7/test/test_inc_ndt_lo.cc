//
// Created by xiang on 2022/7/18.
//

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "ch7/incremental_ndt_lo.h"
#include "common/io_utils.h"
#include "common/timer/timer.h"

DEFINE_string(bag_path, "./dataset/sad/ulhk/test2.bag", "path to rosbag");
DEFINE_string(dataset_type, "ULHK", "NCLT/ULHK/KITTI/WXB3D");  // 数据集类型
DEFINE_bool(use_ndt_nearby_6, false, "use ndt nearby 6?");
DEFINE_bool(display_map, true, "display map?");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    lh::RosbagIO rosbag_io(fLS::FLAGS_bag_path, lh::Str2DatasetType(FLAGS_dataset_type));

    lh::IncrementalNDTLO::Options options;
    options.ndt3d_options_.nearby_type_ = FLAGS_use_ndt_nearby_6 ? lh::IncNdt3d::NearbyType::NEARBY6 : lh::IncNdt3d::NearbyType::CENTER;
    options.display_realtime_cloud_ = FLAGS_display_map;
    lh::IncrementalNDTLO ndt_lo(options);

    rosbag_io
        .AddAutoPointCloudHandler([&ndt_lo](sensor_msgs::PointCloud2::Ptr msg) -> bool {
            lh::Timer::Evaluate(
                [&]() {
                    SE3 pose;
                    ndt_lo.AddCloud(lh::VoxelCloud(lh::PointCloud2ToCloudPtr(msg)), pose);
                },
                "NDT registration");
            return true;
        })
        .Go();

    if (FLAGS_display_map) {
        // 把地图存下来
        ndt_lo.SaveMap("/home/slam_auto_driving/data/ch7/map.pcd");
    }

    lh::Timer::PrintAll();
    LOG(INFO) << "done.";

    return 0;
}