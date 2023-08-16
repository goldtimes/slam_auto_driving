#include <gflags/gflags.h>
#include <glog/logging.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

using PointType = pcl::PointXYZI;
using PointCloudType = pcl::PointCloud<PointType>;

DEFINE_string(pcd_path, "/home/slam_auto_driving/data/ch5/map_example.pcd", "点云文件路径");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    if (FLAGS_pcd_path.empty()) {
        LOG(ERROR) << "pcd path is empty";
        return -1;
    }

    PointCloudType::Ptr cloud_ptr(new PointCloudType);
    pcl::io::loadPCDFile(FLAGS_pcd_path, *cloud_ptr);

    if (cloud_ptr->empty()) {
        LOG(ERROR) << "cannot load cloud file";
        return -1;
    }
    LOG(INFO) << "cloud points: " << cloud_ptr->size();

    // visualize
    pcl::visualization::PCLVisualizer viewer("cloud_viewer");
    pcl::visualization::PointCloudColorHandlerGenericField<PointType> handle(cloud_ptr, "z");  // 使用高度来着色
    viewer.addPointCloud<PointType>(cloud_ptr, handle);
    viewer.spin();

    return 0;
}
