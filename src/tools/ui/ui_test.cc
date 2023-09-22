#include <gflags/gflags.h>
#include <glog/logging.h>
#include <pcl-1.10/pcl/io/pcd_io.h>

#include "tools/ui/pangolin_window.h"

DEFINE_string(source, "/sad/data/ch5/first.pcd", "第一个点云路径");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    lh::ui::PangolinWindow ui;
    ui.Init();

    lh::CloudPtr source(new lh::PointCloudType);
    pcl::io::loadPCDFile(fLS::FLAGS_source, *source);

    LOG(INFO) << "set state";
    ui.UpdateScan(source, SE3());

    LOG(INFO) << "waiting";
    sleep(60);
    ui.Quit();
    return 0;
}