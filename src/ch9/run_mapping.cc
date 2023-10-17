#include <gflags/gflags.h>
#include <glog/logging.h>

#include "frontend.h"
#include "loopclosure.h"
#include "optimization.h"

DEFINE_string(config_yaml, "/home/slam_auto_driving/config/mapping.yaml", "配置文件");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    LOG(INFO) << "testing frontend";
    lh::Frontend frontend(FLAGS_config_yaml);
    if (!frontend.Init()) {
        LOG(ERROR) << "failed to init frontend.";
        return -1;
    }

    frontend.Run();

    lh::Optimization opti(FLAGS_config_yaml);
    if (!opti.Init(1)) {
        LOG(ERROR) << "failed to init opti1.";
        return -1;
    }
    opti.Run();

    lh::LoopClosure lc(FLAGS_config_yaml);
    if (!lc.Init()) {
        LOG(ERROR) << "failed to init loop closure.";
        return -1;
    }
    lc.Run();

    lh::Optimization opti2(FLAGS_config_yaml);
    if (!opti2.Init(2)) {
        LOG(ERROR) << "failed to init opti2.";
        return -1;
    }
    opti2.Run();

    LOG(INFO) << "done.";
    return 0;
}