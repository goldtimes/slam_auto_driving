#include "io_utils.h"
#include <glog/logging.h>

namespace lh {
void TxtIO::Go() {
    if (!fin) {
        LOG(ERROR) << "未找到文件" << std::endl;
        return;
    }
    while (!fin.eof()) {
        std::string line;
        std::getline(fin, line);
        if (line.empty()) {
            continue;
        }
        if (line[0] == '#') {
            // #开头为注释
            continue;
        }
        // load data
        std::stringstream ss;
        ss << line;
        std::string data_type;
        ss >> data_type;
        if (data_type == "IMU" && imu_proc_ != nullptr) {
            double time, gx, gy, gz, ax, ay, az;
            ss >> time >> gx >> gy >> gz >> ax >> ay >> az;
            imu_proc_(IMU(time, Vec3d(gx, gy, gz), Vec3d(ax, ay, az)));
        } else if (data_type == "ODOM") {
            double time, wl, wr;
            ss >> time >> wl >> wr;
            // odom_proc_(Odom(time, wl, wr));
        } else if (data_type == "GNSS") {
            double time, wl, wr;
            ss >> time >> wl >> wr;
            // gnss_proc_(GNSS(time, wl, wr));
        }
    }
    LOG(INFO) << "done.";
}
}  // namespace lh