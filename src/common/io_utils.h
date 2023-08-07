#ifndef SLAM_IN_AUTO_DRIVING_IO_UTILS_H
#define SLAM_IN_AUTO_DRIVING_IO_UTILS_H
#include <fstream>
#include <functional>
#include "imu.h"
#include "math_utils.h"

namespace lh {
// 读取数据文件
class TxtIO {
   public:
    TxtIO(const std::string& file_path) : fin(file_path) {}

    using IMUProcessFuncType = std::function<void(const IMU&)>;

    TxtIO& SetIMUProcessFunc(IMUProcessFuncType imu_proc) {
        imu_proc_ = std::move(imu_proc);
        return *this;
    }
    // 遍历文件内容
    void Go();

   private:
    std::ifstream fin;
    IMUProcessFuncType imu_proc_;
};
}  // namespace lh

#endif