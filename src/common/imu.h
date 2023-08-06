#ifndef IMU_H
#define IMU_H

#include <memory>
#include "common/eigen_sophus.h"

namespace lh {
struct IMU {
    IMU() = default;
    IMU(double t, const Vec3d gyro, const Vec3d& acce) : gyro_(gyro), acce_(acce), timestamp_(t) {}

    double timestamp_ = 0.0;
    Vec3d gyro_ = Vec3d::Zero();
    Vec3d acce_ = Vec3d::Zero();
};
}  // namespace lh

using IMUPtr = std::shared_ptr<lh::IMU>;

#endif