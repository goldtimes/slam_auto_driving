#pragma once
#include "common/eigen_sophus.h"
#include "common/imu.h"
#include "common/nav_state.h"
/**
 * imu 航迹推演
 */
namespace lh {
class ImuIntegration {
   public:
    ImuIntegration() = default;
    ImuIntegration(const Vec3d& ba, const Vec3d& bg) : ba_(ba), bg_(bg) {}
    ~ImuIntegration() = default;
    // imu 的频率为100hz
    void AddImu(const IMU& imu) {
        double dt = imu.timestamp_ - process_imu_time_;
        std::cout << "dt: " << dt << std::endl;
        if (dt > 0 & dt < 0.1) {
            R_ = R_ * Sophus::SO3d::exp((imu.gyro_ - bg_) * dt);
            p_ = p_ + v_ * dt + 0.5 * (R_ * (imu.acce_ - ba_)) * dt * dt + 0.5 * gravity_ * dt * dt;
            v_ = v_ + (R_ * (imu.acce_ - ba_)) * dt + gravity_ * dt;
        }
        // 更新时间
        process_imu_time_ = imu.timestamp_;
    }

    NavStated GetNavStated() { return NavStated(process_imu_time_, R_, p_, v_, bg_, ba_); }

    const SO3 GetR() const { return R_; }
    const Vec3d GetP() const { return p_; }
    const Vec3d GetV() const { return v_; }

   private:
    // 航迹推演的pvq状态
    SO3 R_;
    Vec3d p_;
    Vec3d v_;

    // 零偏状态,现在不是通过静止初始化,而是通过外部设定
    Vec3d bg_;
    Vec3d ba_;

    double process_imu_time_;

    const Vec3d gravity_ = Vec3d(0.0, 0.0, -9.8);
};
}  // namespace lh