#pragma once
#include "common/eigen_sophus.h"
#include "common/imu.h"
#include "common/odom.h"

#include <deque>

namespace lh {
class StaticIMUInit {
   public:
    struct Options {
        Options() {}
        double init_time_secodns_ = 10.0;  // 禁止初始化时间
        int init_imu_queue_max_size_ = 2000;
        int static_odom_pulse_ = 5;  // 禁止时odom噪声
        double max_static_gyro_var = 0.5;
        double max_static_acce_var = 0.05;
        double gravity_norm_ = 9.81;
        bool use_speed_for_static_checking_ = true;  // 是否使用odom来判断车辆静止（部分数据集没有odom选项）
    };

    StaticIMUInit(Options options = Options()) : options_(options) {}

    bool AddIMU(const IMU& imu);
    bool AddOdom(const Odom& odom);

    /// 判定初始化是否成功
    bool InitSuccess() const { return init_success_; }

    /// 获取各Cov, bias, gravity
    Vec3d GetCovGyro() const { return cov_gyro_; }
    Vec3d GetCovAcce() const { return cov_acce_; }
    Vec3d GetInitBg() const { return init_bg_; }
    Vec3d GetInitBa() const { return init_ba_; }
    Vec3d GetGravity() const { return gravity_; }

   private:
    /// 尝试对系统初始化
    bool TryInit();

   private:
    Options options_;
    // bg,ba, ng, na
    Vec3d cov_gyro_ = Vec3d::Zero();
    Vec3d cov_acce_ = Vec3d::Zero();
    Vec3d init_bg_ = Vec3d::Zero();
    Vec3d init_ba_ = Vec3d::Zero();
    Vec3d gravity_ = Vec3d::Zero();
    // 存放imu数据的队列
    std::deque<IMU> init_imu_deque_;
    // 时间
    double current_time_ = 0.0;
    double init_start_time = 0.0;
    bool is_static = false;
    bool init_success_ = false;
};
}  // namespace lh