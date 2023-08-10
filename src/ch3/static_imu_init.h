/**
 * imu静止初始化
 */
#ifndef SLAM_IN_AUTO_DRIVING_STATIC_IMU_INIT_H
#define SLAM_IN_AUTO_DRIVING_STATIC_IMU_INIT_H

#include "common/eigen_sophus.h"
#include "common/imu.h"
#include "common/odom.h"

#include <deque>

namespace lh {
/**
 * IMU水平静止状态下初始化器
 * 使用方法：调用AddIMU, AddOdom添加数据，使用InitSuccess获取初始化是否成功
 * 成功后，使用各Get函数获取内部参数
 *
 * 初始化器在每次调用AddIMU时尝试对系统进行初始化。在有odom的场合，初始化要求odom轮速读数接近零；没有时，假设车辆初期静止。
 * 初始化器收集一段时间内的IMU读数，按照书本3.5.4节估计初始零偏和噪声参数，提供给ESKF或者其他滤波器
 */

class StaticImuInit {
   public:
    struct Options {
        /* data */
        Options() = default;
        double init_time_seconds_ = 10.0;            // 静止的时间
        int init_imu_queue_max_size_ = 2000;         // 初始化时imu队列的最大长度
        double static_odom_pulse_ = 5;               // 静止时轮速记输出噪声
        double max_static_gyro_var = 0.5;            // 静态下陀螺仪测量方差
        double max_static_acce_var = 0.05;           // 静态下加速度计测量方差
        bool use_speed_for_static_checking_ = true;  // 使用odom数据来判断是否静止
    };

    StaticImuInit(Options options = Options()) : options_(options) {}

    // 添加imu数据
    bool AddIMU(const IMU& imu);
    // 添加odom数据
    bool AddOdom(const Odom& odom);

    // 判断初始化是否成功
    bool InitSuccess() const { return init_success_; }
    // 获取各噪声
    Vec3d GetGovGyro() const { return cov_gyro_; }
    Vec3d GetGovAcce() const { return cov_acce_; }
    Vec3d GetInitBg() const { return init_bg_; }
    Vec3d GetInitBa() const { return init_ba_; }
    Vec3d GetGravity() const { return gravity_; }

   private:
    bool TryInit();
    Options options_;
    bool init_success_ = false;
    Vec3d cov_gyro_ = Vec3d::Zero();  // 初始时陀螺仪测量噪声
    Vec3d cov_acce_ = Vec3d::Zero();  // 初始时加速度测量噪声
    Vec3d init_bg_ = Vec3d::Zero();   // 陀螺仪零偏
    Vec3d init_ba_ = Vec3d::Zero();   // 加速计零偏
    Vec3d gravity_ = Vec3d::Zero();   // 重力
    bool is_static_ = false;
    std::deque<IMU> init_imu_deque_;  // 初始化用的imu数据
    double current_time_ = 0.0;
    double init_start_time = 0.0;  // 静止的初始时间
};

}  // namespace lh
#endif