#include "static_imu_init.h"
#include "common/math_utils.h"

#include <glog/logging.h>

namespace lh {
bool StaticImuInit::AddIMU(const IMU& imu) {
    if (init_success_) {
        return true;
    }
    if (options_.use_speed_for_static_checking_ && !is_static_) {
        LOG(WARNING) << "等待车辆静止";
        init_imu_deque_.clear();
        return false;
    }
    // 开始记录静止时间
    if (init_imu_deque_.empty()) {
        init_start_time = imu.timestamp_;
    }

    init_imu_deque_.push_back(imu);
    double init_time = imu.timestamp_ - init_start_time;
    if (init_time > options_.init_time_seconds_) {
        // 静止的时间>10s
        TryInit();
    }
    // 维护队列的长度
    while (init_imu_deque_.size() > options_.init_imu_queue_max_size_) {
        init_imu_deque_.pop_front();
    }

    current_time_ = imu.timestamp_;
    return false;
}

bool StaticImuInit::AddOdom(const Odom& odom) {
    if (init_success_) {
        return true;
    }

    if (odom.left_pulse_ < options_.static_odom_pulse_ && odom.right_pulse_ < options_.static_odom_pulse_) {
        is_static_ = true;
    } else {
        is_static_ = false;
    }
    current_time_ = odom.timestamp_;
    return true;
}

bool StaticImuInit::TryInit() {
    if (init_imu_deque_.size() < 10)
        return false;

    // 计算均值和方差 累加的统计方式
    Vec3d mean_gyro, mean_acce;
    math::ComputeMeanAndCovDiag(init_imu_deque_, mean_gyro, cov_gyro_, [](const IMU& imu) { return imu.gyro_; });
    math::ComputeMeanAndCovDiag(init_imu_deque_, mean_acce, cov_acce_, [this](const IMU& imu) { return imu.acce_; });

    LOG(INFO) << "mean acce: " << mean_acce.transpose();
    gravity_ = -mean_acce / mean_acce.norm() * options_.gravity_norm_;

    // 重新计算加速度计的协方差
    math::ComputeMeanAndCovDiag(init_imu_deque_, mean_acce, cov_acce_,
                                [this](const IMU& imu) { return imu.acce_ + gravity_; });

    // 检查IMU噪声
    if (cov_gyro_.norm() > options_.max_static_gyro_var) {
        LOG(ERROR) << "陀螺仪测量噪声太大" << cov_gyro_.norm() << " > " << options_.max_static_gyro_var;
        return false;
    }

    if (cov_acce_.norm() > options_.max_static_acce_var) {
        LOG(ERROR) << "加计测量噪声太大" << cov_acce_.norm() << " > " << options_.max_static_acce_var;
        return false;
    }

    // 估计测量噪声和零偏
    init_bg_ = mean_gyro;
    init_ba_ = mean_acce;

    LOG(INFO) << "IMU 初始化成功，初始化时间= " << current_time_ - init_start_time << ", bg = " << init_bg_.transpose()
              << ", ba = " << init_ba_.transpose() << ", gyro sq = " << cov_gyro_.transpose()
              << ", acce sq = " << cov_acce_.transpose() << ", grav = " << gravity_.transpose()
              << ", norm: " << gravity_.norm();
    LOG(INFO) << "mean gyro: " << mean_gyro.transpose() << " acce: " << mean_acce.transpose();
    init_success_ = true;
    return true;
}
}  // namespace lh