#include "static_imu_init.hh"
#include "common/math_utils.h"

#include <glog/logging.h>

namespace lh {
bool StaticIMUInit::AddOdom(const Odom& odom) {
    // 初始化成功了
    if (init_success_) {
        return false;
    }

    if (odom.left_pulse_ < options_.static_odom_pulse_ && odom.right_pulse_ < options_.static_odom_pulse_) {
        is_static = true;
    } else {
        is_static = false;
    }
}

bool StaticIMUInit::AddIMU(const IMU& imu) {
    if (init_success_) {
        return false;
    }
    // 等待静止
    if (options_.use_speed_for_static_checking_ && !is_static) {
        LOG(WARNING) << "等待静止";
        init_imu_deque_.clear();
        return false;
    }
    // 开始初始化
    if (init_imu_deque_.empty()) {
        init_start_time = imu.timestamp_;
    }
    init_imu_deque_.push_back(imu);
    double init_dt = imu.timestamp_ - init_start_time;
    // 收集imu数据初始化时间大于10秒
    if (init_dt > options_.init_time_secodns_) {
        TryInit();
    }

    while (init_imu_deque_.size() > options_.init_imu_queue_max_size_) {
        init_imu_deque_.pop_front();
    }
    current_time_ = imu.timestamp_;
    return false;
}

bool StaticIMUInit::TryInit() {
    if (init_imu_deque_.size() < 10) {
        return false;
    }

    // 计算均值和方差
    Vec3d mean_gyro, mean_acce;
    math::ComputeMeanAndCovDiag(init_imu_deque_, mean_gyro, cov_gyro_, [this](const IMU& imu) { return imu.gyro_; });
    math::ComputeMeanAndCovDiag(init_imu_deque_, mean_acce, cov_acce_, [this](const IMU& imu) { return imu.acce_; });

    // 以acce均值为方向，取9.8长度为重力
    LOG(INFO) << "mean acce: " << mean_acce.transpose();
    gravity_ = -mean_acce / mean_acce.norm() * options_.gravity_norm_;

    // 重新计算加计的协方差
    math::ComputeMeanAndCovDiag(init_imu_deque_, mean_acce, cov_acce_, [this](const IMU& imu) { return imu.acce_ + gravity_; });

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
}
}  // namespace lh