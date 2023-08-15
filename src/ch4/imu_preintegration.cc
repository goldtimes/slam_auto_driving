#include "imu_preintegration.h"
#include <glog/logging.h>

namespace lh {
IMUPreintegration::IMUPreintegration(Options options) {
    bg_ = options.init_bg_;
    ba_ = options.init_ba_;
    const float ng2 = options.noise_gyro_ * options.noise_gyro_;
    const float na2 = options.noise_acce_ * options.noise_acce_;

    noise_gyro_acce_.diagonal() << ng2, ng2, ng2, na2, na2, na2;
}

void IMUPreintegration::Integrate(const IMU& imu, double dt) {
    // 去掉零偏的测量
    Vec3d gyro = imu.gyro_ - bg_;
    Vec3d acc = imu.acce_ - ba_;

    // 更新dv, dp, 先不更新dR
    dp_ = dp_ + dv_ * dt + 0.5f * dR_.matrix() * acc * dt * dt;  // 4.13
    dv_ = dv_ + dR_ * acc * dt;                                  // 4.16

    // 运动方程雅可比矩阵系数A，B阵4.29
    Eigen::Matrix<double, 9, 9> A;
    A.setIdentity();
    Eigen::Matrix<double, 9, 6> B;
    B.setZero();
    // acc^
    Mat3d acc_hat = SO3::hat(acc);
    double dt2 = dt * dt;

    A.block<3, 3>(3, 0) = -dR_.matrix() * dt * acc_hat;
    A.block<3, 3>(6, 0) = -0.5 * dR_.matrix() * dt2 * acc_hat;
    A.block<3, 3>(6, 3) = dt * Mat3d::Identity();

    B.block<3, 3>(3, 3) = dR_.matrix() * dt;
    B.block<3, 3>(6, 3) = 0.5 * dR_.matrix() * dt2;

    // 更新雅可比矩阵 4.39
    dp_dba_ = dp_dba_ + dv_dba_ * dt - 0.5f * dR_.matrix() * dt2;
    dp_dbg_ = dp_dbg_ + dv_dbg_ * dt - 0.5f * dR_.matrix() * acc_hat * dR_dbg_ * dt2;

    dv_dba_ = dv_dba_ - dR_.matrix() * dt;
    dv_dbg_ = dv_dbg_ - dR_.matrix() * dt * acc_hat * dR_dbg_;

    // 旋转部分
    Vec3d omega = gyro * dt;  // 主动量
    Mat3d rightJ = SO3::jr(omega);
    SO3 deltaR = SO3::exp(omega);
    dR_ = dR_ * deltaR;

    A.block<3, 3>(0, 0) = deltaR.matrix().transpose();
    B.block<3, 3>(0, 0) = rightJ * dt;

    // 更新噪声项
    cov_ = A * cov_ * A.transpose() + B * noise_gyro_acce_ * B.transpose();

    //更新dR_dbg
    dR_dbg_ = deltaR.matrix().transpose() * dR_dbg_ - rightJ * dt;  // 4.39
    dt_ += dt;
}

SO3 IMUPreintegration::GetDeltaRoation(const Vec3d& bg) { return dR_ * SO3::exp(dR_dbg_ * (bg * bg_)); }

Vec3d IMUPreintegration::GetDeltaVelocity(const Vec3d& bg, const Vec3d& ba) {
    return dv_ + dv_dbg_ * (bg - bg_) + dv_dba_ * (ba - ba_);
}

Vec3d IMUPreintegration::GetDeltaPosition(const Vec3d& bg, const Vec3d& ba) {
    return dp_ + dp_dbg_ * (bg - bg_) + dp_dba_ * (ba - ba_);
}

NavStated IMUPreintegration::Predict(const lh::NavStated& start, const Vec3d& grav) const {
    SO3 Rj = start.R_ * dR_;
    Vec3d vj = start.R_ * dv_ + start.v_ + grav * dt_;
    Vec3d pj = start.R_ * dp_ + start.v_ * dt_ + 0.5f * grav * dt_ * dt_;

    auto state = NavStated(start.timestamp_ + dt_, Rj, pj, vj);
    state.bg_ = bg_;
    state.ba_ = ba_;
    return state;
}
}  // namespace lh