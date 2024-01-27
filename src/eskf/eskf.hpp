#pragma once

#include "common/eigen_sophus.h"
#include "common/gnss.h"
#include "common/imu.h"
#include "common/math_utils.h"
#include "common/nav_state.h"
#include "common/odom.h"

#include <glog/logging.h>
#include <iomanip>

namespace lh {
template <typename S = double>
class ESKF {
   public:
    using SO3 = Sophus::SO3<S>;
    using Vec3T = Eigen::Matrix<S, 3, 1>;
    using Vec18T = Eigen::Matrix<S, 18, 1>;
    using Mat3T = Eigen::Matrix<S, 3, 3>;
    using MotionNoiseT = Eigen::Matrix<S, 18, 18>;  // 运动噪声
    using OdomNoiseT = Eigen::Matrix<S, 3, 3>;      // 里程计噪声
    using GnssNoiseT = Eigen::Matrix<S, 6, 6>;      // GNSS噪声
    using Mat18T = Eigen::Matrix<S, 18, 18>;
    using NavStateT = NavState<S>;

    struct Options {
        Options() = default;
        // imu的频率和 bg,ba的设置
        double imu_dt_ = 0.01;
        double gyro_var_ = 1e-5;  // 陀螺仪标准差
        double acce_var_ = 1e-2;  // 加速度标准差
        double bias_gyro_var_ = 1e-6;
        double bias_acce_var_ = 1e-4;

        // odom 参数
        double odom_var_ = 0.5;
        double odom_dt_ = 0.1;
        double wheel_radius_ = 0.155;
        double circle_pulse_ = 1024.0;

        // rtk
        double gnss_pos_noise_ = 0.1;
        double gnss_height_noise_ = 0.1;
        double gnss_ang_noise_ = 1.0 * math::kDEG2RAD;

        bool update_bias_gyro_ = true;
        bool update_bias_acce_ = true;
    };

   public:
    ESKF(Options options = Options()) : options_(options) { BuildNoise(options); }

    void SetInitialConditions(Options& options, const Vec3T& init_bg, const Vec3T& init_ba, const Vec3T& gravity = Vec3T(0.0, 0.0, -9.8)) {
        BuildNoise(options);
        options_ = options;
        bg_ = init_bg;
        ba_ = init_ba;
        g_ = gravity;
        cov_ = Mat18T::Identity() * 1e-4;
    }
    // 预测
    bool Predict(const IMU& imu);
    // 轮速计更新
    bool ObserveWheelSpeed(const Odom& odom);
    // gps观测
    bool ObserveGps(const GNSS& gnss);

    bool ObserveSE3(const SE3& pose, double trans_noise = 0.1, double ang_noise = 1.0 * math::kDEG2RAD);

    // 获取全量状态
    NavStateT GetNominalState() const { return NavStateT(current_time_, R_, p_, v_, bg_, ba_); }

    SE3 GetNominalSE3() const { return SE3(R_, p_); }

    void SetState(const NavStateT& x, const Vec3d& grav) {
        current_time_ = x.timestamp_;
        p_ = x.p_;
        v_ = x.v_;
        R_ = x.R_;
        bg_ = x.bg_;
        ba_ = x.ba_;
        g_ = grav;
    }

    void SetCov(const Mat18T& cov) { cov_ = cov; }

    Vec3d GetGravity() const { return g_; }

   private:
    void BuildNoise(const Options& options) {
        double ev = options.acce_var_;
        double et = options.gyro_var_;
        double eg = options.bias_gyro_var_;
        double ea = options.bias_acce_var_;

        double ev2 = ev;  // * ev;
        double et2 = et;  // * et;
        double eg2 = eg;  // * eg;
        double ea2 = ea;  // * ea;
        // R, v, p bg, ba, g
        Q_.diagonal() << 0, 0, 0, ev2, ev2, ev2, et2, et2, et2, eg2, eg2, eg2, ea2, ea2, ea2, 0, 0, 0;

        // 设置里程计噪声
        double o2 = options_.odom_var_ * options_.odom_var_;
        odom_noise_.diagonal() << o2, o2, o2;

        // 设置GNSS状态
        double gp2 = options.gnss_pos_noise_ * options.gnss_pos_noise_;
        double gh2 = options.gnss_height_noise_ * options.gnss_height_noise_;
        double ga2 = options.gnss_ang_noise_ * options.gnss_ang_noise_;
        gnss_noise_.diagonal() << gp2, gp2, gh2, ga2, ga2, ga2;
    }

    void UpdateAndReset() {
        p_ += dx_.template block<3, 1>(0, 0);
        v_ += dx_.template block<3, 1>(3, 0);
        R_ = R_ * SO3::exp(dx_.template block<3, 1>(6, 0));

        if (options_.update_bias_gyro_) {
            bg_ += dx_.template block<3, 1>(9, 0);
        }

        if (options_.update_bias_acce_) {
            ba_ += dx_.template block<3, 1>(12, 0);
        }

        g_ += dx_.template block<3, 1>(15, 0);

        ProjectCov();
        dx_.setZero();
    }

    // 对P矩阵进行投影
    void ProjectCov() {
        Mat18T J = Mat18T::Identity();
        J.template block<3, 3>(6, 6) = Mat3T::Identity() - 0.5 * SO3::hat(dx_.template block<3, 1>(6, 0));
        cov_ = J * cov_ * J.transpose();
    }

   private:
    double current_time_ = 0.0;  // eskf 系统当前时间

    Options options_;
    // 名义状态量
    Vec3T p_ = Vec3T::Zero();
    Vec3T v_ = Vec3T::Zero();
    SO3 R_;
    Vec3T bg_ = Vec3T::Zero();
    Vec3T ba_ = Vec3T::Zero();
    Vec3T g_{0.0, 0.0, -9.8};
    // 误差状态量
    Vec18T dx_ = Vec18T::Zero();
    // 协方差
    Mat18T cov_ = Mat18T::Identity();
    // 运动方程噪声
    MotionNoiseT Q_ = MotionNoiseT::Zero();
    // 里程计对v的噪声观测
    OdomNoiseT odom_noise_ = OdomNoiseT::Zero();
    // gnss对 R,p的观测
    GnssNoiseT gnss_noise_ = GnssNoiseT::Zero();

    bool first_gnss_ = true;
};

using ESKFD = ESKF<double>;
using ESKFF = ESKF<float>;

template <typename S>
bool ESKF<S>::Predict(const IMU& imu) {
    assert(imu.timestamp_ > current_time_);

    double dt = imu.timestamp_ - current_time_;
    if (dt > (0.5 * options_.imu_dt_) || dt < 0) {
        // 第一帧的时间间隔不对
        LOG(INFO) << "skip this imu because dt_ = " << dt;
        current_time_ = imu.timestamp_;
        return false;
    }
    // imu 名义状态量的递推

    Vec3T new_p = p_ + v_ * dt + 0.5 * (R_ * (imu.acce_ - ba_)) * dt * dt + 0.5 * g_ * dt * dt;
    Vec3T new_v = v_ + R_ * (imu.acce_ - ba_) * dt + g_ * dt;
    SO3 new_R = R_ * SO3::exp((imu.gyro_ - bg_) * dt);
    R_ = new_R;
    p_ = new_p;
    v_ = new_p;

    // 误差状态的递推, 3.47公式
    Mat18T F = Mat18T::Identity();
    F.template block<3, 3>(0, 3) = Mat3T::Identity() * dt;                         // delta_p / delta_v
    F.template block<3, 3>(3, 6) = -R_.matrix() * SO3::hat(imu.acce_ - ba_) * dt;  // v对theta
    F.template block<3, 3>(3, 12) = -R_.matrix() * dt;                             // v 对 ba
    F.template block<3, 3>(3, 15) = Mat3T::Identity() * dt;                        // v 对 g
    F.template block<3, 3>(6, 6) = SO3::exp(-(imu.gyro_ - bg_) * dt).matrix();     // theta 对 theta
    F.template block<3, 3>(6, 9) = -Mat3T::Identity() * dt;                        // theta 对 bg

    cov_ = F * cov_.eval() * F.transpose() + Q_;
    current_time_ = imu.timestamp_;
    return true;
}

template <typename S>
bool ESKF<S>::ObserveWheelSpeed(const Odom& odom) {
    assert(odom.timestamp_ >= current_time_);

    // 速度的观测 所以观测矩阵3x18
    Eigen::Matrix<S, 3, 18> H = Eigen::Matrix<S, 3, 18>::Zero();
    // 这里jacobian矩阵是对名义状态量求导
    H.template block<3, 3>(0, 3) = Mat3T::Identity();
    // 卡尔曼增益
    Eigen::Matrix<S, 18, 3> K = cov_ * H.transpose() * (H * cov_ * H.transpose() + odom_noise_).inverse();

    // velocity obs 左右轮的速度计算轮子的速度
    double velo_l = options_.wheel_radius_ * odom.left_pulse_ / options_.circle_pulse_ * 2 * M_PI / options_.odom_dt_;
    double velo_r = options_.wheel_radius_ * odom.right_pulse_ / options_.circle_pulse_ * 2 * M_PI / options_.odom_dt_;
    double average_vel = 0.5 * (velo_l + velo_r);

    Vec3T vel_odom(average_vel, 0, 0);
    Vec3T vel_world = R_ * vel_odom;
    // delta_x = K * (z - h(x_pre));  z 观测值 - h(x_pre) x_pre就是之前imu递推的状态量,通过观测函数得到的值,这里的值就是v
    dx_ = K * (vel_world - v_);

    // 更新协方差
    cov_ = (Mat18T::Identity() - K * H) * cov_;

    // 更新状态量
    UpdateAndReset();
    return true;
}

template <typename S>
bool ESKF<S>::ObserveGps(const GNSS& gnss) {
    assert(gnss.unix_time_ >= current_time_);
    if (first_gnss_) {
        R_ = gnss.utm_pose_.so3();
        p_ = gnss.utm_pose_.translation();
        first_gnss_ = false;
        current_time_ = gnss.unix_time_;
        return true;
    }
    assert(gnss.heading_valid_);
    ObserveSE3(gnss.utm_pose_, options_.gnss_pos_noise_, options_.gnss_ang_noise_);
    current_time_ = gnss.unix_time_;
    return true;
}

template <typename S>
bool ESKF<S>::ObserveSE3(const SE3& pose, double trans_noise, double ang_noise) {
    // 这里jacobian矩阵是对误差状态量求导
    Eigen::Matrix<S, 6, 18> H = Eigen::Matrix<S, 6, 18>::Zero();
    H.template block<3, 3>(0, 0) = Mat3T::Identity();  // P部分
    H.template block<3, 3>(3, 6) = Mat3T::Identity();  // R部分（3.66)

    // 卡尔曼增益和更新过程
    Vec6d noise_vec;
    noise_vec << trans_noise, trans_noise, trans_noise, ang_noise, ang_noise, ang_noise;

    Mat6d V = noise_vec.asDiagonal();
    Eigen::Matrix<S, 18, 6> K = cov_ * H.transpose() * (H * cov_ * H.transpose() + V).inverse();

    // 更新x和cov
    Vec6d innov = Vec6d::Zero();
    innov.template head<3>() = (pose.translation() - p_);          // 平移部分
    innov.template tail<3>() = (R_.inverse() * pose.so3()).log();  // 旋转部分(3.67)

    dx_ = K * innov;
    cov_ = (Mat18T::Identity() - K * H) * cov_;

    UpdateAndReset();
    return true;
}
}  // namespace lh