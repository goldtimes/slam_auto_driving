/**
 * eskf
 */

#ifndef SLAM_IN_AUTO_DRIVING_ESKF_HPP
#define SLAM_IN_AUTO_DRIVING_ESKF_HPP

#include "common/eigen_sophus.h"
#include "common/gnss.h"
#include "common/imu.h"
#include "common/math_utils.h"
#include "common/nav_state.h"
#include "common/odom.h"

#include <glog/logging.h>
#include <iomanip>

namespace lh {
/**
 * 误差卡尔曼滤波器
 * imu 融合gnss观测的eskf
 */
template <typename S = double>
class ESKF {
   public:
    using SO3 = Sophus::SO3<S>;                     // 旋转变量类型
    using Vec3T = Eigen::Matrix<S, 3, 1>;           // 向量
    using Vec18T = Eigen::Matrix<S, 18, 1>;         // 18维向量
    using Mat3T = Eigen::Matrix<S, 3, 3>;           // 3x3矩阵
    using MotionNoiseT = Eigen::Matrix<S, 18, 18>;  // 运动噪声
    using OdomNoiseT = Eigen::Matrix<S, 3, 3>;      // 里程计噪声
    using GnssNoiseT = Eigen::Matrix<S, 6, 6>;      // Gnss噪声
    using Mat18T = Eigen::Matrix<S, 18, 18>;        // 18维方差类型
    using NavStateT = NavState<S>;                  // 名义状态变量

    struct Options {
        Options() = default;
        double imu_dt_ = 0.01;
        // imu 噪声
        double gyro_var_ = 1e-5;       // 陀螺仪测量标准差
        double acce_var_ = 1e-2;       // 加速度测量标准差
        double bias_gyro_var_ = 1e-6;  // 陀螺仪零偏标准差
        double bias_acce_var_ = 1e-4;  // 加速度零偏标准差

        // odom参数
        double odom_var_ = 0.5;
        double odom_span_ = 0.1;        // dt
        double wheel_radius_ = 0.155;   // 轮子半径
        double circle_pulse_ = 1024.0;  //编码器每圈脉冲数

        // rtk观测
        double gnss_pos_noise_ = 0.1;                   // gnss位置噪声
        double gnss_height_noise_ = 0.1;                // 高度噪声
        double gnss_ang_noise_ = 1.0 * math::kDEG2RAD;  // gnss旋转噪声

        // 其他
        bool update_bias_gyro_ = true;
        bool update_bias_acce_ = true;
    };

   public:
    /**
     * 初始零偏
     */
    ESKF(Options option = Options()) : options_(option) { BuildNoise(option); }
    /**
     * 设置初始条件
     * @param options 噪声配置
     * @param init_ba 初始零偏 加速度
     * @param init_bg 初始零偏 陀螺仪
     */
    void SetInitialConditions(Options options, const Vec3T& init_bg, const Vec3T& init_ba,
                              const Vec3T gravity = Vec3T{0, 0, -9.8}) {
        BuildNosie(options);
        options_ = options;
        bg_ = bg;
        ba_ = ba;
        cov_ = Mat18T::Identiyu() * 1e-4;
    }
    // imu 递推
    bool Predict(const IMU& imu);
    // 轮速计观测
    bool ObserveWheelSpeed(const Odom& odom);
    // gps观测
    bool ObserveGps(const GNSS& gnss);
    // 使用SE3进行观测
    bool ObserveSE3(const SE3& pose, double trans_noise = 0.1, double ang_noise = 1.0 * math::kDEG2RAD);
    // 获取全量状态
    NavStateT GetNomialState() const { return NavStateT(current_time_, R_, p_, v_, bg_, ba_); }
    // 获取SE3 状态
    SE3 GetNomialSE3() const { return SE3(R_, p_); }
    // 设置状态
    void SetX(const NavStated& x, const Vec3d& grav) {
        current_time_ = x.timestamp_;
        R_ = x.R_;
        p_ = x.p_;
        v_ = x.v_;
        bg_ = x.bg_;
        ba_ = x.ba_;
        g_ = grav;
    }
    // 设置协方差
    void SetCov(const Mat18T& cov) { cov_ = cov; }
    // 获取重力
    Vec3d GetGravity() const { return g_; }

   private:
    void BuildNoise(const Options& options) {
        double ev = options.acce_var_;
        double et = options_.gyro_var_;
        double eg = options.bias_acce_var_;
        double ea = options.bias_acce_var_;

        double ev2 = ev;
        double et2 = et;
        double eg2 = eg;
        double ea2 = ea;
        // 设置过程噪声
        Q_.diagonal() << 0, 0, 0, ev2, ev2, ev2, et2, et2, et2, eg2, eg2, eg2, ea2, ea2, ea2, 0, 0, 0;
        // 设置里程计噪声
        double o2 = options_.odom_var_ * options_.odom_var_;
        odom_noise_.diagonal() << o2, o2, o2;
        // 设置gnss噪声
        double gp2 = options_.gnss_pos_noise_ * options_.gnss_pos_noise_;
        double gh2 = options_.gnss_height_noise_ * options_.gnss_height_noise_;
        double ga2 = options_.gnss_ang_noise_ * options_.gnss_ang_noise_;
        gnss_noise_.diagonal() << gp2, gp2, gh2, ga2, ga2, ga2;
    }

    // 更新名义状态变量和重置error state
    void UpdateAndReset() {
        p_ += dx_.template block<3, 1>(0, 0);
        v_ += dx_.template block<3, 1>(3, 0);
        R_ = R_ * SO3::exp(dx_.template block<3, 1>(6, 0));
        if (options_.update_bias_gyro_) {
            bg_ += dx_.template block<3, 1>(12, 0);
        }
        if (options_.update_bias_acce_) {
            ba_ += dx_.template block<3, 1>(12, 0);
        }
        g_ += dx_.template block<3, 1>(15, 0);

        ProjectCov();
        dx_.setZero();
    }

    // 对P协方差投影 公式3.63
    void ProjectCov() {
        Mat18T J = Mat18T::Identity();
        J.template block<3, 3>(6, 6) = Mat3T::Identity() - 0.5 * SO3::hat(dx_.template block<3, 1>(6, 0));
        cov_ = J * cov_ * J.transpose();
    }

   private:
    double current_time_ = 0.0;
    // 名义状态
    Vec3T p_ = Vec3T::Zero();
    Vec3T v_ = Vec3T::Zero();
    SO3 R_;
    Vec3T bg_ = Vec3T::Zero();
    Vec3T ba_ = Vec3T::Zero();
    Vec3T g_{0, 0, -9.8};

    // 误差状态
    Vec18T dx_ = Vec18T::Zero();

    // 协方差矩阵
    Mat18T cov_ = Mat18T::Identity();

    // 噪声矩阵
    MotionNoiseT Q_ = MotionNoiseT::Zero();
    OdomNoiseT odom_noise_ = OdomNoiseT::Zero();
    GnssNoiseT gnss_noise_ = GnssNoiseT::Zero();
    // 第一帧的gnss数据
    bool first_gnss_ t = true;
    Options options_;
};

using ESKFD = ESKF<double>;
using ESKFF = ESKF<float>;

// 预测
template <typename S>
bool ESKF<S>::Predict(const IMU& imu) {
    assert(imu.timestamp_ >= current_time_);
    double dt = imu.timestamp_ - current_time_;
    if (dt > (5 * options_.imu_dt_) || dt < 0) {
        LOG(INFO) << "skip this imu because dt_ = " << dt;
        current_time_ = imu.timestamp_;
        return false;
    }
    // 名义状态递推
    Vec3T new_p = p_ + v_ * dt + 0.5 * (R_ * (imu.acce_ - ba_)) * dt * dt + 0.5 * g_ * dt * dt;
    Vec3T new_v = v_ + R_ * (imu.acce_ - ba_) * dt + g_ * dt;
    SO3 new_R = R_ * SO3::exp((imu.gyro_ - bg_) * dt);

    R_ = new_R;
    v_ = new_v;
    p_ = new_p;

    // 这里没有更新ba,bg,g
    // error state 递推
    // 计算运动过程雅可比矩阵 3.47
    Mat18T F = Mat18T::Identity();
    F.template block<3, 3>(0, 3) = Mat3T::Identity() * dt;                         // p对v
    F.template block<3, 3>(3, 6) = -R_.matrix() * SO3::hat(imu.acce_ - ba_) * dt;  // v对theta
    F.template block<3, 3>(3, 12) = -R_.matrix() * dt;                             // v对ba
    F.template block<3, 3>(3, 15) = Mat3T::Identity() * dt;                        // v对g
    F.template block<3, 3>(6, 6) = SO3::exp(-(imu.gyro_ - bg_) * dt).matrix();     // theta 对 theta
    F.template block<3, 3>(6, 9) = -Mat3T::Identity() * dt;                        // theta 对 g

    dx_ = F * dx_;                                // 3.48a
    cov_ = F * cov_.eval() * F.transpose() + Q_;  // 3.48b
    current_time_ = imu.timestamp_;
    return true;
}

// 轮速计观测
template <typename S>
bool ESKF<S>::ObserveWheelSpeed(const Odom& odom) {
    assert(odom.timestamp_ >= current_time_);

    // odom修正雅可比矩阵
    Eigen::Matrix<S, 3, 18> H = Eigen::Matrix<S, 3, 18>::Zero();
    // 为什么设置为单位矩阵
    H.template block<3, 3>(0, 3) = Mat3T::Idetity();

    // 卡尔曼增益
    Eigen::Matrix<S, 18, 3> K = cov_ * H.transpose() * (H * cov_ * H.transpose() + odom_noise_).inverse();

    // velocity observe
    double velo_l = options_.wheel_radius_ * odom.left_pulse_ / options_.circle_pulse_ * 2 * M_PI / options_.odom_span_;
    double velo_r =
        options_.wheel_radius_ * odom.right_pulse_ / options_.circle_pulse_ * 2 * M_PI / options_.odom_span_;
    double average_vel = 0.5 * (velo_l + velo_r);

    Vec3T vel_odom(average_vel, 0.0, 0.0);
    Vec3T vel_world = R_ * vel_odom;

    dx_ = K * (vel_world - v_);
    cov_ = (Mat18T::Identity() - K * H) * cov_;

    UpdateAndReset();
    return true;
}

// gnss观测
template <typename S>
bool ESKF<S>::ObserveGps(const GNSS& gnss) {
    // gnss 观测
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

// 位姿观测
template <typename S>
bool ESKF<S>::ObserveSE3(const SE3& pose, double trans_noise, double ang_noise) {
    Eigen::Matrix<S, 6, 18> H = Eigen::Matrix<S, 6, 18>::Zero();
    H.template block<3, 3>(0, 0) = Mat3T::Identity();  // p部分
    H.template block<3, 3>(3, 6) = Mat3T::Identity();  // R 部分 3.66

    // 卡尔曼增益
    Vec6d noise_vec;
    noise_vec << trans_noise, trans_noise, trans_noise, ang_noise, ang_noise, ang_noise;

    Mat6d V = noise_vec.asDiagonal();
    Eigen::Matrix<S, 18, 6> K = cov_ * H.transpose() * (H * cov_ * H.transpose() + V).inverse();

    // 更新x状态和cov
    Vec6d innov = Vec6d::Zero();
    innov.template head<3>() = (pose.translation() - p_);
    innov.template tail<3>() = (R_.inverse() * pose.so3()).log();  // 3.67 李群到李代数
    dx_ = K * innov;
    cov_ = (Mat18T::Identity() - K * H) * cov_;

    UpdateAndReset();
    return true;
}
}  // namespace lh

#endif