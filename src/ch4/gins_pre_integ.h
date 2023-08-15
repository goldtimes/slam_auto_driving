#ifndef MAPPING_DR_PRE_INTEG_H
#define MAPPING_DR_PRE_INTEG_H

#include <deque>
#include <fstream>
#include <memory>

#include "common/eigen_sophus.h"
#include "common/gnss.h"
#include "common/imu.h"
#include "common/math_utils.h"
#include "common/odom.h"
#include "imu_preintegration.h"

namespace lh {
/**
 * 使用预积分优化的gins
 *
 * 使用两帧模型，不会去做全量的优化，两帧模型和eskf更像
 * imu到达时，累积在预积分器中
 * rtk数据到达时，触发一次优化，并边缘化，将边缘化结果写入下一帧先验中
 * 里程计作为速度观测量
 */
class GinsPreInteg {
   public:
    struct Options {
        Options() {}

        Vec3d gravity_ = Vec3d(0, 0, -9.8);
        // 预积分相关噪声
        IMUPreintegration::Options preinteg_options_;
        // 噪声
        double bias_gyro_var_ = 1e-6;
        double bias_acce_var_ = 1e-4;           // 零偏标准差
        Mat3d bg_rw_info_ = Mat3d::Identity();  // 陀螺仪随机游走信息阵
        Mat3d ba_rw_info_ = Mat3d::Identity();  // 加速度计随机游走信息阵
        // gnss噪声
        double gnss_pose_noise_ = 0.1;  // GNSS位置方差
        double gnss_height_noise_ = 0.1;
        double gnss_ang_noise_ = 1.0 * math::kDEG2RAD;
        Mat6d gnss_info_ = Mat6d::Identity();
        // 轮速计
        double odom_var_ = 0.05;
        Mat3d odom_info_ = Mat3d::Identity();
        double odom_span_ = 0.1;        //测量间隔
        double wheel_radius_ = 0.155;   // 轮子半径
        double circle_pulse_ = 1024.0;  // 编码器每圈脉冲数

        bool verbose_ = true;  // 是否输出调试信息
    };

    GinsPreInteg(Options options = Options()) : options_(options) { SetOptions(options_); }

    void SetOptions(Options options);

    void AddImu(const IMU& imu);
    void AddOdom(const Odom& odom);
    void AddGnss(const GNSS& gnss);

    NavStated GetState() const;

   private:
    Options options_;
    double current_time_ = 0.0;
    std::shared_ptr<IMUPreintegration> pre_integ_ = nullptr;
    std::shared_ptr<NavStated> last_frame_ = nullptr;
    std::shared_ptr<NavStated> this_frame_ = nullptr;
    Mat15d prior_info_ = Mat15d::Identity() * 1e2;  // 当前时刻先验

    // 两帧GNSS观测
    GNSS last_gnss_;
    GNSS this_gnss_;

    IMU last_imu_;
    Odom last_odom_;

    // 标志位
    bool first_gnss_received_ = false;
    bool first_imu_received_ = false;
    bool last_odom_set_ = false;

   private:
    void Optimize();
};

}  // namespace lh
#endif