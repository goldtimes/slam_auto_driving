#ifndef IMUTYPES_H
#define IMUTYPES_H

#include <mutex>
#include <opencv2/core/core.hpp>
#include <utility>
#include <vector>

#include "common/eigen_sophus.h"
#include "common/imu.h"
#include "common/nav_state.h"

namespace lh {
/**
 * IMU 预积分器
 *
 * 调用Integrate来插入新的IMU读数，然后通过Get函数得到预积分的值
 * 雅可比也可以通过本类获得，可用于构建g2o的边类
 */

class IMUPreintegration {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // 参数配置项
    struct Options {
        Options() {}
        Vec3d init_bg_ = Vec3d::Zero();  // 初始零偏
        Vec3d init_ba_ = Vec3d::Zero();  // 初始零偏
        double noise_gyro_ = 1e-2;       // 陀螺仪噪声
        double noise_acce_ = 1e-1;       // 加速度计噪声
    };

    IMUPreintegration(Options options = Options());
    /**
     * 插入新的imu数据
     */
    void Integrate(const IMU& imu, double dt);

    /**
     * 从某个起点开始预测积分之后的状态
     */
    NavStated Predict(const NavStated& start, const Vec3d& grav = Vec3d(0, 0, -9.81)) const;

    // 获取修正之后的状态
    SO3 GetDeltaRoation(const Vec3d& bg);
    Vec3d GetDeltaVelocity(const Vec3d& bg, const Vec3d& ba);
    Vec3d GetDeltaPosition(const Vec3d& bg, const Vec3d& ba);

   public:
    double dt_ = 0;                          // 预积分时间
    Mat9d cov_ = Mat9d::Zero();              // 累积噪声时间
    Mat6d noise_gyro_acce_ = Mat6d::Zero();  // 噪声矩阵

    // 零偏
    Vec3d bg_ = Vec3d::Zero();
    Vec3d ba_ = Vec3d::Zero();

    // 预积分的观测量
    SO3 dR_;
    Vec3d dv_ = Vec3d::Zero();
    Vec3d dp_ = Vec3d::Zero();
    // 雅可比矩阵
    Mat3d dR_dbg_ = Mat3d::Zero();
    Mat3d dv_dbg_ = Mat3d::Zero();
    Mat3d dv_dba_ = Mat3d::Zero();
    Mat3d dp_dbg_ = Mat3d::Zero();
    Mat3d dp_dba_ = Mat3d::Zero();
};

}  // namespace lh

#endif