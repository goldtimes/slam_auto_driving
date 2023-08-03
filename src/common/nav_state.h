#ifndef NAV_STATE_H
#define NAV_STATE_H

#include <sophus/so3.hpp>
#include "common/eigen_sophus.h"

namespace lh {
template <typename T>
struct NavState {
    using Vec3 = Eigen::Matrix<T, 3, 1>;
    using SO3 = Sophus::SO3<T>;

    NavState() = default;

    // from time, R, p, v, bg, ba
    explicit NavState(double time, const SO3& R = SO3(), const Vec3& t = Vec3::Zero(), const Vec3& v = Vec3::Zero(),
                      const Vec3& bg = Vec3::Zero(), const Vec3& ba = Vec3::Zero())
        : timestamp_(time), R_(R), p_(t), v_(v), bg_(bg), ba_(ba) {}

    // from pose and vel
    NavState(double time, const SE3& pose, const Vec3& vel = Vec3::Zero())
        : timestamp_(time), R_(pose.so3()), p_(pose.translation()), v_(vel) {}

    /// 转换到Sophus
    Sophus::SE3<T> GetSE3() const { return SE3(R_, p_); }
    // 重载 <<
    friend std::ostream& operator<<(std::ostream& os, const NavState<T>& data) {
        os << "p:" << data.p_.transpose() << ", q: " << data.R_.unit_quaternion().coeffs().transpose()
           << ",v: " << data.v_.transpose() << ", bg: " << data.bg_.transpose() << ", ba: " << data.ba_.transpose();
        return os;
    }

    double timestamp_ = 0;
    SO3 R_;                   // 旋转
    Vec3 p_ = Vec3::Zero();   // 平移
    Vec3 v_ = Vec3::Zero();   // 速度
    Vec3 bg_ = Vec3::Zero();  // gyro 零偏
    Vec3 ba_ = Vec3::Zero();  // acce 零偏
};

using NavStated = NavState<double>;
using NavStatef = NavState<float>;
}  // namespace lh

#endif