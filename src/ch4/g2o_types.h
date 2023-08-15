#ifndef SLAM_IN_AUTO_DRIVING_CH4_G2O_TYPES_H
#define SLAM_IN_AUTO_DRIVING_CH4_G2O_TYPES_H

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_multi_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/robust_kernel.h>

#include "common/eigen_sophus.h"
#include "imu_preintegration.h"

namespace lh {
/**
 * g2o的边定义, 这个边是预积分的边，约束的是上一时刻15维状态与下一时刻的旋转、平移、速度
 * 连接6个顶点: 上一帧的pose, v, bg, ba 下一帧的pose, v
 * 观测量为9维，预积分的残差r_R, r_v, r_p
 */

class EdgeInertial : public g2o::BaseMultiEdge<9, Vec9d> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**
     * 构造函数中需要指定预积分类对象
     * @param preinteg  预积分对象指针
     * @param gravity   重力矢量
     * @param weight    权重
     */
    EdgeInertial(std::shared_ptr<IMUPreintegration> preinteg, const Vec3d& gravity, double weight = 1.0);

    bool read(std::istream& is) override { return false; }
    bool write(std::ostream& os) const override { return false; }

    void computeError() override;
    void linearizeOplus() override;

    Eigen::Matrix<double, 24, 24> GetHessian() {
        linearizeOplus();
        Eigen::Matrix<double, 9, 24> J;
        J.block<9, 6>(0, 0) = _jacobianOplus[0];
        J.block<9, 3>(0, 6) = _jacobianOplus[1];
        J.block<9, 3>(0, 9) = _jacobianOplus[2];
        J.block<9, 3>(0, 12) = _jacobianOplus[3];
        J.block<9, 6>(0, 15) = _jacobianOplus[4];
        J.block<9, 3>(0, 21) = _jacobianOplus[5];
        return J.transpose() * information() * J;
    }

   private:
    const double dt_;
    std::shared_ptr<IMUPreintegration> preint_ = nullptr;
    Vec3d grav_;
};

}  // namespace lh

#endif