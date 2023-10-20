/**
 * 继承LocalParameterization 使用四元素的优化参数
 * 继承SizedCostFunction,定义点到面的cost function
 */
#pragma once

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

Eigen::Matrix3d skew(const Eigen::Vector3d &mat);

void getTransformFromSe3(const Eigen::Matrix<double, 6, 1> &se3, Eigen::Quaterniond &q, Eigen::Vector3d &t);

class Point2PlaneAnalyticCostFunction : public ceres::SizedCostFunction<3, 7> {
   public:
    Point2PlaneAnalyticCostFunction(Eigen::Vector3d curr_pt, std::vector<Eigen::Vector3d> near_pts_);
    virtual ~Point2PlaneAnalyticCostFunction() {}
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

    Eigen::Vector3d cur_pt_, near_pts_;
};

// class PoseSE3Parameterization : public ceres::LocalParameterization {
//    public:
//     PoseSE3Parameterization() {}
//     virtual ~PoseSE3Parameterization(){};
//     virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const;
//     virtual bool ComputeJacobian(const double *x, double *jacobian) const;
//     virtual int GlobalSize() const { return 7; }
//     virtual int LocalSize() const { return 6; }
// };

// class;