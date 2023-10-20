#pragma once
#include "pose_factor.h"

Eigen::Matrix<double, 3, 3> skew(const Eigen::Vector3d &mat) {
    Eigen::Matrix<double, 3, 3> skew_mat;
    skew_mat.setZero();
    skew_mat(0, 1) = -mat(2);
    skew_mat(0, 2) = mat(1);
    skew_mat(1, 2) = -mat(0);
    skew_mat(1, 0) = mat(2);
    skew_mat(2, 0) = -mat(1);
    skew_mat(2, 1) = mat(0);
    return skew_mat;
}

void getTransformFromSe3(const Eigen::Matrix<double, 6, 1> &se3, Eigen::Quaterniond &q, Eigen::Vector3d &t) {
    Eigen::Vector3d omega(se3.data());
    Eigen::Vector3d upsilon(se3.data() + 3);
    Eigen::Matrix3d Omega = skew(omega);

    double theta = omega.norm();
    double half_theta = 0.5 * theta;

    double imag_factor;
    double real_factor = std::cos(half_theta);
    if (theta < 1e-10) {
        double theta_sq = theta * theta;
        double theta_po4 = theta_sq * theta_sq;
        imag_factor = 0.5 - 0.0208333 * theta_sq + 0.000260417 * theta_po4;
    } else {
        double sin_half_theta = sin(half_theta);
        imag_factor = sin_half_theta / theta;
    }
    q = Eigen::Quaterniond(real_factor, imag_factor * omega.x(), imag_factor * omega.y(), imag_factor * omega.z());

    Eigen::Matrix3d J;
    if (theta < 1e-10)
        J = q.matrix();
    else {
        Eigen::Matrix3d Omega2 = Omega * Omega;
        J = (Eigen::Matrix3d::Identity() + (1 - std::cos(theta)) / (theta * theta) * Omega +
             (theta - std::sin(theta)) / (std::pow(theta, 3)) * Omega2);
    }
    t = J * upsilon;
}

Point2PlaneAnalyticCostFunction::Point2PlaneAnalyticCostFunction(Eigen::Vector3d cur_pt, std::vector<Eigen::Vector3d> near_pts)
    : cur_pt_(cur_pt), near_pts_(near_pts) {}

bool Point2PlaneAnalyticCostFunction::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    Eigen::Map<const Eigen::Quaterniond> q_last_curr(parameters[0]);
    Eigen::Map<const Eigen::Vector3d> t_last_curr(parameters[0] + 4);
    // Eigen::Vector3d lp = q_last_curr * cur_pt_ + t_last_curr;
    // residuals[0] = lp.x() - near_pts_.x();
    // residuals[1] = lp.y() - near_pt_.y();
    // residuals[2] = lp.z() - near_pt_.z();

    // if (jacobians != NULL) {
    //     Eigen::Matrix3d skew_lp = skew(cur_pt_);
    //     Eigen::Matrix<double, 3, 6> dp_by_se3;
    //     // dp_by_se3.block<3, 3>(0, 0) = -q_last_curr.matrix() * skew_lp;  //  右乘扰动
    //     dp_by_se3.block<3, 3>(0, 0) = -skew(q_last_curr * cur_pt);
    //     (dp_by_se3.block<3, 3>(0, 3)).setIdentity();
    //     Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>> J_se3(jacobians[0]);
    //     J_se3.setZero();
    //     J_se3.block<3, 6>(0, 0) = dp_by_se3;
    // }

    // return true;
}