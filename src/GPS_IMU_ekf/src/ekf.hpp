#pragma once

#include <stdio.h>
#include <Eigen/Dense>
#include <iostream>

/**
 * @brief ekf 融合imu和gps
 */
class EKF {
   public:
    EKF(bool verbose = false) : verbose(verbose) {}
    ~EKF() {}
    /**
     * @brief ekf预测阶段
     */
    void predict();
    /**
     * @brief ekf利用观测更新
     * @param Z 观测值
     * @param Hx 观测模型
     * @param JH 观测的jacobian
     * @param R 观测噪声
     */
    void update(const Eigen::VectorXd& Z, const Eigen::VectorXd& Hx, const Eigen::MatrixXd& JH, const Eigen::MatrixXd& R);

    /**
     * @brief 对ekf的状态量积分 imu的传播
     */
    void updateJA(const double dt);

    /**
     * @brief 获得系统的状态量
     */
    Eigen::VectorXd get_resulting_state() const;

    /**
     * @brief first time ekf is initialized
     */
    void start(const int nin, const Eigen::VectorXd& xin, const Eigen::MatrixXd& Pin, const Eigen::MatrixXd& Fin, const Eigen::MatrixXd& Qin);

    /**
     * @brief 设置过程噪声
     */
    void setQ(const Eigen::MatrixXd& Q_in);

   private:
    Eigen::VectorXd _state;  // 状态量二维 x,y,heading, velocity, yaw, acc
    Eigen::MatrixXd _K;      // 卡尔曼增益
    Eigen::MatrixXd _S;      // 存储中间变量的矩阵
    Eigen::MatrixXd _P;      // 误差协方差
    Eigen::MatrixXd _Q;      // 过程噪声
    Eigen::MatrixXd _R;      // 观测噪声
    Eigen::MatrixXd _JH;     // 观测的jacobian
    Eigen::MatrixXd _JA;     // 系统方程的jacobian
    Eigen::MatrixXd _I;      // 单位矩阵
    bool _init;              // 系统开始的标志位
    bool verbose;            // 打印信息
    int _num_states;         // 状态量的个数
};