#include "ekf.hpp"

void EKF::start(const int num_state_in, const Eigen::VectorXd& state_in, const Eigen::MatrixXd& Pin, const Eigen::MatrixXd& Fin,
                const Eigen::MatrixXd& Qin) {
    _num_states = num_state_in;
    _I = Eigen::MatrixXd::Identity(_num_states, _num_states);
    if (verbose) {
        std::cout << "ekf num states->" << _num_states << std::endl;
    }
    _state.resize(_num_states);
    _state = state_in;
    _P = Pin;
    _JA = Fin;
    _Q = Qin;
}

Eigen::VectorXd EKF::get_resulting_state() const { return _state; }

void EKF::setQ(const Eigen::MatrixXd& Qin) { _Q = Qin; }

void updateJA(const double dt) {}

// 预测协方差
void EKF::predict() { _P = _JA * _P * _JA.transpose() + _Q; }

void EKF::update(const Eigen::VectorXd& Z, const Eigen::VectorXd& Hx, const Eigen::MatrixXd& JH, const Eigen::MatrixXd& R) {
    // 求卡尔曼增益
    Eigen::MatrixXd JHT = _P * JH.transpose();
    Eigen::MatrixXd _S = JH * JHT + R;
    _K = JHT * _S.inverse();
    // 更新状态量
    Eigen::VectorXd y = Z - Hx;  // 误差
    _state = _state + _K * y;    // 根据卡尔曼增益来调整误差的置信 就是更相信预测的数据还是观测的数据
    // 更新协方差
    _P = (_I - _K * JH) * _P;
}
