#ifndef MAPPING_MATH_UTILS_H
#define MAPPING_MATH_UTILS_H

#include <glog/logging.h>
#include <algorithm>
#include <boost/array.hpp>                 // bpost数组
#include <boost/math/tools/precision.hpp>  // boost 小数
#include <iomanip>                         // c++ 格式设置
#include <limits>                          // 数值范围
#include <map>
#include <numeric>
#include <opencv4/opencv2/core.hpp>

namespace lh::math {
// 常量定义
constexpr double kDEG2RAD = M_PI / 180.0;  // deg->rad
constexpr double kRAD2DEG = 180.0 / M_PI;  // rad -> deg
constexpr double G_m_s2 = 9.81;            // 重力大小
// 非法定义
constexpr size_t kINVALID_ID = std::numeric_limits<size_t>::max();

/**
 * @param C 抽象的容器类型
 * @param D 抽象的累加结果类型
 * @param Getter 获取数据函数, 接收一个容器内数据类型，返回一个D类型
 * @brief Getter&& getter 一定要声明为右值引用，否则会报错，因为在调用该函数时，传入的是一个匿名的lambda函数
 */
// template <typename C, typename D, typename Getter>
// void ComputeMeanAndCovDiag(const C& datas, D& means, D& cov_diag, Getter&& getter) {
//     size_t len = datas.size();
//     assert(len > 1);
//     // clang-format off
//     means = std::accumulate(
//         datas.begin(), datas.end(), D::Zero().eval(),
//         [&getter](const D& sum, const auto& datas) -> D {
//             return sum + getter(datas);
//         });

//     cov_diag = std::accumulate(
//         datas.begin(), datas.end(), D::Zero().eval(),
//         [&means, &getter](const D& sum, const auto& datas) -> D {
//             return sum + (getter(datas) - means).cwiseAbs2().eval();
//         }) / (len - 1);
//     // clang-format on
// }

/**
 * 计算一个容器内数据的均值与对角形式协方差
 * @tparam C    容器类型
 * @tparam D    结果类型
 * @tparam Getter   获取数据函数, 接收一个容器内数据类型，返回一个D类型
 */
template <typename C, typename D, typename Getter>
void ComputeMeanAndCovDiag(const C& data, D& mean, D& cov_diag, Getter&& getter) {
    size_t len = data.size();
    assert(len > 1);
    // clang-format off
    mean = std::accumulate(data.begin(), data.end(), D::Zero().eval(),
                           [&getter](const D& sum, const auto& data) -> D { return sum + getter(data); }) / len;
    cov_diag = std::accumulate(data.begin(), data.end(), D::Zero().eval(),
                               [&mean, &getter](const D& sum, const auto& data) -> D {
                                   return sum + (getter(data) - mean).cwiseAbs2().eval();
                               }) / (len - 1);
    // clang-format on
}

template <typename S>
bool FitPlane(std::vector<Eigen::Matrix<S, 3, 1>>& data, Eigen::Matrix<S, 4, 1>& plane_coeffs, double eps = 1e-2) {
    if (data.size() < 3)
        return false;
    // 每个点都有4个未知参数
    Eigen::MatrixXd A(data.size(), 4);
    for (int i = 0; i < data.size(); i++) {
        A.row(i).head<3>() = data[i].transpose();
        A.row(i)[3] = 1.0;
    }
    Eigen::JacobiSVD svd(A, Eigen::ComputeThinV);
    plane_coeffs = svd.matrixV().col(3);
    // check error
    for (int i = 0; i < data.size(); i++) {
        double err = plane_coeffs.template head<3>().dot(data[i]) + plane_coeffs[3];
        if (err * err > eps) {
            return false;
        }
    }
    return true;
}

template <typename S>
bool FitLine(std::vector<Eigen::Matrix<S, 3, 1>>& data, Eigen::Matrix<S, 3, 1>& origin, Eigen::Matrix<S, 3, 1>& dir, double eps = 0.2) {
    if (data.size() < 2) {
        return false;
    }
    origin = std::accumulate(data.begin(), data.end(), Eigen::Matrix<S, 3, 1>::Zero().eval()) / data.size();

    Eigen::MatrixXd Y(data.size(), 3);
    for (int i = 0; i < data.size(); ++i) {
        Y.row(i) = (data[i] - origin).transpose();
    }
    Eigen::JacobiSVD svd(Y, Eigen::ComputeFullV);
    dir = svd.matrixV().col(0);

    for (const auto& d : data) {
        if (dir.template cross(d - origin).template squaredNorm() > eps) {
            return false;
        }
    }
    return true;
}

}  // namespace lh::math

#endif