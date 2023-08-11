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

/**
 * @param C 抽象的容器类型
 * @param D 抽象的累加结果类型
 * @param Getter 获取数据函数, 接收一个容器内数据类型，返回一个D类型
 * @brief Getter&& getter 一定要声明为右值引用，否则会报错，因为在调用该函数时，传入的是一个匿名的lambda函数
 */
template <typename C, typename D, typename Getter>
void ComputeMeanAndCovDiag(const C& datas, D& means, D& cov_diag, Getter&& getter) {
    size_t len = data.size();
    assert(len > 1);
    // clang-format off
    means = std::accumulate(
        datas.begin(), datas.end(), D::Zero().eval(),
        [&getter](const D& sum, const D& data_in_container) -> D { 
            return sum + getter(data_in_container); 
        });

    cov_diag = std::accumulate(
        datas.begin(), datas.end(), D::Zero().eval(),
        [&mean, &getter](const D& sum, const D& data_in_container) -> D { 
            return sum + (getter(data_in_container) - means).cwiseAb2().eval(); 
        }) / (len - 1);
    // clang-format on
}
}  // namespace lh::math

#endif