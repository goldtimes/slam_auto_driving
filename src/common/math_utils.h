#ifndef MAPPING_MATH_UTILS_H
#define MAPPING_MATH_UTILS_H

#include <glog/logging.h>
#include <boost/array.hpp>                 // bpost数组
#include <boost/math/tools/precision.hpp>  // boost 小数
#include <iomanip>                         // c++ 格式设置
#include <limits>                          // 数值范围
#include <map>
#include <numeric>
#include <opencv4/opencv2/core.hpp>

namespace lh::math {
// 常量定义
constexpr double DEG2RAD = M_PI / 180.0;  // deg->rad
}  // namespace lh::math

#endif