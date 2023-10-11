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

/**
 * 计算一个容器内数据的均值与矩阵形式协方差
 * @tparam C    容器类型
 * @tparam int 　数据维度
 * @tparam Getter   获取数据函数, 接收一个容器内数据类型，返回一个Eigen::Matrix<double, dim,1> 矢量类型
 */
template <typename C, int dim, typename Getter>
void ComputeMeanAndCov(const C& data, Eigen::Matrix<double, dim, 1>& mean, Eigen::Matrix<double, dim, dim>& cov, Getter&& getter) {
    using D = Eigen::Matrix<double, dim, 1>;
    using E = Eigen::Matrix<double, dim, dim>;
    size_t len = data.size();
    assert(len > 1);

    // clang-format off
    mean = std::accumulate(data.begin(), data.end(), Eigen::Matrix<double, dim, 1>::Zero().eval(),
                           [&getter](const D& sum, const auto& data) -> D { return sum + getter(data); }) / len;
    cov = std::accumulate(data.begin(), data.end(), E::Zero().eval(),
                          [&mean, &getter](const E& sum, const auto& data) -> E {
                              D v = getter(data) - mean;
                              return sum + v * v.transpose();
                          }) / (len - 1);
    // clang-format on
}

/**
 * 高斯分布合并 7.24,7.25的公式
 * @tparam S    scalar type
 * @tparam D    dimension
 * @param hist_m        历史点数
 * @param curr_n        当前点数
 * @param hist_mean     历史均值
 * @param hist_var      历史方差
 * @param curr_mean     当前均值
 * @param curr_var      当前方差
 * @param new_mean      新的均值
 * @param new_var       新的方差
 */
template <typename S, int D>
void UpdateMeanAndCov(int hist_m, int curr_n, const Eigen::Matrix<S, D, 1>& hist_mean, const Eigen::Matrix<S, D, D>& hist_var,
                      const Eigen::Matrix<S, D, 1>& curr_mean, const Eigen::Matrix<S, D, D>& curr_var, Eigen::Matrix<S, D, 1>& new_mean,
                      Eigen::Matrix<S, D, D>& new_var) {
    assert(hist_m > 0);
    assert(curr_n > 0);
    new_mean = (hist_m * hist_mean + curr_n * curr_mean) / (hist_m + curr_n);
    new_var = (hist_m * (hist_var + (hist_mean - new_mean) * (hist_mean - new_mean).template transpose()) +
               curr_n * (curr_var + (curr_mean - new_mean) * (curr_mean - new_mean).template transpose())) /
              (hist_m + curr_n);
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

template <typename S>
bool FitLine2D(const std::vector<Eigen::Matrix<S, 2, 1>> data, Eigen::Matrix<S, 3, 1>& coeffs) {
    if (data.size() < 2) {
        return false;
    }

    Eigen::MatrixXd A(data.size(), 3);
    for (int i = 0; i < data.size(); ++i) {
        A.row(i).head<2>() = data[i].transpose();
        A.row(i)[2] = 1.0;
    }
    Eigen::JacobiSVD svd(A, Eigen::ComputeThinV);
    coeffs = svd.matrixV().col(2);
    return true;
}

/**
 * @brief 双线性插值
 */
template <typename T>
inline float GetPixelValue(const cv::Mat& img, float x, float y) {
    if (x < 0)
        x = 0;
    if (y < 0)
        y = 0;
    if (x >= img.cols)
        x = img.cols - 1;
    if (y >= img.rows)
        y = img.rows - 1;
    const T* data = &img.at<T>(floor(y), floor(x));
    float xx = x - floor(x);
    float yy = y - floor(y);

    return ((1 - xx) * (1 - yy) * data[0] + xx * (1 - yy) * data[1] + (1 - xx) * yy * data[img.step / sizeof(T)] +
            xx * yy * data[img.step / sizeof(T) + 1]);
}

/**
 * 将角度保持在正负PI以内
 */
inline void KeepAngleInPI(double& angle) {
    while (angle < -M_PI) {
        angle += 2 * M_PI;
    }
    while (angle > M_PI) {
        angle -= 2 * M_PI;
    }
}

template <typename S>
inline Eigen::Matrix<S, 3, 1> VecFromArray(const std::vector<S>& v) {
    return Eigen::Matrix<S, 3, 1>(v[0], v[1], v[2]);
}

template <typename S>
inline Eigen::Matrix<S, 3, 3> MatFromArray(const std::vector<S>& v) {
    Eigen::Matrix<S, 3, 3> m;
    m << v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8];
    return m;
}
/**
 * pose 插值算法
 * @tparam T    数据类型
 * @tparam C 数据容器类型
 * @tparam FT 获取时间函数
 * @tparam FP 获取pose函数
 * @param query_time 查找时间
 * @param data  数据容器
 * @param take_pose_func 从数据中取pose的谓词，接受一个数据，返回一个SE3
 * @param result 查询结果
 * @param best_match_iter 查找到的最近匹配
 *
 * NOTE 要求query_time必须在data最大时间和最小时间之间(容许0.5s内误差)
 * data的map按时间排序
 * @return
 */
template <typename T, typename C, typename FT, typename FP>
inline bool PoseInterp(double query_time, C&& data, FT&& take_time_func, FP&& take_pose_func, SE3& result, T& best_match, float time_th = 0.5) {
    if (data.empty()) {
        LOG(INFO) << "cannot interp because data is empty. ";
        return false;
    }

    double last_time = take_time_func(*data.rbegin());
    // 查询的时间 > 队列中最晚的时间
    if (query_time > last_time) {
        if (query_time < (last_time + time_th)) {
            result = take_pose_func(*data.rbegin());
            best_match = *data.rbegin();
            return true;
        }
        return false;
    }
    //   data1  < 查询的时间 < data2
    auto match_iter = data.begin();
    for (auto iter = data.begin(); iter != data.end(); ++iter) {
        auto next_iter = iter;
        next_iter++;

        if (take_time_func(*iter) < query_time && take_time_func(*next_iter) >= query_time) {
            match_iter = iter;
            break;
        }
    }

    auto match_iter_n = match_iter;
    match_iter_n++;

    double dt = take_time_func(*match_iter_n) - take_time_func(*match_iter);
    double s = (query_time - take_time_func(*match_iter)) / dt;  // s=0 时为第一帧，s=1时为next
    // 出现了 dt为0的bug
    if (fabs(dt) < 1e-6) {
        best_match = *match_iter;
        result = take_pose_func(*match_iter);
        return true;
    }

    SE3 pose_first = take_pose_func(*match_iter);
    SE3 pose_next = take_pose_func(*match_iter_n);
    result = {pose_first.unit_quaternion().slerp(s, pose_next.unit_quaternion()), pose_first.translation() * (1 - s) + pose_next.translation() * s};
    best_match = s < 0.5 ? *match_iter : *match_iter_n;
    return true;
}

template <typename T>
T rad2deg(const T& radians) {
    return radians * 180.0 / M_PI;
}

template <typename T>
T deg2rad(const T& degrees) {
    return degrees * M_PI / 180.0;
}

/**
 * 边缘化
 * @param H
 * @param start
 * @param end
 * @return
 */
inline Eigen::MatrixXd Marginalize(const Eigen::MatrixXd& H, const int& start, const int& end) {
    // Goal
    // a  | ab | ac       a*  | 0 | ac*
    // ba | b  | bc  -->  0   | 0 | 0
    // ca | cb | c        ca* | 0 | c*

    // Size of block before block to marginalize
    const int a = start;
    // Size of block to marginalize
    const int b = end - start + 1;
    // Size of block after block to marginalize
    const int c = H.cols() - (end + 1);

    // Reorder as follows:
    // a  | ab | ac       a  | ac | ab
    // ba | b  | bc  -->  ca | c  | cb
    // ca | cb | c        ba | bc | b

    Eigen::MatrixXd Hn = Eigen::MatrixXd::Zero(H.rows(), H.cols());
    if (a > 0) {
        Hn.block(0, 0, a, a) = H.block(0, 0, a, a);
        Hn.block(0, a + c, a, b) = H.block(0, a, a, b);
        Hn.block(a + c, 0, b, a) = H.block(a, 0, b, a);
    }
    if (a > 0 && c > 0) {
        Hn.block(0, a, a, c) = H.block(0, a + b, a, c);
        Hn.block(a, 0, c, a) = H.block(a + b, 0, c, a);
    }
    if (c > 0) {
        Hn.block(a, a, c, c) = H.block(a + b, a + b, c, c);
        Hn.block(a, a + c, c, b) = H.block(a + b, a, c, b);
        Hn.block(a + c, a, b, c) = H.block(a, a + b, b, c);
    }
    Hn.block(a + c, a + c, b, b) = H.block(a, a, b, b);

    // Perform marginalization (Schur complement)
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(Hn.block(a + c, a + c, b, b), Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType singularValues_inv = svd.singularValues();
    for (int i = 0; i < b; ++i) {
        if (singularValues_inv(i) > 1e-6)
            singularValues_inv(i) = 1.0 / singularValues_inv(i);
        else
            singularValues_inv(i) = 0;
    }
    Eigen::MatrixXd invHb = svd.matrixV() * singularValues_inv.asDiagonal() * svd.matrixU().transpose();
    Hn.block(0, 0, a + c, a + c) = Hn.block(0, 0, a + c, a + c) - Hn.block(0, a + c, a + c, b) * invHb * Hn.block(a + c, 0, b, a + c);
    Hn.block(a + c, a + c, b, b) = Eigen::MatrixXd::Zero(b, b);
    Hn.block(0, a + c, a + c, b) = Eigen::MatrixXd::Zero(a + c, b);
    Hn.block(a + c, 0, b, a + c) = Eigen::MatrixXd::Zero(b, a + c);

    // Inverse reorder
    // a*  | ac* | 0       a*  | 0 | ac*
    // ca* | c*  | 0  -->  0   | 0 | 0
    // 0   | 0   | 0       ca* | 0 | c*
    Eigen::MatrixXd res = Eigen::MatrixXd::Zero(H.rows(), H.cols());
    if (a > 0) {
        res.block(0, 0, a, a) = Hn.block(0, 0, a, a);
        res.block(0, a, a, b) = Hn.block(0, a + c, a, b);
        res.block(a, 0, b, a) = Hn.block(a + c, 0, b, a);
    }
    if (a > 0 && c > 0) {
        res.block(0, a + b, a, c) = Hn.block(0, a, a, c);
        res.block(a + b, 0, c, a) = Hn.block(a, 0, c, a);
    }
    if (c > 0) {
        res.block(a + b, a + b, c, c) = Hn.block(a, a, c, c);
        res.block(a + b, a, c, b) = Hn.block(a, a + c, c, b);
        res.block(a, a + b, b, c) = Hn.block(a + c, a, b, c);
    }

    res.block(a, a, b, b) = Hn.block(a + c, a + c, b, b);

    return res;
}

}  // namespace lh::math

#endif