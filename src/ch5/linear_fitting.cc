#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/opencv.hpp>
#include "common/eigen_sophus.h"
#include "common/math_utils.h"

DEFINE_int32(num_tested_points_plane, 10, "测试平面拟合的点个数");
DEFINE_int32(num_tested_points_line, 100, "测试直线拟合的点个数");

DEFINE_double(noise_sigma, 0.01, "噪声");

void PlaneFittingTest();
void LineFittingTest();

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    LOG(INFO) << "testing plane fitting";
    PlaneFittingTest();

    LOG(INFO) << "testing line fitting";
    LineFittingTest();
}

void PlaneFittingTest() {
    // 平面参数
    Vec4d true_plane_coeffs(0.1, 0.2, 0.3, 0.4);
    true_plane_coeffs.normalize();

    std::vector<Vec3d> points;

    cv::RNG rng;
    for (int i = 0; i < FLAGS_num_tested_points_plane; ++i) {
        // 生成随机点
        Vec3d p(rng.uniform(0.0, 1.0), rng.uniform(0.0, 1.0), rng.uniform(0.0, 1.0));
        // 将点带入平面方程，计算第4维
        double n4 = -p.dot(true_plane_coeffs.head<3>()) / true_plane_coeffs[3];
        p = p / (n4 + std::numeric_limits<double>::min());  // 加上最小值，防止除零
        // 加上噪声
        p += Vec3d(rng.gaussian(FLAGS_noise_sigma), rng.gaussian(FLAGS_noise_sigma), rng.gaussian(FLAGS_noise_sigma));

        points.emplace_back(p);

        // 验证是否在平面上
        LOG(INFO) << "res of p:" << p.dot(true_plane_coeffs.head<3>()) + true_plane_coeffs[3];
    }
    // 利用生成的点，来得到一个估计的平面参数
    Vec4d estimate_plane_coeffs;
    if (lh::math::FitPlane(points, estimate_plane_coeffs)) {
        LOG(INFO) << "estimated coeffs: " << estimate_plane_coeffs.transpose() << ", true:" << true_plane_coeffs.transpose();
    } else {
        LOG(INFO) << "plane fitting failed";
    }
}

void LineFittingTest() {
    // 定义直线参数,定义了直线的起点和方向
    Vec3d true_line_origin(0.1, 0.2, 0.3);
    Vec3d true_line_dir(0.4, 0.5, 0.6);
    true_line_dir.normalize();
    // 随机生成点
    std::vector<Vec3d> points;
    cv::RNG rng;
    for (int i = 0; i < FLAGS_num_tested_points_line; ++i) {
        double t = rng.uniform(-1.0, 1.0);
        // 构造真实的点
        Vec3d p = true_line_origin + true_line_dir * t;
        // 点+噪声
        p += Vec3d(rng.gaussian(FLAGS_noise_sigma), rng.gaussian(FLAGS_noise_sigma), rng.gaussian(FLAGS_noise_sigma));
        points.emplace_back(p);
    }
    Vec3d esti_origin, esti_dir;
    if (lh::math::FitLine(points, esti_origin, esti_dir)) {
        LOG(INFO) << "est origin:" << esti_origin.transpose() << ",true_origin:" << true_line_origin.transpose();
        LOG(INFO) << "est dir:" << esti_dir.transpose() << ",true_dir:" << true_line_dir.transpose();
    } else {
        LOG(INFO) << "line fitting failed";
    }
}