/**
 * 四元素和so3来在运动学上的差异
 * 做圆周运动
 */

#include <gflags/gflags.h>
#include <glog/logging.h>
#include "common/eigen_sophus.h"
#include "common/math_utils.h"
#include "common/nav_state.h"
#include "tools/ui/pangolin_window.h"

DEFINE_double(angular_velocity, 10.0, "角速度");
DEFINE_double(linear_velocity, 5.0, "线速度");
DEFINE_bool(use_quaternion, false, "是否使用四元素计算");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);

    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    // 可视化

    lh::ui::PangolinWindow ui;
    if (ui.Init() == false)
        return -1;
    double angular_velocity_rad = FLAGS_angular_velocity * lh::math::DEG2RAD;  // 转到弧度制
    SE3 pose;                                                                  // T_B_W 表示的姿态
    Vec3d omega(0, 0, angular_velocity_rad);                                   // 定义角速度
    Vec3d v_body(FLAGS_linear_velocity, 0, 0);                                 // 定义线速度

    const double dt = 0.05;
    while (ui.ShouldQuit() == false) {
        // 初始姿态 * 速度 得到世界坐标下的姿态
        // 1. 更新xyz
        Vec3d v_world = pose.so3() * v_body;
        pose.translation() += v_world * dt;
        if (FLAGS_use_quaternion) {
            // 2. 更新姿态
            Quatd q = pose.unit_quaternion() * Quatd(1, 0.5 * omega[0] * dt, 0.5 * omega[1] * dt, 0.5 * omega[2] * dt);
            q.normalize();
            pose.so3() = SO3(q);
        } else {
            // 2. 更新姿态
            // 旋转向量李代数->李群 旋转矩阵
            auto so3d = SO3::exp(omega * dt);
            pose.so3() = pose.so3() * so3d;
        }
        LOG(INFO) << "pose: " << pose.translation().transpose();
        ui.UpdateNavState(lh::NavStated(0, pose, v_world));
        usleep(dt * 1e6);
    }
    ui.Quit();
    return 0;
}