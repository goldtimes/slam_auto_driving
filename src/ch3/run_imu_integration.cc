#include <glog/logging.h>
#include <iomanip>
#include "imu_integration.h"

#include <fstream>
#include <iostream>
#include "common/io_utils.h"
#include "tools/ui/pangolin_window.h"

DEFINE_string(imu_txt_path, "/home/slam_auto_driving/data/ch3/10.txt", "数据文件路径");
DEFINE_bool(with_ui, true, "是否显示图像界面");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    if (FLAGS_imu_txt_path.empty()) {
        return -1;
    }

    lh::TxtIO io(FLAGS_imu_txt_path);
    // 假设零偏是已经知道的
    Vec3d gravity(0, 0, -9.8);
    Vec3d init_bg(00.000224886, -7.61038e-05, -0.000742259);
    Vec3d init_ba(-0.165205, 0.0926887, 0.0058049);

    lh::IMUIntegration imu_integration(gravity, init_bg, init_ba);
    std::shared_ptr<lh::ui::PangolinWindow> ui = nullptr;
    if (FLAGS_with_ui) {
        ui = std::make_shared<lh::ui::PangolinWindow>();
        ui->Init();
    }
    // 定义lambda函数
    auto save_result = [](std::ofstream& fout, double timestamp, const Sophus::SO3d& R, const Vec3d& v,
                          const Vec3d& p) {
        auto save_vec3 = [](std::ofstream& fout, const Vec3d& v) { fout << v[0] << " " << v[1] << " " << v[2] << " "; };
        auto save_qua = [](std::ofstream& fout, const Quatd& q) {
            fout << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << " ";
        };

        fout << std::setprecision(18) << timestamp << " " << std::setprecision(9);
        save_vec3(fout, p);
        save_qua(fout, R.unit_quaternion());
        save_vec3(fout, v);
        fout << std::endl;
    };

    std::ofstream fout("/home/slam_auto_driving/data/ch3/state.txt");
    // 在io的go中读取到imu数据，然后调用传入的回调函数处理imu数据,积分和更新状态
    io.SetIMUProcessFunc([&imu_integration, &save_result, &fout, &ui](const lh::IMU& imu) {
          imu_integration.AddImu(imu);
          save_result(fout, imu.timestamp_, imu_integration.GetR(), imu_integration.GetV(), imu_integration.GetP());
          if (ui) {
              ui->UpdateNavState(imu_integration.GetNavState());
              usleep(1e2);
          }
      })
        .Go();

    while (ui && !ui->ShouldQuit()) {
        usleep(1e4);
    }
    if (ui) {
        ui->Quit();
    }

    return 0;
}