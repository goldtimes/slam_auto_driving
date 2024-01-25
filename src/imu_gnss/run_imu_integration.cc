#include <glog/logging.h>
#include <iomanip>

#include "common/io_utils.h"
#include "imu_gnss/imu_integration.hh"
#include "tools/ui/pangolin_window.h"

DEFINE_string(imu_txt_path, "/home/slam_auto_driving/data/ch3/10.txt", "数据文件");
DEFINE_bool(with_ui, true, "是否显示图形界面");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::INFO;

    google::ParseCommandLineFlags(&argc, &argv, true);
    if (FLAGS_imu_txt_path.empty()) {
        return -1;
    }

    lh::TxtIO io(FLAGS_imu_txt_path);

    //
    Vec3d gravity(0, 0, -9.8);
    Vec3d init_bg(00.000224886, -7.61038e-05, -0.000742259);
    Vec3d init_ba(-0.165205, 0.0926887, 0.0058049);

    lh::ImuIntegration imu_integ(init_ba, init_bg);

    std::shared_ptr<lh::ui::PangolinWindow> ui = nullptr;
    if (FLAGS_with_ui) {
        ui = std::make_shared<lh::ui::PangolinWindow>();
        ui->Init();
    }

    // 定义一个保存结果的函数
    auto save_result = [](std::ofstream& fout, double time, const Sophus::SO3d& R, const Vec3d& p, const Vec3d& v) {
        //  因为要保存多个vec3,所以定义函数
        auto save_vec3 = [](std::ofstream& fout, const Vec3d& v) { fout << v[0] << " " << v[1] << " " << v[2] << " "; };
        auto save_quat = [](std::ofstream& fout, const Quatd& q) { fout << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << " "; };
        // 设置精度 <iomanip>
        fout << std::setprecision(18) << time << " " << std::setprecision(9);
        save_vec3(fout, p);
        save_quat(fout, R.unit_quaternion());
        save_vec3(fout, v);
        fout << std::endl;
    };
    std::ofstream fout("/home/slam_auto_driving/data/ch3/state.txt");
    // 传入一个lambda函数,正常调用的地方在io 的Go() 循环中
    // 回调函数实现: 在A程序中定义, 传递给B程序, B程序中调用. B程序不管函数的实现
    // 通过lambda可以捕获A中的变量,通过函数传递,可以获取B程序中的变量
    // SetIMUProcessFunc 设置回调函数,捕获A中的imu_integ,save_result, 通过传递参数imu获得了B(调用程序)的imu变量
    io.SetIMUProcessFunc([&imu_integ, &save_result, &fout, &ui](const lh::IMU& imu) {
          // 积分
          imu_integ.AddImu(imu);
          save_result(fout, imu.timestamp_, imu_integ.GetR(), imu_integ.GetP(), imu_integ.GetV());
          if (ui) {
              ui->UpdateNavState(imu_integ.GetNavStated());
              usleep(1e2);
          }
      })
        .Go();

    // 打开了可视化的话，等待界面退出
    while (ui && !ui->ShouldQuit()) {
        usleep(1e4);
    }

    if (ui) {
        ui->Quit();
    }

    return 0;
}