#include "common/io_utils.h"
#include "eskf/eskf.hpp"
#include "eskf/static_imu_init.hh"
#include "imu_gnss/utm_convert.hh"
#include "tools/ui/pangolin_window.h"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <fstream>
#include <iomanip>

DEFINE_string(txt_path, "/home/slam_auto_driving/data/ch3/10.txt", "数据文件路径");
DEFINE_double(antenna_angle, 12.06, "RTK天线安装偏角（角度）");
DEFINE_double(antenna_pox_x, -0.17, "RTK天线安装偏移X");
DEFINE_double(antenna_pox_y, -0.20, "RTK天线安装偏移Y");
DEFINE_bool(with_ui, true, "是否显示图形界面");
DEFINE_bool(with_odom, false, "是否加入轮速计信息");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;

    google::ParseCommandLineFlags(&argc, &argv, true);

    if (fLS::FLAGS_txt_path.empty()) {
        return -1;
    }
    // 初始化
    lh::StaticIMUInit imu_init;
    lh::ESKFD eskf;

    lh::TxtIO io(FLAGS_txt_path);
    Vec2d antenna_pos(FLAGS_antenna_pox_x, FLAGS_antenna_pox_y);

    auto save_vec3 = [](std::ofstream& fout, const Vec3d& v) { fout << v[0] << " " << v[1] << " " << v[2] << " "; };
    auto save_quat = [](std::ofstream& fout, const Quatd& q) { fout << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << " "; };

    auto save_result = [&save_vec3, &save_quat](std::ofstream& fout, const lh::NavStated& save_state) {
        fout << std::setprecision(18) << save_state.timestamp_ << " " << std::setprecision(9);
        save_vec3(fout, save_state.p_);
        save_quat(fout, save_state.R_.unit_quaternion());
        save_vec3(fout, save_state.v_);
        save_vec3(fout, save_state.bg_);
        save_vec3(fout, save_state.ba_);
        fout << std::endl;
    };
    std::ofstream fout("/home/slam_auto_driving/data/ch3/gins.txt");
    bool imu_inited = false, gnss_inited = false;

    std::shared_ptr<lh::ui::PangolinWindow> ui = nullptr;
    if (FLAGS_with_ui) {
        ui = std::make_shared<lh::ui::PangolinWindow>();
        ui->Init();
    }
    /// 设置各类回调函数
    bool first_gnss_set = false;
    Vec3d origin = Vec3d::Zero();

    // io go的流程中先处理imu, gnss, odom
    io.SetIMUProcessFunc([&](const lh::IMU& imu) {
          // 静止初始化成功
          if (!imu_init.InitSuccess()) {
              imu_init.AddIMU(imu);
              return;
          }
          // 设置初始化的状态
          if (!imu_inited) {
              lh::ESKFD::Options options;
              options.gyro_var_ = sqrt(imu_init.GetCovGyro()[0]);
              options.acce_var_ = sqrt(imu_init.GetCovAcce()[0]);

              eskf.SetInitialConditions(options, imu_init.GetInitBg(), imu_init.GetInitBa(), imu_init.GetGravity());
              imu_inited = true;
              return;
          }
          // 等待gps数据
          if (!gnss_inited) {
              return;
          }
          // gps数据有效之后
          eskf.Predict(imu);
          // 发送数据
          auto state = eskf.GetNominalState();
          if (ui) {
              ui->UpdateNavState(state);
          }
          save_result(fout, state);
          usleep(1e3);
      })
        .SetGNSSProcessFunc([&](const lh::GNSS& gnss) {
            if (!imu_inited) {
                return;
            }

            lh::GNSS gnss_convert = gnss;
            // gps的数据无效
            if (!lh::ConvertGps2UTM(gnss_convert, antenna_pos, FLAGS_antenna_angle) || !gnss_convert.heading_valid_) {
                return;
            }
            if (!first_gnss_set) {
                origin = gnss_convert.utm_pose_.translation();
                first_gnss_set = true;
            }
            gnss_convert.utm_pose_.translation() -= origin;
            eskf.ObserveGps(gnss_convert);

            auto state = eskf.GetNominalState();
            if (ui) {
                ui->UpdateNavState(state);
            }
            save_result(fout, state);

            gnss_inited = true;
        })
        .SetOdomProcessFUncType([&](const lh::Odom& odom) {
            /// Odom 处理函数，本章Odom只给初始化使用
            imu_init.AddOdom(odom);
            if (FLAGS_with_odom && imu_inited && gnss_inited) {
                eskf.ObserveWheelSpeed(odom);
            }
        })
        .Go();

    while (ui && !ui->ShouldQuit()) {
        usleep(1e5);
    }
    if (ui) {
        ui->Quit();
    }
    return 0;
}