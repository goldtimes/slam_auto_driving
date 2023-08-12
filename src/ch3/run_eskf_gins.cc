#include "common/io_utils.h"
#include "eskf.hpp"
#include "static_imu_init.h"
#include "tools/ui/pangolin_window.h"
#include "utm_convert.h"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <fstream>
#include <iomanip>

DEFINE_string(txt_path, "/home/slam_auto_driving/data/ch3/10.txt", "数据文件路径");
DEFINE_double(antenna_angle, 12.06, "RTK天线安装偏角(角度)");
DEFINE_double(antenna_pox_x, -0.17, "RTK天线安装偏移X");
DEFINE_double(antenna_pox_y, -0.20, "RTK天线安装偏移Y");
DEFINE_bool(with_ui, true, "是否显示图形界面");
DEFINE_bool(with_odom, true, "是否加入轮速计信息");

/**
 * RTK + IMU 组合导航
 */
int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    if (fLS::FLAGS_txt_path.empty()) {
        return -1;
    }

    // 静止初始化器
    lh::StaticImuInit imu_init;
    lh::ESKFD eskf;

    lh::TxtIO io(FLAGS_txt_path);

    Vec2d antenna_pos(FLAGS_antenna_pox_x, FLAGS_antenna_pox_y);

    auto save_vec3 = [](std::ofstream& fout, const Vec3d& v) { fout << v[0] << " " << v[1] << " " << v[2] << " "; };
    auto save_quat = [](std::ofstream& fout, const Quatd& q) {
        fout << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << " ";
    };

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

    io.SetIMUProcessFunc([&](const lh::IMU& imu) {
          // imu 没初始化成功
          if (!imu_init.InitSuccess()) {
              imu_init.AddIMU(imu);
              return;
          }

          // imu 初始化成功后
          if (!imu_inited) {
              //读取零偏
              lh::ESKFD::Options options;
              options.gyro_var_ = sqrt(imu_init.GetGovGyro()[0]);
              options.acce_var_ = sqrt(imu_init.GetGovAcce()[0]);
              eskf.SetInitialConditions(options, imu_init.GetInitBg(), imu_init.GetInitBa(), imu_init.GetGravity());
              imu_inited = true;
              return;
          }
          // 等待rtk有效数据
          if (!gnss_inited) {
              return;
          }
          // GNSS接收到有效数据之后，开始对imu积分 预测

          eskf.Predict(imu);

          auto state = eskf.GetNomialState();
          if (ui) {
              ui->UpdateNavState(state);
          }

          // 记录数据
          save_result(fout, state);
          usleep(1e3);
      })
        .SetGNSSProcessFunc([&](const lh::GNSS& gnss) {
            // 等待imu初始化完成
            if (!imu_inited)
                return;
            lh::GNSS gnss_convert = gnss;
            // 在heading_valid条件成立时,gnss读书->utm
            if (!lh::ConvertGps2UTM(gnss_convert, antenna_pos, FLAGS_antenna_angle) || !gnss_convert.heading_valid_) {
                return;
            }
            // 第一帧gnss
            if (!first_gnss_set) {
                origin = gnss_convert.utm_pose_.translation();
                first_gnss_set = true;
            }
            // 去除原点
            gnss_convert.utm_pose_.translation() -= origin;
            // 加入观测
            eskf.ObserveGps(gnss_convert);
            auto state = eskf.GetNomialState();
            if (ui) {
                ui->UpdateNavState(state);
            }
            save_result(fout, state);
            gnss_inited = true;
        })
        .SetOdomProcessFUncType([&](const lh::Odom& odom) {
            // imu 初始化需要使用速度信息
            imu_init.AddOdom(odom);
            if (FLAGS_with_odom && imu_inited && gnss_inited) {
                eskf.ObserveWheelSpeed(odom);
            }
        })
        .Go();
    while (ui && !ui->ShouldQuit()) {
        usleep(1e6);
    }
    if (ui) {
        ui->Quit();
    }
    return 0;
}