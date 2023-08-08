#include <glog/logging.h>
#include <memory.h>
#include <iomanip>

#include "common/gnss.h"
#include "common/io_utils.h"
#include "tools/ui/pangolin_window.h"
#include "utm_convert.h"

DEFINE_string(txt_path, "/home/slam_auto_driving/data/ch3/10.txt", "数据文件路径");

DEFINE_double(antenna_angle, 12.06, "RTK 天线安装偏角(角度)");
DEFINE_double(antenna_pox_x, -0.17, "RTK 天线安装偏移x");
DEFINE_double(antenna_pox_y, -0.20, "RTK 天线安装偏移y");
DEFINE_bool(with_ui, true, "是否显示图形界面");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    if (fLS::FLAGS_txt_path.empty()) {
        return -1;
    }

    lh::TxtIO io(fLS::FLAGS_txt_path);

    std::ofstream fout("/home/slam_auto_driving/data/ch3/gnss_output.txt");
    Vec2d antenna_pos(FLAGS_antenna_pox_x, FLAGS_antenna_pox_y);

    auto save_result_func = [](std::ofstream& fout, double timestamp, const SE3& pose) {
        auto save_vec3_func = [](std::ofstream& fout, const Vec3d& v) {
            fout << v[0] << " " << v[1] << " " << v[2] << " ";
        };
        auto save_quat_func = [](std::ofstream& fout, const Quatd& q) {
            fout << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << " ";
        };

        fout << std::setprecision(18) << timestamp << " " << std::setprecision(9);
        save_vec3_func(fout, pose.translation());
        save_quat_func(fout, pose.unit_quaternion());
        fout << std::endl;
    };

    std::shared_ptr<lh::ui::PangolinWindow> ui = nullptr;
    if (FLAGS_with_ui) {
        ui = std::make_shared<lh::ui::PangolinWindow>();
        ui->Init();
    }

    bool first_gnss_set = false;
    Vec3d origin = Vec3d::Zero();

    io.SetGNSSProcessFunc([&](const lh::GNSS& gnss) {
          lh::GNSS gnss_out = gnss;
          if (lh::ConvertGps2UTM(gnss_out, antenna_pos, FLAGS_antenna_angle)) {
              if (!first_gnss_set) {
                  origin = gnss_out.utm_pose_.translation();
                  first_gnss_set = true;
              }
              // 减去原点
              gnss_out.utm_pose_.translation() -= origin;

              save_result_func(fout, gnss_out.unix_time_, gnss_out.utm_pose_);
              ui->UpdateNavState(
                  lh::NavStated(gnss_out.unix_time_, gnss_out.utm_pose_.so3(), gnss_out.utm_pose_.translation()));
              usleep(1e3);
          }
      })
        .Go();

    if (ui) {
        while (!ui->ShouldQuit()) {
            usleep(1e5);
        }
        ui->Quit();
    }
    return 0;
}
