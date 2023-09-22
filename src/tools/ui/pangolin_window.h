#ifndef UI_PANGOLIN_WINDOW_H
#define UI_PANGOLIN_WINDOW_H

#include <eigen3/Eigen/Dense>
#include <map>
#include <memory>
#include "common/eigen_sophus.h"
#include "common/gnss.h"
#include "common/nav_state.h"
#include "common/point_types.h"

namespace lh::ui {
class PangolinWindowImpl;

class PangolinWindow {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    PangolinWindow();
    ~PangolinWindow();

    bool Init();

    /// 等待显示线程结束，并释放资源
    void Quit();

    /// 用户是否已经退出UI
    bool ShouldQuit();

    // 更新kalman滤波状态
    void UpdateNavState(const NavStated& state);

    // 更新激光地图点云，在激光定位中的地图发生改变时，由fusion调用
    void UpdatePointCloudGlobal(const std::map<Vec2i, CloudPtr, less_vec<2>>& cloud);

    // 更新scan和pose
    void UpdateScan(CloudPtr cloud, const SE3& pose);

    // 更新GNSS数据
    void UpdateGps(const GNSS& gps);

    // 设置IMU到雷达的外参
    void SetImuToLidar(const SE3& T_imu_lidar);

    // 设置保留scan的个数
    void SetCurrentScanSize(int current_scan_size);

   private:
    // ...
    std::shared_ptr<PangolinWindowImpl> impl_ = nullptr;
};

}  // namespace lh::ui

#endif