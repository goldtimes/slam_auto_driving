#ifndef FUSION_UI_PANGOLIN_WINDOW_IMPL_H
#define FUSION_UI_PANGOLIN_WINDOW_IMPL_H

#include <pangolin/pangolin.h>
#include <pcl/filters/voxel_grid.h>
#include <atomic>
#include <eigen3/Eigen/Dense>
#include <mutex>
#include <sophus/se3.hpp>
#include <string>
#include <thread>
#include "common/eigen_sophus.h"
#include "tools/ui/ui_car.h"
#include "tools/ui/ui_cloud.h"
#include "tools/ui/ui_trajectory.h"

namespace lh::ui {
struct UIFrame;

class PangolinWindowImpl {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    PangolinWindowImpl() = default;
    ~PangolinWindowImpl() = default;
    // 禁止拷贝
    PangolinWindowImpl(const PangolinWindowImpl& other) = delete;
    PangolinWindowImpl& operator=(const PangolinWindowImpl& other) = delete;
    PangolinWindowImpl(PangolinWindowImpl&& other) = delete;
    PangolinWindowImpl& operator=(PangolinWindowImpl&& other) = delete;
    // 初始化, 创建各种点云,小车实体
    bool Init();
    // 注销
    bool DeInit();
    // 渲染ui
    void Render();

   public:
    std::thread render_thread_;

    std::atomic<bool> exit_flag_;

    // 锁
    std::mutex mtx_nav_state_;
    std::mutex mtx_map_cloud_;
    std::mutex mtx_current_scan_;
    std::mutex mtx_gps_pose_;

    // 状态
    std::atomic<bool> kf_result_need_update_;
    std::atomic<bool> cloud_global_need_update_;
    std::atomic<bool> current_scan_need_update_;
    std::atomic<bool> lidarloc_need_update_;
    std::atomic<bool> gps_need_update_;
    std::atomic<bool> pgoloc_need_update_;

    CloudPtr current_scan_ = nullptr;  // 当前scan
    SE3 current_pose_;                 // 当前scan对应的pose

    // 地图点云
    std::map<Vec2i, CloudPtr, less_vec<2>> cloud_global_map_;

    // gps pose
    SE3 gps_pose_;

    // 滤波器状态
    SE3 pose_;
    Vec3d vel_;
    Vec3d bias_acc_;
    Vec3d bias_gyr_;
    Vec3d gray_;

    SE3 T_imu_lidar_;
    int max_size_of_current_scan_ = 2000;  // 当前扫描数据保留多少个

    // render相关
   private:
    void AllocateBuffer();
    void ReleaseBuffer();
    void CreateDisplayLayout();

    void DrawAll();

    // 渲染
    void RenderLabels();
    void RenderClouds();

    bool UpdateGps();
    bool UpdateGlobalMap();
    bool UpdateCurrentScan();
    bool UpdateState();

   private:
    // 窗口layout
    int win_width_ = 1920;
    int win_height_ = 1080;
    static constexpr float cam_focus_ = 5000;
    static constexpr float cam_z_near_ = 1.0;
    static constexpr float cam_z_far_ = 1e10;
    static constexpr int menu_width_ = 200;

    const std::string win_name_ = "LH.UI";
    const std::string dis_main_name_ = "main";
    const std::string dis_3d_name_ = "Cam 3D";
    const std::string dis_3d_main_name_ = "Cam 3D Main";
    const std::string dis_plot_name_ = "Plot";
    const std::string dis_imgs_name_ = "Images";

    bool following_loc_ = true;  // 相机跟随定位结果

    // text
    pangolin::GlText gltext_label_global_;
    // camera
    pangolin::OpenGlRenderState s_cam_main_;
    // cloud rendering
    ui::UiCar car_{Vec3f(0.2, 0.2, 0.2)};                                  // 白色车
    std::map<Vec2i, std::shared_ptr<UiCloud>, less_vec<2>> cloud_map_ui_;  // 用来渲染的点云地图
    std::shared_ptr<ui::UiCloud> current_scan_ui_;                         // 当前scan
    std::deque<std::shared_ptr<UiCloud>> scans_;                           // 点云队列

    // 绘制轨迹
    SE3 T_map_odom_for_lio_traj_ui_;      // 显示lio的轨迹
    SE3 T_map_baselink_for_lio_traj_ui_;  // 显示lio的轨迹

    // trajectory
    std::shared_ptr<ui::UiTrajectory> traj_lidarloc_ui_ = nullptr;
    std::shared_ptr<ui::UiTrajectory> traj_gps_ui_ = nullptr;

    // data logger
    pangolin::DataLog log_vel_;
    pangolin::DataLog log_vel_baselink_;
    pangolin::DataLog log_bias_acc_;
    pangolin::DataLog log_bias_gyr_;

    std::unique_ptr<pangolin::Plotter> plotter_vel_ = nullptr;
    std::unique_ptr<pangolin::Plotter> plotter_vel_baselink_ = nullptr;
    std::unique_ptr<pangolin::Plotter> plotter_bias_acc_ = nullptr;
    std::unique_ptr<pangolin::Plotter> plotter_bias_gyr_ = nullptr;
};
}  // namespace lh::ui

#endif