#ifndef FUSION_UI_PANGOLIN_WINDOW_IMPL_H
#define FUSION_UI_PANGOLIN_WINDOW_IMPL_H

#include <pangolin/pangolin.h>
#include <atomic>
#include <eigen3/Eigen/Dense>
#include <mutex>
#include <sophus/se3.hpp>
#include <string>
#include <thread>
#include "common/eigen_sophus.h"
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
    // 初始化
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

    // 状态
    std::atomic<bool> kf_result_need_update_;

    // 滤波器状态
    SE3 pose_;
    Vec3d vel_;
    Vec3d bias_acc_;
    Vec3d bias_gyr_;
    Vec3d gray_;

    SE3 current_pose_;

    // render相关
   private:
    void AllocateBuffer();
    void ReleaseBuffer();
    void CreateDisplayLayout();

    void DrawAll();

    // 渲染
    void RenderLabels();
    void RenderClouds();

    bool UpdateState();

   private:
    // 窗口layout
    int win_width_ = 1920;
    int win_height_ = 1080;
    static constexpr float cam_focus_ = 500;
    static constexpr float cam_z_near_ = 1.0;
    static constexpr float cam_z_far_ = 1e10;
    static constexpr int menu_width_ = 200;

    const std::string win_name_ = "LH.UI";
    const std::string dis_main_name_ = "main";
    const std::string dis_3d_name_ = "Cam 3D";
    const std::string dis_3d_main_name_ = "Cam 3D Main";
    const std::string dis_plot_name_ = "Plot";
    const std::string dis_imgs_name_ = "Images";

    // text
    pangolin::GlText gltext_label_global_;

    // data logger
    pangolin::DataLog log_vel_;
    pangolin::DataLog log_vel_baselink_;
    pangolin::DataLog log_bias_acc_;
    pangolin::DataLog log_bias_gyr_;

    std::unique_ptr<pangolin::Plotter> plotter_vel_ = nullptr;
    std::unique_ptr<pangolin::Plotter> plotter_vel_baselink_ = nullptr;
    std::unique_ptr<pangolin::Plotter> plotter_bias_acc_ = nullptr;
    std::unique_ptr<pangolin::Plotter> plotter_bias_gyr_ = nullptr;
    // 轨迹
    std::shared_ptr<ui::UiTrajectory> traj_lidarloc_ui_ = nullptr;
};
}  // namespace lh::ui

#endif