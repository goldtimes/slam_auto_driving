#include "tools/ui/pangolin_window.h"
#include <glog/logging.h>
#include "tools/ui/pangolin_window_impl.h"

namespace lh::ui {
PangolinWindow::PangolinWindow() { impl_ = std::make_shared<PangolinWindowImpl>(); }

PangolinWindow::~PangolinWindow() {
    LOG(INFO) << "pangolin window is deallocated.";
    Quit();
}

bool PangolinWindow::Init() {
    impl_->kf_result_need_update_.store(false);
    bool inited = impl_->Init();
    if (inited) {
        impl_->render_thread_ = std::thread([this]() { impl_->Render(); });
    }
    return inited;
}

void PangolinWindow::Quit() {
    if (impl_->render_thread_.joinable()) {
        impl_->exit_flag_.store(true);
        impl_->render_thread_.join();
    }
    impl_->DeInit();
}

void PangolinWindow::UpdateNavState(const NavStated& state) {
    // 锁
    std::unique_lock<std::mutex> lock_lio_res(impl_->mtx_nav_state_);

    impl_->pose_ = SE3(state.R_, state.p_);
    impl_->vel_ = state.v_;
    impl_->bias_acc_ = state.ba_;
    impl_->bias_gyr_ = state.bg_;

    impl_->kf_result_need_update_.store(true);
}

bool PangolinWindow::ShouldQuit() { return pangolin::ShouldQuit(); }

}  // namespace lh::ui