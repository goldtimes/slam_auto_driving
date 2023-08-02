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

bool PangolinWindow::ShouldQuit() { return pangolin::ShouldQuit(); }

}  // namespace lh::ui