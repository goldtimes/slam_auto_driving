#ifndef UI_PANGOLIN_WINDOW_H
#define UI_PANGOLIN_WINDOW_H

#include <eigen3/Eigen/Dense>
#include <map>
#include <memory>

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

   private:
    // ...
    std::shared_ptr<PangolinWindowImpl> impl_ = nullptr;
};

}  // namespace lh::ui

#endif