#ifndef UI_TRAJECTORY_H
#define UI_TRAJECTORY_H

#include <pangolin/gl/glvbo.h>
#include "common/eigen_sophus.h"

namespace lh::ui {
class UiTrajectory {
   public:
    UiTrajectory(const Vec3f& color) : color_(color) { pos_.reserve(max_size); }

    void AddPose(const SE3& pose);
    // 渲染轨迹
    void Render();
    void Clear() {
        pos_.clear();
        pos_.reserve(max_size);
        vbo_.Free();
    }

   private:
    Vec3f color_ = Vec3f::Zero();
    int max_size = 1e6;       // 记录的最大点数
    std::vector<Vec3f> pos_;  // 轨迹
    pangolin::GlBuffer vbo_;  // 显存顶点信息
};
}  // namespace lh::ui

#endif