#ifndef SLAM_IN_AUTO_DRIVING_MAPPING_2D_H
#define SLAM_IN_AUTO_DRIVING_MAPPING_2D_H

#include "ch6/frame.h"
#include "common/eigen_sophus.h"
#include "common/lidar_utils.h"

#include <memory>
#include <opencv2/core.hpp>

namespace lh {
class Submap;

class Mapping2D {
   public:
    bool Init(bool with_loop_closing = false);

    bool ProcessScan(Scan2d::Ptr scan);

    /**
     * @brief 显示全局地图
     * @param max_size 全局地图的大小
     * @return 全局地图图像
     */
    cv::Mat ShowGlobalMap(int max_size = 500);

   private:
    bool IsKeyFrame();
    void AddKeyFrame();
    void ExpandSubmap();

   private:
    size_t frame_id_ = 0;
    size_t submap_id_ = 0;
    size_t keyframe_id_ = 0;

    bool first_scan_ = true;
    std::shared_ptr<Frame> current_frame_ = nullptr;
    std::shared_ptr<Frame> last_frame_ = nullptr;
    SE2 motion_guess_;
    std::shared_ptr<Frame> last_keyframe_ = nullptr;
    std::shared_ptr<Submap> current_submap_ = nullptr;

    std::vector<std::shared_ptr<Submap>> all_submaps_;

    inline static constexpr double keyframe_pos_th_ = 0.3;              // 帧位移量
    inline static constexpr double keyframe_ang_th_ = 15 * M_PI / 180;  // 帧角度量
};
}  // namespace lh

#endif