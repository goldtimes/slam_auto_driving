#pragma once

#include <opencv2/core.hpp>
#include "common/eigen_sophus.h"
#include "common/lidar_utils.h"
#include "g2o_types.h"

namespace lh {
class LikelihoodField {
   public:
    // 2D 场的模板，在设置target_scan 或者 map时候生成
    struct ModelPoint {
        ModelPoint(int dx, int dy, float res) : dx_(dx), dy_(dy), residual_(res) {}
        int dx_ = 0;
        int dy_ = 0;
        float residual_ = 0;
    };

    LikelihoodField() { BuildModel(); }

    /**
     * @brief 增加一个2d的目标scan
     */
    void SetTargetScan(Scan2d::Ptr scan);

    /**
     * @brief 设置被配准的scan
     */
    void SetSourceScan(Scan2d::Ptr scan);
    /**
     * @brief 占据栅格地图生成似然场
     */
    void SetFieldImageFromOccuMap(const cv::Mat& occu_map);
    /**
     * 使用高斯牛顿配准法
     */
    bool AlignGaussNewton(SE2& init_pose);

    /**
     * @brief g2o配准
     */
    bool AlignG2O(SE2& init_pose);
    /**
     * 获取场函数，转换为RGB图像
     */
    cv::Mat GetFieldImage();

    bool HasOutsidePoints() const { return has_outside_pts_; }

    void SetPose(const SE2& pose) { pose_ = pose; }

   private:
    void BuildModel();
    SE2 pose_;  // T_w_s  submap in world
    Scan2d::Ptr target_ = nullptr;
    Scan2d::Ptr source_ = nullptr;

    std::vector<ModelPoint> model_;  // 2D 模板
    cv::Mat field_;
    bool has_outside_pts_ = false;
    inline static const float resolution_ = 20;  // 每米多少个像素 1/0.05
};
}  // namespace lh