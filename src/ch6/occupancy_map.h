#ifndef SLAM_IN_AUTO_DRIVING_OCCUPANCY_MAP_H
#define SLAM_IN_AUTO_DRIVING_OCCUPANCY_MAP_H
#include <opencv2/core.hpp>
#include "frame.h"

namespace lh {
class OccupancyMap {
   public:
    struct Model2DPoint {
        int dx_ = 0;
        int dy_ = 0;
        double angle_ = 0;
        float range_ = 0;
    };

    enum class GridMethod {
        MODEL_POINTS,  // 模板化算法
        BRESENHAM,     // 直接栅格化
    };

    OccupancyMap();
    /**
     * @brief 往占据栅格地图中增加一个frame
     */
    void AddLidarFrame(std::shared_ptr<Frame> frame, GridMethod method = GridMethod::BRESENHAM);
    /**
     * @brief 获取原始的占据栅格地图
     */
    cv::Mat GetOccupancyGrid() const { return occpuancy_grid_; }

    /**
     * @brief 黑白灰的占据栅格，可视化
     */
    cv::Mat GetOccupancyGridBlackWhite() const;

    /**
     * @brief 设置位姿 中心点
     */
    void SetPose(const SE2 pose) { pose_ = pose; }

    bool HasOutSidePoints() const { return has_outside_pts_; }

    double GetResolution() const { return resolution_; }
    /**
     *  @brief 在某个点填入占据或者非占据信息
     */
    void SetPoint(const Vec2i& pt, bool occupy);

   private:
    void BuildModel();
    /**
     * @brief 世界坐标系->图像坐标系 模板函数
     * p_i^map = T_mtow.inverse() * p_i^W
     */
    template <class T>
    inline Vec2i world2Image(const Eigen::Matrix<T, 2, 1>& pt) {
        Vec2d pt_map = (pose_.inverse() * pt) * resolution_ + center_image_;
        int x = int(pt_map[0]);
        int y = int(pt_map[1]);
        return Vec2i(x, y);
    }

    /**
     * @brief 查找某个角度下的range值 线性插值
     */
    double FindRangeInAngle(double angle, Scan2d::Ptr scan);

    /**
     *  直线填充，给定起始点和终止点，中间区域填充为白色
     */
    void BresenhamFilling(const Vec2i& pi, const Vec2i& p2);

   private:
    cv::Mat occpuancy_grid_;
    SE2 pose_;  // T scan to world
    Vec2d center_image_ = Vec2d(image_size_ / 2, image_size_ / 2);

    bool has_outside_pts_ = false;
    // 模板生成栅格地图
    std::vector<Model2DPoint> model_;

    inline static constexpr double closest_th_ = 0.2;         // 近距离阈值
    inline static constexpr double endpoint_close_th_ = 0.1;  // 末端点近距离阈值
    inline static constexpr int image_size_ = 1000;           // 图像大小
    inline static constexpr double resolution_ = 20.0;        // 1 / 0.05
    inline static constexpr double inv_resolution_ = 0.05;    // 分辨率
    inline static constexpr int model_size_ = 400;            // 模板像素大小
};
}  // namespace lh
#endif