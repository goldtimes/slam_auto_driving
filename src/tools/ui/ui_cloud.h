#ifndef UI_CLOUD_H
#define UI_CLOUD_H

#include "common/eigen_sophus.h"
#include "common/point_types.h"

#include <pangolin/gl/glvbo.h>

namespace lh {
/**
 * @brief UI中使用的点云
 * 固定不变的点云可以用这个来渲染
 */
class UiCloud {
   public:
    enum UseColor {
        PCL_COLOR,
        INTENSITY_COLOR,
        HEIGHT_COLOR,
        GRAY_COLOR,
    };

    UiCloud() {}
    UiCloud(CloudPtr cloud);

    /**
     * @brief 设置点云，和点云位姿，将点云转换到全局坐标系
     */
    void SetCloud(CloudPtr, const SE3& pose);

    void Render();
    void SetRenderColor(UseColor use_color);
    // 颜色相关
   private:
    Vec4f IntensityToRgbPCL(const float& intensity) const {
        int index = int(intensity * 6);
        index = index % intensity_color_table_pcl_.size();
        return intensity_color_table_pcl_[index];
    }

    UseColor use_color_ = UseColor::PCL_COLOR;

    std::vector<Vec3f> xyz_data_;              // XYZ buffer
    std::vector<Vec4f> color_data_pcl_;        // color buffer
    std::vector<Vec4f> color_data_intensity_;  // color buffer
    std::vector<Vec4f> color_data_height_;     // color buffer
    std::vector<Vec4f> color_data_gray_;       // color buffer

    pangolin::GlBuffer vbo_;  // 显存顶点信息
    pangolin::GlBuffer cbo_;  // 颜色顶点信息

    /// PCL中intensity table
    void BuildIntensityTable();
    static std::vector<Vec4f> intensity_color_table_pcl_;
};
}  // namespace lh

#endif