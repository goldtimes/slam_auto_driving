#include "lidar_2d_utils.h"
#include <opencv2/imgproc.hpp>

namespace lh {
void Visualize2DScan(Scan2d::Ptr scan, const SE2& pose, cv::Mat& image, const Vec3b& color, int image_size, float resolution,
                     const SE2& pose_submap) {
    if (image.data == nullptr) {
        image = cv::Mat(image_size, image_size, CV_8UC3, cv::Vec3b(255, 255, 255));
    }
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        // 超出2d测量范围的点
        if (scan->ranges[i] < scan->range_min || scan->ranges[i] > scan->range_max) {
            continue;
        }
        double real_angle = scan->angle_min + i * scan->angle_increment;
        double x = scan->ranges[i] * std::cos(real_angle);
        double y = scan->ranges[i] * std::sin(real_angle);
        // 过滤掉不合法的角度
        if (real_angle < scan->angle_min + 30 * M_PI / 180.0 || real_angle > scan->angle_max - 30 * M_PI / 180.0) {
            continue;
        }
        // pose_submap 和 pose 是单位矩阵
        // 当前submap的坐标.inverse() * 机器人的坐标pose * Vec2d(x, y) 激光点的坐标
        Vec2d psubmap = pose_submap.inverse() * (pose * Vec2d(x, y));

        int image_x = int(psubmap[0] * resolution + image_size / 2);
        int image_y = int(psubmap[1] * resolution + image_size / 2);
        if (image_x >= 0 && image_x < image.cols && image_y >= 0 && image_y < image.rows) {
            image.at<cv::Vec3b>(image_y, image_x) = cv::Vec3b(color[0], color[1], color[2]);
        }
    }
    // 画出pose自身所在位置
    // 机器人在submap的坐标 * 分辨率 + 图像中心
    // std::cout << "trans: " << pose.translation() << std::endl;
    Vec2d pose_in_image = pose_submap.inverse() * (pose.translation()) * double(resolution) + Vec2d(image_size / 2, image_size / 2);
    cv::circle(image, cv::Point2f(pose_in_image[0], pose_in_image[1]), 5, cv::Scalar(color[0], color[1], color[2]), 2);
}

}  // namespace lh