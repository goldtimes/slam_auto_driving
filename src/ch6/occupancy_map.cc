#include "occupancy_map.h"
#include "common/eigen_sophus.h"
#include "common/math_utils.h"

#include <glog/logging.h>
#include <execution>

namespace lh {
OccupancyMap::OccupancyMap() {
    BuildModel();
    occpuancy_grid_ = cv::Mat(image_size_, image_size_, CV_8U, 127);
}

void OccupancyMap::BuildModel() {
    for (int x = -model_size_; x < model_size_; ++x) {
        for (int y = -model_size_; y < model_size_; ++y) {
            Model2DPoint pt;
            pt.dx_ = x;
            pt.dy_ = y;
            // 距离 * 0.05?
            pt.range_ = sqrt(x * x + y * y) * inv_resolution_;
            pt.angle_ = std::atan2(y, x);
            // 限制在2pi 内
            pt.angle_ = pt.angle_ > M_PI ? pt.angle_ - 2 * M_PI : pt.angle_;
            model_.push_back(pt);
        }
    }
}

double OccupancyMap::FindRangeInAngle(double angle, Scan2d::Ptr scan) {
    math::KeepAngleInPI(angle);
    if (angle < scan->angle_min || angle > scan->angle_max) {
        return 0.0;
    }
    int angle_index = int((angle - scan->angle_min) / scan->angle_increment);
    if (angle_index < 0 || angle_index >= scan->ranges.size()) {
        return 0.0;
    }
    int angle_index_p = angle_index + 1;
    double real_angle = angle;

    double range = 0;
    if (angle_index_p >= scan->ranges.size()) {
        range = scan->ranges[angle_index];
    } else {
        // 线性插值
        double ratio = ((angle - scan->angle_min) / scan->angle_increment) - angle_index;
        double range1 = scan->ranges[angle_index];
        double range2 = scan->ranges[angle_index_p];

        double real_angle1 = scan->angle_min + scan->angle_increment * angle_index;
        double real_angle2 = scan->angle_min + scan->angle_increment * angle_index_p;

        if (range2 < scan->range_min || range2 > scan->range_max) {
            range = range1;
            real_angle = real_angle1;
        } else if (range1 < scan->range_min || range1 > scan->range_max) {
            range = range2;
            real_angle = real_angle2;
        } else {
            range = range1 * (1 - ratio) + range2 * ratio;
        }
    }
    return range;
}
void OccupancyMap::AddLidarFrame(std::shared_ptr<Frame> frame, GridMethod method = GridMethod::BRESENHAM) {
    auto& scan = frame->scan_;
    // 不能直接使用frame->pose_, 因为frame可能来自上一个地图
    // 此时的frame->pose_未更新，依旧是frame上一个地图中的pose
    SE2 pose_in_submap = pose_.inverse() * frame->pose_;  // 我这里理解的是增量？
    float theta = pose_in_submap.so2().log();

    has_outside_pts_ = false;
    // 计算末端点所在的网格
    std::set<Vec2i, less_vec<2>> endpoints;
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        if (scan->ranges[i] < scan->range_min || scan->ranges[i] > scan->range_max) {
            continue;
        }

        double real_angle = scan->angle_min + i * scan->angle_increment;
        double x = scan->ranges[i] * std::cos(real_angle);
        double y = scan->ranges[i] * std::sin(real_angle);
        // 先获得世界坐标系下的点 p_w = T_m_w * p_m, 在将p_w转到图像上的像素点
        endpoints.emplace(world2Image(frame->pose_ * Vec2d(x, y)));
    }

    if (method == GridMethod::MODEL_POINTS) {
        // 遍历模板，生成白色点
        std::for_each(std::execution::par_unseq, model_.begin(), model_.end(), [&](const Model2DPoint& pt) {
            Vec2i pose_in_image = world2Image(frame->pose_.translation());
            Vec2i pw = pose_in_image + Vec2i(pt.dx_, pt.dy_);  // submap下
            // < 0.2
            if (pt.range_ < closest_th_) {
                // 小距离认为无物体
                SetPoint(pw, false);
                return;
            }

            double angle = pt.angle_ - theta;  // 激光系下的角度
            double range = this->FindRangeInAngle(angle, scan);
            if (range < scan->range_min || range > scan->range_max) {
                // 无测量值
                // 但是离机器人比较近
                if (pt.range_ < endpoint_close_th_) {
                    SetPoint(pw, false);
                }
                return;
            }

            if (range > pt.range_ && endpoints.find(pw) == endpoints.end()) {
                SetPoint(pw, false);
            }
        });
    } else {
        Vec2i start = world2Image(frame->pose_.translation());
        std::for_each(std::execution::par_unseq, endpoints.begin(), endpoints.end(), [this, &start](const auto& pt) { BresenhamFilling(start, pt); });
    }
}

void OccupancyMap::SetPoint(const Vec2i& pt, bool occupy) {
    // 检查点是否超过的图像大小
    int x = pt[0], y = pt[1];

    if (x < 0 || y << 0 || x >= occpuancy_grid_.cols || y >= occpuancy_grid_.rows) {
        if (occupy) {
            has_outside_pts_ = true;
        }
        return;
    }
    // 下标合法
    uchar value = occpuancy_grid_.at<uchar>(y, x);
    // 将颜色限制在117-137  0-255是从黑到白
    if (occupy) {
        if (value > 117) {
            occpuancy_grid_.ptr<uchar>(y)[x] -= 1;
        }
    } else {
        if (value < 137) {
            occpuancy_grid_.ptr<uchar>(y)[x] += 1;
        }
    }
}

cv::Mat OccupancyMap::GetOccupancyGridBlackWhite() const {
    cv::Mat image(image_size_, image_size_, CV_8UC3);
    for (int x = 0; x < occpuancy_grid_.cols; ++x) {
        for (int y = 0; y < occpuancy_grid_.rows; ++y) {
            if (occpuancy_grid_.at<uchar>(y, x) == 127) {
                // 灰色
                image.at<cv::Vec3b>(y, x) = cv::Vec3b(127, 127, 127);
            } else if (occpuancy_grid_.at<uchar>(y, x) < 127) {
                // 黑色
                image.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0);
            } else if (occpuancy_grid_.at<uchar>(y, x) > 127) {
                // 白色
                image.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 255, 255);
            }
        }
    }
    return image;
}

void OccupancyMap::BresenhamFilling(const Vec2i& p1, const Vec2i& p2) {
    int dx = p2.x() - p1.x();
    int dy = p2.y() - p1.y();
    int ux = dx > 0 ? 1 : -1;
    int uy = dy > 0 ? 1 : -1;
    dx = abs(dx);
    dy = abs(dy);

    int x = p1.x();
    int y = p1.y();

    if (dx > dy) {
        // 以x为增量
        int e = -dx;
        for (int i = 0; i < dx; ++i) {
            x += ux;
            e += 2 * dy;
            if (e >= 0) {
                y += uy;
                e -= 2 * dx;
            }

            if (Vec2i(x, y) != p2) {
                SetPoint(Vec2i(x, y), false);
            }
        }
    } else {
        int e = -dy;
        for (int i = 0; i < dy; ++i) {
            y += uy;
            e += 2 * dx;
            if (e >= 0) {
                x += ux;
                e -= 2 * dy;
            }
            if (Vec2i(x, y) != p2) {
                SetPoint(Vec2i(x, y), false);
            }
        }
    }
}

}  // namespace lh