#ifndef SLAM_IN_AUTO_DRIVING_G2O_TYPES_H
#define SLAM_IN_AUTO_DRIVING_G2O_TYPES_H

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>

#include <glog/logging.h>
#include <opencv2/core.hpp>
#include "common/eigen_sophus.h"
#include "common/math_utils.h"

namespace lh {
class VertexSE2 : public g2o::BaseVertex<3, SE2> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    void setToOriginImpl() override { _estimate = SE2(); }
    void oplusImpl(const double* update) override {
        _estimate.translation()[0] += update[0];
        _estimate.translation()[1] += update[1];
        _estimate.so2() = _estimate.so2() * SO2::exp(update[2]);
    }

    bool read(std::istream& is) override { return true; }
    bool write(std::ostream& os) const override { return false; }
};

class EdgeSE2LikelihoodField : public g2o::BaseUnaryEdge<1, double, VertexSE2> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeSE2LikelihoodField(const cv::Mat& field_image, double range, double angle, float resolution = 10.0)
        : field_image_(field_image), range_(range), angle_(angle), resolution_(resolution) {}
    bool read(std::istream& is) override { return true; }
    bool write(std::ostream& os) const override { return true; }

    /**
     * @brief 判断边是否在field_image外面
     */
    bool IsOutSide() {
        VertexSE2* v = (VertexSE2*)_vertices[0];
        SE2 pose = v->estimate();
        Vec2d pw = pose * Vec2d(range_ * std::cos(angle_), range_ * std::sin(angle_));
        Vec2i pf = (pw * resolution_ + Vec2d(field_image_.rows / 2, field_image_.cols / 2)).cast<int>();

        if (pf[0] >= image_boarder_ && pf[0] < field_image_.cols - image_boarder_ && pf[1] >= image_boarder_ &&
            pf[1] < field_image_.rows - image_boarder_) {
            return false;
        } else {
            return true;
        }
    }

    void computeError() override {
        VertexSE2* v = (VertexSE2*)_vertices[0];
        SE2 pose = v->estimate();
        Vec2d pw = pose * Vec2d(range_ * std::cos(angle_), range_ * std::sin(angle_));
        Vec2i pf = (pw * resolution_ + Vec2d(field_image_.rows / 2, field_image_.cols / 2)).cast<int>();
        if (pf[0] >= image_boarder_ && pf[0] < field_image_.cols - image_boarder_ && pf[1] >= image_boarder_ &&
            pf[1] < field_image_.rows - image_boarder_) {
            _error[0] = math::GetPixelValue<float>(field_image_, pf[0], pf[1]);
        } else {
            _error[0] = 0;
            setLevel(1);
        }
    }

    void linearizeOplus() override {
        VertexSE2* v = (VertexSE2*)_vertices[0];
        SE2 pose = v->estimate();
        float theta = pose.so2().log();
        Vec2d pw = pose * Vec2d(range_ * std::cos(angle_), range_ * std::sin(angle_));
        Vec2i pf = (pw * resolution_ + Vec2d(field_image_.rows / 2, field_image_.cols / 2)).cast<int>();

        if (pf[0] >= image_boarder_ && pf[0] < field_image_.cols - image_boarder_ && pf[1] >= image_boarder_ &&
            pf[1] < field_image_.rows - image_boarder_) {
            // 图像梯度
            // x方向的梯度 pf[0] + 1, pf[0] - 1;
            float dx =
                0.5 * (math::GetPixelValue<float>(field_image_, pf[0] + 1, pf[1]) - math::GetPixelValue<float>(field_image_, pf[0] - 1, pf[1]));
            float dy =
                0.5 * (math::GetPixelValue<float>(field_image_, pf[0], pf[1] + 1) - math::GetPixelValue<float>(field_image_, pf[0], pf[1] - 1));

            // 雅可比矩阵
            _jacobianOplusXi << resolution_ * dx, resolution_ * dy,
                -resolution_ * dx * range_ * std::sin(angle_ + theta) + resolution_ * dy * range_ * std::cos(angle_ + theta);
        } else {
            _jacobianOplusXi.setZero();
            setLevel(1);
        }
    }

   private:
    const cv::Mat& field_image_;
    double range_ = 0;
    double angle_ = 0;
    float resolution_ = 10.0;
    inline static const int image_boarder_ = 10;
};

/**
 * SE2 pose graph
 * error = v1.inv * v2 * measurement.inv
 */
class EdgeSE2 : public g2o::BaseBinaryEdge<3, SE2, VertexSE2, VertexSE2> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeSE2() {}

    void computeError() override {
        VertexSE2* v1 = (VertexSE2*)_vertices[0];
        VertexSE2* v2 = (VertexSE2*)_vertices[1];
        _error = (v1->estimate().inverse() * v2->estimate() * measurement().inverse()).log();
    }

    // TODO jacobian

    bool read(std::istream& is) override { return true; }
    bool write(std::ostream& os) const override { return true; }
};
}  // namespace lh

#endif