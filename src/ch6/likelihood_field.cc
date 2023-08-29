#include "g2o_types.h"

#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>
#include <glog/logging.h>
#include "likelihood_field.h"

#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

namespace lh {
void LikelihoodField::SetTargetScan(Scan2d::Ptr scan) {
    target_ = scan;
    // target点上生成场函数
    // 指定了图像的大小 1000px * 1000px
    field_ = cv::Mat(1000, 1000, CV_32F, 30.0);
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double r = scan->ranges[i];
        if (r > scan->range_max || r < scan->range_min) {
            continue;
        }

        double real_angle = scan->angle_min + i * scan->angle_increment;
        // 500 为图像尺寸的一半
        double x = r * std::cos(real_angle) * resolution_ + 500;
        double y = r * std::sin(real_angle) * resolution_ + 500;
        // model_ 40*40像素大小，以点云的点为中心
        for (auto& model_pt : model_) {
            int xx = int(x + model_pt.dx_);
            int yy = int(y + model_pt.dy_);
            // 确保不会搜索到图像尺寸外面
            if (xx >= 0 && xx < field_.cols && yy >= 0 && yy < field_.rows && field_.at<float>(yy, xx) > model_pt.residual_) {
                field_.at<float>(yy, xx) = model_pt.residual_;
            }
        }
    }
}
// 在点云的点附近附近
void LikelihoodField::BuildModel() {
    const int range = 20;  // 生成多少个像素的模板
    for (int x = -range; x <= range; ++x) {
        for (int y = -range; y <= range; ++y) {
            model_.emplace_back(x, y, std::sqrt((x * x) + (y * y)));
        }
    }
}
void LikelihoodField::SetSourceScan(Scan2d::Ptr scan) { source_ = scan; }

bool LikelihoodField::AlignGaussNewton(SE2& init_pose) {
    int iterations = 10;
    double cost = 0;
    double last_cost = 0;
    SE2 current_pose = init_pose;
    const int min_effect_pts = 20;
    const int image_boarder = 20;

    has_outside_pts_ = false;
    for (int iter = 0; iter < iterations; ++iter) {
        Mat3d H = Mat3d::Zero();
        Vec3d b = Vec3d::Zero();
        cost = 0;

        int effective_num = 0;
        // 遍历source
        for (size_t i = 0; i < source_->ranges.size(); ++i) {
            float r = source_->ranges[i];
            if (r < source_->range_min || r > source_->range_max) {
                continue;
            }

            float angle = source_->angle_min + i * source_->angle_increment;
            if (angle < source_->angle_min + 30 * M_PI / 180.0 || angle > source_->angle_max - 30 * M_PI / 180.0) {
                continue;
            }

            float theta = current_pose.so2().log();
            // p_i^w = T_BtoW * p_i^B
            Vec2d pw = current_pose * Vec2d(r * std::cos(angle), r * std::sin(angle));
            // 在field中的图像坐标
            Vec2i pf = (pw * resolution_ + Vec2d(500, 500)).cast<int>();

            if (pf[0] >= image_boarder && pf[0] < field_.cols - image_boarder && pf[1] >= image_boarder && pf[1] < field_.rows - image_boarder) {
                effective_num++;

                // 图像梯度
                float dx = 0.5 * (field_.at<float>(pf[1], pf[0] + 1) - field_.at<float>(pf[1], pf[0] - 1));
                float dy = 0.5 * (field_.at<float>(pf[1] + 1, pf[0]) - field_.at<float>(pf[1] - 1, pf[0]));
                Vec3d J;
                J << resolution_ * dx, resolution_ * dy,
                    -resolution_ * dx * r * std::sin(angle + theta) + resolution_ * dy * r * std::cos(angle + theta);
                H += J * J.transpose();

                float e = field_.at<float>(pf[1], pf[0]);
                b += -J * e;
                cost += e * e;
            } else {
                has_outside_pts_ = true;
            }
        }
        if (effective_num < min_effect_pts) {
            return false;
        }

        Vec3d dx = H.ldlt().solve(b);
        if (isnan(dx[0])) {
            break;
        }

        cost /= effective_num;
        if (iter > 0 && cost >= last_cost) {
            break;
        }
        LOG(INFO) << "iter " << iter << " cost = " << cost << ", effect num: " << effective_num;

        current_pose.translation() += dx.head<2>();
        current_pose.so2() = current_pose.so2() * SO2::exp(dx[2]);
        last_cost = cost;
    }
    init_pose = current_pose;
    return true;
}

bool LikelihoodField::AlignG2O(SE2& init_pose) {
    using BlockSloverType = g2o::BlockSolver<g2o::BlockSolverTraits<3, 1>>;
    using LinearSloverType = g2o::LinearSolverCholmod<BlockSloverType::PoseMatrixType>;
    auto* solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<BlockSloverType>(g2o::make_unique<LinearSloverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    auto* v = new VertexSE2();
    v->setId(0);
    v->setEstimate(init_pose);
    optimizer.addVertex(v);
    // 不考虑太远的scan
    const double range_th = 15.0;
    const double rk_delta = 0.8;

    has_outside_pts_ = false;
    // 遍历source
    for (size_t i = 0; i < source_->ranges.size(); ++i) {
        float r = source_->ranges[i];
        if (r < source_->range_min || r > source_->range_max) {
            continue;
        }
        if (r > range_th) {
            continue;
        }

        float angle = source_->angle_min + i * source_->angle_increment;
        if (angle < source_->angle_min + 30 * M_PI / 180.0 || angle > source_->angle_max - 30 * M_PI / 180.0) {
            continue;
        }

        auto edge = new EdgeSE2LikelihoodField(field_, r, angle, resolution_);
        edge->setVertex(0, v);

        if (edge->isOutSide) {
            has_outside_pts_ = true;
            delete edge;
            continue;
        }

        edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
        auto rk = new g2o::RobustKernelHuber;
        rk->setDelta(rk_delta);
        edge->setRobustKernel(rk);
        optimizer.addEdge(edge);
    }
    optimizer.setVerbose(false);
    optimizer.initializeOptimization();
    optimizer.optimize(10);

    init_pose = v->estimate();
    return true;
}

cv::Mat LikelihoodField::GetFieldImage() {
    cv::Mat image(field_.rows, field_.cols, CV_8UC3);
    for (int x = 0; x < field_.cols; ++x)
        for (int y = 0; y < field_.rows; ++y) {
            float r = field_.at<float>(y, x) * 255.0 / 30.0;
            image.at<cv::Vec3b>(y, x) = cv::Vec3b(uchar(r), uchar(r), uchar(r));
        }
    return image;
}

void LikelihoodField::SetFieldImageFromOccuMap(const cv::Mat& occu_map) {}
}  // namespace lh