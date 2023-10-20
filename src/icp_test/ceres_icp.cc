#include "ceres_icp.h"
#include "lidarFactor.hpp"
#include "lidarOptimization/lidarCeres.h"
#include "pose_factor.h"
inline Eigen::Vector3d ToVec3d(const PointType &pt) { return pt.getVector3fMap().cast<double>(); }

template <typename S>
bool FitPlane(std::vector<Eigen::Matrix<S, 3, 1>> &data, Eigen::Matrix<S, 4, 1> &plane_coeffs, double eps = 1e-2) {
    if (data.size() < 3)
        return false;
    // 每个点都有4个未知参数
    Eigen::MatrixXd A(data.size(), 4);
    for (int i = 0; i < data.size(); i++) {
        A.row(i).head<3>() = data[i].transpose();
        A.row(i)[3] = 1.0;
    }
    Eigen::JacobiSVD svd(A, Eigen::ComputeThinV);
    plane_coeffs = svd.matrixV().col(3);
    // 平面上的点到平面的距离
    for (int i = 0; i < data.size(); i++) {
        double err = plane_coeffs.template head<3>().dot(data[i]) + plane_coeffs[3];
        if (err * err > eps) {
            return false;
        }
    }
    return true;
}

CeresIcp::CeresIcp(const YAML::Node &node) : kdtree_flann(new pcl::KdTreeFLANN<PointType>) {
    max_coresspoind_dis = node["max_corr_dist"].as<double>();
    trans_eps = node["trans_eps"].as<double>();
    euc_fitness_eps = node["euc_fitness_eps"].as<double>();
    max_iterations = node["max_iter"].as<int>();

    std::cout << "ICP params:" << std::endl
              << "max_corr_dist: " << max_coresspoind_dis << ", "
              << "trans_eps: " << trans_eps << ", "
              << "euc_fitness_eps: " << euc_fitness_eps << ", "
              << "max_iter: " << max_iterations << std::endl
              << std::endl;
}
CeresIcp::~CeresIcp() {}

bool CeresIcp::setTargetCloud(const CloudPtr &input_target) {
    target_ptr = input_target;
    kdtree_flann->setInputCloud(input_target);
    return true;
}

bool CeresIcp::P2P(const CloudPtr &source, const Eigen::Matrix4f &predict_pose, CloudPtr &transformed_source_ptr, Eigen::Matrix4f &result_pose) {
    source_ptr = source;
    CloudPtr transform_cloud(new PointCloud());
    Eigen::Matrix4d T = predict_pose.cast<double>();
    q_w_curr = Eigen::Quaterniond(T.block<3, 3>(0, 0));
    t_w_curr = T.block<3, 1>(0, 3);

    for (int i = 0; i < max_iterations; ++i) {
        pcl::transformPointCloud(*source_ptr, *transform_cloud, T);
        ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
        ceres::Problem::Options problem_options;
        ceres::Problem problem(problem_options);

        problem.AddParameterBlock(parameters, 7, new test_ceres::PoseSE3Parameterization());

        // std::cout << "------------ " << i << "------------" << std::endl;
        for (int j = 0; j < transform_cloud->size(); ++j) {
            const PointType &origin_pt = source_ptr->points[j];
            if (!pcl::isFinite(origin_pt))
                continue;

            const PointType &transform_pt = transform_cloud->at(j);
            std::vector<float> res_dis;
            std::vector<int> indices;
            kdtree_flann->nearestKSearch(transform_pt, 1, indices, res_dis);
            if (res_dis.front() > max_coresspoind_dis)
                continue;

            Eigen::Vector3d nearest_pt =
                Eigen::Vector3d(target_ptr->at(indices.front()).x, target_ptr->at(indices.front()).y, target_ptr->at(indices.front()).z);

            Eigen::Vector3d origin_eigen(origin_pt.x, origin_pt.y, origin_pt.z);

            ceres::CostFunction *cost_function = new test_ceres::EdgeAnalyticCostFuntion(origin_eigen, nearest_pt);
            problem.AddResidualBlock(cost_function, loss_function, parameters);
        }
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.max_num_iterations = 10;
        options.minimizer_progress_to_stdout = false;
        options.check_gradients = false;
        options.gradient_check_relative_precision = 1e-4;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        T.setIdentity();
        T.block<3, 1>(0, 3) = t_w_curr;
        T.block<3, 3>(0, 0) = q_w_curr.toRotationMatrix();

        // std::cout << "T\n"
        //           << T << std::endl;
    }

    final_pose = T.cast<float>();
    result_pose = T.cast<float>();
    // pcl::transformPointCloud(*source_ptr, *transformed_source_ptr, result_pose);
    return true;
}
bool CeresIcp::P2Plane(const CloudPtr &source, const Eigen::Matrix4f &predict_pose, CloudPtr &transformed_source_ptr, Eigen::Matrix4f &result_pose) {
    std::cout << "target size: " << target_ptr->size() << ",input cloud size:" << source->size() << std::endl;
    source_ptr = source;
    CloudPtr transform_cloud(new PointCloud);
    Eigen::Matrix4d T = predict_pose.cast<double>();
    q_w_curr = Eigen::Quaterniond(T.block<3, 3>(0, 0));
    t_w_curr = T.block<3, 1>(0, 3);
    // 迭代次数
    for (int i = 0; i < max_iterations; ++i) {
        pcl::transformPointCloud(*source, *transform_cloud, T);
        // ceres::LossFunction *lost_function = new ceres::HuberLoss(0.1);

        ceres::Problem::Options problem_options;
        ceres::Problem problem(problem_options);
        // 添加带优化的变量, PoseSE3Parameterization继承了localparam, 告诉雅克比矩阵的计算和误差的相加
        // problem.AddParameterBlock(parameters, 7, new test_ceres::PoseSE3Parameterization());
        int effect_num = 0;

        std::vector<int> index(source_ptr->points.size());
        for (int i = 0; i < index.size(); ++i) {
            index[i] = i;
        }

        std::vector<bool> effect_pts(index.size(), false);
        // 遍历transformed点云
        for (int j = 0; j < transform_cloud->size(); ++j) {
            const PointType &origin_pt = source_ptr->points[j];
            Eigen::Vector3d origin_pt_v3d = ToVec3d(origin_pt);
            // 去除无效点
            if (!pcl::isFinite(origin_pt)) {
                continue;
            }
            const PointType &transform_pt = transform_cloud->at(j);

            std::vector<int> indices;
            std::vector<float> dist;
            // 搜索transform_pt周围的5个点
            kdtree_flann->nearestKSearch(transform_pt, 5, indices, dist);
            // kdtree_flann->nearestKSearch(transform_pt, 1, indices, dist);
            // // 确保找到周围的点>3个
            if (indices.size() > 3) {
                // 拟合平面
                // 将target点云中的点转换为向量
                std::vector<Eigen::Vector3d> nn_eigen;
                for (int k = 0; k < indices.size(); ++k) {
                    nn_eigen.emplace_back(ToVec3d(target_ptr->points[indices[k]]));
                }
                // 获取平面的法向量
                Eigen::Vector4d n;
                if (!FitPlane(nn_eigen, n)) {
                    effect_pts[j] = false;
                    continue;
                }
                // transform_pt 距离平面的误差
                Eigen::Vector3d pt = ToVec3d(transform_pt);
                double e = n.head<3>().dot(pt) + n[3];
                // std::cout << "e: " << e << std::endl;
                // 点到平面的距离
                if (fabs(e) > 0.05) {
                    effect_pts[j] = false;
                    continue;
                }
                // 平面拟合成功且误差距离小给定的阈值
                effect_pts[j] = true;
                // 构建优化问题
                ceres::CostFunction *cost_function = LidarPlaneNormFactor::Create(origin_pt_v3d, n.head<3>(), n[3]);
                problem.AddResidualBlock(cost_function, nullptr, parameters, parameters + 4);
                effect_num++;
            } else {
                effect_pts[j] = false;
            }
        }
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.max_num_iterations = 10;
        options.minimizer_progress_to_stdout = false;
        options.check_gradients = false;
        options.gradient_check_relative_precision = 1e-4;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        T.setIdentity();
        T.block<3, 1>(0, 3) = t_w_curr;
        T.block<3, 3>(0, 0) = q_w_curr.toRotationMatrix();
    }
    final_pose = T.cast<float>();
    result_pose = T.cast<float>();
    // pcl::transformPointCloud(*source_ptr, *transformed_source_ptr, result_pose);
    return true;
}
