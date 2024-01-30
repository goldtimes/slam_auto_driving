#pragma once

#include "common/eigen_sophus.h"
#include "common/point_types.h"

namespace lh {

class Ndt3d {
   public:
    enum class NearbyType {
        CENTER,
        NEARBY6,
    };

    using KeyType = Eigen::Matrix<int, 3, 1>;  // 体素的索引
    // 单个体素中的性质,均值和方差
    struct VoxelValue {
        VoxelValue() {}
        VoxelValue(size_t id) { idx_.emplace_back(id); }
        std::vector<size_t> idx_;                // 存储在体素中的点云索引
        Vec3d mu_ = Vec3d::Zero();               // 均值
        Eigen::Matrix3d sigma_ = Mat3d::Zero();  // 协方差
        Eigen::Matrix3d info_ = Mat3d::Zero();   // 信息矩阵,协方差的逆
    };
    // 参数配置
    struct Options {
        int max_iteration_ = 20;        // 最大迭代次数
        double voxel_size_ = 1.0;       // 体素大小
        double inv_voxel_size_ = 1.0;   // 体素
        int min_effective_pts_ = 10;    // 最近邻 点数阈值
        int min_pts_in_voxel_ = 3;      // 每个栅格中最小点数
        double eps_ = 1e-2;             // 收敛判定
        double res_outlier_th_ = 20.0;  // 异常值拒绝阈值
        bool remove_centroid_ = false;  //是否计算两个点云中心并移除中心?

        NearbyType nearby_type_ = NearbyType::NEARBY6;
    };

   public:
    Ndt3d() {
        options_.inv_voxel_size_ = 1.0 / options_.voxel_size_;
        GenerateNearbyGrids();
    }
    Ndt3d(Options options) : options_(options) {
        options_.inv_voxel_size_ = 1.0 / options_.voxel_size_;
        GenerateNearbyGrids();
    }

    void SetTarget(CloudPtr target) {
        target_cloud_ = target;

        BuildVoxels();
        target_center_ = std::accumulate(target_cloud_->begin(), target_cloud_->end(), Vec3d::Zero(),
                                         [](const Vec3d& c, const PointType& pt) -> Vec3d { return c + ToVec3d(pt); }) /
                         target_cloud_->size();
    }

    void SetSource(CloudPtr source) {
        source_cloud_ = source;
        source_center_ = std::accumulate(source_cloud_->begin(), source_cloud_->end(), Vec3d::Zero(),
                                         [](const Vec3d& c, const PointType& pt) -> Vec3d { return c + ToVec3d(pt); }) /
                         source_cloud_->size();
    }

    void SetGtPose(const SE3& gt_pose) {
        gt_pose_ = gt_pose;
        gt_set_ = true;
    }

    // gauss-newton 方法
    bool AlignNdt(SE3& init_pose);

   private:
    // 建立体素
    void BuildVoxels();
    // 生成周边的体素信息
    void GenerateNearbyGrids();

   private:
    Options options_;

    // 我们用什么数据类型来存储体素点云呢? 显然用map会好很多,key为体素的索引,value为体素的均值和方差
    // 键值,value和求hash值的方法
    std::unordered_map<KeyType, VoxelValue, hash_vec<3>> voxel_grid_;
    // 单个体素的周围体素
    std::vector<KeyType> nearby_grids_;
    // 点云
    CloudPtr target_cloud_;
    CloudPtr source_cloud_;
    // 点云中心
    Vec3d target_center_;
    Vec3d source_center_;

    bool gt_set_ = false;
    SE3 gt_pose_;
};
}  // namespace lh