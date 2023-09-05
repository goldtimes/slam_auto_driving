#ifndef SLAM_IN_AUTO_DRIVING_NDT_INC_H
#define SLAM_IN_AUTO_DRIVING_NDT_INC_H

#include <list>
#include "common/eigen_sophus.h"
#include "common/g2o_types.h"
#include "common/point_types.h"

namespace lh {
/**
 * @brief 增量版本的NDT，内部会维护增量式的voxels, 自动删除旧的voxel,往voxel里添加点云时，更新均值和协方差估计
 */
class IncNdt3d {
   public:
    enum class NearbyType {
        CENTER,   // 只考虑中心
        NEARBY6,  // 上下左右前后
    };

    struct Options {
        int max_iteration_ = 4;        // 最大迭代次数
        double voxel_size_ = 1.0;      // 体素大小
        double inv_voxel_size_ = 1.0;  // 体素大小之逆
        int min_effective_pts_ = 10;   // 最近邻点数阈值
        int min_pts_in_voxel_ = 5;     // 每个栅格中最小点数
        int max_pts_in_voxel_ = 50;    // 每个栅格中最大点数
        double eps_ = 1e-3;            // 收敛判定条件
        double res_outlier_th_ = 5.0;  // 异常值拒绝阈值
        size_t capacity_ = 100000;     // 缓存的体素数量

        NearbyType nearby_type_ = NearbyType::NEARBY6;
    };

    using KeyType = Eigen::Matrix<int, 3, 1>;  // 体素的索引
    // 体素内置结构
    struct VoxelData {
        VoxelData() {}
        VoxelData(const Vec3d& pt) {}
        void AddPoint(const Vec3d& pt) {}

        std::vector<Vec3d> pts_;       // 每个体素的内部点，多于一定数量之后再估计均值和协方差
        Vec3d mu_ = Vec3d::Zero();     // 均值
        Mat3d sigma_ = Mat3d::Zero();  // 协方差
        Mat3d info_ = Mat3d::Zero();   // 协方差的逆

        bool ndt_estimated_ = false;  // NDT是否已经估计
        int num_pts_ = 0;             // 总共的点数
    };

    IncNdt3d() {}
    IncNdt3d(Options options) : options_(options) {}
    // 获取体素个数
    int NumGrids() const { return grids_.size(); }

    void AddCloud(CloudPtr cloud_world);

    void SetSource(CloudPtr source) { source_ = source; }

    bool AlignNdt(SE3& init_pose);

    /**
     * @brief 计算给定Pose下的雅可比和残差矩阵，符合IEKF中符号（8.17, 8.19）
     */
    void ComputeResidualAndJacobians(const SE3& pose, Mat18d& HTVHm, Vec18d& HTVr);
    /**
     * 根据估计的NDT建立edges
     * @param v
     * @param edges
     */
    // void BuildNDTEdges(VertexPose* v, std::vector<EdgeNDT*>& edges);

   private:
    void GenerateNearbyGrids();
    void UpdateVoxel(VoxelData& v);

   private:
    CloudPtr source_ = nullptr;
    Options options_;

    using KeyAndData = std::pair<KeyType, VoxelData>;
    std::list<KeyAndData> data_;                                                       // 真实数据，缓存
    std::unordered_map<KeyType, std::list<KeyAndData>::iterator, hash_vec<3>> grids_;  // 栅格数据
    std::vector<KeyType> nearby_grids_;                                                // 附近的栅格
    bool flag_first_scan_ = true;
};
}  // namespace lh

#endif