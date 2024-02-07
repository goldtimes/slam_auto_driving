#pragma once
#include <list>
#include <unordered_map>
#include <vector>
#include "common/eigen_sophus.h"
#include "common/g2o_types.h"
#include "common/point_types.h"

namespace lh {
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
    size_t capacity_ = 1e5;        // 缓存的体素数量

    NearbyType nearby_type_ = NearbyType::NEARBY6;
};

// 体素的坐标,存放在map中的key
using VOXEL_LOC = Eigen::Matrix<int, 3, 1>;

// 单个体素的数据结构
struct VoxelData {
    VoxelData() {}
    VoxelData(const Vec3d& pt) {
        pts_.emplace_back(pt);
        num_pts_ = 1;
    }

    void AddPoint(const Vec3d& pt) {
        pts_.emplace_back(pt);
        if (!ndt_estimated_) {
            num_pts_++;
        }
    }

    std::vector<Vec3d> pts_;              //  体素中的点
    Vec3d mu_ = Eigen::Vector3d::Zero();  // 均值
    Mat3d sigma_ = Mat3d::Zero();         // 协方差
    Mat3d info_ = Mat3d::Zero();          // 协方差的逆

    bool ndt_estimated_ = false;  // ndt 是否已经估计
    int num_pts_ = 0;             // 总共的点数
};

class IncNdt3d {
   public:
    IncNdt3d() {
        options_.inv_voxel_size_ = 1.0 / options_.voxel_size_;
        GenerateNearbyGrids();
    }
    IncNdt3d(Options options = Options()) {
        options_.inv_voxel_size_ = 1.0 / options_.voxel_size_;
        GenerateNearbyGrids();
    }
    // 往grid中添加点云
    void AddCloud(CloudPtr cloud_world);
    //  设置代配准的点云
    void SetSource(CloudPtr source) { source_ = source; }

    bool AlignNdt(SE3& init_pose);

    int GetNumGrids() const { return grids_.size(); }
    // iekf
    void ComputeResidualAndJacobians(const SE3& pose, Mat18d& HTVH, Vec18d& HTVr);
    // g2o
    void BuildNDTEdges(VertexPose* v, std::vector<EdgeNDT*>& edges);

   private:
    void GenerateNearbyGrids();
    void UpdateVoxel(VoxelData& v);

   private:
    Options options_;

    CloudPtr source_ = nullptr;

    using KeyAndData = std::pair<VOXEL_LOC, VoxelData>;
    std::list<KeyAndData> data_;                                                         // 存放保存在map中的数据, LRU 缓存机制
    std::unordered_map<VOXEL_LOC, std::list<KeyAndData>::iterator, hash_vec<3>> grids_;  // 3d栅格
    std::vector<VOXEL_LOC> nearby_grids_;                                                // 附近的栅格

    bool flag_first_scan_ = true;
};
}  // namespace lh