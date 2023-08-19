#ifndef SLAM_IN_AUTO_DRIVING_GRID2D_HPP
#define SLAM_IN_AUTO_DRIVING_GRID2D_HPP

#include "bfnn.h"
#include "common/eigen_sophus.h"
#include "common/math_utils.h"
#include "common/point_types.h"

#include <glog/logging.h>
#include <execution>
#include <map>
#include <unordered_map>
#include <vector>

namespace lh {

/**
 * 栅格法最邻近
 * @tparam dim 模板参数，使用2D或3D栅格
 */
template <int dim>
class GridNN {
   public:
    using KeyType = Eigen::Matrix<int, dim, 1>;
    using PtType = Eigen::Matrix<float, dim, 1>;

    enum class NearbyType {
        CENTER,
        // for 2d
        NEARBY4,
        NEARBY8,
        // for3d
        NEARBY6,
    };
    // 构造函数
    explicit GridNN(float resoluation = 0.1, NearbyType near_by_type = NearbyType::NEARBY4) : resolution_(resoluation), nearby_type_(near_by_type) {
        inv_resolution_ = 1.0 / resolution_;
        // 检查维度
        if (dim == 2 && nearby_type_ == NearbyType::NEARBY6) {
            LOG(INFO) << "2D grid does not support nearby6, using nearby4 instead.";
            nearby_type_ = NearbyType::NEARBY4;
        } else if (dim == 3 && (nearby_type_ != NearbyType::NEARBY6 && nearby_type_ != NearbyType::CENTER)) {
            LOG(INFO) << "3D grid does not support nearby4/8, using nearby6 instead.";
            nearby_type_ = NearbyType::NEARBY6;
        }
        GenerateNearByGrids();
    }
    // 设置点云，建立栅格
    bool SetPointCloud(CloudPtr cloud);
    bool GetClosestPoint(const PointType& pt, PointType& closest_pt, size_t& idx);
    // 对比两个点云
    bool GetClosestPointForCloud(CloudPtr ref, CloudPtr query, std::vector<std::pair<size_t, size_t>>& matches);
    bool GetClosestPointForCloudMT(CloudPtr ref, CloudPtr query, std::vector<std::pair<size_t, size_t>>& matches);

   private:
    void GenerateNearByGrids();
    KeyType Pos2Grid(const PtType& pt);

   private:
    float resolution_;
    float inv_resolution_;
    NearbyType nearby_type_ = NearbyType::NEARBY4;

    std::unordered_map<KeyType, std::vector<size_t>, hash_vec<dim>> grids_;  // 栅格数据
    CloudPtr cloud_;
    std::vector<KeyType> nearby_grids_;  // 附近栅格
};

// 实现
template <int dim>
bool GridNN<dim>::SetPointCloud(CloudPtr cloud) {
    std::vector<size_t> index(cloud->size());
    std::for_each(index.begin(), index.end(), [idx = 0](size_t& i) mutable { i = idx++; });

    std::for_each(index.begin(), index.end(), [&cloud, this](const size_t& idx) {
        auto pt = cloud->points[idx];
        auto key = Pos2Grid(ToEigen<float, dim>(pt));
        if (grids_.find(key) == grids_.end()) {
            grids_.insert({key, {idx}});
        } else {
            grids_[key].emplace_back(idx);
        }
    });

    cloud_ = cloud;
    LOG(INFO) << "grids: " << grids_.size();
    return true;
}

template <int dim>
Eigen::Matrix<int, dim, 1> GridNN<dim>::Pos2Grid(const Eigen::Matrix<float, dim, 1>& pt) {
    return (pt * inv_resolution_).template cast<int>();
}

// 定义2d 栅格周边
template <>
void GridNN<2>::GenerateNearByGrids() {
    if (nearby_type_ == NearbyType::CENTER) {
        nearby_grids_.emplace_back(KeyType::Zero());
    } else if (nearby_type_ == NearbyType::NEARBY4) {
        nearby_grids_ = {Vec2i(0, 0), Vec2i(-1, 0), Vec2i(1, 0), Vec2i(0, 1), Vec2i(0, -1)};
    } else if (nearby_type_ == NearbyType::NEARBY8) {
        nearby_grids_ = {
            Vec2i(0, 0), Vec2i(-1, 0), Vec2i(1, 0), Vec2i(0, 1), Vec2i(0, -1), Vec2i(-1, -1), Vec2i(-1, 1), Vec2i(1, -1), Vec2i(1, 1),
        };
    }
}

// 定义3d 栅格的周边
template <>
void GridNN<3>::GenerateNearByGrids() {
    if (nearby_type_ == NearbyType::CENTER) {
        nearby_grids_.emplace_back(KeyType::Zero());
    } else if (nearby_type_ == NearbyType::NEARBY6) {
        nearby_grids_ = {KeyType(0, 0, 0),  KeyType(-1, 0, 0), KeyType(1, 0, 0), KeyType(0, 1, 0),
                         KeyType(0, -1, 0), KeyType(0, 0, -1), KeyType(0, 0, 1)};
    }
}
// 在pt点栅格周边寻找最邻近
template <int dim>
bool GridNN<dim>::GetClosestPoint(const PointType& pt, PointType& closest_pt, size_t& idx) {
    std::vector<size_t> idx_to_check;
    auto key = Pos2Grid(ToEigen<float, dim>(pt));
    // pt点周围的栅格
    std::for_each(nearby_grids_.begin(), nearby_grids_.end(), [&key, &idx_to_check, this](const KeyType& delta) {
        auto dkey = key + delta;
        auto iter = grids_.find(dkey);
        // 找到了
        if (iter != grids_.end()) {
            idx_to_check.insert(idx_to_check.end(), iter->second.begin(), iter->second.end());
        }
    });
    if (idx_to_check.empty()) {
        return false;
    }

    // 暴力查找
    CloudPtr nearby_cloud(new PointCloudType);
    std::vector<size_t> nearby_idx;
    for (auto& idx : idx_to_check) {
        nearby_cloud->points.template emplace_back(cloud_->points[idx]);
        nearby_idx.emplace_back(idx);
    }

    size_t closest_point_idx = bfnn_point(nearby_cloud, ToVec3f(pt));
    idx = nearby_idx.at(closest_point_idx);
    closest_pt = cloud_->points[idx];
    return true;
}

template <int dim>
bool GridNN<dim>::GetClosestPointForCloud(CloudPtr ref, CloudPtr query, std::vector<std::pair<size_t, size_t>>& matches) {
    matches.clear();
    std::vector<size_t> index(query->size());
    std::for_each(index.begin(), index.end(), [idx = 0](size_t& i) mutable { i = idx++; });
    std::for_each(index.begin(), index.end(), [this, &matches, &query](const size_t& idx) {
        PointType cp;
        size_t cp_idx;
        if (GetClosestPoint(query->points[idx], cp, cp_idx)) {
            matches.emplace_back(cp_idx, idx);
        }
    });

    return true;
}

template <int dim>
bool GridNN<dim>::GetClosestPointForCloudMT(CloudPtr ref, CloudPtr query, std::vector<std::pair<size_t, size_t>>& matches) {
    matches.clear();
    // 与串行版本基本一样，但matches需要预先生成，匹配失败时填入非法匹配
    std::vector<size_t> index(query->size());
    std::for_each(index.begin(), index.end(), [idx = 0](size_t& i) mutable { i = idx++; });
    matches.resize(index.size());

    std::for_each(std::execution::par_unseq, index.begin(), index.end(), [this, &matches, &query](const size_t& idx) {
        PointType cp;
        size_t cp_idx;
        if (GetClosestPoint(query->points[idx], cp, cp_idx)) {
            matches[idx] = {cp_idx, idx};
        } else {
            matches[idx] = {math::kINVALID_ID, math::kINVALID_ID};
        }
    });

    return true;
}

}  // namespace lh

#endif