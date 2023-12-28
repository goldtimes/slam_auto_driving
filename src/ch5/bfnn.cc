#include "bfnn.h"
// c++17 多线程
#include <execution>

namespace lh {
/**
 * std::min_element 寻找范围内最小的元素，自定义比较函数和point比较，返回迭代器的位置
 * 返回的迭代器位置-cloud->points.begin() = 下标
 */
int bfnn_point(CloudPtr cloud, const Vec3f& point) {
    return std::min_element(cloud->points.begin(), cloud->points.end(),
                            [&point](const PointType& pt1, const PointType& pt2) -> bool {
                                return (pt1.getVector3fMap() - point).squaredNorm() < (pt2.getVector3fMap() - point).squaredNorm();
                            }) -
           cloud->points.begin();
}

/**
 * 最近k邻域
 */

std::vector<int> bfnn_point_k(CloudPtr cloud, const Vec3f& point, int k) {
    /// 函数中定义了一个结构体,存储点云中的点的下标和point的距离
    struct IndexAndDist {
        IndexAndDist() {}
        IndexAndDist(int index, double dis2) : index_(index), dis2_(dis2) {}
        int index_ = 0;
        double dis2_ = 0;
    };

    std::vector<IndexAndDist> index_and_dist(cloud->size());
    // 计算点云中每个点和point的距离
    for (int i = 0; i < cloud->size(); ++i) {
        index_and_dist[i] = {i, (cloud->points[i].getVector3fMap() - point).squaredNorm()};
    }
    // 根据距离从小到大的排列
    std::sort(index_and_dist.begin(), index_and_dist.end(), [](const auto& d1, const auto& d2) { return d1.dis2_ < d2.dis2_; });

    std::vector<int> ret;
    // 将一个容器的值->给到另一个容器
    // 将index_and_dist的前k个距离最小的点云下标放到ret中
    std::transform(index_and_dist.begin(), index_and_dist.begin() + k, std::back_inserter(ret), [](const auto& d1) { return d1.index_; });
    return ret;
}
// 单线程版本
void bfnn_cloud(CloudPtr target, CloudPtr source, std::vector<std::pair<size_t, size_t>>& matches) {
    std::vector<size_t> index(source->size());
    // 遍历index，给index元素赋值
    std::for_each(index.begin(), index.end(), [idx = 0](size_t& i) mutable { i = idx++; });
    matches.resize(index.size());
    // mathes中包含点云1中的点的索引 和 给它的一个idx
    std::for_each(std::execution::seq, index.begin(), index.end(), [&](auto idx) {
        // 包含了source中点云的下标和target点云中的最邻点
        matches[idx].second = idx;
        matches[idx].first = bfnn_point(target, ToVec3f(source->points[idx]));
    });
}

// 多线程版本
void bfnn_cloud_mt(CloudPtr cloud1, CloudPtr cloud2, std::vector<std::pair<size_t, size_t>>& matches) {
    std::vector<size_t> index(cloud2->size());
    // 遍历index，给index元素赋值
    std::for_each(index.begin(), index.end(), [idx = 0](size_t& i) mutable { i = idx++; });
    matches.resize(index.size());
    // 区别在于std::execution::par_unseq
    std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](auto idx) {
        matches[idx].second = idx;
        matches[idx].first = bfnn_point(cloud1, ToVec3f(cloud2->points[idx]));
    });
}

void bfnn_cloud_mt_k(CloudPtr cloud1, CloudPtr cloud2, std::vector<std::pair<size_t, size_t>>& matches, int k) {
    // 生成索引
    std::vector<size_t> index(cloud2->size());
    std::for_each(index.begin(), index.end(), [idx = 0](size_t& i) mutable { i = idx++; });

    // 并行化
    matches.resize(index.size() * k);
    std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](auto idx) {
        auto v = bfnn_point_k(cloud1, ToVec3f(cloud2->points[idx]), k);
        for (int i = 0; i < v.size(); ++i) {
            matches[idx * k + i].first = v[i];
            matches[idx * k + i].second = idx;
        }
    });
}

}  // namespace lh
