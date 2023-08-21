#ifndef SLAM_IN_AUTO_DRIVING_KDTREE_H
#define SLAM_IN_AUTO_DRIVING_KDTREE_H

#include <glog/logging.h>
#include <map>
#include <queue>
#include "common/eigen_sophus.h"
#include "common/point_types.h"

namespace lh {
/**
 * kd数节点，二叉树结构
 */
struct KdTreeNode {
    int id_ = -1;
    int point_idx_ = 0;            // 点的索引
    int axis_index_ = 0;           // 分割轴
    float split_thresh_ = 0.0;     // 分割阈值
    KdTreeNode* left_ = nullptr;   // 左子树
    KdTreeNode* right_ = nullptr;  // 右子树

    bool IsLeaf() const { return left_ == nullptr && right_ == nullptr; }
};

/**
 * 手写kd树
 */

class KdTree {
   public:
    explicit KdTree() = default;
    ~KdTree() { Clear(); }

    bool BuildTree(const CloudPtr& cloud);
    // 获取k最近邻
    bool GetClosestPoint(const PointType& pt, std::vector<int>& closest_idx, int k = 5);
    // 并行为点云寻找最近邻
    bool GetClosestPointMT(const CloudPtr& cloud, std::vector<std::pair<size_t, size_t>>& matches, int k = 5);

    // 计算最近邻的倍数
    void SetEnableANN(bool use_ann = true, float alpha = 0.1) {
        approximate_ = use_ann;
        alpha_ = alpha;
    }
    // 返回节点数量
    size_t size() const { return size_; }
    // 清理数据
    void Clear();

    // 打印节点信息
    void PrintAll();

   private:
    /**
     * 在node插入点
     */
    void Insert(const IndexVec& points, KdTreeNode* node);

    bool FindSplitAxisAndThresh(const IndexVec& point_idx, int& axis, float& th, IndexVec& left, IndexVec& right);

    void Reset();

    static inline float Dis2(const Vec3f& p1, const Vec3f& p2) {}

   private:
    int k_ = 5;                                   // knn最近邻数量
    std::shared_ptr<KdTreeNode> root_ = nullptr;  // 根节点
    std::vector<Vec3f> cloud_;                    // 输入点云
    std::unordered_map<int, KdTreeNode*> nodes_;

    size_t size_ = 0;       // 叶子节点数量
    int tree_node_id_ = 0;  // 为kdtree node分配id
    // 近似最近邻
    bool approximate_ = true;
    float alpha_ = 0.1;
};

}  // namespace lh

#endif