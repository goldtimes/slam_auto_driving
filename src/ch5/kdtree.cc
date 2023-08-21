#include "kdtree.h"
#include <glog/logging.h>
#include "common/math_utils.h"

#include <execution>
#include <set>

namespace lh {
bool KdTree::BuildTree(const CloudPtr& cloud) {
    if (cloud->empty())
        return false;
    cloud_.clear();
    cloud_.resize(cloud->size());

    for (size_t i = 0; i < cloud->points.size(); ++i) {
        cloud_[i] = ToVec3f(cloud->points[i]);
    }

    Clear();
    Reset();

    IndexVec idx(cloud->size());
    for (int i = 0; i < cloud->points.size(); ++i) {
        idx[i] = i;
    }
    Insert(idx, root_.get());
    return true;
}

/**
 * 递归创建
 */
void KdTree::Insert(const IndexVec& points, KdTreeNode* node) {
    nodes_.insert({node->id_, node});
    if (points.empty()) {
        return;
    }
    if (points.size() == 1) {
        size_++;
        node->point_idx_ = points[0];
        return;
    }
    IndexVec left, right;

    if (!FindSplitAxisAndThresh(points, node->axis_index_, node->split_thresh_, left, right)) {
        size_++;
        node->point_idx_ = points[0];
        return;
    }

    const auto create_if_not_empty = [&node, this](KdTreeNode*& new_node, const IndexVec& index) {
        if (!index.empty()) {
            new_node = new KdTreeNode;
            new_node->id_ = tree_node_id_++;
            Insert(index, new_node);
        }
    };

    create_if_not_empty(node->left_, left);
    create_if_not_empty(node->right_, right);
}

/**
 * 计算三个轴上的散布情况
 */
bool KdTree::FindSplitAxisAndThresh(const IndexVec& point_idx, int& axis, float& th, IndexVec& left, IndexVec& right) {
    Vec3f var;
    Vec3f mean;

    math::ComputeMeanAndCovDiag(point_idx, mean, var, [this](int idx) { return cloud_[idx]; });

    int max_i, max_j;
    // 取最大的方差作为分割轴，均值作为阈值
    var.maxCoeff(&max_i, &max_j);
    axis = max_i;
    th = mean[axis];

    for (const auto& idx : point_idx) {
        if (cloud_[idx][axis] < th) {
            left.emplace_back(idx);
        } else {
            right.emplace_back(idx);
        }
    }
    // 边界情况检查：输入的points等于同一个值，上面的判定是>=号，所以都进了右侧
    // 这种情况不需要继续展开，直接将当前节点设为叶子就行
    if (point_idx.size() > 1 && (left.empty() || right.empty())) {
        return false;
    }
    return true;
}

void KdTree::Reset() {
    tree_node_id_ = 0;
    root_.reset(new KdTreeNode());
    root_->id_ = tree_node_id_++;
    size_ = 0;
}

void KdTree::Clear() {
    for (const auto& np : nodes_) {
        if (np.second != root_.get()) {
            delete np.second;
        }
    }

    nodes_.clear();
    root_ = nullptr;
    size_ = 0;
    tree_node_id_ = 0;
}

void KdTree::PrintAll() {
    for (const auto& np : nodes_) {
        auto node = np.second;
        if (node->left_ == nullptr && node->right_ == nullptr) {
            LOG(INFO) << "leaf node: " << node->id_ << ", idx:" << node->point_idx_;
        } else {
            LOG(INFO) << "node: " << node->id_ << ",axis:" << node->axis_index_ << ",th:" << node->split_thresh_;
        }
    }
}
}  // namespace lh