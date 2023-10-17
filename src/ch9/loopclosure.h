#ifndef SLAM_IN_AUTO_DRIVING_LOOP_CLOSING_H
#define SLAM_IN_AUTO_DRIVING_LOOP_CLOSING_H

#include "common/eigen_sophus.h"
#include "common/point_types.h"
#include "keyframe.h"

namespace lh {
/**
 * 回环检测候选帧
 */
struct LoopCandidate {
    LoopCandidate() {}
    /**
     * 两帧之间的相对变换
     */
    LoopCandidate(IdType id1, IdType id2, SE3 Tij) : idx1_(id1), idx2_(id2), Tij_(Tij) {}

    IdType idx1_ = 0;
    IdType idx2_ = 0;

    SE3 Tij_;
    double ndt_score_ = 0.0;
};

class LoopClosure {
   public:
    explicit LoopClosure(const std::string& config_yaml);

    bool Init();

    void Run();

   private:
    // 提取回环的候选
    void DetectLoopCandidates();

    // 计算回环候选帧的相对运动
    void ComputeLoopCandidates();

    /// 计算一个回环候选是否成立
    void ComputeForCandidate(LoopCandidate& c);

    /// 保存计算结果
    void SaveResults();

   private:
    std::vector<LoopCandidate> loop_candiates_;
    int min_id_interval_ = 50;   // 候选的两个关键帧之间的id差值
    double min_distance_ = 30;   // 被候选的最小距离
    int skip_id_ = 5;            // 如果选择了一个候选帧,那么隔开多少个id之后在选择一个
    double ndt_score_th_ = 2.5;  // 有效回环的ndt分值

    std::map<IdType, KFPtr> keyframes_;
    std::string yaml_;
};

}  // namespace lh

#endif