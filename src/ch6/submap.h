#pragma once

#include "frame.h"
#include "likelihood_field.h"
#include "occupancy_map.h"

namespace lh {
/**
 * 子地图类
 * 子地图关联到若干个key_frame,维护自己的栅格地图和似然场
 * 往子地图中添加关键帧时，会更新它的栅格地图与似然场
 * 比如我们定义一个子地图中存放8个key_frame,如果超过8个key_frame，那么就需要建立新的子地图
 * 关键帧如何定义呢？
 */
class Submap {
   public:
    Submap(const SE2& pose) : pose_(pose) {
        occu_map_.SetPose(pose_);
        field_.SetPose(pose_);
    }

    /**
     * @brief 复制其他submap的占据栅格
     */
    void SetOccuFromOtherSubmap(std::shared_ptr<Submap> other);

    /**
     * @brief 将frame与本submap匹配，计算frame->pose
     */
    bool MatchScan(std::shared_ptr<Frame> frame);

    bool HasOutsidePoint() const;

    /**
     * @brief scan数据去更新栅格地图
     */
    void AddScanInOccupancyMap(std::shared_ptr<Frame> scan);

    /**
     * @brief 子地图中增加关键帧
     */
    void AddKeyFrame(std::shared_ptr<Frame> frame) { frames_.emplace_back(frame); }

    /**
     * @brief 当子地图的位姿更新时，重设每个frame的世界位姿
     */
    void UpdateFramePoseWorld();

    OccupancyMap& GetOccumap() { return occu_map_; }
    LikelihoodField& GetLikelihood() { return field_; }

    std::vector<std::shared_ptr<Frame>>& GetFrames() { return frames_; }

    size_t NumFrames() const { return frames_.size(); }

    void SetId(size_t id) { id_ = id; }
    size_t GetId() const { return id_; }

    void SetPose(const SE2& pose);
    SE2 GetPose() const { return pose_; }

   private:
    SE2 pose_;                                    // submap的pose Tstow
    size_t id_ = 0;                               // submap的id
    std::vector<std::shared_ptr<Frame>> frames_;  // 子地图中的关键帧容器
    LikelihoodField field_;                       // 用于匹配
    OccupancyMap occu_map_;                       // 用于生成栅格地图
};
}  // namespace lh