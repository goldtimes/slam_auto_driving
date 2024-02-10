#pragma once
#include "common/point_types.h"

namespace lh {
/**
 * 点云需要有线束信息
 */
class FeatureExtraction {
   public:
    struct IdAndValue {
        // 线束id和曲率的结构体
        IdAndValue() {}
        IdAndValue(int id, double value) : id_(id), value_(value) {}
        int id_;
        double value_;
    };
    FeatureExtraction() = default;
    void Extract(FullCloudPtr pc, CloudPtr pc_out_edge, CloudPtr pc_out_sur);

    void ExtractFromSector(const CloudPtr& pc_in, std::vector<IdAndValue>& cloud_curvature, CloudPtr& pc_out_edge, CloudPtr& pc_out_surf);

   private:
};
}  // namespace lh