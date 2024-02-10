#pragma once
#include <deque>
#include "3d_slam/3d_icp.hh"
#include "ch5/kdtree.h"
#include "common/eigen_sophus.h"
#include "common/point_types.h"
#include "tools/pcl_map_viewer.h"

namespace lh {
class FeatureExtraction;

class LoamLikeOdom {
   public:
    struct Options {};

    void ProcessPointCloud(FullCloudPtr full_cloud);

    void SaveMap(const std::string& path);

   private:
    SE3 AlignWithLocalMap(CloudPtr edge, CloudPtr surf);
    bool IsKeyframe(SE3& current_pose);

   private:
    Options options_;
};
}  // namespace lh