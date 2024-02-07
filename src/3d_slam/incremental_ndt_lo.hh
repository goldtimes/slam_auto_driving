#pragma once
#include "3d_slam/ndt_inc.hh"
#include "common/eigen_sophus.h"
#include "common/point_types.h"
#include "tools/pcl_map_viewer.h"

namespace lh {
class INcrementalNDTLO {
   public:
    struct Options {
        double kf_distance = 0.5;
        double kf_angle = 30;
        bool display_realtime_cloud = true;

        IncNdt3d::Options ndt_inc_options;
    };
    INcrementalNDTLO() = default;
    INcrementalNDTLO(Options options = Options()) : options_(options) {
        if (options_.display_realtime_cloud) {
            viewer_ = std::make_shared<PCLMapViewer>(0.5);
        }

        inc_ndt_ = IncNdt3d(options_.ndt_inc_options);
    }

    void AddCloud(const CloudPtr scan, SE3& pose, bool use_guess = false);

    void SaveMap(const std::string& map_path);

   private:
    bool IsKeyframe(const SE3& current_pose) {}

   private:
    Options options_;
    std::shared_ptr<PCLMapViewer> viewer_;
    IncNdt3d inc_ndt_;
    int cnt_frame_ = 0;
    bool first_frame = true;
    std::vector<SE3> estimated_poses_;
    SE3 last_kf_pose_;
};
}  // namespace lh