#ifndef SLAM_IN_AUTO_DRIVING_GEN_SIMU_DATA_H
#define SLAM_IN_AUTO_DRIVING_GEN_SIMU_DATA_H

#include "common/eigen_sophus.h"
#include "common/point_types.h"

/**
 * 生成本章所需要的仿真数据
 * 仿真数据可以获取真值位姿，也能保证真值的点到点匹配关系，可以检测我们算法是否写对
 *
 * 本程序模拟一个简单的长方体，可以给它施加一个随机的6 dof变换，然后输出source和target点云
 */
namespace lh {
class GenSimuData {
   public:
    struct Options {
        Options() {}
        int num_points_ = 2000;  // 点数中点的数量
        // 盒子参数
        double width_ = 5.0;    // 宽度,y方向
        double length_ = 10.0;  // 长度,x方向
        double height_ = 1.0;   // 宽度,z方向
        // pose 参数
        double pose_rot_sigma_ = 0.05;   // 旋转方向的sigma
        double pose_trans_sigma_ = 0.3;  // 平移方向的sigma
    };

    GenSimuData(Options options = Options()) : options_(options) {}

    void Gen();
    CloudPtr GetTarget() const { return target_; }
    CloudPtr GetSource() const { return source_; }
    SE3 GetPose() const { return gt_pose_; }

   private:
    void GenTarget();
    CloudPtr target_ = nullptr, source_ = nullptr;
    Options options_;
    SE3 gt_pose_;  // 真值,从target转到source
};
}  // namespace lh

#endif