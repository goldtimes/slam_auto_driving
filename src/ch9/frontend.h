#ifndef SLAM_IN_AUTO_DRIVING_FRONTEND_H
#define SLAM_IN_AUTO_DRIVING_FRONTEND_H

#include <map>
#include <memory>
#include <string>

#include "common/gnss.h"
#include "common/nav_state.h"
#include "keyframe.h"

namespace lh {
class LioIEKF;
/**
 * 建图的前端部分, 将IMU和lidar点云交给lio处理,将RTK解析为rtk_pose
 */
class Frontend {
   public:
    struct Options {};
    explicit Frontend(const std::string& config_yaml);
    // 初始化，创建LIO对象，检查数据存在性
    bool Init();

    /// 运行前端
    void Run();

   private:
    /// 看某个state是否能提取到RTK pose
    void ExtractKeyFrame(const NavStated& state);

    /// 确定某个关键帧的GPS pose
    void FindGPSPose(std::shared_ptr<Keyframe> kf);

    /// 保存各个关键帧位姿，点云在生成时已经保存
    void SaveKeyframes();

    /// 提取RTK地图原点并移除此原点
    void RemoveMapOrigin();

   private:
    std::shared_ptr<Keyframe> last_kf_ = nullptr;            // 最新关键帧
    IdType kf_id_ = 0;                                       // 最新关键帧的id
    std::map<IdType, std::shared_ptr<Keyframe>> keyframes_;  // 抽取的关键帧
    std::shared_ptr<LioIEKF> lio_ = nullptr;                 // LIO
    std::string config_yaml_;                                // 配置文件路径
    std::map<double, GNSSPtr> gnss_;                         // gnss数据
    Vec3d map_origin_ = Vec3d::Zero();                       // 地图的原点
    std::string bag_path_;
    std::string lio_yaml_;
    double kf_dis_th_ = 1.0;       // 关键帧距离阈值
    double kf_ang_th_deg_ = 10.0;  // 关键帧角度阈值
};
}  // namespace lh

#endif