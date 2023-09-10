#ifndef FASTER_LIO_LASER_MAPPING_H
#define FASTER_LIO_LASER_MAPPING_H

#include <livox_ros_driver/CustomMsg.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>

#include "ch3/eskf.hpp"
#include "ch3/static_imu_init.h"

#include "ch7/incremental_ndt_lo.h"
#include "ch7/loosely_coupled_lio/cloud_convert.h"
#include "ch7/loosely_coupled_lio/measure_sync.h"
#include "tools/ui/pangolin_window.h"

namespace lh {
/**
 * @brief 使用3章的ekf, 7.3节的增量NDT 里程计来实现
 * 第一个imu和雷达的程序
 */
class LooselyLIO {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    struct Options {
        Options() {}
        bool save_motion_undistortion_pcd_ = false;
        bool with_ui_ = true;
    };
    LooselyLIO(const Options& options);
    ~LooselyLIO() = default;

    bool Init(const std::string& config_yaml);

    void PCLCallBack(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void LivoxPCLCallBack(const livox_ros_driver::CustomMsg::ConstPtr& msg);

    // imu 回调
    void IMUCallBack(IMUPtr msg_in);

    void Finish();

   private:
    void ProcessMeasurements(const MeasureGroup& meas);

    bool LoadFromYAML(const std::string& yaml);
    // imu 尝试初始化
    void TryInitIMU();
    // 利用imu预测状态信息
    // 预测数据会放到imu_states_;
    void Predict();

    // 对measures_中的点云去畸变
    void Undistort();
    // 配准
    void Align();

   private:
    std::shared_ptr<MessageSync> sync_ = nullptr;  // 消息同步
    StaticImuInit imu_init_;                       // imu 静止初始化
    std::shared_ptr<IncrementalNDTLO> inc_ndt_lo_ = nullptr;

    FullCloudPtr scan_undistort_{new FullPointCloudType()};
    SE3 pose_of_lo_;
    Options options_;

    // flags
    bool imu_need_init_ = true;   // imu的初始零偏
    bool flg_first_scan_ = true;  // 是否第一个雷达
    int frame_num_ = 0;           // 帧数计算

    // EKF data
    MeasureGroup measures_;              // 同步之后的imu和点云
    std::vector<NavStated> imu_states_;  // eskf 预测期间的状态
    ESKFD eskf_;

    SE3 TIL_;  // lidar_imu的外参
    std::shared_ptr<ui::PangolinWindow> ui_ = nullptr;
};
}  // namespace lh

#endif