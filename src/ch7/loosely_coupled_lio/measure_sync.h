#ifndef SLAM_IN_AUTO_DRIVING_MEASURE_SYNC_H
#define SLAM_IN_AUTO_DRIVING_MEASURE_SYNC_H

#include <glog/logging.h>
#include <deque>
#include "cloud_convert.h"
#include "common/imu.h"
#include "common/point_types.h"

namespace lh {
/**
 * @brief IMU数据和雷达同步
 */
struct MeasureGroup {
    MeasureGroup() { this->lidar_.reset(new FullPointCloudType()); }

    double lidar_begin_time_ = 0;   // 雷达包的起始时间
    double lidar_end_time_ = 0;     // 雷达的终止时间
    FullCloudPtr lidar_ = nullptr;  // 雷达点云
    std::deque<IMUPtr> imu_;        // 上一时时刻到现在的IMU读数
};
/**
 * 将激光数据和imu数据同步
 */
class MessageSync {
   public:
    using Callback = std::function<void(const MeasureGroup&)>;

    MessageSync(Callback callback) : callback_(callback), conv_(new CloudConvert()) {}

    // 初始化
    void Init(const std::string& yaml);

    /**
     * @brief 处理imu数据
     */
    void ProcessIMU(IMUPtr& imu) {
        double timestamp = imu->timestamp_;
        if (timestamp < last_timestamp_imu_) {
            LOG(WARNING) << "imu loop back, clear buuff";
            imu_buffer_.clear();
        }
        last_timestamp_imu_ = timestamp;
        imu_buffer_.emplace_back(imu);
    }

    void ProcessCloud(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        if (msg->header.stamp.toSec() < last_timestamp_lidar_) {
            LOG(WARNING) << "lidar loop back, clear buuff";
            lidar_buffer_.clear();
        }

        FullCloudPtr cloud(new FullPointCloudType());
        conv_->Process(msg, cloud);
        lidar_buffer_.push_back(cloud);
        time_buffer_.push_back(msg->header.stamp.toSec());
        last_timestamp_lidar_ = msg->header.stamp.toSec();

        sync();
    }

    void ProcessCloud(const livox_ros_driver::CustomMsg::ConstPtr& msg) {
        if (msg->header.stamp.toSec() < last_timestamp_lidar_) {
            LOG(WARNING) << "lidar loop back, clear buffer";
            lidar_buffer_.clear();
        }

        last_timestamp_lidar_ = msg->header.stamp.toSec();
        FullCloudPtr ptr(new FullPointCloudType());
        conv_->Process(msg, ptr);

        if (ptr->empty()) {
            return;
        }

        lidar_buffer_.emplace_back(ptr);
        time_buffer_.emplace_back(last_timestamp_lidar_);

        sync();
    }

   private:
    // 尝试同步imu与激光数据，成功返回true
    bool sync();

   private:
    Callback callback_;                             // 同步数据后的回调函数
    std::shared_ptr<CloudConvert> conv_ = nullptr;  // 点云转换
    std::deque<FullCloudPtr> lidar_buffer_;         // 雷达数据缓冲
    std::deque<IMUPtr> imu_buffer_;                 // imu数据缓冲
    double last_timestamp_imu_ = -1.0;              // 最近imu数据
    double last_timestamp_lidar_ = 0;               // 最近雷达数据

    std::deque<double> time_buffer_;
    bool lidar_pushed_ = false;
    MeasureGroup measures_;
    double lidar_end_time_ = 0;
};
}  // namespace lh

#endif