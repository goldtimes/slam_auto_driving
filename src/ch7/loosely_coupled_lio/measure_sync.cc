#include "measure_sync.h"

namespace lh {
bool MessageSync::sync() {
    // 如果点云和imu数据为空
    if (lidar_buffer_.empty() || imu_buffer_.empty()) {
        return false;
    }

    if (!lidar_pushed_) {
        // 取出最早的lidar数据
        measures_.lidar_ = lidar_buffer_.front();
        measures_.lidar_begin_time_ = time_buffer_.front();
        lidar_end_time_ = measures_.lidar_begin_time_ + measures_.lidar_->points.back().time / double(1000);
        // 计算雷达帧尾的数据
        measures_.lidar_end_time_ = lidar_end_time_;
        lidar_pushed_ = true;
    }
    // imu的时间 < lidar的时间
    if (last_timestamp_imu_ < lidar_end_time_) {
        return false;
    }

    double imu_time = imu_buffer_.front()->timestamp_;
    measures_.imu_.clear();
    // 确保imu不为空，并且imu数据早于lidar数据，然后要做时间同步
    while ((!imu_buffer_.empty()) && (imu_time < lidar_end_time_)) {
        // 取出imu的时间
        imu_time = imu_buffer_.front()->timestamp_;
        // imu 时间 > 雷达的帧尾数据
        if (imu_time > lidar_end_time_) {
            break;
        }
        measures_.imu_.push_back(imu_buffer_.front());
        imu_buffer_.pop_front();
    }
    lidar_buffer_.pop_front();
    time_buffer_.pop_front();
    lidar_pushed_ = false;
    if (callback_) {
        callback_(measures_);
    }
    return true;
}

void MessageSync::Init(const std::string& yaml) { conv_->LoadFromYAML(yaml); }
}  // namespace lh