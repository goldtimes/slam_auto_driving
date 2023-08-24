#ifndef SLAM_IN_AUTO_DRIVING_GNSS_H
#define SLAM_IN_AUTO_DRIVING_GNSS_H

#include <memory>
#include "common/eigen_sophus.h"
#include "common/message_def.h"

namespace lh {
/// GNSS状态位信息
/// 通常由GNSS厂商提供，这里使用千寻提供的状态位
enum class GpsStatusType {
    GNSS_FLOAT_SOLUTION = 5,         // 浮点解（cm到dm之间）
    GNSS_FIXED_SOLUTION = 4,         // 固定解（cm级）
    GNSS_PSEUDO_SOLUTION = 2,        // 伪距差分解（分米级）
    GNSS_SINGLE_POINT_SOLUTION = 1,  // 单点解（10m级）
    GNSS_NOT_EXIST = 0,              // GPS无信号
    GNSS_OTHER = -1,                 // 其他
};

// UTM坐标
struct UTMCoordinate {
    UTMCoordinate() = default;
    explicit UTMCoordinate(int zone, const Vec2d& xy = Vec2d::Zero(), bool north = true) : zone_(zone), xy_(xy), north_(north) {}
    int zone_ = 0;  // utm区域
    Vec2d xy_ = Vec2d::Zero();
    double z_ = 0;       // 高度
    bool north_ = true;  // 是否在北半球
};

// GNSS读数结构
struct GNSS {
    GNSS() = default;
    GNSS(double unix_time, int status, const Vec3d& lat_lon_alt, double heading, bool heading_valid)
        : unix_time_(unix_time), lat_lon_alt_(lat_lon_alt), heading_(heading), heading_valid_(heading_valid) {
        status_ = GpsStatusType(status);
    }
    GNSS(sensor_msgs::NavSatFix::Ptr msg) {
        unix_time_ = msg->header.stamp.toSec();
        if (int(msg->status.status) >= int(sensor_msgs::NavSatStatus::STATUS_FIX)) {
            status_ = GpsStatusType::GNSS_FIXED_SOLUTION;
        } else {
            status_ = GpsStatusType::GNSS_OTHER;
        }
        lat_lon_alt_ << msg->latitude, msg->longitude, msg->altitude;
    }

    double unix_time_ = 0;
    GpsStatusType status_ = GpsStatusType::GNSS_NOT_EXIST;
    Vec3d lat_lon_alt_ = Vec3d::Zero();  // 经纬度数、高度
    double heading_ = 0.0;               //  双天线读到的方位角
    bool heading_valid_ = false;         // 方位角是否有效
    UTMCoordinate utm_;
    bool utm_valid_ = false;

    SE3 utm_pose_;  // 处理后6 DoF pose
};

}  // namespace lh

using GNSSPtr = std::shared_ptr<lh::GNSS>;

#endif