#include "imu_gnss/utm_convert.hh"
#include "common/math_utils.h"
#include "utm_convert/utm.h"

#include <glog/logging.h>

namespace lh {

// 经纬度转utm
bool LatLon2UTM(const Vec2d& latlon, UTMCoordinate& utm_coor) {
    long zone = 0;
    char char_north = 0;
    long ret =
        Convert_Geodetic_To_UTM(latlon[0] * math::kDEG2RAD, latlon[1] * math::kDEG2RAD, &zone, &char_north, &utm_coor.xy_[0], &utm_coor.xy_[1]);

    utm_coor.zone_ = static_cast<int>(zone);
    utm_coor.north_ = char_north == 'N';

    return ret == 0;
}

bool UTM2LatLon(const UTMCoordinate& utm_coor, Vec2d& latlon) {
    bool ret = Convert_UTM_To_Geodetic((long)utm_coor.zone_, utm_coor.north_ ? 'N' : 'S', utm_coor.xy_[0], utm_coor.xy_[1], &latlon[0], &latlon[1]);
    latlon *= math::kRAD2DEG;
    return ret == 0;
}

bool ConvertGps2UTM(GNSS& gnss_reading, const Vec2d& antenna_pos, const double antenna_angle, const Vec3d& map_origin) {
    UTMCoordinate utm_rtk;
    if (!LatLon2UTM(gnss_reading.lat_lon_alt_.head<2>(), utm_rtk)) {
        return false;
    }
    utm_rtk.zone_ = gnss_reading.lat_lon_alt_[2];

    // rtk的有朝向信息时, 北东地转东北天
    double heading = 0;
    if (gnss_reading.heading_valid_) {
        heading = (90 - gnss_reading.heading_) * math::kDEG2RAD;
    }

    // T_WB = T_WG * T_GB
    SE3 TBG(SO3::rotZ(antenna_angle * math::kDEG2RAD), Vec3d(antenna_pos[0], antenna_pos[1], 0));
    SE3 TGB = TBG.inverse();

    // 减去原点
    double x = utm_rtk.xy_[0] - map_origin[0];
    double y = utm_rtk.xy_[1] - map_origin[1];
    double z = utm_rtk.z_ - map_origin[2];

    SE3 TWG(SO3::rotZ(heading * math::kDEG2RAD), Vec3d(x, y, z));
    SE3 TWB = TWG * TGB;

    // 把TWB 写到gnss类型中
    gnss_reading.utm_valid_ = true;
    gnss_reading.utm_.xy_[0] = TWB.translation().x();
    gnss_reading.utm_.xy_[1] = TWB.translation().y();
    gnss_reading.utm_.z_ = TWB.translation().z();

    if (gnss_reading.heading_valid_) {
        gnss_reading.utm_pose_ = TWB;
    } else {
        gnss_reading.utm_pose_ = SE3(SO3(), TWB.translation());
    }
    return true;
}

bool ConvertGps2UTMOnlyTrans(GNSS& gps_msg) {
    /// 经纬高转换为UTM
    UTMCoordinate utm_rtk;
    LatLon2UTM(gps_msg.lat_lon_alt_.head<2>(), utm_rtk);
    gps_msg.utm_valid_ = true;
    gps_msg.utm_.xy_ = utm_rtk.xy_;
    gps_msg.utm_.z_ = gps_msg.lat_lon_alt_[2];
    gps_msg.utm_pose_ = SE3(SO3(), Vec3d(gps_msg.utm_.xy_[0], gps_msg.utm_.xy_[1], gps_msg.utm_.z_));
    return true;
}

}  // namespace lh