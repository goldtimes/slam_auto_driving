/**
 * 因为是要学习多传感器的融合定位，而不是gps的读数
 */
#include "ch3/utm_convert.h"
#include "common/math_utils.h"
#include "utm_convert/utm.h"

#include <glog/logging.h>

namespace lh {
bool LatLon2UTM(const Vec2d& latlon, UTMCoordinate& utm_coor) {
    long zone = 0;
    char char_north = 0;
    // 经纬度转换成utm坐标
    long ret = Convert_Geodetic_To_UTM(latlon[0] * math::kDEG2RAD, latlon[1] * math::kDEG2RAD, &zone, &char_north,
                                       &utm_coor.xy_[0], &utm_coor.xy_[1]);
    utm_coor.zone_ = (int)zone;
    utm_coor.north_ = char_north == 'N';

    return ret == 0;
}

bool UTM2LatLon(const UTMCoordinate& utm_coor, Vec2d& latlon) {
    bool ret = Convert_UTM_To_Geodetic((long)utm_coor.zone_, utm_coor.north_ ? 'N' : 'S', utm_coor.xy_[0],
                                       utm_coor.xy_[1], &latlon[0], &latlon[1]);
    latlon *= math::kRAD2DEG;
    return ret == 0;
}

bool ConvertGps2UTM(GNSS& gps_msg, const Vec2d& antenna_pos, const double& antenna_angle, const Vec3d& map_origin) {
    // 经纬高转utm
    UTMCoordinate utm_rtk;
    if (!LatLon2UTM(gps_msg.lat_lon_alt_.head<2>(), utm_rtk)) {
        return false;
    }
    // 高度
    utm_rtk.z_ = gps_msg.lat_lon_alt_[2];
    // gps heading
    double heading = 0;
    if (gps_msg.heading_valid_) {
        heading = (90 - gps_msg.heading_) * math::kDEG2RAD;  // 北东地->东北地
    }

    // TWG -> TWB  T_GtoW * T_BtoG = T_BtoW
    // gps的安装角度和安装平移,构造SE3
    SE3 T_GtoB(SO3::rotZ(antenna_angle * math::kDEG2RAD), Vec3d(antenna_pos[0], antenna_pos[1], 0));
    SE3 T_BtoG = T_GtoB.inverse();

    // 指定了地图原点
    double x = utm_rtk.xy_[0] - map_origin[0];
    double y = utm_rtk.xy_[1] - map_origin[1];
    double z = utm_rtk.z_ - map_origin[2];

    SE3 T_GtoW(SO3::rotZ(heading), Vec3d(x, y, z));
    SE3 T_BtoW = T_GtoW * T_BtoG;

    gps_msg.utm_valid_ = true;
    gps_msg.utm_.xy_[0] = T_BtoW.translation().x();
    gps_msg.utm_.xy_[1] = T_BtoW.translation().y();
    gps_msg.utm_.z_ = T_BtoW.translation().z();

    if (gps_msg.heading_valid_) {
        // gps的读数是T_GtoW的，我们将这个读数转到T_BtoW下，并记录
        gps_msg.utm_pose_ = T_BtoW;
    } else {
        gps_msg.utm_pose_ = SE3(SO3(), T_BtoW.translation());
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