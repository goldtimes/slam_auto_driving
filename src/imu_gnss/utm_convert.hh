#pragma once
#include "common/gnss.h"

namespace lh {
bool ConvertGps2UTM(GNSS& gnss_reading, const Vec2d& antenna_pos, const double antenna_angle, const Vec3d& map_origin = Vec3d::Zero());

bool ConvertGps2UTMOnlyTrans(const GNSS& gnss_reading);

bool LatLon2UTM(const Vec2d& latlon, UTMCoordinate& utm_coor);

bool UTM2LatLon(const UTMCoordinate& utm_coor, Vec2d& latlon);
}  // namespace lh