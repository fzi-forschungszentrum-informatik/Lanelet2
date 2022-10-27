#include "lanelet2_projection/Geocentric.h"

#include <GeographicLib/Geocentric.hpp>

namespace lanelet {
namespace projection {

BasicPoint3d GeocentricProjector::forward(const GPSPoint& gps) const {
  BasicPoint3d ecef{0., 0., 0.};
  GeographicLib::Geocentric::WGS84().Forward(gps.lat,
                                           gps.lon,
                                           gps.ele,
                                           ecef[0],
                                           ecef[1],
                                           ecef[2]);
  return ecef;
}

GPSPoint GeocentricProjector::reverse(const BasicPoint3d& ecef) const {
  GPSPoint gps{0., 0., 0.};
  GeographicLib::Geocentric::WGS84().Reverse(ecef[0],
                                           ecef[1],
                                           ecef[2],
                                           gps.lat,
                                           gps.lon,
                                           gps.ele);
  return gps;
}

}  // namespace projection
}  // namespace lanelet
