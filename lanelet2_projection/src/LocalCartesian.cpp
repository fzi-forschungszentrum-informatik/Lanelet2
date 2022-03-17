#include "lanelet2_projection/LocalCartesian.h"

namespace lanelet {
namespace projection {

LocalCartesianProjector::LocalCartesianProjector(Origin origin) :
      Projector(origin),
      localCartesian(origin.position.lat, origin.position.lon, origin.position.ele, GeographicLib::Geocentric::WGS84()) {
}

BasicPoint3d LocalCartesianProjector::forward(const GPSPoint& gps) const {
  BasicPoint3d local{0., 0., 0.};
  this->localCartesian.Forward(gps.lat,
                               gps.lon,
                               gps.ele,
                               local[0],
                               local[1],
                               local[2]);
  return local;
}

GPSPoint LocalCartesianProjector::reverse(const BasicPoint3d& local) const {
  GPSPoint gps{0., 0., 0.};
  this->localCartesian.Reverse(local[0],
                               local[1],
                               local[2],
                               gps.lat,
                               gps.lon,
                               gps.ele);
  return gps;
}

}  // namespace projection
}  // namespace lanelet
