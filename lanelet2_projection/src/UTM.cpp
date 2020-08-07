#include "lanelet2_projection/UTM.h"

#include <GeographicLib/UTMUPS.hpp>

namespace lanelet {
namespace projection {

UtmProjector::UtmProjector(Origin origin, const bool useOffset, const bool throwInPaddingArea)
    : Projector(origin), useOffset_{useOffset}, throwInPaddingArea_{throwInPaddingArea} {
  double x = 0;
  double y = 0;
  GeographicLib::UTMUPS::Forward(this->origin().position.lat, this->origin().position.lon, zone_,
                                 isInNorthernHemisphere_, x, y);
  if (useOffset_) {
    xOffset_ = x;
    yOffset_ = y;
  }
}

BasicPoint3d UtmProjector::forward(const GPSPoint& gps) const {
  BasicPoint3d utm{0., 0., gps.ele};
  int zone{};
  bool northp{};
  try {
    GeographicLib::UTMUPS::Forward(gps.lat, gps.lon, zone, northp, utm.x(), utm.y());
  } catch (GeographicLib::GeographicErr& e) {
    throw ForwardProjectionError(e.what());
  }

  if (zone != zone_ || northp != isInNorthernHemisphere_) {
    if (throwInPaddingArea_) {
      throw ForwardProjectionError("You have left the UTM zone or changed the hemisphere!");
    }
    // try to transfer to the desired zone
    double xAfterTransfer = 0;
    double yAfterTransfer = 0;
    int zoneAfterTransfer = 0;
    try {
      GeographicLib::UTMUPS::Transfer(zone, northp, utm.x(), utm.y(), zone_, isInNorthernHemisphere_, xAfterTransfer,
                                      yAfterTransfer, zoneAfterTransfer);
    } catch (GeographicLib::GeographicErr& e) {
      throw ForwardProjectionError(e.what());
    }

    if (zoneAfterTransfer != zone_) {
      throw ForwardProjectionError("You have left the padding area of the UTM zone!");
    }
    utm.x() = xAfterTransfer;
    utm.y() = yAfterTransfer;
  }

  if (useOffset_) {
    utm.x() -= xOffset_;
    utm.y() -= yOffset_;
  }

  return utm;
}

GPSPoint UtmProjector::reverse(const BasicPoint3d& utm) const {
  GPSPoint gps{0., 0., utm.z()};
  try {
    GeographicLib::UTMUPS::Reverse(zone_, isInNorthernHemisphere_, useOffset_ ? utm.x() + xOffset_ : utm.x(),
                                   useOffset_ ? utm.y() + yOffset_ : utm.y(), gps.lat, gps.lon);
  } catch (GeographicLib::GeographicErr& e) {
    throw ReverseProjectionError(e.what());
  }

  if (throwInPaddingArea_) {
    // for zone compliance testing:
    try {
      forward(gps);
    } catch (ForwardProjectionError& e) {
      throw ReverseProjectionError(e.what());
    };
  }
  return gps;
}

}  // namespace projection
}  // namespace lanelet
