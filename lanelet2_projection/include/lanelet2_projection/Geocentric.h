#pragma once
#include <lanelet2_io/Projection.h>

namespace lanelet {
namespace projection {

class GeocentricProjector : public Projector {
 public:
  // initialize the origin so that it's not the default one which causes
  // IOHandler::handleDefaultProjector to throw an exception
  GeocentricProjector() : Projector{Origin({90.0, 0.0, -6356752.3})} {}
  BasicPoint3d forward(const GPSPoint& gps) const override;
  GPSPoint reverse(const BasicPoint3d& enu) const override;
};

}  // namespace projection
}  // namespace lanelet
