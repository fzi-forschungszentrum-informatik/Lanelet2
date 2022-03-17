#pragma once
#include <lanelet2_io/Exceptions.h>
#include <lanelet2_io/Projection.h>

#include <GeographicLib/Geocentric.hpp>

namespace lanelet {
namespace projection {

class GeocentricProjector : public Projector {
 public:
  BasicPoint3d forward(const GPSPoint& gps) const override;
  GPSPoint reverse(const BasicPoint3d& enu) const override;
};

}  // namespace projection
}  // namespace lanelet
