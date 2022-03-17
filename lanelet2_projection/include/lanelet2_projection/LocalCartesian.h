#pragma once
#include <lanelet2_io/Exceptions.h>
#include <lanelet2_io/Projection.h>

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

namespace lanelet {
namespace projection {
class LocalCartesianProjector : public Projector {
 public:
  explicit LocalCartesianProjector(Origin origin);

  BasicPoint3d forward(const GPSPoint& gps) const override;

  GPSPoint reverse(const BasicPoint3d& enu) const override;
 private:
  GeographicLib::LocalCartesian localCartesian;
};

}  // namespace projection
}  // namespace lanelet
