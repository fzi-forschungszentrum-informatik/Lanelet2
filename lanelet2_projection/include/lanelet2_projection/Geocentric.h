#pragma once
#include <lanelet2_io/Projection.h>

namespace lanelet {
namespace projection {

class GeocentricProjector : public Projector {
 public:
  BasicPoint3d forward(const GPSPoint& gps) const override;
  GPSPoint reverse(const BasicPoint3d& enu) const override;
};

}  // namespace projection
}  // namespace lanelet
