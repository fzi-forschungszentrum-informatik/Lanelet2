#pragma once
#include <lanelet2_io/Exceptions.h>
#include <lanelet2_io/Projection.h>

namespace lanelet {
namespace projection {
class UtmProjector : public Projector {
 public:
  explicit UtmProjector(Origin origin, bool useOffset = true, bool throwInPaddingArea = false);

  BasicPoint3d forward(const GPSPoint& gps) const override;

  GPSPoint reverse(const BasicPoint3d& utm) const override;

  //! UTM zone of the origin, all points are projected into this zone
  int zone() const { return zone_; }
  //! true if the origin is in the northern hemisphere
  bool isInNorthernHemisphere() const { return isInNorthernHemisphere_; }
  //! UTM coordinates of the origin that are subtracted from the projected points ((0, 0) if useOffset is false)
  BasicPoint2d offset() const { return {xOffset_, yOffset_}; }

 private:
  int zone_{};
  bool isInNorthernHemisphere_{true}, useOffset_{}, throwInPaddingArea_{};
  double xOffset_{}, yOffset_{};
};

}  // namespace projection
}  // namespace lanelet
