#pragma once
#include <lanelet2_core/primitives/GPSPoint.h>
#include <lanelet2_core/primitives/Point.h>
#include <memory>
#include <vector>

namespace lanelet {
//!< When transforming between global and local coordinates, this offset will be
//!< applied to the lat/lon coordinates
struct Origin {
  static Origin defaultOrigin() { return {}; }
  GPSPoint position;
};

//! Base class for implementing projection models. To use your own projection,
//! implement this class and and pass it to the write/load functions.
class Projector {  // NOLINT
 public:
  using Ptr = std::shared_ptr<Projector>;
  explicit Projector(Origin origin = Origin::defaultOrigin()) : origin_{origin} {}
  virtual ~Projector() noexcept = default;

  //! @brief Project a point from lat/lon coordinates to a local coordinate
  //! system
  //! @throws ForwardProjectionError if projection is impossible
  virtual BasicPoint3d forward(const GPSPoint& p) const = 0;

  //! @brief Project a point from local coordinates to global lat/lon
  //! coordinates
  //! @throws ReverseProjectionError if projection is impossible
  virtual GPSPoint reverse(const BasicPoint3d& p) const = 0;

  //! Obtain the internal origin
  const Origin& origin() const { return origin_; }

 private:
  Origin origin_;
};

namespace projection {
/**
 * @brief implements a simple spherical mercator projection.
 *
 * Will be too unprecise for most calculations, but its fast and needs no
 * external dependencies. If you want more, check out lanelet2_projection!
 */
class SphericalMercatorProjector : public Projector {
 public:
  using Projector::Projector;
  BasicPoint3d forward(const GPSPoint& p) const override {
    const auto scale = std::cos(origin().position.lat * M_PI / 180.0);
    const double x{scale * p.lon * M_PI * EarthRadius / 180.0};
    const double y{scale * EarthRadius * std::log(std::tan((90.0 + p.lat) * M_PI / 360.0))};
    return {x, y, p.ele};
  }

  GPSPoint reverse(const BasicPoint3d& p) const override {
    const double scale = std::cos(origin().position.lat * M_PI / 180.0);
    const double lon = p.x() * 180.0 / (M_PI * EarthRadius * scale);
    const double lat = 360.0 * std::atan(std::exp(p.y() / (EarthRadius * scale))) / M_PI - 90.0;
    return {lat, lon, p.z()};
  }

 private:
  static constexpr double EarthRadius{6378137.0};
};
}  // namespace projection

using DefaultProjector = projection::SphericalMercatorProjector;

inline DefaultProjector defaultProjection(Origin origin = Origin::defaultOrigin()) { return DefaultProjector(origin); }
}  // namespace lanelet
