#pragma once
#include <lanelet2_core/primitives/GPSPoint.h>
#include <lanelet2_core/primitives/Point.h>
#include <memory>
#include <vector>

namespace lanelet {
//!< When transforming between global and local coordinates, this offset will be applied to the lat/lon coordinates.
//! The object is also used to determine a senseful local coordinate system (e.g. choosing the correct UTM tile).
struct Origin {
  explicit Origin(const GPSPoint& position) : position{position}, isDefault{false} {}
  Origin() = default;

  static Origin defaultOrigin() { return {}; }
  GPSPoint position;     //! The position of the origin
  bool isDefault{true};  //! The Parser/Writer check for this if appropriate to make sure they are using a valid origin
};

//! Base class for implementing projection models. To use your own projection,
//! implement this class and and pass it to the write/load functions.
class Projector {  // NOLINT
 public:
  using Ptr = std::shared_ptr<Projector>;
  explicit Projector(Origin origin = Origin::defaultOrigin(),
              const std::string& pName = "") : origin_{origin}, name_{pName} {}
  Projector(Projector&& rhs) noexcept = default;
  Projector& operator=(Projector&& rhs) noexcept = default;
  Projector(const Projector& rhs) = default;
  Projector& operator=(const Projector& rhs) = default;
  virtual ~Projector() noexcept = default;

  //! @brief Project a point from lat/lon coordinates to a local coordinate system
  //! @throws ForwardProjectionError if projection is impossible
  virtual BasicPoint3d forward(const GPSPoint& p) const = 0;

  //! @brief Project a point from local coordinates to global lat/lon coordinates
  //! @throws ReverseProjectionError if projection is impossible
  virtual GPSPoint reverse(const BasicPoint3d& p) const = 0;

  //! Obtain the internal origin
  const Origin& origin() const { return origin_; }

  //! Get the name of the type of projector in use (internal use mainly)
  const std::string & name() const { return name_;}

 private:
  Origin origin_;
  std::string name_;
};

namespace projection {
/**
 * @brief implements a simple spherical mercator projection.
 *
 * Will be too unprecise for most calculations, but its fast and needs no external dependencies. If you want more, check
 * out lanelet2_projection! This is the same projection that was used by lanelet1.
 */
class SphericalMercatorProjector : public Projector {
 public:
  using Projector::Projector;
  explicit SphericalMercatorProjector(Origin origin = Origin::defaultOrigin()) :
                    Projector(origin, "SphericalMercatorProjector") {}
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


/**
 * @brief a null class to allow input points that are already in geo-spatial coordinates.
 *
 * It will not operate any convertion to local/utm/other coordinate system
 */
class NullProjector : public Projector {
 public:
  using Projector::Projector;
  explicit NullProjector(Origin origin = Origin::defaultOrigin()) :
                  Projector(origin, "NullProjector") {}
  BasicPoint3d forward(const GPSPoint& p) const override {
    return {p.lat, p.lon, p.ele};
  }

  GPSPoint reverse(const BasicPoint3d& p) const override {
    return {p.x(), p.y(), p.z()};
  }
};


}  // namespace projection

using DefaultProjector = projection::SphericalMercatorProjector;

inline DefaultProjector defaultProjection(Origin origin = Origin::defaultOrigin()) { return DefaultProjector(origin); }
}  // namespace lanelet
