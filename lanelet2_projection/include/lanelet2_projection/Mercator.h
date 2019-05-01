#pragma once
#include <lanelet2_io/Projection.h>

namespace lanelet {
namespace projection {

//! Implements the usual elliptical mercator projection
//! This is the same transformation used by liblanelet(1).
class Mercator : public Projector {
 public:
  explicit Mercator(const Origin& origin = Origin::defaultOrigin())
      : Projector(origin), offset_{rawForward(origin.position)} {}

  BasicPoint3d forward(const GPSPoint& pGps) const override { return rawForward(pGps) - offset_; }
  GPSPoint reverse(const BasicPoint3d& p) const override { return rawReverse(p + offset_); }

 private:
  static BasicPoint3d rawForward(const GPSPoint& p) {
    double lat = std::min(89.5, std::max(p.lat, -89.5));
    double phi = lat * M_PI / 180.0;
    double con = Eccent * std::sin(phi);
    con = std::pow((1.0 - con) / (1.0 + con), 0.5 * Eccent);
    double ts = std::tan(0.5 * (M_PI * 0.5 - phi)) / con;
    const double y = -RMajor * std::log(ts);
    const double x = RMajor * p.lon * M_PI / 180.;
    return {x, y, p.ele};
  }
  static GPSPoint rawReverse(const BasicPoint3d& p) {
    double ts = std::exp(-p.y() / RMajor);
    double phi = M_PI / 2 - 2 * std::atan(ts);
    double dphi = 1.0;
    for (int i = 0; fabs(dphi) > 0.000000001 && i < 15; i++) {
      double con = Eccent * sin(phi);
      dphi = M_PI / 2 - 2 * atan(ts * pow((1.0 - con) / (1.0 + con), 0.5 * Eccent)) - phi;
      phi += dphi;
    }
    const double lat = 180 / M_PI * phi;
    const double lon = 180 / M_PI * p.x() / RMajor;
    return {lat, lon, p.z()};
  }

  BasicPoint3d offset_{0, 0, 0};
  static constexpr double RMajor{6378137.0};
  static constexpr double RMinor{6356752.3142};
  static constexpr double Eccent{0.081819190928906924};  //=std::sqrt(1.0 - RMinor*RMinor/RMajor/RMajor)
};

}  // namespace projection
}  // namespace lanelet
