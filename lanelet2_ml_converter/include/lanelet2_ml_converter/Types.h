#pragma once
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LaneletOrArea.h>

#include <boost/geometry.hpp>
#include <functional>
#include <type_traits>

#include "lanelet2_ml_converter/Forward.h"

namespace lanelet {
namespace ml_converter {

enum class LaneletRepresentationType { Centerline, Boundaries };
enum class ParametrizationType { Bezier, BezierEndpointFixed, LineString };
enum class LineStringType {
  RoadBorder,
  Dashed,
  Solid,
  Mixed,
  Virtual,
  Centerline,
  Unknown
};  // Mixed == DashedSolid or SolidDashed
enum class TEType { TrafficLight, TrafficSign, Unknown };

struct OrientedRect {
  BasicPoint3d center;
  double yaw;
  bool from2d{false};
  boost::geometry::model::polygon<BasicPoint2d> bg_poly;
  boost::geometry::model::polygon<BasicPoint2d>::ring_type& bounds() { return bg_poly.outer(); }
  const boost::geometry::model::polygon<BasicPoint2d>::ring_type& bounds_const() { return bg_poly.outer(); }

  friend OrientedRect getRotatedRect(const BasicPoint3d& center, double extentLongitudinal, double extentLateral,
                                     double yaw, bool from2dPos);

 private:
  OrientedRect() noexcept {}
};

}  // namespace ml_converter
}  // namespace lanelet
