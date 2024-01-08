#pragma once
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LaneletOrArea.h>

#include <boost/geometry.hpp>
#include <functional>
#include <type_traits>

#include "lanelet2_map_learning/Forward.h"

namespace lanelet {
namespace map_learning {

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

// wrapper for boost::python reasons
struct OrientedRect {
  boost::geometry::model::polygon<BasicPoint2d> bg_poly;
  boost::geometry::model::polygon<BasicPoint2d>::ring_type& bounds() { return bg_poly.outer(); }
  const boost::geometry::model::polygon<BasicPoint2d>::ring_type& bounds_const() { return bg_poly.outer(); }
};

}  // namespace map_learning
}  // namespace lanelet
