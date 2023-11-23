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
enum class LineStringType { RoadBorder, Dashed, Solid, Virtual, Centerline, Unknown };
enum class TEType { TrafficLight, TrafficSign, Unknown };

using OrientedRect = boost::geometry::model::polygon<BasicPoint2d>;

}  // namespace map_learning
}  // namespace lanelet
