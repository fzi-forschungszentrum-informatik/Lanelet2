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

//! Represents the relation of a lanelet to another lanelet
struct LaneletRelation {
  ConstLanelet lanelet;       //!< the lanelet this relation refers to
  RelationType relationType;  //!< the type of relation to that
};
inline bool operator==(const LaneletRelation& lhs, const LaneletRelation& rhs) {
  return lhs.lanelet == rhs.lanelet && lhs.relationType == rhs.relationType;
}
inline bool operator!=(const LaneletRelation& rhs, const LaneletRelation& lhs) { return !(rhs == lhs); }

using LaneletRelations = std::vector<LaneletRelation>;

}  // namespace map_learning
}  // namespace lanelet
