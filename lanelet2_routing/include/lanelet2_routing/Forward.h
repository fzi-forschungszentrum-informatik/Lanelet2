#pragma once

#include <lanelet2_core/Forward.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace lanelet {
namespace routing {
namespace internal {

using IdPair = std::pair<Id, Id>;

template <typename BaseGraphT>
class Graph;

class RoutingGraphGraph;
class RouteGraph;
}  // namespace internal

using LaneId = uint16_t;
class RoutingGraph;
using RoutingGraphPtr = std::shared_ptr<RoutingGraph>;
using RoutingGraphUPtr = std::unique_ptr<RoutingGraph>;
using RoutingGraphConstPtr = std::shared_ptr<const RoutingGraph>;

class RoutingGraphContainer;
using RoutingGraphContainerUPtr = std::unique_ptr<RoutingGraphContainer>;

class Route;
struct LaneletRelation;
using LaneletRelations = std::vector<LaneletRelation>;

using RouteUPtr = std::unique_ptr<Route>;
using Routes = std::vector<Route>;
class RoutingCost;
using RoutingCostPtr = std::shared_ptr<RoutingCost>;
using RoutingCostUPtr = std::unique_ptr<RoutingCost>;
using RoutingCosts = std::vector<RoutingCost>;
using RoutingCostUPtrs = std::vector<RoutingCostUPtr>;
using RoutingCostPtrs = std::vector<RoutingCostPtr>;
using RoutingCostId = uint16_t;

class LaneletPath;
using LaneletPaths = std::vector<LaneletPath>;
class LaneletOrAreaPath;
using LaneletOrAreaPaths = std::vector<LaneletOrAreaPath>;

//! This enum expresses the types of relations lanelet2 distiguishes internally. Between two lanelets a and b (in this
//! order), exactly one of these relation exists.
//!
//! @note The relation between b and a is different than between a and b. There is also no obvious
//! symmetry. When a is left of b, b can be either right or adjacent right to b.
enum class RelationType : uint8_t {
  None = 0,                 //!< No relation
  Successor = 0b1,          //!< A (the only) direct, reachable successor. Not merging and not diverging.
  Left = 0b10,              //!< (the only) directly adjacent, reachable left neighbour
  Right = 0b100,            //!< (the only) directly adjacent, reachable right neighbour
  AdjacentLeft = 0b1000,    //!< directly adjacent, unreachable left neighbor
  AdjacentRight = 0b10000,  //!< directly adjacent, unreachable right neighbor
  Conflicting = 0b100000,   //!< Unreachable but with overlapping shape
  Area = 0b1000000          //!< Adjacent to a reachable area
};

using RelationUnderlyingType = std::underlying_type_t<RelationType>;

constexpr RelationType allRelations() { return static_cast<RelationType>(0b1111111); }
static_assert(allRelations() > RelationType::Area, "allRelations is wrong!");

constexpr RelationType operator~(RelationType r) { return RelationType(~RelationUnderlyingType(r)); }
constexpr RelationType operator&(RelationType r1, RelationType r2) {
  return RelationType(RelationUnderlyingType(r1) & RelationUnderlyingType(r2));
}
constexpr RelationType operator&=(RelationType& r1, RelationType r2) { return r1 = r1 & r2; }
constexpr RelationType operator|(RelationType r1, RelationType r2) {
  return RelationType(std::underlying_type_t<RelationType>(r1) | RelationUnderlyingType(r2));
}
constexpr RelationType operator|=(RelationType& r1, RelationType r2) { return r1 = r1 | r2; }

// Used for graph export
inline std::string relationToString(RelationType type) {
  switch (type) {
    case RelationType::None:
      return "None";
    case RelationType::Successor:
      return "Successor";
    case RelationType::Left:
      return "Left";
    case RelationType::Right:
      return "Right";
    case RelationType::AdjacentLeft:
      return "AdjacentLeft";
    case RelationType::AdjacentRight:
      return "AdjacentRight";
    case RelationType::Conflicting:
      return "Conflicting";
    case RelationType::Area:
      return "Area";
  }
  return "";  // some compilers need that
}

inline std::string relationToColor(RelationType type) {
  switch (type) {
    case RelationType::None:
      return "";
    case RelationType::Successor:
      return "green";
    case RelationType::Left:
      return "blue";
    case RelationType::Right:
      return "magenta";
    case RelationType::Conflicting:
      return "red";
    case RelationType::AdjacentLeft:
    case RelationType::AdjacentRight:
      return "black";
    case RelationType::Area:
      return "yellow";
  }
  return "";  // some compilers need that
}
}  // namespace routing
}  // namespace lanelet
