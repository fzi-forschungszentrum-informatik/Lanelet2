#pragma once

#include <lanelet2_core/Forward.h>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace lanelet {
namespace routing {
class RouteElement;
using RouteElements = std::vector<RouteElement>;
using RouteElementRawPtrs = std::vector<RouteElement*>;
using RouteElementUPtr = std::unique_ptr<RouteElement>;
using RouteElementUPtrs = std::vector<RouteElementUPtr>;

using IdPair = std::pair<Id, Id>;

class Graph;

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
using ConstLaneletRouteElementMap = std::unordered_map<ConstLanelet, RouteElementUPtr>;
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

enum class RelationType : uint16_t {
  None = 0,                    //!< No relation
  Successor = 0b00000001,      //!< A (the only) direct, reachable successor. Not merging and not diverging.
  Left = 0b10,                 //!< (the only) directly adjacent, reachable left neighbour
  Right = 0b100,               //!< (the only) directly adjacent, reachable right neighbour
  Conflicting = 0b1000,        //!< Unreachable but with overlapping shape
  Merging = 0b10000,           //!< predecessor of a lanelet with multiple predecessors
  Diverging = 0b100000,        //!< successor of a lanelet with multiple successors
  AdjacentLeft = 0b1000000,    //!< directly adjacent, unreachable left neighbor
  AdjacentRight = 0b10000000,  //!< directly adjacent, unreachable right neighbor
  Area = 0b100000000           //!< Adjacent to a reachable area
};

constexpr RelationType allRelations() { return static_cast<RelationType>(0b111111111); }
static_assert(allRelations() > RelationType::Area, "allRelations is wrong!");

constexpr RelationType operator~(RelationType r) { return RelationType(~std::underlying_type_t<RelationType>(r)); }
constexpr RelationType operator&(RelationType r1, RelationType r2) {
  return RelationType(std::underlying_type_t<RelationType>(r1) & std::underlying_type_t<RelationType>(r2));
}
constexpr RelationType operator&=(RelationType& r1, RelationType r2) { return r1 = r1 & r2; }
constexpr RelationType operator|(RelationType r1, RelationType r2) {
  return RelationType(std::underlying_type_t<RelationType>(r1) | std::underlying_type_t<RelationType>(r2));
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
    case RelationType::Conflicting:
      return "Conflicting";
    case RelationType::Merging:
      return "Merging";
    case RelationType::Diverging:
      return "Diverging";
    case RelationType::AdjacentLeft:
      return "AdjacentLeft";
    case RelationType::AdjacentRight:
      return "AdjacentRight";
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
    case RelationType::Merging:
      return "orange";
    case RelationType::Diverging:
      return "orange";
    case RelationType::AdjacentLeft:
      return "black";
    case RelationType::AdjacentRight:
      return "black";
    case RelationType::Area:
      return "yellow";
  }
  return "";  // some compilers need that
}
}  // namespace routing
}  // namespace lanelet
