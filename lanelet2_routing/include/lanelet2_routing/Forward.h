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

struct Graph;
struct FilteredGraphContainer;

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

enum class RelationType { Successor, Left, Right, Conflicting, Merging, Diverging, AdjacentLeft, AdjacentRight, Area };
using RelationTypes = std::vector<RelationType>;

constexpr inline size_t numRelationTypes() { return static_cast<size_t>(RelationType::Area) + 1; }

// Used for graph export
inline std::string relationToString(RelationType type) {
  switch (type) {
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
