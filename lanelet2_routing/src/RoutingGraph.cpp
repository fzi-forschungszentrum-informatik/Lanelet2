#include "RoutingGraph.h"
#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/Exceptions.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/Area.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_traffic_rules/TrafficRules.h>
#include <algorithm>
#include <cassert>  // Asserts
#include <memory>
#include <queue>
#include <utility>
#include "Exceptions.h"
#include "Forward.h"
#include "Graph.h"
#include "Route.h"
#include "ShortestPath.h"
#include "internal/RouteBuilder.h"
#include "internal/RoutingGraphBuilder.h"

#include "RoutingGraphVisualization.h"
namespace lanelet {
namespace routing {

#if __cplusplus < 201703L
constexpr const char RoutingGraph::ParticipantHeight[];
#endif

namespace {
template <typename T>
T reservedVector(size_t size) {
  T t;
  t.reserve(size);
  return t;
}

//! Helper function to get all relation types except the provided ones.
RelationTypes getRemainingRelations(const RelationTypes& typesToExclude) {
  assert(std::is_sorted(typesToExclude.begin(), typesToExclude.end()));
  RelationTypes relations;
  for (size_t it = 0; it < numRelationTypes(); it++) {
    auto relation = static_cast<RelationType>(it);
    if (!std::binary_search(typesToExclude.begin(), typesToExclude.end(), relation)) {
      relations.emplace_back(relation);
    }
  }
  return relations;
}

ConstLanelets toLanelets(const ConstLaneletOrAreas& la) {
  return utils::transform(la, [](auto& la) { return static_cast<const ConstLanelet&>(la); });
}

/** @brief Helper objecct to save state of possible routes search */
struct PossibleRoutesInfo;
using PossibleRoutesInfoUPtr = std::unique_ptr<PossibleRoutesInfo>;
struct PossibleRoutesInfo {
  uint32_t vertexId;
  double totalDistance{0};
  size_t numLaneChanges{0};
  ConstLaneletOrAreas laneletsOrAreas;
  explicit PossibleRoutesInfo(uint32_t startVertex, const ConstLaneletOrArea& startLanelet) : vertexId(startVertex) {
    laneletsOrAreas.emplace_back(startLanelet);
  }
};

//! sort functor for possible routes
struct MoreLaneChanges {
  bool operator()(const PossibleRoutesInfo& lhs, const PossibleRoutesInfo& rhs) const {
    if (lhs.numLaneChanges == rhs.numLaneChanges) {
      return lhs.totalDistance > rhs.totalDistance;
    }
    return lhs.numLaneChanges > rhs.numLaneChanges;
  }
};

/** @brief Simple helper function to combine shortest paths */
bool addToPath(ConstLanelets& path, const Optional<LaneletPath>& newElements) {
  if (newElements) {
    path.insert(path.end(), ++newElements->begin(), newElements->end());
    return true;  // NOLINT
  }
  return false;
}

//! Helper function to create a new point that represents a lanelet.
template <typename PointT>
PointT createPoint(const ConstLaneletOrArea& ll) {
  PointT p;
  p.setId(ll.id());
  p.setAttribute("id", Attribute(ll.id()));
  if (ll.isLanelet()) {
    boost::geometry::centroid(utils::toHybrid(ll.lanelet()->polygon2d()), p);
  }
  if (ll.isArea()) {
    boost::geometry::centroid(utils::toHybrid(utils::to2D(ll.area()->outerBoundPolygon())), p);
  }
  return p;
}

/** @brief Implementation function to retrieve a neighboring vertex
 *  @throws RoutingGraphError if 'throwOnError' is true and there is more than one neighboring lanelet
 *  @param vertex Start vertex
 *  @param graph Filtered graph with the allowed type of edge
 *  @param throwOnError Decides wheter to throw or ignore an error
 *  @return Neighboring lanelet */
Optional<ConstLaneletOrArea> neighboringImpl(const GraphType::vertex_descriptor vertex, const FilteredGraph& graph,
                                             bool throwOnError = false) {
  std::pair<FilteredGraphTraits::out_edge_iterator, FilteredGraphTraits::out_edge_iterator> outEdges =
      boost::out_edges(vertex, graph);
  if (throwOnError && std::distance(outEdges.first, outEdges.second) > 1) {
    std::string ids;
    std::for_each(outEdges.first, outEdges.second, [&graph, &ids](const auto& edge) {
      ids += " " + std::to_string(graph[boost::target(edge, graph)].laneletOrArea.id());
    });
    throw RoutingGraphError("More than one neighboring lanelet to " + std::to_string(graph[vertex].laneletOrArea.id()) +
                            " with this relation:" + ids);
  }
  if (outEdges.first != outEdges.second) {
    return graph[boost::target(*(outEdges.first), graph)].laneletOrArea;
  }
  return {};
}

Optional<ConstLanelet> neighboringLaneletImpl(const GraphType::vertex_descriptor vertex, const FilteredGraph& graph,
                                              bool throwOnError = false) {
  auto value = neighboringImpl(vertex, graph, throwOnError);
  if (!!value && value->isLanelet()) {
    return value->lanelet();
  }
  return {};
}

template <typename Func>
Optional<ConstLaneletOrArea> ifInGraph(const Graph& g, const ConstLaneletOrArea& llt, Func f) {
  auto vertex = g.getVertex(llt);
  if (!vertex) {
    return {};
  }
  return f(*vertex);
}

template <typename Func>
Optional<ConstLanelet> ifLaneletInGraph(const Graph& g, const ConstLanelet& llt, Func f) {
  auto laneletVertex = g.getVertex(llt);
  if (!laneletVertex) {
    return {};
  }
  return f(*laneletVertex);
}

template <typename Func>
ConstLanelets getUntilEnd(const ConstLanelet& start, Func next) {
  auto result = reservedVector<ConstLanelets>(3);
  Optional<ConstLanelet> current = start;
  while (!!(current = next(*current))) {
    result.emplace_back(*current);
  }
  return result;
}

ConstLaneletOrAreas getAllEdgesFromGraph(const Graph& graph, const FilteredGraph& subgraph,
                                         const ConstLaneletOrArea& laneletOrArea, bool edgesOut) {
  ConstLaneletOrAreas result;
  auto laneletVertex = graph.getVertex(laneletOrArea);
  if (!laneletVertex) {
    return result;
  }
  auto processEdges = [&](auto edgeRange) {
    result.reserve(size_t(std::distance(edgeRange.first, edgeRange.second)));
    for (; edgeRange.first != edgeRange.second; edgeRange.first++) {
      auto node =
          edgesOut ? boost::target(*edgeRange.first, graph.graph) : boost::source(*edgeRange.first, graph.graph);
      result.emplace_back(graph.graph[node].laneletOrArea);
    }
    return result;
  };
  return edgesOut ? processEdges(boost::out_edges(*laneletVertex, subgraph))
                  : processEdges(boost::in_edges(*laneletVertex, subgraph));
}

ConstLanelets getLaneletEdgesFromGraph(const Graph& graph, const FilteredGraph& subgraph, const ConstLanelet& lanelet,
                                       bool edgesOut) {
  ConstLanelets result;
  auto allEdges = getAllEdgesFromGraph(graph, subgraph, lanelet, edgesOut);
  result = reservedVector<ConstLanelets>(allEdges.size());
  for (auto& edge : allEdges) {
    if (edge.isLanelet()) {
      result.push_back(*edge.lanelet());
    }
  }
  return result;
}
}  // namespace

RoutingGraph::RoutingGraph(RoutingGraph&& /*other*/) noexcept = default;
RoutingGraph& RoutingGraph::operator=(RoutingGraph&& /*other*/) noexcept = default;
RoutingGraph::~RoutingGraph() = default;

RoutingGraphUPtr RoutingGraph::build(const LaneletMap& laneletMap, const traffic_rules::TrafficRules& trafficRules,
                                     const RoutingCostPtrs& routingCosts, const RoutingGraph::Configuration& config) {
  return RoutingGraphBuilder(trafficRules, routingCosts, config).build(laneletMap);
}

Optional<Route> RoutingGraph::getRoute(const ConstLanelet& from, const ConstLanelet& to,
                                       RoutingCostId routingCostId) const {
  auto optPath{shortestPath(from, to, routingCostId)};
  if (!optPath) {
    return {};
  };
  return RouteBuilder(*this).getRouteFromShortestPath(*optPath);
}

Optional<Route> RoutingGraph::getRouteVia(const ConstLanelet& from, const ConstLanelets& via, const ConstLanelet& to,
                                          RoutingCostId routingCostId) const {
  auto optPath{shortestPathVia(from, via, to, routingCostId)};
  if (!optPath) {
    return {};
  };
  return RouteBuilder(*this).getRouteFromShortestPath(*optPath);
}

Optional<LaneletPath> RoutingGraph::shortestPath(const ConstLanelet& from, const ConstLanelet& to,
                                                 RoutingCostId routingCostId) const {
  auto startVertex = graph_->getVertex(from);
  auto endVertex = graph_->getVertex(to);
  if (!startVertex || !endVertex) {
    return {};
  };
  ShortestPath<FilteredGraph> pathSearch(graph_->withLaneChanges(routingCostId), *startVertex);
  std::deque<GraphType::vertex_descriptor> vertexPath{pathSearch.shortestPath(*endVertex)};
  if (vertexPath.empty()) {
    return {};
  };
  return LaneletPath(utils::transform(vertexPath, [&graph = graph_->graph](auto& vi) { return graph[vi].lanelet(); }));
}

Optional<LaneletPath> RoutingGraph::shortestPathVia(const ConstLanelet& start, const ConstLanelets& via,
                                                    const ConstLanelet& end, RoutingCostId routingCostId) const {
  ConstLanelets routePoints = utils::concatenate({ConstLanelets{start}, via, ConstLanelets{end}});

  ConstLanelets path;
  for (size_t it = 0; it < routePoints.size() - 1; it++) {
    auto results = shortestPath(routePoints[it], routePoints[it + 1], routingCostId);
    if (!!results && !results->empty() && path.empty()) {
      path.push_back(results->front());
    }
    if (!addToPath(path, results)) {
      return {};
    };
  }
  return LaneletPath(path);
}

Optional<RelationType> RoutingGraph::routingRelation(const ConstLanelet& from, const ConstLanelet& to,
                                                     bool includeConflicting) const {
  auto edgeInfo = includeConflicting ? graph_->getEdgeInfo(from, to)
                                     : graph_->getEdgeInfoFor(from, to, graph_->filteredGraphs().withoutConflicting);
  if (!!edgeInfo) {
    return edgeInfo->relation;
  }
  return {};
}

ConstLanelets RoutingGraph::following(const ConstLanelet& lanelet, bool withLaneChanges) const {
  auto& subgraph = withLaneChanges ? graph_->withLaneChanges(0) : graph_->withoutLaneChanges(0);
  return getLaneletEdgesFromGraph(*graph_, subgraph, lanelet, true);
}

LaneletRelations RoutingGraph::followingRelations(const ConstLanelet& lanelet, bool withLaneChanges) const {
  ConstLanelets foll{following(lanelet, withLaneChanges)};
  LaneletRelations result;
  for (auto const& it : foll) {
    result.emplace_back(LaneletRelation{it, *routingRelation(lanelet, it)});
  }
  return result;
}  // namespace routing

ConstLanelets RoutingGraph::previous(const ConstLanelet& lanelet, bool withLaneChanges) const {
  auto& subgraph = withLaneChanges ? graph_->withLaneChanges(0) : graph_->withoutLaneChanges(0);
  return getLaneletEdgesFromGraph(*graph_, subgraph, lanelet, false);
}

LaneletRelations RoutingGraph::previousRelations(const ConstLanelet& lanelet, bool withLaneChanges) const {
  ConstLanelets prev{previous(lanelet, withLaneChanges)};
  LaneletRelations result;
  for (auto const& it : prev) {
    Optional<RelationType> relation{routingRelation(it, lanelet)};
    if (!!relation) {
      result.emplace_back(LaneletRelation{it, *relation});
    } else {
      assert(false && "Two Lanelets in a route are not connected. This shouldn't happen.");  // NOLINT
    }
  }
  return result;
}

ConstLanelets RoutingGraph::besides(const ConstLanelet& lanelet) const {
  auto move = [](auto it) { return std::make_move_iterator(it); };
  ConstLanelets left{lefts(lanelet)};
  ConstLanelets right{rights(lanelet)};
  ConstLanelets result;
  result.reserve(left.size() + right.size() + 1);
  result.insert(std::end(result), move(left.rbegin()), move(left.rend()));
  result.push_back(lanelet);
  result.insert(std::end(result), move(std::begin(right)), move(std::end(right)));
  return result;
}

Optional<ConstLanelet> RoutingGraph::left(const ConstLanelet& lanelet) const {
  return ifLaneletInGraph(
      *graph_, lanelet, [this](auto& vertex) { return neighboringLaneletImpl(vertex, graph_->filteredGraphs().left); });
}

Optional<ConstLanelet> RoutingGraph::adjacentLeft(const ConstLanelet& lanelet) const {
  return ifLaneletInGraph(*graph_, lanelet, [this](auto& vertex) {
    return neighboringLaneletImpl(vertex, graph_->filteredGraphs().adjacentLeft);
  });
}

Optional<ConstLanelet> RoutingGraph::right(const ConstLanelet& lanelet) const {
  return ifLaneletInGraph(*graph_, lanelet, [this](auto& vertex) {
    return neighboringLaneletImpl(vertex, graph_->filteredGraphs().right);
  });
}

Optional<ConstLanelet> RoutingGraph::adjacentRight(const ConstLanelet& lanelet) const {
  return ifLaneletInGraph(*graph_, lanelet, [this](auto& vertex) {
    return neighboringLaneletImpl(vertex, graph_->filteredGraphs().adjacentRight);
  });
}

ConstLanelets RoutingGraph::lefts(const ConstLanelet& lanelet) const {
  return getUntilEnd(lanelet, [this](const ConstLanelet& llt) { return left(llt); });
}

ConstLanelets RoutingGraph::adjacentLefts(const ConstLanelet& lanelet) const {
  return getUntilEnd(lanelet, [this](const ConstLanelet& llt) { return adjacentLeft(llt); });
}

LaneletRelations RoutingGraph::leftRelations(const ConstLanelet& lanelet) const {
  bool leftReached{false};
  ConstLanelet current = lanelet;
  LaneletRelations result;
  while (!leftReached) {
    const ConstLanelets leftOf{lefts(current)};
    for (auto const& it : leftOf) {
      result.emplace_back(LaneletRelation{it, RelationType::Left});
      current = it;
    }
    const ConstLanelets adjacentLeftOf{adjacentLefts(current)};
    for (auto const& it : adjacentLeftOf) {
      result.emplace_back(LaneletRelation{it, RelationType::AdjacentLeft});
      current = it;
    }
    leftReached = (leftOf.empty() && adjacentLeftOf.empty());
  }
  return result;
}

ConstLanelets RoutingGraph::rights(const ConstLanelet& lanelet) const {
  return getUntilEnd(lanelet, [this](const ConstLanelet& llt) { return right(llt); });
}

ConstLanelets RoutingGraph::adjacentRights(const ConstLanelet& lanelet) const {
  return getUntilEnd(lanelet, [this](const ConstLanelet& llt) { return adjacentRight(llt); });
}

LaneletRelations RoutingGraph::rightRelations(const ConstLanelet& lanelet) const {
  bool rightReached{false};
  ConstLanelet current = lanelet;
  auto result = reservedVector<LaneletRelations>(3);
  while (!rightReached) {
    const ConstLanelets rightOf{rights(current)};
    for (auto const& it : rightOf) {
      result.emplace_back(LaneletRelation{it, RelationType::Right});
      current = it;
    }
    const ConstLanelets adjacentRightOf{adjacentRights(current)};
    for (auto const& it : adjacentRightOf) {
      result.emplace_back(LaneletRelation{it, RelationType::AdjacentRight});
      current = it;
    }
    rightReached = (rightOf.empty() && adjacentRightOf.empty());
  }
  return result;
}

ConstLaneletOrAreas RoutingGraph::conflicting(const ConstLaneletOrArea& laneletOrArea) const {
  return getAllEdgesFromGraph(*graph_, graph_->filteredGraphs().conflicting, laneletOrArea, true);
}

template <typename Func>
std::vector<ConstLaneletOrAreas> possiblePathsImpl(const GraphType::vertex_descriptor& start,
                                                   const FilteredGraph& graph, bool keepBeforeStop,
                                                   Func stopCriterion) {
  std::vector<ConstLaneletOrAreas> result;
  std::priority_queue<PossibleRoutesInfo, std::vector<PossibleRoutesInfo>, MoreLaneChanges> candidates;
  candidates.push(PossibleRoutesInfo(static_cast<uint32_t>(start), graph[start].laneletOrArea));
  auto weights = boost::get(&EdgeInfo::routingCost, graph);
  FilteredGraph::out_edge_iterator edgesIt, edgesEnd;
  auto tryToVisit = [visited = std::set<Id>{graph[start].laneletOrArea.id()}](const ConstLaneletOrArea& la) mutable {
    if (visited.find(la.id()) != visited.end()) {
      return false;
    }
    visited.insert(la.id());
    return true;
  };
  while (!candidates.empty()) {
    PossibleRoutesInfo candidate = candidates.top();
    candidates.pop();
    bool continued = false;
    for (std::tie(edgesIt, edgesEnd) = out_edges(candidate.vertexId, graph); edgesIt != edgesEnd; ++edgesIt) {
      auto idx = boost::target(*edgesIt, graph);
      if (!tryToVisit(graph[idx].laneletOrArea)) {
        continue;
      }
      const auto cost = boost::get(weights, *edgesIt);
      PossibleRoutesInfo newCandidate{candidate};
      newCandidate.totalDistance += cost;
      newCandidate.vertexId = static_cast<uint32_t>(idx);
      newCandidate.laneletsOrAreas.emplace_back(graph[idx].laneletOrArea);
      newCandidate.numLaneChanges += graph[*edgesIt].relation != RelationType::Successor;
      if (stopCriterion(newCandidate)) {
        if (keepBeforeStop) {
          continue;
        }
        result.emplace_back(std::move(newCandidate.laneletsOrAreas));
      } else {
        candidates.push(std::move(newCandidate));
      }
      continued = true;
    }
    if (!continued && keepBeforeStop) {
      result.emplace_back(candidate.laneletsOrAreas);
    }
  }
  return result;
}

ConstLanelets RoutingGraph::reachableSet(const ConstLanelet& lanelet, double maxRoutingCost,
                                         RoutingCostId routingCostId) const {
  ConstLanelets result;
  auto start = graph_->getVertex(lanelet);
  if (!start) {
    return result;
  }
  auto stopIfMoreThanMinCost = [maxRoutingCost](const PossibleRoutesInfo& route) {
    return route.totalDistance > maxRoutingCost;
  };
  auto& graph = graph_->withLaneChanges(routingCostId);
  auto possibleRoutes = possiblePathsImpl(*start, graph, true, stopIfMoreThanMinCost);
  // the result may contain duplicates where lanes have diverged
  auto reachableSet = toLanelets(utils::concatenateRange(possibleRoutes, [&possibleRoutes](auto& path) {
    assert(!path.empty());
    if (&path == possibleRoutes.begin().base()) {
      return std::make_pair(path.begin(), path.end());
    }
    return std::make_pair(path.begin() + 1, path.end());
  }));
  std::sort(reachableSet.begin(), reachableSet.end(), [](auto& rhs, auto& lhs) { return rhs.id() <= lhs.id(); });
  auto remove = std::unique(reachableSet.begin(), reachableSet.end());
  reachableSet.erase(remove, reachableSet.end());
  return reachableSet;
}

ConstLaneletOrAreas RoutingGraph::reachableSetIncludingAreas(const ConstLaneletOrArea& llOrAr, double maxRoutingCost,
                                                             RoutingCostId routingCostId) const {
  ConstLaneletOrAreas result;
  auto start = graph_->getVertex(llOrAr);
  if (!start) {
    return result;
  }
  auto stopIfMoreThanMinCost = [maxRoutingCost](const PossibleRoutesInfo& route) {
    return route.totalDistance > maxRoutingCost;
  };
  auto& graph = graph_->filteredGraphs().withAreasAndLaneChanges[routingCostId];
  auto possibleRoutes = possiblePathsImpl(*start, graph, true, stopIfMoreThanMinCost);
  // the result may contain duplicates where lanes have diverged
  auto reachableSet = utils::concatenateRange(possibleRoutes, [&possibleRoutes](auto& path) {
    assert(!path.empty());
    if (&path == possibleRoutes.begin().base()) {
      return std::make_pair(path.begin(), path.end());
    }
    return std::make_pair(path.begin() + 1, path.end());
  });
  std::sort(reachableSet.begin(), reachableSet.end(), [](auto& rhs, auto& lhs) { return rhs.id() <= lhs.id(); });
  auto remove = std::unique(reachableSet.begin(), reachableSet.end());
  reachableSet.erase(remove, reachableSet.end());
  return reachableSet;
}

LaneletPaths RoutingGraph::possiblePaths(const ConstLanelet& startPoint, double minRoutingCost,
                                         RoutingCostId routingCostId, bool allowLaneChanges) const {
  auto start = graph_->getVertex(startPoint);
  if (!start) {
    return {};
  };
  auto stopIfMoreThanMinCost = [minRoutingCost](const PossibleRoutesInfo& route) {
    return route.totalDistance >= minRoutingCost;
  };
  auto& graph = allowLaneChanges ? graph_->withLaneChanges(routingCostId) : graph_->withoutLaneChanges(routingCostId);
  return utils::transform(possiblePathsImpl(*start, graph, false, stopIfMoreThanMinCost),
                          [](auto& v) { return LaneletPath(toLanelets(v)); });
}

LaneletPaths RoutingGraph::possiblePaths(const ConstLanelet& startPoint, uint32_t minLanelets,
                                         bool allowLaneChanges) const {
  auto start = graph_->getVertex(startPoint);
  if (!start) {
    return {};
  };
  auto stopIfMinLanelets = [minLanelets](const PossibleRoutesInfo& route) {
    return route.laneletsOrAreas.size() >= minLanelets;
  };
  auto& graph = allowLaneChanges ? graph_->withLaneChanges(0) : graph_->withoutLaneChanges(0);
  return utils::transform(possiblePathsImpl(*start, graph, false, stopIfMinLanelets),
                          [](auto& v) { return LaneletPath(toLanelets(v)); });
}

LaneletOrAreaPaths RoutingGraph::possiblePathsIncludingAreas(const ConstLaneletOrArea& startPoint,
                                                             double minRoutingCost, RoutingCostId routingCostId,
                                                             bool allowLaneChanges) const {
  auto start = graph_->getVertex(startPoint);
  if (!start) {
    return {};
  };
  auto stopIfMoreThanMinCost = [minRoutingCost](const PossibleRoutesInfo& route) {
    return route.totalDistance >= minRoutingCost;
  };
  auto& graph = allowLaneChanges ? graph_->withAreasAndLaneChanges(routingCostId)
                                 : graph_->withAreasWithoutLaneChanges(routingCostId);
  return utils::transform(possiblePathsImpl(*start, graph, false, stopIfMoreThanMinCost),
                          [](auto& v) { return LaneletOrAreaPath(v); });
}

LaneletOrAreaPaths RoutingGraph::possiblePathsIncludingAreas(const ConstLaneletOrArea& startPoint, uint32_t minElements,
                                                             bool allowLaneChanges) const {
  auto start = graph_->getVertex(startPoint);
  if (!start) {
    return {};
  };
  auto stopIfMinVertices = [minElements](const PossibleRoutesInfo& route) {
    return route.laneletsOrAreas.size() >= minElements;
  };
  auto& graph = allowLaneChanges ? graph_->withAreasAndLaneChanges(0) : graph_->withAreasWithoutLaneChanges(0);
  return utils::transform(possiblePathsImpl(*start, graph, false, stopIfMinVertices),
                          [](auto& v) { return LaneletOrAreaPath(v); });
}

void RoutingGraph::exportGraphML(const std::string& filename, const RelationTypes& edgeTypesToExclude,
                                 RoutingCostId routingCostId) const {
  if (filename.empty()) {
    throw InvalidInputError("No filename passed");
  }
  if (routingCostId >= graph_->numRoutingCosts) {
    throw InvalidInputError("Routing Cost ID is higher than the number of routing modules.");
  }
  RelationTypes relations{getRemainingRelations(edgeTypesToExclude)};
  exportGraphMLImpl<GraphType>(filename, graph_->graph, relations, routingCostId);
}

void RoutingGraph::exportGraphViz(const std::string& filename, const RelationTypes& edgeTypesToExclude,
                                  RoutingCostId routingCostId) const {
  if (filename.empty()) {
    throw InvalidInputError("No filename passed");
  }
  if (routingCostId >= graph_->numRoutingCosts) {
    throw InvalidInputError("Routing Cost ID is higher than the number of routing modules.");
  }
  RelationTypes relations{getRemainingRelations(edgeTypesToExclude)};
  exportGraphVizImpl<GraphType>(filename, graph_->graph, relations, routingCostId);
}

//! Helper function to slim down getDebugLaneletMap
RelationTypes allowedRelationsfromConfiguration(bool includeAdjacent, bool includeConflicting) {
  RelationTypes allowedRelations{RelationType::Successor, RelationType::Left, RelationType::Right,
                                 RelationType::Merging, RelationType::Diverging};
  if (includeAdjacent) {
    allowedRelations.push_back(RelationType::AdjacentLeft);
    allowedRelations.push_back(RelationType::AdjacentRight);
  }
  if (includeConflicting) {
    allowedRelations.push_back(RelationType::Conflicting);
  }
  std::sort(allowedRelations.begin(), allowedRelations.end());
  return allowedRelations;
}

LineString3d createLineString(const Point2d& from, const Point2d& to, RelationType relation, double routingCost) {
  LineString2d lineString(utils::getId());
  lineString.push_back(from);
  lineString.push_back(to);
  LineString3d lineString3d(lineString);
  lineString3d.setAttribute("relation", relationToString(relation));
  lineString3d.setAttribute("routing_cost", routingCost);
  return lineString3d;
}

class DebugMapBuilder {
 public:
  using LaneletOrAreaPair = std::pair<ConstLaneletOrArea, ConstLaneletOrArea>;
  explicit DebugMapBuilder(const FilteredGraph& graph) : graph_{graph} {}
  LaneletMapPtr run(const LaneletOrAreaToVertex& loa) {
    LaneletMapPtr output = std::make_shared<LaneletMap>();
    for (auto& vertex : loa) {
      visitVertex(vertex);
    }
    auto lineStrings = utils::transform(lineStringMap_, [](auto& mapLs) { return mapLs.second; });
    auto map = utils::createMap(lineStrings);
    for (auto& p : pointMap_) {
      map->add(utils::to3D(p.second));
    }
    return map;
  }

 private:
  void visitVertex(const LaneletOrAreaToVertex::value_type& vertex) {
    addPoint(vertex.first);
    auto edges = boost::out_edges(vertex.second, graph_);
    for (auto edge = edges.first; edge != edges.second; ++edge) {
      const auto& target = graph_[boost::target(*edge, graph_)].laneletOrArea;
      addPoint(target);
      const auto& edgeInfo = graph_[*edge];
      addEdge(vertex.first, target, edgeInfo);
    }
  }

  LaneletOrAreaPair getPair(const ConstLaneletOrArea& first, const ConstLaneletOrArea& second) {
    return first.id() < second.id() ? LaneletOrAreaPair(first, second) : LaneletOrAreaPair(second, first);
  }

  void addPoint(const ConstLaneletOrArea& point) {
    auto inMap = pointMap_.find(point);
    if (inMap == pointMap_.end()) {
      pointMap_.emplace(point, createPoint<Point2d>(point));
    }
  }

  void addEdge(const ConstLaneletOrArea& from, const ConstLaneletOrArea& to, EdgeInfo edge) {
    auto pair = getPair(from, to);
    auto inMap = lineStringMap_.find(pair);
    if (inMap != lineStringMap_.end()) {
      inMap->second.setAttribute("relation_reverse", relationToString(edge.relation));
      inMap->second.setAttribute("routing_cost_reverse", std::to_string(edge.routingCost));

    } else {
      auto pFrom = pointMap_.at(from);
      auto pTo = pointMap_.at(to);
      LineString3d lineString3d{createLineString(pFrom, pTo, edge.relation, edge.routingCost)};
      lineStringMap_.emplace(pair, lineString3d);
    }
  }

 private:
  const FilteredGraph& graph_;
  std::unordered_map<LaneletOrAreaPair, LineString3d> lineStringMap_;  // Stores all relations
  std::unordered_map<ConstLaneletOrArea, Point2d> pointMap_;           // Stores all 'edges'
};

LaneletMapPtr RoutingGraph::getDebugLaneletMap(RoutingCostId routingCostId, bool includeAdjacent,
                                               bool includeConflicting) const {
  if (routingCostId >= graph_->numRoutingCosts) {
    throw InvalidInputError("Routing Cost ID is higher than the number of routing modules.");
  }
  EdgeCostFilter edgeFilter(graph_->graph, routingCostId,
                            allowedRelationsfromConfiguration(includeAdjacent, includeConflicting));
  FilteredGraph filteredGraph(graph_->graph, edgeFilter);
  return DebugMapBuilder(filteredGraph).run(graph_->laneletOrAreaToVertex);
}

RoutingGraph::Errors RoutingGraph::checkValidity(bool throwOnError) const {
  Errors errors;
  for (const auto& laWithVertex : graph_->laneletOrAreaToVertex) {
    auto& la = laWithVertex.first;
    auto ll = laWithVertex.first.lanelet();
    auto& vertex = laWithVertex.second;
    auto id = la.id();
    // Check left relation
    Optional<ConstLanelet> left;
    try {
      left = neighboringLaneletImpl(vertex, graph_->filteredGraphs().left, true);

    } catch (RoutingGraphError& e) {
      errors.emplace_back(std::string("Left: ") + e.what());
    }
    Optional<ConstLanelet> adjacentLeft;
    try {
      adjacentLeft = neighboringLaneletImpl(vertex, graph_->filteredGraphs().adjacentLeft, true);
    } catch (RoutingGraphError& e) {
      errors.emplace_back(std::string("Adjacent left: ") + e.what());
    }
    if (left && adjacentLeft) {
      errors.emplace_back("Lanelet " + std::to_string(id) + " has both 'left' (id: " + std::to_string(left->id()) +
                          ") and 'adjancent_left' (id: " + std::to_string(adjacentLeft->id()) + ") lanelet");
    }
    if (left) {
      LaneletRelations rel{rightRelations(*left)};
      if (rel.empty()) {
        errors.emplace_back("There is a 'left' relation from " + std::to_string(id) + " to " +
                            std::to_string(left->id()) + " but no relation back");
      } else if (rel.front().lanelet != ll) {
        errors.emplace_back("There is a 'left' relation from " + std::to_string(id) + " to " +
                            std::to_string(left->id()) + ", but " + std::to_string(id) +
                            " isn't the closest lanelet the other way round");
      }
    }
    if (adjacentLeft) {
      LaneletRelations rel{rightRelations(*adjacentLeft)};
      if (rel.empty()) {
        errors.emplace_back("There is a 'adjacentLeft' relation from " + std::to_string(id) + " to " +
                            std::to_string(adjacentLeft->id()) + " but no relation back");
      } else if (rel.front().lanelet != ll) {
        errors.emplace_back("There is a 'adjacentLeft' relation from " + std::to_string(id) + " to " +
                            std::to_string(adjacentLeft->id()) + ", but " + std::to_string(id) +
                            " isn't the closest lanelet the other way round");
      }
    }
    // Check right
    Optional<ConstLanelet> right;
    try {
      right = neighboringLaneletImpl(vertex, graph_->filteredGraphs().right, true);
    } catch (RoutingGraphError& e) {
      errors.emplace_back(std::string("Right: ") + e.what());
    }
    Optional<ConstLanelet> adjacentRight;
    try {
      adjacentRight = neighboringLaneletImpl(vertex, graph_->filteredGraphs().adjacentRight, true);
    } catch (RoutingGraphError& e) {
      errors.emplace_back(std::string("Adjacent right: ") + e.what());
    }
    if (right && adjacentRight) {
      errors.emplace_back("Lanelet " + std::to_string(id) + " has both 'right' (id: " + std::to_string(right->id()) +
                          ") and 'adjancent_right' (id: " + std::to_string(adjacentRight->id()) + ") lanelet");
    }
    if (right) {
      LaneletRelations rel{leftRelations(*right)};
      if (rel.empty()) {
        errors.emplace_back("There is a 'right' relation from " + std::to_string(id) + " to " +
                            std::to_string(right->id()) + " but no relation back");
      } else if (rel.front().lanelet != ll) {
        errors.emplace_back("There is a 'right' relation from " + std::to_string(id) + " to " +
                            std::to_string(right->id()) + ", but " + std::to_string(id) +
                            " isn't the closest lanelet the other way round");
      }
    }
    if (adjacentRight) {
      LaneletRelations rel{leftRelations(*adjacentRight)};
      if (rel.empty()) {
        errors.emplace_back("There is a 'adjacentRight' relation from " + std::to_string(id) + " to " +
                            std::to_string(adjacentRight->id()) + " but no relation back");
      } else if (rel.front().lanelet != ll) {
        errors.emplace_back("There is a 'adjacentRight' relation from " + std::to_string(id) + " to " +
                            std::to_string(adjacentRight->id()) + ", but " + std::to_string(id) +
                            " isn't the closest lanelet the other way round");
      }
    }
  }
  if (throwOnError && !errors.empty()) {
    std::stringstream ss;
    ss << "Errors found in routing graph:";
    for (const auto& err : errors) {
      ss << "\n\t- " << err;
    }
    throw RoutingGraphError(ss.str());
  }
  return errors;
}

RoutingGraph::RoutingGraph(std::unique_ptr<Graph>&& graph, LaneletMapConstPtr&& passableMap)
    : graph_{std::move(graph)}, passableLaneletMap_{std::move(passableMap)} {}

}  // namespace routing
}  // namespace lanelet
