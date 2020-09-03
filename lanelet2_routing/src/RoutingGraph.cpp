#include "lanelet2_routing/RoutingGraph.h"

#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/Exceptions.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/Area.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_traffic_rules/TrafficRules.h>

#include <algorithm>
#include <boost/graph/reverse_graph.hpp>
#include <cassert>  // Asserts
#include <memory>
#include <utility>

#include "lanelet2_routing/Exceptions.h"
#include "lanelet2_routing/Forward.h"
#include "lanelet2_routing/Route.h"
#include "lanelet2_routing/internal/Graph.h"
#include "lanelet2_routing/internal/GraphUtils.h"
#include "lanelet2_routing/internal/RouteBuilder.h"
#include "lanelet2_routing/internal/RoutingGraphBuilder.h"
#include "lanelet2_routing/internal/ShortestPath.h"

// needs to be included after shotestPath due to some overload resolution quirks
#include "lanelet2_routing/internal/RoutingGraphVisualization.h"

namespace lanelet {
namespace routing {

#if __cplusplus < 201703L
constexpr const char RoutingGraph::ParticipantHeight[];
#endif

namespace {
using internal::DijkstraSearchMap;
using internal::DijkstraStyleSearch;
using internal::FilteredRoutingGraph;
using internal::GraphType;
using internal::LaneletVertexId;
using internal::RoutingGraphGraph;
using internal::VertexVisitInformation;

template <typename T>
T reservedVector(size_t size) {
  T t;
  t.reserve(size);
  return t;
}

/** @brief Helper objecct to save state of possible routes search */
struct PossibleRoutesInfo;
using PossibleRoutesInfoUPtr = std::unique_ptr<PossibleRoutesInfo>;
struct PossibleRoutesInfo {
  uint32_t vertexId{};
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
template <typename T, typename U>
bool addToPath(T& path, const Optional<U>& newElements) {
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
Optional<ConstLaneletOrArea> neighboringImpl(const GraphType::vertex_descriptor vertex,
                                             const FilteredRoutingGraph& graph, bool throwOnError = false) {
  auto outEdges = boost::out_edges(vertex, graph);
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

Optional<ConstLanelet> neighboringLaneletImpl(const GraphType::vertex_descriptor vertex,
                                              const FilteredRoutingGraph& graph, bool throwOnError = false) {
  auto value = neighboringImpl(vertex, graph, throwOnError);
  if (!!value && value->isLanelet()) {
    return value->lanelet();
  }
  return {};
}

template <typename Func>
Optional<ConstLaneletOrArea> ifInGraph(const RoutingGraphGraph& g, const ConstLaneletOrArea& llt, Func f) {
  auto vertex = g.getVertex(llt);
  if (!vertex) {
    return {};
  }
  return f(*vertex);
}

template <typename Func>
Optional<ConstLanelet> ifLaneletInGraph(const RoutingGraphGraph& g, const ConstLanelet& llt, Func f) {
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

ConstLaneletOrAreas getAllEdgesFromGraph(const RoutingGraphGraph& graph, const FilteredRoutingGraph& subgraph,
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
          edgesOut ? boost::target(*edgeRange.first, graph.get()) : boost::source(*edgeRange.first, graph.get());
      result.emplace_back(graph.get()[node].laneletOrArea);
    }
    return result;
  };
  return edgesOut ? processEdges(boost::out_edges(*laneletVertex, subgraph))
                  : processEdges(boost::in_edges(*laneletVertex, subgraph));
}

ConstLanelets getLaneletEdgesFromGraph(const RoutingGraphGraph& graph, const FilteredRoutingGraph& subgraph,
                                       const ConstLanelet& lanelet, bool edgesOut) {
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

template <bool Backw>
struct GetGraph {};
template <>
struct GetGraph<true> {
  template <typename G>
  auto operator()(const G& g) {
    return boost::make_reverse_graph(g);
  }
};
template <>
struct GetGraph<false> {
  template <typename G>
  auto operator()(const G& g) {
    return g;
  }
};

template <bool Backw, typename OutVertexT, typename GraphT>
std::vector<OutVertexT> buildPath(const DijkstraSearchMap<LaneletVertexId>& map, LaneletVertexId vertex, GraphT g) {
  const auto* currInfo = &map.at(vertex);
  auto size = currInfo->length;
  std::vector<OutVertexT> path(size);
  while (true) {
    auto idx = Backw ? size - currInfo->length : currInfo->length - 1;
    path[idx] = static_cast<OutVertexT>(g[vertex].laneletOrArea);
    if (currInfo->predecessor == vertex) {
      break;
    }
    vertex = currInfo->predecessor;
    currInfo = &map.at(vertex);
  }
  return path;
}

template <typename Cost1, typename Cost2>
struct CombinedCost {
  CombinedCost(const Cost1& c1, const Cost2& c2) : c1{c1}, c2{c2} {}
  template <typename T>
  inline bool operator()(const T& v) const {
    return c1(v) && c2(v);
  }
  Cost1 c1;
  Cost2 c2;
};

template <bool Eq = false>
struct StopIfLaneletsMoreThan {
  explicit StopIfLaneletsMoreThan(size_t n) : n{n} {}
  template <typename T>
  inline bool operator()(const T& v) const {
    return Eq ? v.length <= n : v.length < n;
  }
  size_t n;
};

template <bool Eq = false>
struct StopIfCostMoreThan {
  explicit StopIfCostMoreThan(double c) : c{c} {}
  template <typename T>
  inline bool operator()(const T& v) const {
    return Eq ? v.cost <= c : v.cost < c;
  }

  template <typename Other>
  CombinedCost<StopIfCostMoreThan, Other> operator&&(const Other& o) {
    return {*this, o};
  }
  double c;
};

template <bool Backw, bool KeepShorter, typename OutVertexT, typename OutContainerT, typename Func>
std::vector<OutContainerT> possiblePathsImpl(const GraphType::vertex_descriptor& start,
                                             const FilteredRoutingGraph& graph, Func stopCriterion) {
  auto g = GetGraph<Backw>{}(graph);
  DijkstraStyleSearch<decltype(g)> search(g);
  search.query(start, stopCriterion);
  auto keepPath = [&](auto& vertex) { return vertex.second.isLeaf && (KeepShorter || !vertex.second.predicate); };
  auto numPaths = size_t(std::count_if(search.getMap().begin(), search.getMap().end(), keepPath));
  std::vector<OutContainerT> result;
  result.reserve(numPaths);
  for (auto& vertex : search.getMap()) {
    if (!keepPath(vertex)) {
      continue;
    }
    result.emplace_back(buildPath<Backw, OutVertexT>(search.getMap(), vertex.first, graph));
  }
  return result;
}

template <bool Backw, typename OutVertexT, typename OutContainerT>
std::vector<OutContainerT> possiblePathsImpl(const GraphType::vertex_descriptor& start,
                                             const FilteredRoutingGraph& graph, const PossiblePathsParams& params) {
  if (params.routingCostLimit && !params.elementLimit && !params.includeShorterPaths) {
    return possiblePathsImpl<Backw, false, OutVertexT, OutContainerT>(start, graph,
                                                                      StopIfCostMoreThan<>(*params.routingCostLimit));
  }
  if (params.routingCostLimit && !params.elementLimit && params.includeShorterPaths) {
    return possiblePathsImpl<Backw, true, OutVertexT, OutContainerT>(start, graph,
                                                                     StopIfCostMoreThan<>(*params.routingCostLimit));
  }
  if (!params.routingCostLimit && params.elementLimit && !params.includeShorterPaths) {
    return possiblePathsImpl<Backw, false, OutVertexT, OutContainerT>(start, graph,
                                                                      StopIfLaneletsMoreThan<>(*params.elementLimit));
  }
  if (!params.routingCostLimit && params.elementLimit && params.includeShorterPaths) {
    return possiblePathsImpl<Backw, true, OutVertexT, OutContainerT>(start, graph,
                                                                     StopIfLaneletsMoreThan<>(*params.elementLimit));
  }
  if (params.routingCostLimit && params.elementLimit && !params.includeShorterPaths) {
    return possiblePathsImpl<Backw, false, OutVertexT, OutContainerT>(
        start, graph, StopIfCostMoreThan<>(*params.routingCostLimit) && StopIfLaneletsMoreThan<>(*params.elementLimit));
  }
  if (params.routingCostLimit && params.elementLimit && params.includeShorterPaths) {
    return possiblePathsImpl<Backw, true, OutVertexT, OutContainerT>(
        start, graph, StopIfCostMoreThan<>(*params.routingCostLimit) && StopIfLaneletsMoreThan<>(*params.elementLimit));
  }
  throw InvalidInputError("Possible paths called with invalid cost limit AND invalid element limit!");
}

template <bool Backw, typename OutVertexT, typename Func>
std::vector<OutVertexT> reachableSetImpl(const GraphType::vertex_descriptor& start, const FilteredRoutingGraph& graph,
                                         Func stopCriterion) {
  auto g = GetGraph<Backw>{}(graph);
  DijkstraStyleSearch<decltype(g)> search(g);
  search.query(start, stopCriterion);
  std::vector<OutVertexT> result;
  result.reserve(search.getMap().size());
  for (auto& vertex : search.getMap()) {
    if (vertex.second.predicate) {
      result.emplace_back(static_cast<OutVertexT>(graph[vertex.first].laneletOrArea));
    }
  }
  return result;
}

template <typename PathT, typename PrimT>
Optional<PathT> shortestPathImpl(const PrimT& from, const PrimT& to, RoutingCostId routingCostId, bool withLaneChanges,
                                 bool withAreas, const internal::RoutingGraphGraph& graph) {
  auto startVertex = graph.getVertex(from);
  auto endVertex = graph.getVertex(to);
  if (!startVertex || !endVertex) {
    return {};
  }
  auto filteredGraph =
      withLaneChanges
          ? withAreas ? graph.withAreasAndLaneChanges(routingCostId) : graph.withLaneChanges(routingCostId)
          : withAreas ? graph.withAreasWithoutLaneChanges(routingCostId) : graph.withoutLaneChanges(routingCostId);
  DijkstraStyleSearch<FilteredRoutingGraph> search(filteredGraph);
  class DestinationReached {};
  try {
    search.query(*startVertex, [endVertex](const internal::VertexVisitInformation& i) {
      if (i.vertex == *endVertex) {
        throw DestinationReached{};
      }
      return true;
    });
  } catch (DestinationReached) {  // NOLINT
    return PathT{buildPath<false, PrimT>(search.getMap(), *endVertex, filteredGraph)};
  }
  return {};
}

template <typename RetT, typename Primitives, typename ShortestPathFunc>
Optional<RetT> shortestPathViaImpl(Primitives routePoints, ShortestPathFunc&& shortestPath) {
  Primitives path;
  for (size_t it = 0; it < routePoints.size() - 1; it++) {
    auto results = shortestPath(routePoints[it], routePoints[it + 1]);
    if (!!results && !results->empty() && path.empty()) {
      path.push_back(results->front());
    }
    if (!addToPath(path, results)) {
      return Optional<RetT>();
    }
  }
  return RetT(path);
}
}  // namespace

RoutingGraph::RoutingGraph(RoutingGraph&& /*other*/) noexcept = default;
RoutingGraph& RoutingGraph::operator=(RoutingGraph&& /*other*/) noexcept = default;
RoutingGraph::~RoutingGraph() = default;

RoutingGraphUPtr RoutingGraph::build(const LaneletMap& laneletMap, const traffic_rules::TrafficRules& trafficRules,
                                     const RoutingCostPtrs& routingCosts, const RoutingGraph::Configuration& config) {
  return internal::RoutingGraphBuilder(trafficRules, routingCosts, config).build(laneletMap);
}

RoutingGraphUPtr RoutingGraph::build(const LaneletSubmap& laneletSubmap,
                                     const traffic_rules::TrafficRules& trafficRules,
                                     const RoutingCostPtrs& routingCosts, const RoutingGraph::Configuration& config) {
  return internal::RoutingGraphBuilder(trafficRules, routingCosts, config).build(laneletSubmap);
}

Optional<Route> RoutingGraph::getRoute(const ConstLanelet& from, const ConstLanelet& to, RoutingCostId routingCostId,
                                       bool withLaneChanges) const {
  auto optPath{shortestPath(from, to, routingCostId, withLaneChanges)};
  if (!optPath) {
    return {};
  }
  return internal::RouteBuilder(*graph_).getRouteFromShortestPath(*optPath, withLaneChanges, routingCostId);
}

Optional<Route> RoutingGraph::getRouteVia(const ConstLanelet& from, const ConstLanelets& via, const ConstLanelet& to,
                                          RoutingCostId routingCostId, bool withLaneChanges) const {
  auto optPath{shortestPathVia(from, via, to, routingCostId, withLaneChanges)};
  if (!optPath) {
    return {};
  }
  return internal::RouteBuilder(*graph_).getRouteFromShortestPath(*optPath, withLaneChanges, routingCostId);
}

Optional<LaneletPath> RoutingGraph::shortestPath(const ConstLanelet& from, const ConstLanelet& to,
                                                 RoutingCostId routingCostId, bool withLaneChanges) const {
  return shortestPathImpl<LaneletPath, ConstLanelet>(from, to, routingCostId, withLaneChanges, false, *graph_);
}

Optional<LaneletOrAreaPath> RoutingGraph::shortestPathIncludingAreas(const ConstLaneletOrArea& from,
                                                                     const ConstLaneletOrArea& to,
                                                                     RoutingCostId routingCostId,
                                                                     bool withLaneChanges) const {
  return shortestPathImpl<LaneletOrAreaPath, ConstLaneletOrArea>(from, to, routingCostId, withLaneChanges, true,
                                                                 *graph_);
}

Optional<LaneletPath> RoutingGraph::shortestPathVia(const ConstLanelet& start, const ConstLanelets& via,
                                                    const ConstLanelet& end, RoutingCostId routingCostId,
                                                    bool withLaneChanges) const {
  ConstLanelets routePoints = utils::concatenate({ConstLanelets{start}, via, ConstLanelets{end}});
  return shortestPathViaImpl<LaneletPath>(
      routePoints, [&](auto& from, auto& to) { return this->shortestPath(from, to, routingCostId, withLaneChanges); });
}

Optional<LaneletOrAreaPath> RoutingGraph::shortestPathIncludingAreasVia(const ConstLaneletOrArea& start,
                                                                        const ConstLaneletOrAreas& via,
                                                                        const ConstLaneletOrArea& end,
                                                                        RoutingCostId routingCostId,
                                                                        bool withLaneChanges) const {
  ConstLaneletOrAreas routePoints = utils::concatenate({ConstLaneletOrAreas{start}, via, ConstLaneletOrAreas{end}});
  return shortestPathViaImpl<LaneletOrAreaPath>(routePoints, [&](auto& from, auto& to) {
    return this->shortestPathIncludingAreas(from, to, routingCostId, withLaneChanges);
  });
}

Optional<RelationType> RoutingGraph::routingRelation(const ConstLanelet& from, const ConstLanelet& to,
                                                     bool includeConflicting) const {
  auto edgeInfo = includeConflicting ? graph_->getEdgeInfo(from, to)
                                     : graph_->getEdgeInfoFor(from, to, graph_->withoutConflicting());
  if (!!edgeInfo) {
    return edgeInfo->relation;
  }
  return {};
}

ConstLanelets RoutingGraph::following(const ConstLanelet& lanelet, bool withLaneChanges) const {
  auto subgraph = withLaneChanges ? graph_->withLaneChanges() : graph_->withoutLaneChanges();
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
  auto subgraph = withLaneChanges ? graph_->withLaneChanges(0) : graph_->withoutLaneChanges(0);
  return getLaneletEdgesFromGraph(*graph_, subgraph, lanelet, false);
}

LaneletRelations RoutingGraph::previousRelations(const ConstLanelet& lanelet, bool withLaneChanges) const {
  ConstLanelets prev{previous(lanelet, withLaneChanges)};
  LaneletRelations result;
  result.reserve(prev.size());
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

ConstLanelets RoutingGraph::besides(const ConstLanelet& lanelet, RoutingCostId routingCostId) const {
  auto move = [](auto it) { return std::make_move_iterator(it); };
  ConstLanelets left{lefts(lanelet, routingCostId)};
  ConstLanelets right{rights(lanelet, routingCostId)};
  ConstLanelets result;
  result.reserve(left.size() + right.size() + 1);
  result.insert(std::end(result), move(left.rbegin()), move(left.rend()));
  result.push_back(lanelet);
  result.insert(std::end(result), move(std::begin(right)), move(std::end(right)));
  return result;
}

Optional<ConstLanelet> RoutingGraph::left(const ConstLanelet& lanelet, RoutingCostId routingCostId) const {
  return ifLaneletInGraph(*graph_, lanelet,
                          [&](auto& vertex) { return neighboringLaneletImpl(vertex, graph_->left(routingCostId)); });
}

Optional<ConstLanelet> RoutingGraph::adjacentLeft(const ConstLanelet& lanelet, RoutingCostId routingCostId) const {
  return ifLaneletInGraph(*graph_, lanelet, [&](auto& vertex) {
    return neighboringLaneletImpl(vertex, graph_->adjacentLeft(routingCostId));
  });
}

Optional<ConstLanelet> RoutingGraph::right(const ConstLanelet& lanelet, RoutingCostId routingCostId) const {
  return ifLaneletInGraph(*graph_, lanelet,
                          [&](auto& vertex) { return neighboringLaneletImpl(vertex, graph_->right(routingCostId)); });
}

Optional<ConstLanelet> RoutingGraph::adjacentRight(const ConstLanelet& lanelet, RoutingCostId routingCostId) const {
  return ifLaneletInGraph(*graph_, lanelet, [&](auto& vertex) {
    return neighboringLaneletImpl(vertex, graph_->adjacentRight(routingCostId));
  });
}

ConstLanelets RoutingGraph::lefts(const ConstLanelet& lanelet, RoutingCostId routingCostId) const {
  return getUntilEnd(lanelet, [&](const ConstLanelet& llt) { return left(llt, routingCostId); });
}

ConstLanelets RoutingGraph::adjacentLefts(const ConstLanelet& lanelet, RoutingCostId routingCostId) const {
  return getUntilEnd(lanelet, [&](const ConstLanelet& llt) { return adjacentLeft(llt, routingCostId); });
}

LaneletRelations RoutingGraph::leftRelations(const ConstLanelet& lanelet, RoutingCostId routingCostId) const {
  bool leftReached{false};
  ConstLanelet current = lanelet;
  LaneletRelations result;
  while (!leftReached) {
    const ConstLanelets leftOf{lefts(current, routingCostId)};
    for (auto const& it : leftOf) {
      result.emplace_back(LaneletRelation{it, RelationType::Left});
      current = it;
    }
    const ConstLanelets adjacentLeftOf{adjacentLefts(current, routingCostId)};
    for (auto const& it : adjacentLeftOf) {
      result.emplace_back(LaneletRelation{it, RelationType::AdjacentLeft});
      current = it;
    }
    leftReached = (leftOf.empty() && adjacentLeftOf.empty());
  }
  return result;
}

ConstLanelets RoutingGraph::rights(const ConstLanelet& lanelet, RoutingCostId routingCostId) const {
  return getUntilEnd(lanelet, [&](const ConstLanelet& llt) { return right(llt, routingCostId); });
}

ConstLanelets RoutingGraph::adjacentRights(const ConstLanelet& lanelet, RoutingCostId routingCostId) const {
  return getUntilEnd(lanelet, [&](const ConstLanelet& llt) { return adjacentRight(llt, routingCostId); });
}

LaneletRelations RoutingGraph::rightRelations(const ConstLanelet& lanelet, RoutingCostId routingCostId) const {
  bool rightReached{false};
  ConstLanelet current = lanelet;
  auto result = reservedVector<LaneletRelations>(3);
  while (!rightReached) {
    const ConstLanelets rightOf{rights(current, routingCostId)};
    for (auto const& it : rightOf) {
      result.emplace_back(LaneletRelation{it, RelationType::Right});
      current = it;
    }
    const ConstLanelets adjacentRightOf{adjacentRights(current, routingCostId)};
    for (auto const& it : adjacentRightOf) {
      result.emplace_back(LaneletRelation{it, RelationType::AdjacentRight});
      current = it;
    }
    rightReached = (rightOf.empty() && adjacentRightOf.empty());
  }
  return result;
}

ConstLaneletOrAreas RoutingGraph::conflicting(const ConstLaneletOrArea& laneletOrArea) const {
  return getAllEdgesFromGraph(*graph_, graph_->conflicting(), laneletOrArea, true);
}

ConstLanelets RoutingGraph::reachableSet(const ConstLanelet& lanelet, double maxRoutingCost,
                                         RoutingCostId routingCostId, bool allowLaneChanges) const {
  auto start = graph_->getVertex(lanelet);
  if (!start) {
    return {};
  }
  auto graph = allowLaneChanges ? graph_->withLaneChanges(routingCostId) : graph_->withoutLaneChanges(routingCostId);
  return reachableSetImpl<false, ConstLanelet>(*start, graph, StopIfCostMoreThan<true>{maxRoutingCost});
}

ConstLaneletOrAreas RoutingGraph::reachableSetIncludingAreas(const ConstLaneletOrArea& llOrAr, double maxRoutingCost,
                                                             RoutingCostId routingCostId) const {
  auto start = graph_->getVertex(llOrAr);
  if (!start) {
    return {};
  }
  auto graph = graph_->withAreasAndLaneChanges(routingCostId);
  return reachableSetImpl<false, ConstLaneletOrArea>(*start, graph, StopIfCostMoreThan<true>{maxRoutingCost});
}

ConstLanelets RoutingGraph::reachableSetTowards(const ConstLanelet& lanelet, double maxRoutingCost,
                                                RoutingCostId routingCostId, bool allowLaneChanges) const {
  auto start = graph_->getVertex(lanelet);
  if (!start) {
    return {};
  }
  auto graph = allowLaneChanges ? graph_->withLaneChanges(routingCostId) : graph_->withoutLaneChanges(routingCostId);
  return reachableSetImpl<true, ConstLanelet>(*start, graph, StopIfCostMoreThan<true>{maxRoutingCost});
}

LaneletPaths RoutingGraph::possiblePaths(const ConstLanelet& startPoint, const PossiblePathsParams& params) const {
  auto start = graph_->getVertex(startPoint);
  if (!start) {
    return {};
  }
  auto graph = params.includeLaneChanges ? graph_->withLaneChanges(params.routingCostId)
                                         : graph_->withoutLaneChanges(params.routingCostId);
  return possiblePathsImpl<false, ConstLanelet, LaneletPath>(*start, graph, params);
}

LaneletPaths RoutingGraph::possiblePaths(const ConstLanelet& startPoint, double minRoutingCost,
                                         RoutingCostId routingCostId, bool allowLaneChanges) const {
  return possiblePaths(startPoint, PossiblePathsParams{minRoutingCost, {}, routingCostId, allowLaneChanges, false});
}

LaneletPaths RoutingGraph::possiblePaths(const ConstLanelet& startPoint, uint32_t minLanelets, bool allowLaneChanges,
                                         RoutingCostId routingCostId) const {
  return possiblePaths(startPoint, PossiblePathsParams{{}, minLanelets, routingCostId, allowLaneChanges, false});
}

LaneletPaths RoutingGraph::possiblePathsTowards(const ConstLanelet& targetLanelet,
                                                const PossiblePathsParams& params) const {
  auto start = graph_->getVertex(targetLanelet);
  if (!start) {
    return {};
  }
  auto graph = params.includeLaneChanges ? graph_->withLaneChanges(params.routingCostId)
                                         : graph_->withoutLaneChanges(params.routingCostId);
  return possiblePathsImpl<true, ConstLanelet, LaneletPath>(*start, graph, params);
}

LaneletPaths RoutingGraph::possiblePathsTowards(const ConstLanelet& targetLanelet, double minRoutingCost,
                                                RoutingCostId routingCostId, bool allowLaneChanges) const {
  return possiblePathsTowards(targetLanelet,
                              PossiblePathsParams{minRoutingCost, {}, routingCostId, allowLaneChanges, false});
}

LaneletPaths RoutingGraph::possiblePathsTowards(const ConstLanelet& targetLanelet, uint32_t minLanelets,
                                                bool allowLaneChanges, RoutingCostId routingCostId) const {
  return possiblePathsTowards(targetLanelet,
                              PossiblePathsParams{{}, minLanelets, routingCostId, allowLaneChanges, false});
}

LaneletOrAreaPaths RoutingGraph::possiblePathsIncludingAreas(const ConstLaneletOrArea& startPoint,
                                                             const PossiblePathsParams& params) const {
  auto start = graph_->getVertex(startPoint);
  if (!start) {
    return {};
  }
  auto graph = params.includeLaneChanges ? graph_->withAreasAndLaneChanges(params.routingCostId)
                                         : graph_->withAreasWithoutLaneChanges(params.routingCostId);
  return possiblePathsImpl<false, ConstLaneletOrArea, LaneletOrAreaPath>(*start, graph, params);
}

LaneletOrAreaPaths RoutingGraph::possiblePathsIncludingAreas(const ConstLaneletOrArea& startPoint,
                                                             double minRoutingCost, RoutingCostId routingCostId,
                                                             bool allowLaneChanges) const {
  return possiblePathsIncludingAreas(startPoint,
                                     PossiblePathsParams{minRoutingCost, {}, routingCostId, allowLaneChanges, false});
}

LaneletOrAreaPaths RoutingGraph::possiblePathsIncludingAreas(const ConstLaneletOrArea& startPoint, uint32_t minElements,
                                                             bool allowLaneChanges, RoutingCostId routingCostId) const {
  return possiblePathsIncludingAreas(startPoint,
                                     PossiblePathsParams{{}, minElements, routingCostId, allowLaneChanges, false});
}

void RoutingGraph::forEachSuccessor(const ConstLanelet& lanelet, const LaneletVisitFunction& f, bool allowLaneChanges,
                                    RoutingCostId routingCostId) const {
  auto start = graph_->getVertex(lanelet);
  if (!start) {
    return;
  }
  auto graph = allowLaneChanges ? graph_->withLaneChanges(routingCostId) : graph_->withoutLaneChanges(routingCostId);
  DijkstraStyleSearch<FilteredRoutingGraph> search(graph);
  search.query(*start, [&](const VertexVisitInformation& i) -> bool {
    return f(LaneletVisitInformation{graph_->get()[i.vertex].lanelet(), graph_->get()[i.predecessor].lanelet(), i.cost,
                                     i.length, i.numLaneChanges});
  });
}

void RoutingGraph::forEachSuccessorIncludingAreas(const ConstLaneletOrArea& lanelet,
                                                  const LaneletOrAreaVisitFunction& f, bool allowLaneChanges,
                                                  RoutingCostId routingCostId) const {
  auto start = graph_->getVertex(lanelet);
  if (!start) {
    return;
  }
  auto graph = allowLaneChanges ? graph_->withAreasAndLaneChanges(routingCostId)
                                : graph_->withAreasWithoutLaneChanges(routingCostId);
  DijkstraStyleSearch<FilteredRoutingGraph> search(graph);
  search.query(*start, [&](const VertexVisitInformation& i) -> bool {
    return f(LaneletOrAreaVisitInformation{graph_->get()[i.vertex].laneletOrArea,
                                           graph_->get()[i.predecessor].laneletOrArea, i.cost, i.length,
                                           i.numLaneChanges});
  });
}

void RoutingGraph::forEachPredecessor(const ConstLanelet& lanelet, const LaneletVisitFunction& f, bool allowLaneChanges,
                                      RoutingCostId routingCostId) const {
  auto start = graph_->getVertex(lanelet);
  if (!start) {
    return;
  }
  auto forwGraph =
      allowLaneChanges ? graph_->withLaneChanges(routingCostId) : graph_->withoutLaneChanges(routingCostId);
  auto graph = boost::make_reverse_graph(forwGraph);  // forwGraph needs to stay on the stack
  internal::DijkstraStyleSearch<decltype(graph)> search(graph);
  search.query(*start, [&](const VertexVisitInformation& i) -> bool {
    return f(LaneletVisitInformation{graph_->get()[i.vertex].lanelet(), graph_->get()[i.predecessor].lanelet(), i.cost,
                                     i.length, i.numLaneChanges});
  });
}

void RoutingGraph::forEachPredecessorIncludingAreas(const ConstLaneletOrArea& lanelet,
                                                    const LaneletOrAreaVisitFunction& f, bool allowLaneChanges,
                                                    RoutingCostId routingCostId) const {
  auto start = graph_->getVertex(lanelet);
  if (!start) {
    return;
  }
  auto forwGraph = allowLaneChanges ? graph_->withAreasAndLaneChanges(routingCostId)
                                    : graph_->withAreasWithoutLaneChanges(routingCostId);
  auto graph = boost::make_reverse_graph(forwGraph);  // forwGraph needs to stay on the stack
  internal::DijkstraStyleSearch<decltype(graph)> search(graph);
  search.query(*start, [&](const VertexVisitInformation& i) -> bool {
    return f(LaneletOrAreaVisitInformation{graph_->get()[i.vertex].laneletOrArea,
                                           graph_->get()[i.predecessor].laneletOrArea, i.cost, i.length,
                                           i.numLaneChanges});
  });
}

void RoutingGraph::exportGraphML(const std::string& filename, const RelationType& edgeTypesToExclude,
                                 RoutingCostId routingCostId) const {
  if (filename.empty()) {
    throw InvalidInputError("No filename passed");
  }
  if (routingCostId >= graph_->numRoutingCosts()) {
    throw InvalidInputError("Routing Cost ID is higher than the number of routing modules.");
  }
  RelationType relations = allRelations() & ~edgeTypesToExclude;
  internal::exportGraphMLImpl<GraphType>(filename, graph_->get(), relations, routingCostId);
}

void RoutingGraph::exportGraphViz(const std::string& filename, const RelationType& edgeTypesToExclude,
                                  RoutingCostId routingCostId) const {
  if (filename.empty()) {
    throw InvalidInputError("No filename passed");
  }
  if (routingCostId >= graph_->numRoutingCosts()) {
    throw InvalidInputError("Routing Cost ID is higher than the number of routing modules.");
  }
  RelationType relations = allRelations() & ~edgeTypesToExclude;
  internal::exportGraphVizImpl<GraphType>(filename, graph_->get(), relations, routingCostId);
}

//! Helper function to slim down getDebugLaneletMap
RelationType allowedRelationsfromConfiguration(bool includeAdjacent, bool includeConflicting) {
  RelationType allowedRelations{RelationType::Successor | RelationType::Left | RelationType::Right |
                                RelationType::Area};
  if (includeAdjacent) {
    allowedRelations |= RelationType::AdjacentLeft;
    allowedRelations |= RelationType::AdjacentRight;
  }
  if (includeConflicting) {
    allowedRelations |= RelationType::Conflicting;
  }
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
  explicit DebugMapBuilder(const FilteredRoutingGraph& graph) : graph_{graph} {}
  LaneletMapPtr run(const internal::LaneletOrAreaToVertex& loa) {
    LaneletMapPtr output = std::make_shared<LaneletMap>();
    for (const auto& vertex : loa) {
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
  void visitVertex(const internal::LaneletOrAreaToVertex::value_type& vertex) {
    addPoint(vertex.first);
    auto edges = boost::out_edges(vertex.second, graph_);
    for (auto edge = edges.first; edge != edges.second; ++edge) {
      const auto& target = graph_[boost::target(*edge, graph_)].laneletOrArea;
      addPoint(target);
      const auto& edgeInfo = graph_[*edge];
      addEdge(vertex.first, target, edgeInfo);
    }
  }

  static LaneletOrAreaPair getPair(const ConstLaneletOrArea& first, const ConstLaneletOrArea& second) {
    return first.id() < second.id() ? LaneletOrAreaPair(first, second) : LaneletOrAreaPair(second, first);
  }

  void addPoint(const ConstLaneletOrArea& point) {
    auto inMap = pointMap_.find(point);
    if (inMap == pointMap_.end()) {
      pointMap_.emplace(point, createPoint<Point2d>(point));
    }
  }

  void addEdge(const ConstLaneletOrArea& from, const ConstLaneletOrArea& to, internal::EdgeInfo edge) {
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

  FilteredRoutingGraph graph_;
  std::unordered_map<LaneletOrAreaPair, LineString3d> lineStringMap_;  // Stores all relations
  std::unordered_map<ConstLaneletOrArea, Point2d> pointMap_;           // Stores all 'edges'
};

LaneletMapPtr RoutingGraph::getDebugLaneletMap(RoutingCostId routingCostId, bool includeAdjacent,
                                               bool includeConflicting) const {
  if (routingCostId >= graph_->numRoutingCosts()) {
    throw InvalidInputError("Routing Cost ID is higher than the number of routing modules.");
  }
  internal::EdgeCostFilter<GraphType> edgeFilter(
      graph_->get(), routingCostId, allowedRelationsfromConfiguration(includeAdjacent, includeConflicting));
  FilteredRoutingGraph filteredGraph(graph_->get(), edgeFilter);
  return DebugMapBuilder(filteredGraph).run(graph_->vertexLookup());
}

RoutingGraph::Errors RoutingGraph::checkValidity(bool throwOnError) const {
  Errors errors;
  for (const auto& laWithVertex : graph_->vertexLookup()) {
    const auto& la = laWithVertex.first;
    auto ll = laWithVertex.first.lanelet();
    const auto& vertex = laWithVertex.second;
    auto id = la.id();
    // Check left relation
    Optional<ConstLanelet> left;
    try {
      left = neighboringLaneletImpl(vertex, graph_->left(), true);

    } catch (RoutingGraphError& e) {
      errors.emplace_back(std::string("Left: ") + e.what());
    }
    Optional<ConstLanelet> adjacentLeft;
    try {
      adjacentLeft = neighboringLaneletImpl(vertex, graph_->adjacentLeft(), true);
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
      right = neighboringLaneletImpl(vertex, graph_->right(), true);
    } catch (RoutingGraphError& e) {
      errors.emplace_back(std::string("Right: ") + e.what());
    }
    Optional<ConstLanelet> adjacentRight;
    try {
      adjacentRight = neighboringLaneletImpl(vertex, graph_->adjacentRight(), true);
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

RoutingGraph::RoutingGraph(std::unique_ptr<RoutingGraphGraph>&& graph, LaneletSubmapConstPtr&& passableMap)
    : graph_{std::move(graph)}, passableLaneletSubmap_{std::move(passableMap)} {}

}  // namespace routing
}  // namespace lanelet
