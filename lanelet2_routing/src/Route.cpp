#include "lanelet2_routing/Route.h"

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/utility/Utilities.h>

#include <boost/graph/reverse_graph.hpp>
#include <unordered_map>

#include "lanelet2_routing/Exceptions.h"
#include "lanelet2_routing/internal/Graph.h"
#include "lanelet2_routing/internal/GraphUtils.h"
#include "lanelet2_routing/internal/ShortestPath.h"

namespace lanelet {
namespace routing {

namespace {
using internal::FilteredRouteGraph;
using internal::RouteGraph;
using internal::RouteVertexInfo;

using ConstLaneletPointMapIt = std::map<Id, Point2d>::iterator;
using DebugEdge = std::pair<Id, Id>;

/** @brief Creates a new point that represents a RouteElement in the debug lanelet map
 *  A new point at the center of gravity of the lanelet is created and added to the pointMap. Some basic information
 * like the ID is integrated in the attributes.
 *  @param pointMap Map to add the point to
 *  @param element Route element that should be represented
 *  @return Iterator to the new entry in the point map */
ConstLaneletPointMapIt createAndAddPoint(std::map<Id, Point2d>& pointMap, const RouteVertexInfo& element) {
  ConstLanelet lanelet{element.lanelet};
  Point2d point;
  point.setId(lanelet.id());
  point.setAttribute("id", lanelet.id());
  point.setAttribute("lane_id", element.laneId);
  boost::geometry::centroid(CompoundHybridPolygon2d(lanelet.polygon2d()), point);
  const auto emplace{pointMap.emplace(lanelet.id(), point)};
  return emplace.first;
}

/** @brief Adds a relation between two RouteElement objects to the debug lanelet map
 *  @param lineStringMap Map of relations between lanelets
 *  @param pointMap Map of points that represent RouteElements
 *  @param from Start route element
 *  @param relations Relations that should be added */
void addRelation(std::map<DebugEdge, LineString3d>& edgeMap, std::map<Id, Point2d>& vertexMap,
                 const RouteVertexInfo& from, const RouteVertexInfo& to, RelationType relation) {
  auto fromPointIt = vertexMap.find(from.lanelet.id());
  if (fromPointIt == vertexMap.end()) {
    fromPointIt = createAndAddPoint(vertexMap, from);
  }
  DebugEdge orderedPair = std::minmax(from.lanelet.id(), to.lanelet.id());
  auto lineStringMapIt = edgeMap.find(orderedPair);
  if (lineStringMapIt == edgeMap.end()) {
    auto toPointIt = vertexMap.find(to.lanelet.id());
    if (toPointIt == vertexMap.end()) {
      toPointIt = createAndAddPoint(vertexMap, to);
    }
    LineString2d lineString;
    lineString.push_back(fromPointIt->second);
    lineString.push_back(toPointIt->second);
    lineStringMapIt = edgeMap.emplace(orderedPair, LineString3d(lineString)).first;
    lineStringMapIt->second.setAttribute("relation_1", relationToString(relation));
    return;
  }
  std::string direction =
      lineStringMapIt->second.front().id() == fromPointIt->first ? "relation_" : "relation_reverse_";
  for (size_t it = 1; it >= 1; it++) {  /// Finding the next unused attribute.
    auto attr = direction + std::to_string(it);
    if (!lineStringMapIt->second.hasAttribute(attr)) {
      lineStringMapIt->second.setAttribute(attr, relationToString(relation));
      break;
    }
  }
}

LaneletSequence remainingLaneImpl(RouteGraph::Vertex v, const FilteredRouteGraph& g) {
  ConstLanelets lane;
  auto start = v;
  while (true) {
    lane.push_back(g[v].lanelet);
    auto outEdges = boost::out_edges(v, g);
    if (std::distance(outEdges.first, outEdges.second) != 1) {
      break;  // lane splits
    }
    v = boost::target(*outEdges.first, g);
    if (boost::in_degree(v, g) != 1) {
      break;  // other lane merges
    }
    if (v == start) {
      break;  // we are in a loop
    }
  }
  return LaneletSequence{std::move(lane)};
}

template <bool Backwards = false>
LaneletRelations getRelations(RouteGraph::Vertex v, const FilteredRouteGraph& g) {
  auto out = internal::GetEdges<Backwards>{}(v, g);
  return utils::transform(out.first, out.second, [&g](auto e) {
    return LaneletRelation{g[internal::GetTarget<Backwards>{}(e, g)].lanelet, g[e].relation};
  });
}
template <bool Backwards = false>
ConstLanelets getLanelets(RouteGraph::Vertex v, const FilteredRouteGraph& g) {
  auto out = internal::GetEdges<Backwards>{}(v, g);
  return utils::transform(out.first, out.second,
                          [&g](auto e) { return g[internal::GetTarget<Backwards>{}(e, g)].lanelet; });
}

Optional<LaneletRelation> getSingleRelation(RouteGraph::Vertex v, const FilteredRouteGraph& g) {
  auto outEdges = boost::out_edges(v, g);
  if (outEdges.first == outEdges.second) {
    return {};
  }
  return LaneletRelation{g[boost::target(*outEdges.first, g)].lanelet, g[*outEdges.first].relation};
}

template <bool Backwards = false>
std::pair<Optional<RouteGraph::Vertex>, RelationType> getNextVertex(RouteGraph::Vertex v, const FilteredRouteGraph& g) {
  auto out = internal::GetEdges<Backwards>{}(v, g);
  if (out.first == out.second) {
    return {};
  }
  return {internal::GetTarget<Backwards>{}(*out.first, g), g[*out.first].relation};
}

}  // anonymous namespace

Route::Route() = default;
Route::~Route() noexcept = default;
Route& Route::operator=(Route&& other) noexcept = default;
Route::Route(Route&& other) noexcept = default;
Route::Route(LaneletPath shortestPath, std::unique_ptr<RouteGraph> graph, LaneletSubmapConstPtr laneletSubmap) noexcept
    : graph_{std::move(graph)}, shortestPath_{std::move(shortestPath)}, laneletSubmap_{std::move(laneletSubmap)} {}

LaneletPath Route::remainingShortestPath(const ConstLanelet& ll) const {
  auto iter = std::find(shortestPath_.begin(), shortestPath_.end(), ll);
  if (iter == shortestPath_.end()) {
    return LaneletPath{};
  }
  if (!shortestPath_.empty() && shortestPath_.front() == shortestPath_.back()) {  // circular
    ConstLanelets llts{shortestPath_.begin(), shortestPath_.end()};
    llts.pop_back();
    std::rotate(llts.begin(), llts.begin() + std::distance(shortestPath_.begin(), iter), llts.end());
    return LaneletPath{llts};
  }
  return LaneletPath{ConstLanelets{iter, shortestPath_.end()}};
}

LaneletSequence Route::fullLane(const ConstLanelet& ll) const {
  // go back to the first lanelet of the lane (ie the first lanelet that has not exactly one predecessor) and then call
  // remaining lane on it
  auto v = graph_->getVertex(ll);
  if (!v) {
    return LaneletSequence{};
  }
  auto g = graph_->withoutLaneChanges();
  auto begin = *v;
  while (true) {
    auto inEdges = boost::in_edges(*v, g);
    if (std::distance(inEdges.first, inEdges.second) != 1) {
      break;  // lanes merge
    }
    auto next = boost::source(*inEdges.first, g);
    if (boost::out_degree(next, g) != 1) {
      break;  // other lane merges
    }
    v = next;
    if (v == begin) {
      break;  // we are in a loop
    }
  }
  return remainingLaneImpl(*v, g);
}

LaneletSequence Route::remainingLane(const ConstLanelet& ll) const {
  auto v = graph_->getVertex(ll);
  if (!v) {
    return LaneletSequence{};
  }
  return remainingLaneImpl(*v, graph_->withoutLaneChanges());
}

double Route::length2d() const {
  return std::accumulate(shortestPath_.begin(), shortestPath_.end(), 0.,
                         [](double num, const ConstLanelet& ll) { return num + geometry::length2d(ll); });
}

size_t Route::numLanes() const {
  std::set<LaneId> lanes;
  auto& g = graph_->get();
  for (auto v : g.vertex_set()) {
    lanes.emplace(g[v].laneId);
  }
  return lanes.size();
}

LaneletMapPtr Route::getDebugLaneletMap() const {
  // we need std::map because of its iterator validity guarantee at insertion
  std::map<DebugEdge, LineString3d> edgeMap;
  std::map<Id, Point2d> vertexMap;
  const auto& g = graph_->get();
  auto addEdge = [&](RouteGraph::Edge e) {
    addRelation(edgeMap, vertexMap, g[boost::source(e, g)], g[boost::target(e, g)], g[e].relation);
  };
  for (auto v : graph_->get().vertex_set()) {
    auto outEdges = boost::out_edges(v, graph_->get());
    std::for_each(outEdges.first, outEdges.second, addEdge);
  }
  // Mark shortest path
  for (const auto& el : shortestPath_) {
    vertexMap[el.id()].setAttribute("shortest_path", true);
  }
  auto map = utils::createMap(utils::transform(edgeMap, [](auto& e) { return e.second; }));
  for (const auto& it : vertexMap) {
    map->add(utils::to3D(it.second));
  }
  return map;
}

size_t Route::size() const { return boost::num_vertices(graph_->get()); }

LaneletRelations Route::followingRelations(const ConstLanelet& lanelet) const {
  auto v = graph_->getVertex(lanelet);
  if (!v) {
    return {};
  }
  return getRelations(*v, graph_->withoutLaneChanges());
}

ConstLanelets Route::following(const ConstLanelet& lanelet) const {
  auto v = graph_->getVertex(lanelet);
  if (!v) {
    return {};
  }
  return getLanelets(*v, graph_->withoutLaneChanges());
}

LaneletRelations Route::previousRelations(const ConstLanelet& lanelet) const {
  auto v = graph_->getVertex(lanelet);
  if (!v) {
    return {};
  }
  return getRelations<true>(*v, graph_->withoutLaneChanges());
}

ConstLanelets Route::previous(const ConstLanelet& lanelet) const {
  auto v = graph_->getVertex(lanelet);
  if (!v) {
    return {};
  }
  return getLanelets<true>(*v, graph_->withoutLaneChanges());
}

Optional<LaneletRelation> Route::leftRelation(const ConstLanelet& lanelet) const {
  auto v = graph_->getVertex(lanelet);
  if (!v) {
    return {};
  }
  return getSingleRelation(*v, graph_->somehowLeft());
}

LaneletRelations Route::leftRelations(const ConstLanelet& lanelet) const {
  LaneletRelations result;
  auto next = graph_->getVertex(lanelet);
  RelationType type;
  auto g = graph_->somehowLeft();
  std::tie(next, type) = getNextVertex(*next, g);
  while (!!next) {
    result.emplace_back(LaneletRelation{g[*next].lanelet, type});
    std::tie(next, type) = getNextVertex(*next, g);
  }
  return result;
}

Optional<LaneletRelation> Route::rightRelation(const ConstLanelet& lanelet) const {
  auto v = graph_->getVertex(lanelet);
  if (!v) {
    return {};
  }
  return getSingleRelation(*v, graph_->somehowRight());
}

LaneletRelations Route::rightRelations(const ConstLanelet& lanelet) const {
  LaneletRelations result;
  auto next = graph_->getVertex(lanelet);
  RelationType type;
  auto g = graph_->somehowRight();
  std::tie(next, type) = getNextVertex(*next, g);
  while (!!next) {
    result.emplace_back(LaneletRelation{g[*next].lanelet, type});
    std::tie(next, type) = getNextVertex(*next, g);
  }
  return result;
}

void Route::forEachSuccessor(const ConstLanelet& lanelet, const LaneletVisitFunction& f) const {
  auto v = graph_->getVertex(lanelet);
  if (!v) {
    return;
  }
  auto g = graph_->withLaneChanges();
  internal::DijkstraStyleSearch<FilteredRouteGraph> search(g);
  search.query(*v, [&](const internal::VertexVisitInformation& i) -> bool {
    return f(LaneletVisitInformation{graph_->get()[i.vertex].lanelet, graph_->get()[i.predecessor].lanelet, i.cost,
                                     i.length, i.numLaneChanges});
  });
}

void Route::forEachPredecessor(const ConstLanelet& lanelet, const LaneletVisitFunction& f) const {
  auto v = graph_->getVertex(lanelet);
  if (!v) {
    return;
  }
  auto g = graph_->withLaneChanges();
  auto gInv = boost::make_reverse_graph(g);
  internal::DijkstraStyleSearch<decltype(gInv)> search(gInv);
  search.query(*v, [&](const internal::VertexVisitInformation& i) -> bool {
    return f(LaneletVisitInformation{graph_->get()[i.vertex].lanelet, graph_->get()[i.predecessor].lanelet, i.cost,
                                     i.length, i.numLaneChanges});
  });
}

ConstLanelets Route::conflictingInRoute(const ConstLanelet& lanelet) const {
  auto v = graph_->getVertex(lanelet);
  if (!v) {
    return {};
  }
  return getLanelets(*v, graph_->conflicting());
}

ConstLaneletOrAreas Route::conflictingInMap(const ConstLanelet& lanelet) const {
  auto v = graph_->getVertex(lanelet);
  if (!v) {
    return {};
  }
  return graph_->get()[*v].conflictingInMap;
}

ConstLaneletOrAreas lanelet::routing::Route::allConflictingInMap() const {
  auto& g = graph_->get();
  return utils::concatenateRange(g.vertex_set(), [&](auto v) {
    auto& conf = g[v].conflictingInMap;
    return std::make_pair(std::begin(conf), std::end(conf));
  });
}

bool Route::contains(const ConstLanelet& lanelet) const { return !!graph_->getVertex(lanelet); }

template <RelationType Expect>
void checkRelationIs(Route::Errors& errors, Id source, Id dest, RelationType sourceRel, RelationType targetRel) {
  if ((Expect & targetRel) != RelationType::None) {
    auto sourceStr = std::to_string(source);
    auto destStr = std::to_string(dest);
    errors.emplace_back("Lanelet " + sourceStr + " is " + relationToString(sourceRel) + "of/with " + destStr +
                        ", but " + destStr + " is " + relationToString(targetRel) + " with/of if!");
  }
}
Route::Errors Route::checkValidity(bool throwOnError) const {
  Errors errors;
  // All elements of the shortest path are in the route
  for (const auto& ll : shortestPath_) {
    if (!contains(ll)) {
      errors.emplace_back("Lanelet " + std::to_string(ll.id()) + " of shortest path is not part of the route!");
    }
  }
  // Check if all relations are back and forth
  auto g = graph_->get();
  auto edges = boost::edges(g);
  std::for_each(edges.first, edges.second, [&](internal::RouteGraph::Edge e) {
    // get reverse edge
    decltype(e) eRev;
    bool exists = false;
    std::tie(eRev, exists) = boost::edge(boost::target(e, g), boost::source(e, g), g);
    auto sourceId = g[boost::source(e, g)].lanelet.id();
    auto targetId = g[boost::target(e, g)].lanelet.id();
    auto sourceRelation = g[e].relation;
    if (!exists) {
      if (g[e].relation != RelationType::Successor) {
        errors.emplace_back("Lanelet " + std::to_string(sourceId) + " is " + relationToString(sourceRelation) +
                            " of/with lanelet " + std::to_string(targetId) + ", but there is no relation back!");
      }
      return;
    }
    auto targetRelation = g[eRev].relation;
    switch (sourceRelation) {
      case RelationType::Conflicting:
        checkRelationIs<RelationType::Conflicting>(errors, sourceId, targetId, sourceRelation, targetRelation);
        break;
      case RelationType::Left:
      case RelationType::AdjacentLeft:
        checkRelationIs<RelationType::Right | RelationType::AdjacentRight>(errors, sourceId, targetId, sourceRelation,
                                                                           targetRelation);
        break;
      case RelationType::Right:
      case RelationType::AdjacentRight:
        checkRelationIs<RelationType::Left | RelationType::AdjacentLeft>(errors, sourceId, targetId, sourceRelation,
                                                                         targetRelation);
        break;
      case RelationType::Successor:  // anything is ok here or already checked for the reverse edge
        break;
      default:
        errors.emplace_back("Unsupported relation type found in graph for lanelet " + std::to_string(sourceId) + ": " +
                            std::to_string(static_cast<RelationUnderlyingType>(g[eRev].relation)));
    }
  });

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

}  // namespace routing
}  // namespace lanelet
