#include "lanelet2_map_learning/MapGraph.h"

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

#include "lanelet2_map_learning/Exceptions.h"
#include "lanelet2_map_learning/Forward.h"
#include "lanelet2_map_learning/internal/Graph.h"
#include "lanelet2_map_learning/internal/GraphUtils.h"
#include "lanelet2_map_learning/internal/MapGraphBuilder.h"
#include "lanelet2_map_learning/internal/MapGraphVisualization.h"

namespace lanelet {
namespace map_learning {

#if __cplusplus < 201703L
constexpr const char MapGraph::ParticipantHeight[];
#endif

using internal::FilteredMapGraph;
using internal::GraphType;
using internal::LaneletVertexId;
using internal::MapGraphGraph;

template <typename T>
T reservedVector(size_t size) {
  T t;
  t.reserve(size);
  return t;
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
 *  @throws MapGraphError if 'throwOnError' is true and there is more than one neighboring lanelet
 *  @param vertex Start vertex
 *  @param graph Filtered graph with the allowed type of edge
 *  @param throwOnError Decides wheter to throw or ignore an error
 *  @return Neighboring lanelet */
Optional<ConstLaneletOrArea> neighboringImpl(const GraphType::vertex_descriptor vertex, const FilteredMapGraph& graph,
                                             bool throwOnError = false) {
  auto outEdges = boost::out_edges(vertex, graph);
  if (throwOnError && std::distance(outEdges.first, outEdges.second) > 1) {
    std::string ids;
    std::for_each(outEdges.first, outEdges.second, [&graph, &ids](const auto& edge) {
      ids += " " + std::to_string(graph[boost::target(edge, graph)].laneletOrArea.id());
    });
    throw MapGraphError("More than one neighboring lanelet to " + std::to_string(graph[vertex].laneletOrArea.id()) +
                        " with this relation:" + ids);
  }
  if (outEdges.first != outEdges.second) {
    return graph[boost::target(*(outEdges.first), graph)].laneletOrArea;
  }
  return {};
}

Optional<ConstLanelet> neighboringLaneletImpl(const GraphType::vertex_descriptor vertex, const FilteredMapGraph& graph,
                                              bool throwOnError = false) {
  auto value = neighboringImpl(vertex, graph, throwOnError);
  if (!!value && value->isLanelet()) {
    return value->lanelet();
  }
  return {};
}

template <typename Func>
Optional<ConstLaneletOrArea> ifInGraph(const MapGraphGraph& g, const ConstLaneletOrArea& llt, Func f) {
  auto vertex = g.getVertex(llt);
  if (!vertex) {
    return {};
  }
  return f(*vertex);
}

template <typename Func>
Optional<ConstLanelet> ifLaneletInGraph(const MapGraphGraph& g, const ConstLanelet& llt, Func f) {
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

ConstLaneletOrAreas getAllEdgesFromGraph(const MapGraphGraph& graph, const FilteredMapGraph& subgraph,
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

ConstLanelets getLaneletEdgesFromGraph(const MapGraphGraph& graph, const FilteredMapGraph& subgraph,
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

ConstLaneletOrAreas MapGraph::getLaneletEdges(const ConstLanelet& lanelet, bool edgesOut) const {
  ConstLaneletOrAreas result;
  auto allEdges = getAllEdgesFromGraph(*graph_, graph_->withAllRelations(), lanelet, edgesOut);
  result = reservedVector<ConstLaneletOrAreas>(allEdges.size());
  for (auto& edge : allEdges) {
    if (edge.isLanelet()) {
      result.push_back(edge);
    }
  }
  return result;
}

MapGraph::MapGraph(MapGraph&& /*other*/) noexcept = default;
MapGraph& MapGraph::operator=(MapGraph&& /*other*/) noexcept = default;
MapGraph::~MapGraph() = default;

MapGraphUPtr MapGraph::build(const LaneletMap& laneletMap, const traffic_rules::TrafficRules& trafficRules,
                             const MapGraph::Configuration& config) {
  return internal::MapGraphBuilder(trafficRules, config).build(laneletMap);
}

MapGraphUPtr MapGraph::build(const LaneletSubmap& laneletSubmap, const traffic_rules::TrafficRules& trafficRules,
                             const MapGraph::Configuration& config) {
  return internal::MapGraphBuilder(trafficRules, config).build(laneletSubmap);
}

Optional<RelationType> MapGraph::routingRelation(const ConstLanelet& from, const ConstLanelet& to,
                                                 bool includeConflicting) const {
  auto edgeInfo = includeConflicting ? graph_->getEdgeInfo(from, to)
                                     : graph_->getEdgeInfoFor(from, to, graph_->withoutConflicting());
  if (!!edgeInfo) {
    return edgeInfo->relation;
  }
  return {};
}

ConstLanelets MapGraph::following(const ConstLanelet& lanelet, bool withLaneChanges) const {
  auto subgraph = withLaneChanges ? graph_->withLaneChanges() : graph_->withoutLaneChanges();
  return getLaneletEdgesFromGraph(*graph_, subgraph, lanelet, true);
}

LaneletRelations MapGraph::followingRelations(const ConstLanelet& lanelet, bool withLaneChanges) const {
  ConstLanelets foll{following(lanelet, withLaneChanges)};
  LaneletRelations result;
  for (auto const& it : foll) {
    result.emplace_back(LaneletRelation{it, *routingRelation(lanelet, it)});
  }
  return result;
}  // namespace map_learning

ConstLanelets MapGraph::previous(const ConstLanelet& lanelet, bool withLaneChanges) const {
  auto subgraph = withLaneChanges ? graph_->withLaneChanges() : graph_->withoutLaneChanges();
  return getLaneletEdgesFromGraph(*graph_, subgraph, lanelet, false);
}

LaneletRelations MapGraph::previousRelations(const ConstLanelet& lanelet, bool withLaneChanges) const {
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

ConstLanelets MapGraph::besides(const ConstLanelet& lanelet) const {
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

Optional<ConstLanelet> MapGraph::left(const ConstLanelet& lanelet) const {
  return ifLaneletInGraph(*graph_, lanelet,
                          [&](auto& vertex) { return neighboringLaneletImpl(vertex, graph_->left()); });
}

Optional<ConstLanelet> MapGraph::adjacentLeft(const ConstLanelet& lanelet) const {
  return ifLaneletInGraph(*graph_, lanelet,
                          [&](auto& vertex) { return neighboringLaneletImpl(vertex, graph_->adjacentLeft()); });
}

Optional<ConstLanelet> MapGraph::right(const ConstLanelet& lanelet) const {
  return ifLaneletInGraph(*graph_, lanelet,
                          [&](auto& vertex) { return neighboringLaneletImpl(vertex, graph_->right()); });
}

Optional<ConstLanelet> MapGraph::adjacentRight(const ConstLanelet& lanelet) const {
  return ifLaneletInGraph(*graph_, lanelet,
                          [&](auto& vertex) { return neighboringLaneletImpl(vertex, graph_->adjacentRight()); });
}

ConstLanelets MapGraph::lefts(const ConstLanelet& lanelet) const {
  return getUntilEnd(lanelet, [&](const ConstLanelet& llt) { return left(llt); });
}

ConstLanelets MapGraph::adjacentLefts(const ConstLanelet& lanelet) const {
  return getUntilEnd(lanelet, [&](const ConstLanelet& llt) { return adjacentLeft(llt); });
}

LaneletRelations MapGraph::leftRelations(const ConstLanelet& lanelet) const {
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

ConstLanelets MapGraph::rights(const ConstLanelet& lanelet) const {
  return getUntilEnd(lanelet, [&](const ConstLanelet& llt) { return right(llt); });
}

ConstLanelets MapGraph::adjacentRights(const ConstLanelet& lanelet) const {
  return getUntilEnd(lanelet, [&](const ConstLanelet& llt) { return adjacentRight(llt); });
}

LaneletRelations MapGraph::rightRelations(const ConstLanelet& lanelet) const {
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

ConstLaneletOrAreas MapGraph::conflicting(const ConstLaneletOrArea& laneletOrArea) const {
  return getAllEdgesFromGraph(*graph_, graph_->conflicting(), laneletOrArea, true);
}

void MapGraph::exportGraphML(const std::string& filename, const RelationType& edgeTypesToExclude) const {
  if (filename.empty()) {
    throw InvalidInputError("No filename passed");
  }
  RelationType relations = allRelations() & ~edgeTypesToExclude;
  internal::exportGraphMLImpl<GraphType>(filename, graph_->get(), relations);
}

void MapGraph::exportGraphViz(const std::string& filename, const RelationType& edgeTypesToExclude) const {
  if (filename.empty()) {
    throw InvalidInputError("No filename passed");
  }
  RelationType relations = allRelations() & ~edgeTypesToExclude;
  internal::exportGraphVizImpl<GraphType>(filename, graph_->get(), relations);
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

LineString3d createLineString(const Point2d& from, const Point2d& to, RelationType relation) {
  LineString2d lineString(utils::getId());
  lineString.push_back(from);
  lineString.push_back(to);
  LineString3d lineString3d(lineString);
  lineString3d.setAttribute("relation", relationToString(relation));
  return lineString3d;
}

class DebugMapBuilder {
 public:
  using LaneletOrAreaPair = std::pair<ConstLaneletOrArea, ConstLaneletOrArea>;
  explicit DebugMapBuilder(const FilteredMapGraph& graph) : graph_{graph} {}
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

    } else {
      auto pFrom = pointMap_.at(from);
      auto pTo = pointMap_.at(to);
      LineString3d lineString3d{createLineString(pFrom, pTo, edge.relation)};
      lineStringMap_.emplace(pair, lineString3d);
    }
  }

  FilteredMapGraph graph_;
  std::unordered_map<LaneletOrAreaPair, LineString3d> lineStringMap_;  // Stores all relations
  std::unordered_map<ConstLaneletOrArea, Point2d> pointMap_;           // Stores all 'edges'
};

LaneletMapPtr MapGraph::getDebugLaneletMap(bool includeAdjacent, bool includeConflicting) const {
  internal::EdgeFilter<GraphType> edgeFilter(graph_->get(),
                                             allowedRelationsfromConfiguration(includeAdjacent, includeConflicting));
  FilteredMapGraph filteredGraph(graph_->get(), edgeFilter);
  return DebugMapBuilder(filteredGraph).run(graph_->vertexLookup());
}

MapGraph::Errors MapGraph::checkValidity(bool throwOnError) const {
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

    } catch (MapGraphError& e) {
      errors.emplace_back(std::string("Left: ") + e.what());
    }
    Optional<ConstLanelet> adjacentLeft;
    try {
      adjacentLeft = neighboringLaneletImpl(vertex, graph_->adjacentLeft(), true);
    } catch (MapGraphError& e) {
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
    } catch (MapGraphError& e) {
      errors.emplace_back(std::string("Right: ") + e.what());
    }
    Optional<ConstLanelet> adjacentRight;
    try {
      adjacentRight = neighboringLaneletImpl(vertex, graph_->adjacentRight(), true);
    } catch (MapGraphError& e) {
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
    ss << "Errors found in graph:";
    for (const auto& err : errors) {
      ss << "\n\t- " << err;
    }
    throw MapGraphError(ss.str());
  }
  return errors;
}

MapGraph::MapGraph(std::unique_ptr<MapGraphGraph>&& graph, LaneletSubmapConstPtr&& passableMap)
    : graph_{std::move(graph)}, passableLaneletSubmap_{std::move(passableMap)} {}

}  // namespace map_learning
}  // namespace lanelet
