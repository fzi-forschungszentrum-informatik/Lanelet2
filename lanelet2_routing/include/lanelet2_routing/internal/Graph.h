#pragma once

#include <lanelet2_core/primitives/LaneletOrArea.h>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <map>
#include <utility>

#include "lanelet2_routing/Exceptions.h"
#include "lanelet2_routing/Forward.h"

namespace lanelet {
namespace routing {
namespace internal {
/** @brief Internal information of a vertex in the graph
 *  If A* search is adapted, this could hold information about longitude and latitude. */
struct VertexInfo {
  // careful. You must be sure that this is indeed a lanelet
  const ConstLanelet& lanelet() const { return static_cast<const ConstLanelet&>(laneletOrArea); }
  const ConstArea& area() const { return static_cast<const ConstArea&>(laneletOrArea); }
  const ConstLaneletOrArea& get() const { return laneletOrArea; }
  ConstLaneletOrArea laneletOrArea;
};

/** @brief Internal information of a vertex in the route graph */
struct RouteVertexInfo {
  const ConstLanelet& get() const { return lanelet; }

  ConstLanelet lanelet;
  LaneId laneId{};
  ConstLaneletOrAreas conflictingInMap;
};

/** @brief Internal information of an edge in the graph */
struct EdgeInfo {
  double routingCost;     ///< Calculcated routing cost. Infinity if not routable
  RoutingCostId costId;   ///< ID of the routing cost module that was used to calculate cost
  RelationType relation;  ///< Relation between the two lanelets. E.g. SUCCESSOR or CONFLICTING.
};

/// General graph type definitions
using GraphType = boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS, VertexInfo, EdgeInfo>;
using RouteGraphType =
    boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS, RouteVertexInfo, EdgeInfo>;
using GraphTraits = boost::graph_traits<GraphType>;

template <typename BaseGraphT>
struct EdgeCostFilter;

/// Filtered graphs provide a filtered view on a graph. Here we can filter out types of relations (e.g. CONFLICTING) or
/// filter by routing cost module ID.
template <typename BaseGraphT>
using FilteredGraphT = boost::filtered_graph<BaseGraphT, EdgeCostFilter<BaseGraphT>>;

template <typename BaseGraphT>
using FilteredGraphTraits = boost::graph_traits<FilteredGraphT<BaseGraphT>>;

using FilteredRoutingGraph = FilteredGraphT<GraphType>;
using FilteredRouteGraph = FilteredGraphT<RouteGraphType>;

/** @brief An internal edge filter to get a filtered view on the graph. */
template <typename GraphT>
struct EdgeCostFilter {
  /// Needed to be able to iterate edges
  EdgeCostFilter() = default;

  /// @brief Constructor for a filter that includes all relation types.
  /// @param graph Base routing graph
  /// @param routingCostId Edges with this routing cost ID will pass
  EdgeCostFilter(const GraphT& graph, RoutingCostId routingCostId)
      : routingCostId_{routingCostId},
        pmRelation_{boost::get(&EdgeInfo::relation, graph)},
        pmIds_{boost::get(&EdgeInfo::costId, graph)} {}

  /// @brief Constructor for a filter that includes all allowed relations in 'relation'
  /// @param graph Base routing graph
  /// @param routingCostId Edges with this routing cost ID will pass
  /// @param relation Relation that will pass the filter
  EdgeCostFilter(const GraphT& graph, RoutingCostId routingCostId, const RelationType& relation)
      : routingCostId_{routingCostId},
        relations_{relation},
        pmRelation_{boost::get(&EdgeInfo::relation, graph)},
        pmIds_{boost::get(&EdgeInfo::costId, graph)} {}

  /// @brief Operator that determines wheter an edge should pass the filter or not.
  template <typename Edge>
  inline bool operator()(const Edge& e) const {
    return boost::get(pmIds_, e) == routingCostId_ &&
           (relations_ == allRelations() || bool(relations_ & boost::get(pmRelation_, e)));
  }

 private:
  RoutingCostId routingCostId_{0};          ///< Id of the routing cost functor to use
  RelationType relations_{allRelations()};  ///< Relations that pass the filter
  typename boost::property_map<const GraphT, RelationType EdgeInfo::*>::const_type
      pmRelation_;  ///< Property map to the relations of edges in the graph
  typename boost::property_map<const GraphT, RoutingCostId EdgeInfo::*>::const_type
      pmIds_;  ///< Property map to the routing cost IDs of the edges
};

using LaneletOrAreaToVertex = std::unordered_map<ConstLaneletOrArea, std::uint32_t>;
using FilteredGraphDesc = std::pair<size_t, RelationType>;

/// @brief Manages the actual routing graph and provieds different views on the edges (lazily computed)
template <typename BaseGraphT>
class Graph {
 public:
  using FilteredGraph = FilteredGraphT<BaseGraphT>;
  using CostFilter = EdgeCostFilter<BaseGraphT>;
  using Vertex = typename boost::graph_traits<BaseGraphT>::vertex_descriptor;
  using Edge = typename boost::graph_traits<BaseGraphT>::edge_descriptor;

  explicit Graph(size_t numRoutingCosts) : numRoutingCosts_{numRoutingCosts} {}

  inline const BaseGraphT& get() const noexcept { return graph_; }
  inline BaseGraphT& get() noexcept { return graph_; }
  inline size_t numRoutingCosts() const noexcept { return numRoutingCosts_; }
  inline const LaneletOrAreaToVertex& vertexLookup() const noexcept { return laneletOrAreaToVertex_; }

  FilteredGraph withLaneChanges(RoutingCostId routingCostId = 0) const {
    return getFilteredGraph(routingCostId, RelationType::Successor | RelationType::Left | RelationType::Right);
  }

  FilteredGraph withoutLaneChanges(RoutingCostId routingCostId = 0) const {
    return getFilteredGraph(routingCostId, RelationType::Successor);
  }

  FilteredGraph withAreasAndLaneChanges(RoutingCostId routingCostId = 0) const {
    return getFilteredGraph(routingCostId,
                            RelationType::Successor | RelationType::Left | RelationType::Right | RelationType::Area);
  }

  FilteredGraph withAreasWithoutLaneChanges(RoutingCostId routingCostId = 0) const {
    return getFilteredGraph(routingCostId, RelationType::Successor | RelationType::Area);
  }

  FilteredGraph left(RoutingCostId routingCostId = 0) const {
    return getFilteredGraph(routingCostId, RelationType::Left);
  }
  FilteredGraph somehowLeft(RoutingCostId routingCostId = 0) const {
    return getFilteredGraph(routingCostId, RelationType::Left | RelationType::AdjacentLeft);
  }

  FilteredGraph right(RoutingCostId routingCostId = 0) const {
    return getFilteredGraph(routingCostId, RelationType::Right);
  }
  FilteredGraph somehowRight(RoutingCostId routingCostId = 0) const {
    return getFilteredGraph(routingCostId, RelationType::Right | RelationType::AdjacentRight);
  }

  FilteredGraph adjacentLeft(RoutingCostId routingCostId = 0) const {
    return getFilteredGraph(routingCostId, RelationType::AdjacentLeft);
  }

  FilteredGraph adjacentRight(RoutingCostId routingCostId = 0) const {
    return getFilteredGraph(routingCostId, RelationType::AdjacentRight);
  }

  FilteredGraph conflicting(RoutingCostId routingCostId = 0) const {
    return getFilteredGraph(routingCostId, RelationType::Conflicting);
  }

  FilteredGraph withoutConflicting(RoutingCostId routingCostId = 0) const {
    return getFilteredGraph(routingCostId, allRelations() | ~RelationType::Conflicting);
  }

  inline bool empty() const noexcept { return laneletOrAreaToVertex_.empty(); }

  //! add new lanelet to graph
  inline Vertex addVertex(const typename BaseGraphT::vertex_property_type& property) {
    GraphType::vertex_descriptor vd = 0;
    vd = boost::add_vertex(graph_);
    graph_[vd] = property;
    laneletOrAreaToVertex_.emplace(property.get(), vd);
    return vd;
  }

  void addEdge(const ConstLaneletOrArea& from, const ConstLaneletOrArea& to, const EdgeInfo& edgeInfo) {
    auto fromVertex = getVertex(from);
    auto toVertex = getVertex(to);
    if (!fromVertex || !toVertex) {
      assert(false && "Lanelet/Area is not part of the graph.");  // NOLINT
      return;
    }
    addEdge(*fromVertex, *toVertex, edgeInfo);
  }

  void addEdge(Vertex from, Vertex to, const EdgeInfo& edgeInfo) {
    if (!std::isfinite(edgeInfo.routingCost)) {
      return;
    }
    if (edgeInfo.routingCost < 0.) {
      throw RoutingGraphError{"Negative costs calculated by routing cost module!"};
    }
    auto edge = boost::add_edge(from, to, graph_);
    assert(edge.second && "Edge could not be added to the graph.");
    graph_[edge.first] = edgeInfo;
  }

  /** @brief Received the edgeInfo between two given vertices
   *  @return EdgeInfo or nothing if there's no edge */
  Optional<EdgeInfo> getEdgeInfo(const ConstLanelet& from, const ConstLanelet& to) const noexcept {
    return getEdgeInfoFor(from, to, graph_);
  }

  /** @brief Received the edgeInfo between two given vertices and a given (filtered)graph
   *  @return EdgeInfo or nothing if there's no edge */
  template <typename Graph>
  Optional<EdgeInfo> getEdgeInfoFor(const ConstLanelet& from, const ConstLanelet& to, const Graph& g) const noexcept {
    auto fromVertex = getVertex(from);
    auto toVertex = getVertex(to);
    if (!fromVertex || !toVertex) {
      return {};
    }
    auto edgeToNext{boost::edge(*fromVertex, *toVertex, g)};
    if (edgeToNext.second) {
      return g[edgeToNext.first];
    }
    return {};
  }

  //! Helper function to determine the graph vertex of a given lanelet
  Optional<typename boost::graph_traits<BaseGraphT>::vertex_descriptor> getVertex(
      const ConstLaneletOrArea& lanelet) const noexcept {
    try {
      return laneletOrAreaToVertex_.at(lanelet);
    } catch (std::out_of_range&) {
      return {};
    }
  }

 private:
  FilteredGraph getFilteredGraph(RoutingCostId routingCostId, RelationType relations) const {
    if (routingCostId >= numRoutingCosts_) {
      throw InvalidInputError("Routing Cost ID is higher than the number of routing modules.");
    }
    return FilteredGraph(graph_, CostFilter(graph_, routingCostId, relations));
  }
  BaseGraphT graph_;                             //!< The actual graph object
  LaneletOrAreaToVertex laneletOrAreaToVertex_;  //!< Mapping of lanelets/areas to vertices of the graph
  size_t numRoutingCosts_;                       //!< Number of available routing cost calculation methods
};

class RoutingGraphGraph : public Graph<GraphType> {
  using Graph::Graph;
};
class RouteGraph : public Graph<RouteGraphType> {
  using Graph::Graph;
};

}  // namespace internal
}  // namespace routing
}  // namespace lanelet
