#pragma once

#include <Forward.h>
#include <lanelet2_core/primitives/LaneletOrArea.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <map>
#include <utility>

namespace lanelet {
namespace routing {

/** @brief Internal information of a vertex in the graph
 *  If A* search is adapted, this could hold information about longitude and latitude. */
struct VertexInfo {
  // careful. You must be sure that this is indeed a lanelet
  const ConstLanelet& lanelet() const { return static_cast<const ConstLanelet&>(laneletOrArea); }
  const ConstArea& area() const { return static_cast<const ConstArea&>(laneletOrArea); }
  ConstLaneletOrArea laneletOrArea;
};

/** @brief Internal information of an edge in the graph */
struct EdgeInfo {
  RoutingCostId costId;   ///< ID of the routing cost module that was used to calculate cost
  double routingCost;     ///< Calculcated routing cost. Infinity if not routable
  RelationType relation;  ///< Relation between the two lanelets. E.g. SUCCESSOR or CONFLICTING.
};

/// General graph type definitions
using GraphType = boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS, VertexInfo, EdgeInfo>;
using GraphTraits = boost::graph_traits<GraphType>;

struct EdgeCostFilter;
/// Filtered graphs provide a filtered view on a graph. Here we can filter out types of relations (e.g. CONFLICTING) or
/// filter by routing cost module ID.
using FilteredGraph = boost::filtered_graph<GraphType, EdgeCostFilter>;
using FilteredGraphTraits = boost::graph_traits<FilteredGraph>;

/** @brief An internal edge filter to get a filtered view on the graph. */
struct EdgeCostFilter {
  /// Needed to be able to iterate edges
  EdgeCostFilter() = default;

  /// @brief Constructor for a filter that includes all relation types.
  /// @param graph Base routing graph
  /// @param routingCostId Edges with this routing cost ID will pass
  EdgeCostFilter(const GraphType& graph, RoutingCostId routingCostId)
      : routingCostId_{routingCostId},
        pmRelation_{boost::get(&EdgeInfo::relation, graph)},
        pmIds_{boost::get(&EdgeInfo::costId, graph)} {}

  /// @brief Constructor for a filter that includes all allowed relations in 'relation'
  /// @param graph Base routing graph
  /// @param routingCostId Edges with this routing cost ID will pass
  /// @param relation Relation that will pass the filter
  EdgeCostFilter(const GraphType& graph, RoutingCostId routingCostId, const RelationType& relation)
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
  boost::property_map<const GraphType, RelationType EdgeInfo::*>::const_type
      pmRelation_;  ///< Property map to the relations of edges in the graph
  boost::property_map<const GraphType, RoutingCostId EdgeInfo::*>::const_type
      pmIds_;  ///< Property map to the routing cost IDs of the edges
};

using LaneletOrAreaToVertex = std::unordered_map<ConstLaneletOrArea, std::uint32_t>;
using FilteredGraphDesc = std::pair<size_t, RelationType>;

/// @brief Manages the actual routing graph and provieds different views on the edges (lazily computed)
class Graph {
 public:
  explicit Graph(size_t numRoutingCosts) : numRoutingCosts_{numRoutingCosts} {}

  inline const GraphType& get() const noexcept { return graph_; }
  inline size_t numRoutingCosts() const noexcept { return numRoutingCosts_; }
  inline const LaneletOrAreaToVertex& vertexLookup() const noexcept { return laneletOrAreaToVertex_; }

  const FilteredGraph& withLaneChanges(RoutingCostId routingCostId = 0) {
    return getFilteredGraph(routingCostId, RelationType::Successor | RelationType::Left | RelationType::Right);
  }

  const FilteredGraph& withoutLaneChanges(RoutingCostId routingCostId = 0) {
    return getFilteredGraph(routingCostId, RelationType::Successor);
  }

  const FilteredGraph& withAreasAndLaneChanges(RoutingCostId routingCostId = 0) {
    return getFilteredGraph(routingCostId,
                            RelationType::Successor | RelationType::Left | RelationType::Right | RelationType::Area);
  }

  const FilteredGraph& withAreasWithoutLaneChanges(RoutingCostId routingCostId = 0) {
    return getFilteredGraph(routingCostId, RelationType::Successor | RelationType::Area);
  }

  const FilteredGraph& left(RoutingCostId routingCostId = 0) {
    return getFilteredGraph(routingCostId, RelationType::Left);
  }

  const FilteredGraph& right(RoutingCostId routingCostId = 0) {
    return getFilteredGraph(routingCostId, RelationType::Right);
  }

  const FilteredGraph& adjacentLeft(RoutingCostId routingCostId = 0) {
    return getFilteredGraph(routingCostId, RelationType::AdjacentLeft);
  }

  const FilteredGraph& adjacentRight(RoutingCostId routingCostId = 0) {
    return getFilteredGraph(routingCostId, RelationType::AdjacentRight);
  }

  const FilteredGraph& conflicting(RoutingCostId routingCostId = 0) {
    return getFilteredGraph(routingCostId, RelationType::Conflicting);
  }

  const FilteredGraph& withoutConflicting(RoutingCostId routingCostId = 0) {
    return getFilteredGraph(routingCostId, allRelations() | ~RelationType::Conflicting);
  }

  //! add new lanelet to graph
  inline void addVertex(const ConstLaneletOrArea& ll) {
    if (!filteredGraphs_.empty()) {
      filteredGraphs_.clear();
    }
    GraphType::vertex_descriptor vd;
    vd = boost::add_vertex(graph_);
    graph_[vd].laneletOrArea = ll;
    laneletOrAreaToVertex_.emplace(std::pair<ConstLaneletOrArea, uint32_t>(ll, vd));
  }

  void addEdge(const ConstLaneletOrArea& from, const ConstLaneletOrArea& to, const EdgeInfo& edgeInfo) {
    if (!filteredGraphs_.empty()) {
      filteredGraphs_.clear();
    }
    using EdgePair = std::pair<GraphType::edge_descriptor, bool>;
    auto fromVertex = getVertex(from);
    auto toVertex = getVertex(to);
    if (!fromVertex || !toVertex) {
      assert(false && "Lanelet/Area is not part of the graph.");  // NOLINT
      return;
    }
    EdgePair edge = boost::add_edge(*fromVertex, *toVertex, graph_);
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
  Optional<std::uint32_t> getVertex(const ConstLaneletOrArea& lanelet) const noexcept {
    try {
      return laneletOrAreaToVertex_.at(lanelet);
    } catch (std::out_of_range&) {
      return {};
    }
  }

 private:
  FilteredGraph& getFilteredGraph(RoutingCostId routingCostId, RelationType relations) {
    if (routingCostId >= numRoutingCosts_) {
      throw InvalidInputError("Routing Cost ID is higher than the number of routing modules.");
    }
    FilteredGraphDesc descr{routingCostId, relations};
    auto it = filteredGraphs_.find(descr);
    if (it == filteredGraphs_.end()) {
      EdgeCostFilter costFilter(graph_, routingCostId, relations);
      it = filteredGraphs_
               .emplace(std::piecewise_construct, std::forward_as_tuple(routingCostId, relations),
                        std::forward_as_tuple(graph_, costFilter))
               .first;
    }
    return it->second;
  }
  GraphType graph_;                                            //!< The actual graph object
  LaneletOrAreaToVertex laneletOrAreaToVertex_;                //!< Mapping of lanelets/areas to vertices of the graph
  size_t numRoutingCosts_;                                     //!< Number of available routing cost calculation methods
  std::map<FilteredGraphDesc, FilteredGraph> filteredGraphs_;  //!< Cache for already computed filtered graphs
};

}  // namespace routing
}  // namespace lanelet
