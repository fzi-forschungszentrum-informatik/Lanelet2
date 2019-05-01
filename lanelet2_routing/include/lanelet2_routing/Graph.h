#pragma once

#include <Forward.h>
#include <lanelet2_core/primitives/LaneletOrArea.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <set>
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
  RelationType relation;  ///< Relation between the two lanelets. E.g. SUCCEEDING or CONFLICTING.
};

/// General graph type definitions
using GraphType = boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS, VertexInfo, EdgeInfo>;
using GraphTraits = boost::graph_traits<GraphType>;

struct EdgeCostFilter;
/// Filtered graphs provide a filtered view on a graph. Here we can filter out types of relations (e.g. CONFLICTING) or
/// filter by routing cost module ID.
using FilteredGraph = boost::filtered_graph<GraphType, EdgeCostFilter>;
using FilteredGraphs = std::vector<FilteredGraph>;
using FilteredGraphTraits = boost::graph_traits<FilteredGraph>;

/// @brief Returns all valid relations.
static RelationTypes relationTypeSet() {
  RelationTypes completeSet;
  for (auto i = 0u; i < numRelationTypes(); i++) {
    completeSet.emplace_back(static_cast<RelationType>(i));
  }
  return completeSet;
}

/** @brief An internal edge filter to get a filtered view on the graph.
 *  It is always restricted to one routing cost module ID. But the relations between lanelets that can pass the filter
 * can be configured to:
 *    * one specific type (e.g. CONFLICTING)
 *    * all types
 *    * all except one specified type
 *    * a given set of types */
struct EdgeCostFilter {
  /// Needed to be able to iterate edges
  EdgeCostFilter() = default;

  /// @brief Constructor for a filter that includes all relation types.
  /// @param graph Base routing graph
  /// @param routingCostId Edges with this routing cost ID will pass
  EdgeCostFilter(const GraphType& graph, RoutingCostId routingCostId)
      : routingCostId_{routingCostId},
        relations_{relationTypeSet()},
        pmRelation_{boost::get(&EdgeInfo::relation, graph)},
        pmIds_{boost::get(&EdgeInfo::costId, graph)} {}

  /// @brief Constructor for a filter that includes one specific relation type.
  /// @param graph Base routing graph
  /// @param routingCostId Edges with this routing cost ID will pass
  /// @param relation Relation that will pass the filter
  EdgeCostFilter(const GraphType& graph, RoutingCostId routingCostId, const RelationType& relation)
      : routingCostId_{routingCostId},
        relations_{relation},
        pmRelation_{boost::get(&EdgeInfo::relation, graph)},
        pmIds_{boost::get(&EdgeInfo::costId, graph)} {}

  /// @brief Constructor for a filter that includes a given set of relations.
  /// @param graph Base routing graph
  /// @param routingCostId Edges with this routing cost ID will pass
  /// @param relationSet Relations that will pass the filter
  EdgeCostFilter(const GraphType& graph, RoutingCostId routingCostId, RelationTypes relationSet)
      : routingCostId_{routingCostId},
        relations_{std::move(relationSet)},
        pmRelation_{boost::get(&EdgeInfo::relation, graph)},
        pmIds_{boost::get(&EdgeInfo::costId, graph)} {
    std::sort(relations_.begin(), relations_.end());
    relations_.erase(std::unique(relations_.begin(), relations_.end()), relations_.end());
  }

  /// @brief Constructor for a filter that includes all types of edges except one.
  /// @param graph Base routing graph
  /// @param routingCostId Edges with this routing cost ID will pass
  /// @param relationSet All allowed relations including the one that will be excluded. Typically RelationTypeSet().
  /// @param exclude Relation that will be erased from the ones specified in relationSet
  /// This constructor is typically used to get all edges except CONFLICTING
  EdgeCostFilter(const GraphType& graph, RoutingCostId routingCostId, RelationTypes relationSet,
                 const RelationType exclude)
      : routingCostId_{routingCostId},
        relations_{std::move(relationSet)},
        pmRelation_{boost::get(&EdgeInfo::relation, graph)},
        pmIds_{boost::get(&EdgeInfo::costId, graph)} {
    std::sort(relations_.begin(), relations_.end());
    relations_.erase(std::unique(relations_.begin(), relations_.end()), relations_.end());
    relations_.erase(std::lower_bound(relations_.begin(), relations_.end(), exclude));
  }

  /// @brief Operator that determines wheter an edge should pass the filter or not.
  template <typename Edge>
  inline bool operator()(const Edge& e) const {
    return boost::get(pmIds_, e) == routingCostId_ &&
           (relations_.size() == numRelationTypes() ||
            std::binary_search(relations_.begin(), relations_.end(), boost::get(pmRelation_, e)));
  }

 private:
  RoutingCostId routingCostId_{0};
  RelationTypes relations_;  ///< Relations that pass the filter
  boost::property_map<const GraphType, RelationType EdgeInfo::*>::const_type
      pmRelation_;  ///< Property map to the relations of edges in the graph
  boost::property_map<const GraphType, RoutingCostId EdgeInfo::*>::const_type
      pmIds_;  ///< Property map to the routing cost IDs of the edges
};

/** @brief Helper struct to implement PImpl slim down the header file.
 *  Provides various filtered views on graphs needed to answer queries. */
struct FilteredGraphContainer {  // NOLINT
  FilteredGraphContainer() = delete;
  explicit FilteredGraphContainer(const GraphType& graph)
      : left{FilteredGraph(graph, EdgeCostFilter(graph, 0, RelationType::Left))},
        adjacentLeft{FilteredGraph(graph, EdgeCostFilter(graph, 0, RelationType::AdjacentLeft))},
        right{FilteredGraph(graph, EdgeCostFilter(graph, 0, RelationType::Right))},
        adjacentRight{FilteredGraph(graph, EdgeCostFilter(graph, 0, RelationType::AdjacentRight))},
        conflicting{FilteredGraph(graph, EdgeCostFilter(graph, 0, RelationType::Conflicting))},
        withoutConflicting{
            FilteredGraph(graph, EdgeCostFilter(graph, 0, relationTypeSet(), RelationType::Conflicting))} {}
  FilteredGraphs withLaneChanges;
  FilteredGraphs withoutLaneChanges;
  FilteredGraphs withAreasAndLaneChanges;
  FilteredGraphs withAreasWithoutLaneChanges;
  FilteredGraph left;
  FilteredGraph adjacentLeft;
  FilteredGraph right;
  FilteredGraph adjacentRight;
  FilteredGraph conflicting;
  FilteredGraph withoutConflicting;
};

using LaneletOrAreaToVertex = std::unordered_map<ConstLaneletOrArea, std::uint32_t>;

/// @brief Dummy object to be able to apply PImpl pattern
struct Graph {
  GraphType graph;                              //!< The actual graph object
  LaneletOrAreaToVertex laneletOrAreaToVertex;  //!< Mapping of lanelets/areas to vertices of the graph
  size_t numRoutingCosts;                       ///< Number of available routing cost calculation methods

  explicit Graph(size_t numRoutingCosts) : numRoutingCosts{numRoutingCosts} {}

  const FilteredGraph& withLaneChanges(size_t routingCostId) {
    if (routingCostId >= numRoutingCosts) {
      throw InvalidInputError("Routing Cost ID is higher than the number of routing modules.");
    }
    return filteredGraphs().withLaneChanges[routingCostId];
  }

  const FilteredGraph& withoutLaneChanges(size_t routingCostId) {
    if (routingCostId >= numRoutingCosts) {
      throw InvalidInputError("Routing Cost ID is higher than the number of routing modules.");
    }
    return filteredGraphs().withoutLaneChanges[routingCostId];
  }
  const FilteredGraph& withAreasAndLaneChanges(size_t routingCostId) {
    if (routingCostId >= numRoutingCosts) {
      throw InvalidInputError("Routing Cost ID is higher than the number of routing modules.");
    }
    return filteredGraphs().withAreasAndLaneChanges[routingCostId];
  }

  const FilteredGraph& withAreasWithoutLaneChanges(size_t routingCostId) {
    if (routingCostId >= numRoutingCosts) {
      throw InvalidInputError("Routing Cost ID is higher than the number of routing modules.");
    }
    return filteredGraphs().withAreasWithoutLaneChanges[routingCostId];
  }

  const FilteredGraphContainer& filteredGraphs() {
    if (!filteredGraphs_) {
      filteredGraphs_ = std::make_unique<FilteredGraphContainer>(graph);
      // Set up filtered graphs
      filteredGraphs_->withLaneChanges.reserve(numRoutingCosts);
      filteredGraphs_->withoutLaneChanges.reserve(numRoutingCosts);
      RelationTypes withoutLaneChanges{RelationType::Successor, RelationType::Merging, RelationType::Diverging};
      RelationTypes withLaneChanges{withoutLaneChanges};
      withLaneChanges.push_back(RelationType::Left);
      withLaneChanges.push_back(RelationType::Right);
      RelationTypes withAreaWithoutLaneChanges{withoutLaneChanges};
      RelationTypes withAreaAndLaneChanges{withLaneChanges};
      withAreaAndLaneChanges.push_back(RelationType::Area);
      withAreaWithoutLaneChanges.push_back(RelationType::Area);
      auto& g = *filteredGraphs_;
      for (RoutingCostId rci = 0; rci < numRoutingCosts; rci++) {
        EdgeCostFilter predWithLaneChanges(graph, rci, withLaneChanges);
        EdgeCostFilter predWithoutLaneChanges(graph, rci, withoutLaneChanges);
        EdgeCostFilter predAreaWithLaneChanges(graph, rci, withAreaAndLaneChanges);
        EdgeCostFilter predAreaWithoutLaneChanges(graph, rci, withAreaWithoutLaneChanges);

        g.withLaneChanges.emplace_back(FilteredGraph(graph, predWithLaneChanges));
        g.withoutLaneChanges.emplace_back(FilteredGraph(graph, predWithoutLaneChanges));
        g.withAreasAndLaneChanges.emplace_back(FilteredGraph(graph, predAreaWithLaneChanges));
        g.withAreasWithoutLaneChanges.emplace_back(FilteredGraph(graph, predAreaWithoutLaneChanges));
      }
    }
    return *filteredGraphs_;
  }

  //! add new lanelet to graph. Can no longer be called after filteredGraphs has been called!
  inline void addVertex(const ConstLaneletOrArea& ll) {
    assert(!filteredGraphs_);
    GraphType::vertex_descriptor vd;
    vd = boost::add_vertex(graph);
    graph[vd].laneletOrArea = ll;
    laneletOrAreaToVertex.emplace(std::pair<ConstLaneletOrArea, uint32_t>(ll, vd));
  }

  void addEdge(const ConstLaneletOrArea& from, const ConstLaneletOrArea& to, const EdgeInfo& edgeInfo) {
    using EdgePair = std::pair<GraphType::edge_descriptor, bool>;
    auto fromVertex = getVertex(from);
    auto toVertex = getVertex(to);
    if (!fromVertex || !toVertex) {
      assert(false && "Lanelet/Area is not part of the graph.");  // NOLINT
      return;
    }
    EdgePair edge = boost::add_edge(*fromVertex, *toVertex, graph);
    assert(edge.second && "Edge could not be added to the graph.");
    graph[edge.first] = edgeInfo;
  }

  /** @brief Received the edgeInfo between two given vertices
   *  @return EdgeInfo or nothing if there's no edge */
  Optional<EdgeInfo> getEdgeInfo(const ConstLanelet& from, const ConstLanelet& to) const noexcept {
    return getEdgeInfoFor(from, to, graph);
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
      return laneletOrAreaToVertex.at(lanelet);
    } catch (std::out_of_range&) {
      return {};
    }
  }

 private:
  std::unique_ptr<FilteredGraphContainer>
      filteredGraphs_;  //!< Wrapper around all filtered graphs needed for the queries
};

}  // namespace routing
}  // namespace lanelet
