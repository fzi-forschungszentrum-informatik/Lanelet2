#pragma once

#include <lanelet2_core/primitives/LaneletOrArea.h>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <map>
#include <utility>

#include "lanelet2_map_learning/Exceptions.h"
#include "lanelet2_map_learning/Forward.h"

namespace lanelet {
namespace map_learning {
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

/** @brief Internal information of an edge in the graph */
struct EdgeInfo {
  RelationType relation;  ///< Relation between the two lanelets. E.g. SUCCESSOR or CONFLICTING.
};

/// General graph type definitions
using GraphType = boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS, VertexInfo, EdgeInfo>;
using GraphTraits = boost::graph_traits<GraphType>;

template <typename BaseGraphT>
struct EdgeFilter;

/// Filtered graphs provide a filtered view on a graph. Here we can filter out types of relations (e.g. CONFLICTING)
template <typename BaseGraphT>
using FilteredGraphT = boost::filtered_graph<BaseGraphT, EdgeFilter<BaseGraphT>>;

template <typename BaseGraphT>
using FilteredGraphTraits = boost::graph_traits<FilteredGraphT<BaseGraphT>>;

using FilteredMapGraph = FilteredGraphT<GraphType>;

/** @brief An internal edge filter to get a filtered view on the graph. */
template <typename GraphT>
struct EdgeFilter {
  /// Needed to be able to iterate edges
  EdgeFilter() = default;

  /// @brief Constructor for a filter that includes all relation types.
  /// @param graph Base graph
  EdgeFilter(const GraphT& graph) : pmRelation_{boost::get(&EdgeInfo::relation, graph)} {}

  /// @brief Constructor for a filter that includes all allowed relations in 'relation'
  /// @param graph Base graph
  /// @param relation Relation that will pass the filter
  EdgeFilter(const GraphT& graph, const RelationType& relation)
      : relations_{relation}, pmRelation_{boost::get(&EdgeInfo::relation, graph)} {}

  /// @brief Operator that determines wheter an edge should pass the filter or not.
  template <typename Edge>
  inline bool operator()(const Edge& e) const {
    return relations_ == allRelations() || bool(relations_ & boost::get(pmRelation_, e));
  }

 private:
  RelationType relations_{allRelations()};  ///< Relations that pass the filter
  typename boost::property_map<const GraphT, RelationType EdgeInfo::*>::const_type
      pmRelation_;  ///< Property map to the relations of edges in the graph
};

using LaneletOrAreaToVertex = std::unordered_map<ConstLaneletOrArea, std::uint32_t>;
using FilteredGraphDesc = std::pair<size_t, RelationType>;

/// @brief Manages the actual graph and provides different views on the edges (lazily computed)
template <typename BaseGraphT>
class Graph {
 public:
  using FilteredGraph = FilteredGraphT<BaseGraphT>;
  using Filter = EdgeFilter<BaseGraphT>;
  using Vertex = typename boost::graph_traits<BaseGraphT>::vertex_descriptor;
  using Edge = typename boost::graph_traits<BaseGraphT>::edge_descriptor;

  explicit Graph() {}

  inline const BaseGraphT& get() const noexcept { return graph_; }
  inline BaseGraphT& get() noexcept { return graph_; }
  inline const LaneletOrAreaToVertex& vertexLookup() const noexcept { return laneletOrAreaToVertex_; }

  FilteredGraph withAllRelations() const { return FilteredGraph(graph_, Filter(graph_)); }

  FilteredGraph withLaneChanges() const {
    return getFilteredGraph(RelationType::Successor | RelationType::Left | RelationType::Right);
  }

  FilteredGraph withoutLaneChanges() const { return getFilteredGraph(RelationType::Successor); }

  FilteredGraph withAreasAndLaneChanges() const {
    return getFilteredGraph(RelationType::Successor | RelationType::Left | RelationType::Right | RelationType::Area);
  }

  FilteredGraph withAreasWithoutLaneChanges() const {
    return getFilteredGraph(RelationType::Successor | RelationType::Area);
  }

  FilteredGraph left() const { return getFilteredGraph(RelationType::Left); }
  FilteredGraph somehowLeft() const { return getFilteredGraph(RelationType::Left | RelationType::AdjacentLeft); }

  FilteredGraph right() const { return getFilteredGraph(RelationType::Right); }
  FilteredGraph somehowRight() const { return getFilteredGraph(RelationType::Right | RelationType::AdjacentRight); }

  FilteredGraph adjacentLeft() const { return getFilteredGraph(RelationType::AdjacentLeft); }

  FilteredGraph adjacentRight() const { return getFilteredGraph(RelationType::AdjacentRight); }

  FilteredGraph conflicting() const { return getFilteredGraph(RelationType::Conflicting); }

  FilteredGraph withoutConflicting() const { return getFilteredGraph(allRelations() | ~RelationType::Conflicting); }

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
  FilteredGraph getFilteredGraph(RelationType relations) const {
    return FilteredGraph(graph_, Filter(graph_, relations));
  }
  BaseGraphT graph_;                             //!< The actual graph object
  LaneletOrAreaToVertex laneletOrAreaToVertex_;  //!< Mapping of lanelets/areas to vertices of the graph
};

class MapGraphGraph : public Graph<GraphType> {
  using Graph::Graph;
};

}  // namespace internal
}  // namespace map_learning
}  // namespace lanelet
