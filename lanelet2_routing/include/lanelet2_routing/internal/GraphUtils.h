#pragma once
#include <lanelet2_core/utility/Optional.h>

#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/two_bit_color_map.hpp>

#include "lanelet2_routing/internal/Graph.h"

namespace lanelet {
namespace routing {
namespace internal {
using LaneletVertexId = GraphTraits::vertex_descriptor;
using LaneletVertexIds = std::vector<LaneletVertexId>;
using RouteLanelets = std::set<LaneletVertexId>;

template <typename ContainerT, typename T>
inline bool has(const ContainerT& c, const T& t) {
  return std::find(c.begin(), c.end(), t) != c.end();
}
template <typename ContainerT, typename T>
inline bool has(const std::set<T>& c, const T& t) {
  return c.find(t) != c.end();
}
template <RelationType R, typename GraphT, typename EdgeT>
inline bool hasRelation(const GraphT& g, EdgeT e) {
  return (g[e].relation & R) != RelationType::None;
}
template <RelationType R, typename Graph>
Optional<LaneletVertexId> getNext(LaneletVertexId ofVertex, const Graph& g) {
  auto edges = boost::out_edges(ofVertex, g);
  auto it = std::find_if(edges.first, edges.second, [&](auto e) { return hasRelation<R>(g, e); });
  if (it != edges.second) {
    return boost::target(*it, g);
  }
  return {};
}

// Class that selects between in_edges and out_edges (i wish I had c++17...)
template <bool Backwards>
struct GetEdges {};
template <>
struct GetEdges<true> {
  template <typename Id, typename Graph>
  inline auto operator()(Id id, Graph& g) {
    return in_edges(id, g);
  }
};
template <>
struct GetEdges<false> {
  template <typename Id, typename Graph>
  inline auto operator()(Id id, Graph& g) {
    return out_edges(id, g);
  }
};

// Class that selects between in_edges and out_edges (i wish I had c++17...)
template <bool Backwards>
struct GetTarget {};
template <>
struct GetTarget<true> {
  using T = FilteredRoutingGraph::vertex_descriptor;
  template <typename Id, typename Graph>
  inline T operator()(Id id, Graph& g) {
    return boost::source(id, g);
  }
};
template <>
struct GetTarget<false> {
  using T = FilteredRoutingGraph::vertex_descriptor;
  template <typename Id, typename Graph>
  inline T operator()(Id id, Graph& g) {
    return boost::target(id, g);
  }
};

template <typename Vertex, typename Graph, typename Func>
bool anySidewayNeighbourIs(Vertex v, const Graph& g, Func&& f) {
  Optional<LaneletVertexId> currVertex = v;
  while (!!currVertex && !f(*currVertex)) {
    currVertex = getNext<RelationType::Left>(*currVertex, g);
  }
  if (!!currVertex) {
    return true;
  }
  currVertex = getNext<RelationType::Right>(v, g);
  while (!!currVertex && !f(*currVertex)) {
    currVertex = getNext<RelationType::Right>(*currVertex, g);
  }
  return !!currVertex;
}

template <typename Graph>
std::set<LaneletVertexId> getAllNeighbourLanelets(LaneletVertexId ofVertex, const Graph& ofRoute) {
  std::set<LaneletVertexId> result{ofVertex};
  Optional<LaneletVertexId> currVertex = ofVertex;
  while (!!currVertex) {
    result.insert(*currVertex);
    currVertex = getNext<RelationType::Left>(*currVertex, ofRoute);
  }
  currVertex = ofVertex;
  while (!!currVertex) {
    result.insert(*currVertex);
    currVertex = getNext<RelationType::Right>(*currVertex, ofRoute);
  }
  return result;
}

//! Filter that reduces the original graph by edges that belong to different cost types or lane changes
class OriginalGraphFilter {
 public:
  OriginalGraphFilter() = default;
  OriginalGraphFilter(const GraphType& g, bool withLaneChange, RoutingCostId costId)
      : g_{&g}, costId_{costId}, filterMask_{RelationType::Successor | RelationType::Conflicting} {
    if (withLaneChange) {
      filterMask_ |=
          RelationType::Left | RelationType::Right | RelationType::AdjacentLeft | RelationType::AdjacentRight;
    }
  }
  bool operator()(const GraphTraits::edge_descriptor& v) const {
    const auto& edge = (*g_)[v];
    return edge.costId == costId_ && (edge.relation & filterMask_) != RelationType::None;
  }

 private:
  const GraphType* g_;
  RoutingCostId costId_;
  RelationType filterMask_;
};
using OriginalGraph = boost::filtered_graph<GraphType, OriginalGraphFilter>;

//! Reduces the graph to a set of desired vertices
class OnRouteFilter {
 public:
  OnRouteFilter() = default;
  explicit OnRouteFilter(const RouteLanelets& onRoute) : onRoute_{&onRoute} {}

  bool operator()(LaneletVertexId vertexId) const { return onRoute_->find(vertexId) != onRoute_->end(); }

 private:
  const RouteLanelets* onRoute_{};
};

template <RelationType relation, typename GraphType>
class EdgeRelationFilter {
 public:
  EdgeRelationFilter() = default;
  explicit EdgeRelationFilter(const GraphType& graph) : graph_{&graph} {}
  bool operator()(FilteredRoutingGraph::edge_descriptor e) const {
    auto type = (*graph_)[e].relation;
    return (type & relation) != RelationType::None;
  }

 private:
  const GraphType* graph_{};
};

//! Removes edges from the graph that are not drivable (e.g. adjacent or conflicing)
using OnlyDrivableEdgesFilter =
    EdgeRelationFilter<RelationType::Successor | RelationType::Left | RelationType::Right, OriginalGraph>;
using OnlyConflictingFilter = EdgeRelationFilter<RelationType::Conflicting, OriginalGraph>;

//! Removes conflicting edges from the graph
class NoConflictingFilter {
 public:
  NoConflictingFilter() = default;
  explicit NoConflictingFilter(const OriginalGraph& originalGraph) : originalGraph_{&originalGraph} {}
  bool operator()(FilteredRoutingGraph::edge_descriptor e) const {
    auto type = (*originalGraph_)[e].relation;
    return type != RelationType::Conflicting;
  }

 private:
  const OriginalGraph* originalGraph_{};
};

//! Removes vertices from the graph that are not adjacent to a set of vertices. Adjacent can also mean conflicting!
class NextToRouteFilter {
 public:
  NextToRouteFilter() = default;
  NextToRouteFilter(const RouteLanelets& onRoute, const OriginalGraph& originalGraph)
      : onRoute_{&onRoute}, originalGraph_{&originalGraph} {}

  bool operator()(LaneletVertexId vertexId) const {
    // at least one out edge must be connected to lanelets on the route. This includes conflicting and adjacent!
    if (onRoute_->find(vertexId) != onRoute_->end()) {
      return true;
    }
    auto outEdges = boost::out_edges(vertexId, *originalGraph_);
    auto connectedToRoute = std::any_of(outEdges.first, outEdges.second, [&](OriginalGraph::edge_descriptor e) {
      auto dest = boost::target(e, *originalGraph_);
      return onRoute_->find(dest) != onRoute_->end() ||
             onRoute_->find(boost::source(e, *originalGraph_)) != onRoute_->end();
    });
    return connectedToRoute;
  }

 private:
  const RouteLanelets* onRoute_{};
  const OriginalGraph* originalGraph_{};
};

//! Removes edges from the graph that are not drivable (e.g. adjacent or conflicing) OR leave the route at its end
class OnlyDrivableEdgesWithinFilter {
  using SuccessorFilter = EdgeRelationFilter<RelationType::Successor, OriginalGraph>;

 public:
  OnlyDrivableEdgesWithinFilter() = default;
  explicit OnlyDrivableEdgesWithinFilter(RouteLanelets withinLanelets, const OriginalGraph& originalGraph)
      : drivableEdge_{originalGraph}, successorEdge_{originalGraph}, withinLanelets_{std::move(withinLanelets)} {}
  bool operator()(FilteredRoutingGraph::edge_descriptor e) const {
    return drivableEdge_(e) && (!successorEdge_(e) || withinLanelets_.find(e.m_source) == withinLanelets_.end());
  }

 private:
  OnlyDrivableEdgesFilter drivableEdge_;
  SuccessorFilter successorEdge_;
  RouteLanelets withinLanelets_;
};

//! Finds vertices that are in conflict or adjacent to some vertices (not reachable though bidirectional lane changes)
class ConflictingSectionFilter {
 public:
  ConflictingSectionFilter() = default;
  explicit ConflictingSectionFilter(const OriginalGraph& g, const RouteLanelets& onRoute)
      : g_{&g}, onRoute_{&onRoute} {}

  bool operator()(LaneletVertexId vertexId) const {
    // conflicting if it is not yet part of the route but in conflict with a route lanelet
    if (has(*onRoute_, vertexId)) {
      return false;
    }
    auto outEdges = boost::out_edges(vertexId, *g_);
    bool isNeighbour = false;
    bool isConflicting = false;
    std::for_each(outEdges.first, outEdges.second, [&](auto edge) {
      if (onRoute_->find(boost::target(edge, *g_)) == onRoute_->end()) {
        return;
      }
      auto type = (*g_)[edge].relation;
      auto neighbourTypes = RelationType::Left | RelationType::Right;
      auto conflictTypes = RelationType::Conflicting | RelationType::AdjacentLeft | RelationType::AdjacentRight;
      if ((type & (neighbourTypes)) != RelationType::None) {
        auto outEdge = boost::edge(boost::target(edge, *g_), boost::source(edge, *g_), *g_);
        auto reverseIsNeigh = outEdge.second && ((*g_)[outEdge.first].relation & neighbourTypes) != RelationType::None;
        isNeighbour |= reverseIsNeigh;
        isConflicting |= !isNeighbour;
      }
      isConflicting |= (type & (conflictTypes)) != RelationType::None;
    });
    return isConflicting && !isNeighbour;
  }

 private:
  const OriginalGraph* g_{};
  const RouteLanelets* onRoute_{};
};

//! Finds vertices within a set of vertices that are in conflict with another set of vertices
class OnRouteAndConflictFilter {
 public:
  OnRouteAndConflictFilter() = default;
  explicit OnRouteAndConflictFilter(const RouteLanelets& onRoute, const std::vector<LaneletVertexId>& conflictWith,
                                    const OriginalGraph& g)
      : onRoute_{&onRoute}, conflictWith_{&conflictWith}, g_{&g} {}

  bool operator()(LaneletVertexId vertexId) const {
    auto isOk = anySidewayNeighbourIs(vertexId, *g_, [&](auto v) {
      if (!has(*onRoute_, vertexId)) {
        return false;
      }
      auto outEdges = boost::out_edges(v, *g_);
      auto isConflicting = [&](OriginalGraph::edge_descriptor e) {
        auto conflictTypes = RelationType::Conflicting | RelationType::AdjacentLeft | RelationType::AdjacentRight;
        auto type = (*g_)[e].relation;
        return (type & conflictTypes) != RelationType::None && has(*conflictWith_, boost::target(e, *g_));
      };
      return v == start_ || v == end_ || std::any_of(outEdges.first, outEdges.second, isConflicting);
    });
    return isOk;
  }

  void setConflicting(const LaneletVertexIds& conflictWith) { conflictWith_ = &conflictWith; }
  void setStart(LaneletVertexId start) { start_ = start; }
  void setEnd(LaneletVertexId end) { end_ = end; }

 private:
  const RouteLanelets* onRoute_{};
  const LaneletVertexIds* conflictWith_{};
  LaneletVertexId start_{};
  LaneletVertexId end_{};
  const OriginalGraph* g_{};
};

//! For graph queries, we implement our own color map because boost's color does not perform well on sparse graphs
struct SparseColorMap {
  using key_type = LaneletVertexId;                     // NOLINT
  using value_type = boost::two_bit_color_type;         // NOLINT
  using reference = void;                               // NOLINT
  using category = boost::read_write_property_map_tag;  // NOLINT
  using MapT = std::map<key_type, std::uint8_t>;
  std::shared_ptr<MapT> data{std::make_shared<MapT>()};
};

inline SparseColorMap::value_type get(const SparseColorMap& map, LaneletVertexId key) {
  auto val = map.data->find(key);
  if (val != map.data->end()) {
    return static_cast<SparseColorMap::value_type>(val->second);
  }
  return SparseColorMap::value_type::two_bit_white;
}

inline void put(SparseColorMap& map, LaneletVertexId key, SparseColorMap::value_type value) {
  (*map.data)[key] = static_cast<std::uint8_t>(value);
}

//! An iterator that finds paths from a start vertex to all reachable destinations.
template <typename GraphT>
class ConnectedPathIterator {
 public:
  using Vertices = std::vector<LaneletVertexId>;

 private:
  template <typename Func>
  class PathVisitor : public boost::default_dfs_visitor {
   public:
    PathVisitor(Func& f, Vertices& path) : path_{&path}, f_{&f} {}
    void discover_vertex(typename GraphT::vertex_descriptor v, const GraphT& /*g*/) {  // NOLINT
      path_->push_back(v);
      movingForward_ = true;
    }
    void finish_vertex(typename GraphT::vertex_descriptor v, const GraphT& /*g*/) {  // NOLINT
      if (movingForward_) {
        (*f_)(*path_);
      }
      movingForward_ = false;
      assert(!path_->empty());
      assert(path_->back() == v);
      path_->pop_back();
    }

   private:
    bool movingForward_{true};
    Vertices* path_;
    const std::decay_t<Func>* f_;
  };

 public:
  ConnectedPathIterator() = default;
  explicit ConnectedPathIterator(const GraphT& g) : g_{g} {}

  //! Calls a function for all full paths starting from start. A full path is a path from start to either a leaf or the
  //! last unvisited vertex in loops.
  template <typename Func>
  void forEachPath(LaneletVertexId start, Func&& f) {
    assert(g_.m_vertex_pred(start));
    path_.clear();
    PathVisitor<Func> vis(f, path_);
    SparseColorMap cm;
    boost::depth_first_visit(g_, start, vis, cm);
  }

  //! Returns whether a path exists in the graph that connects from and to
  bool hasPathFromTo(LaneletVertexId from, LaneletVertexId to) {
    class PathFound {};
    auto destinationReached = [&](const auto& path) {
      return std::any_of(std::make_reverse_iterator(path.end()), std::make_reverse_iterator(path.begin()),
                         [&](auto p) { return p == to; });
    };
    try {
      forEachPath(from, [&](const auto& path) {
        if (destinationReached(path)) {
          throw PathFound{};
        }
      });
    } catch (PathFound) {  // NOLINT(misc-throw-by-value-catch-by-reference)
      return true;
    }
    return false;
  }

  GraphT& graph() { return g_; }

 private:
  GraphT g_;
  Vertices path_;
};

// Aliases for some graphs needed by us
using OnRouteGraph = boost::filtered_graph<OriginalGraph, boost::keep_all, OnRouteFilter>;
using DrivableGraph = boost::filtered_graph<OriginalGraph, OnlyDrivableEdgesFilter>;
using NoConflictingGraph = boost::filtered_graph<OriginalGraph, NoConflictingFilter>;
using OnlyConflictingGraph = boost::filtered_graph<OriginalGraph, OnlyConflictingFilter>;
using NextToRouteGraph = boost::filtered_graph<OriginalGraph, OnlyDrivableEdgesWithinFilter, NextToRouteFilter>;
using ConflictOrAdjacentToRouteGraph =
    boost::filtered_graph<OriginalGraph, OnlyDrivableEdgesFilter, ConflictingSectionFilter>;
using ConflictsWithPathGraph = boost::filtered_graph<OriginalGraph, NoConflictingFilter, OnRouteAndConflictFilter>;

}  // namespace internal
}  // namespace routing
}  // namespace lanelet
