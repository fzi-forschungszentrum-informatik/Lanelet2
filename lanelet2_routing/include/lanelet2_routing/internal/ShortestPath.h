#pragma once
#include <boost/graph/dijkstra_shortest_paths_no_color_map.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/property_map/property_map.hpp>
#include "Exceptions.h"
#include "Graph.h"
#include "GraphUtils.h"

namespace lanelet {
namespace routing {

/** @brief Internal custom visitor for Dijkstra Shortest Path.
 * Throws if goal vertice is reached and modifies the colormap. The color map keeps track of the state of vertices. Even
 * though one can be directly provided to the algorithm it is ignored. See
 * http://boost.2283326.n4.nabble.com/Boost-Graph-Library-Dijkstra-Shortest-Path-ignores-color-map-in-named-parameters-td2650984.html
 * This visitor implements a basic color map modification that can be used to avoid searches that have already been
 * made. Furthermore the visitor throws an FoundGoalException once the goal vertice is reached to avoid exploration of
 * the whole graph.
 * @see Dijkstra */
template <class Vertex>
class DijkstraGoalVisitor : public boost::default_dijkstra_visitor {
 public:
  DijkstraGoalVisitor(const Vertex goal, std::vector<boost::default_color_type>* colorMap)
      : goal_(goal), colorMap_(colorMap) {}
  template <class Graph>
  void examine_vertex(Vertex u, Graph& /*g*/) {  // NOLINT
    if (u == goal_) {
      (*colorMap_)[u] = boost::default_color_type::gray_color;
      throw FoundGoalError("Found goal");
    }
  }
  template <class Graph>
  void finish_vertex(Vertex u, Graph& /*g*/) {  // NOLINT
    (*colorMap_)[u] = boost::default_color_type::black_color;
  }

 private:
  const Vertex goal_;                                 // NOLINT
  std::vector<boost::default_color_type>* colorMap_;  ///< Colormap that saves the state of vertices
};

template <typename G>
struct ShortestPath {
  using VertexType = typename G::vertex_descriptor;
  const G& graph;          ///< Routing graph
  const VertexType start;  ///< Start vertex

  ShortestPath() = delete;

  /** @brief Constructor of a shortest path search with a defined start vertex.
   *  @param graph Routing graph to perform search
   *  @param start Start vertex */
  ShortestPath(const G& graph, const VertexType start) : graph(graph), start(start) {
    abortIfVertexInvalid(start);
    size_t numVertices = boost::num_vertices(graph);
    this->predecessors_.resize(numVertices);
    this->distances_.resize(numVertices);
    this->colorMap_.resize(numVertices);
    std::fill(distances_.begin(), distances_.end(), std::numeric_limits<double>::max());
  }

  /** @brief Main function to get a shortest path
   *  @param target Goal vertex
   *  @return Vertices passed to reach the target or an empty list.
   *  @throws InvalidInputException if the target vertex isn't reasonable */
  std::deque<VertexType> shortestPath(const VertexType target) {
    abortIfVertexInvalid(target);

    // Only perform search if goal vertex has not been visited
    if (colorMap_[target] == boost::default_color_type::white_color) {
      DijkstraGoalVisitor<VertexType> goalVisitor(target, &colorMap_);
      try {
        // NOLINTNEXTLINE
        boost::dijkstra_shortest_paths_no_color_map(graph, start,
                                                    boost::predecessor_map(predecessors_.data())
                                                        .distance_map(distances_.data())
                                                        .weight_map(boost::get(&EdgeInfo::routingCost, graph))
                                                        .visitor(goalVisitor));
      } catch (const FoundGoalError&) {  //! Thrown regulary to avoid exploitation of unneeded vertices
      }
    }

    if (!reached(target)) {
      return {};
    }

    std::deque<VertexType> sp;
    for (VertexType v = target; v != start; v = predecessors_[v]) {
      sp.push_front(v);
    }
    sp.push_front(start);

    assert((sp.empty() || ((sp.front() == start) && (sp.back() == target))) &&
           "Dijkstra returns either empty list or path from source to target.");
    return sp;
  }

 private:
  //! Simple helper function to determine if the target has been reached.
  bool reached(const VertexType target) {
    abortIfVertexInvalid(target);
    return colorMap_[target] != boost::default_color_type::white_color;
  }

  /** @brief Provides the routing cost to reach the taget
   *  @param target Goal vertex.
   *  @throws InvalidInputError if the target vertex isn't reasonable
   *  @return The cost or nothing if it hasn't been reached. */
  Optional<double> distance(const VertexType target) {
    abortIfVertexInvalid(target);
    if (reached(target)) {
      return distances_[target];
    }
    return {};
  }

  //! Basic check if vertex ID is reasonable
  void abortIfVertexInvalid(const VertexType v) {
    if (v >= boost::num_vertices(graph)) {
      throw InvalidInputError("Invalid input vertex with number " + std::to_string(v));
    }
  }

  std::vector<VertexType> predecessors_;             ///< All predecessors passed to reach the goal
  std::vector<double> distances_;                    ///< Routing cost of all steps
  std::vector<boost::default_color_type> colorMap_;  ///< Color map to avoid repetitive calculations
};

//! This object carries the required information for the graph neighbourhood search
struct VertexVisitInformation {
  RoutingGraphGraph::Vertex vertex;
  RoutingGraphGraph::Vertex predecessor;
  double cost{};
  size_t length{};
  size_t numLaneChanges{};
};

struct VertexState {
  RoutingGraphGraph::Vertex predecessor;  //! The vertex this refers to
  double cost{};                          //! Current accumulated cost
  size_t length{};                        //! Number of vertices to this vertex (including this one)
  size_t numLaneChanges{};                //! Required lane changes along the shortest path in order to get here
  bool predicate{true};                   //! False if disabled by predicate
  bool isLeaf{true};                      //! True if it has no successor that is on the shortest path
};

template <typename VertexT>
using DijkstraSearchMap = std::map<VertexT, VertexState>;

template <typename VertexT>
struct DijkstraCostMap {
  using key_type = VertexT;                             // NOLINT
  using value_type = double;                            // NOLINT
  using reference = void;                               // NOLINT
  using category = boost::read_write_property_map_tag;  // NOLINT
  DijkstraSearchMap<VertexT>* map{};
};

template <typename VertexT>
inline typename DijkstraCostMap<VertexT>::value_type get(const DijkstraCostMap<VertexT>& map, VertexT key) {
  auto val = map.map->find(key);
  if (val != map.map->end()) {
    return val->second.cost;
  }
  return std::numeric_limits<double>::infinity();
}

template <typename VertexT>
inline void put(DijkstraCostMap<VertexT>& map, VertexT key, typename DijkstraCostMap<VertexT>::value_type value) {
  (*map.map)[key].cost = value;
}

template <typename G>
class DijkstraStyleSearch {
 public:
  using VertexType = typename boost::graph_traits<G>::vertex_descriptor;
  using EdgeType = typename boost::graph_traits<G>::edge_descriptor;
  using DijkstraSearchMapType = DijkstraSearchMap<VertexType>;
  using VisitCallback = std::function<bool(const VertexVisitInformation&)>;

 private:
  class LeafFilter {
   public:
    LeafFilter() = default;
    LeafFilter(const DijkstraSearchMapType& m, const G& g) : m_{&m}, g_{&g} {}
    bool operator()(EdgeType e) const { return (*m_).at(boost::source(e, *g_)).predicate; }

   private:
    const DijkstraSearchMapType* m_{};
    const G* g_{};
  };
  using SearchGraph = boost::filtered_graph<G, LeafFilter>;

  template <typename Func>
  class DijkstraStyleVisitor : public boost::default_dijkstra_visitor {
    using FuncT = std::remove_reference_t<Func>;

   public:
    DijkstraStyleVisitor() = default;
    DijkstraStyleVisitor(DijkstraSearchMapType& map, FuncT* cb) : map_{&map}, cb_{cb} {}

    // called whenever a minimal edge is discovered
    void examine_vertex(VertexType v, const SearchGraph& /*g*/) {
      auto& state = map_->at(v);
      state.predicate =
          ((*cb_)(VertexVisitInformation{v, state.predecessor, state.cost, state.length, state.numLaneChanges}));
      map_->at(state.predecessor).isLeaf = v == state.predecessor;  // necessary for the initial vertex
    }

    void edge_relaxed(EdgeType e, const SearchGraph& g) {
      // called whenever a shorter path to e is discovered and before "examine vertex" is called
      // cost is automatically updated by the dijkstra algorithm
      auto& predecessor = (*map_).at(boost::source(e, g));
      auto& follower = (*map_)[boost::target(e, g)];
      follower.length = predecessor.length + 1;
      follower.predecessor = boost::source(e, g);
      follower.numLaneChanges = predecessor.numLaneChanges + (g[e].relation != RelationType::Successor);
    }

   private:
    DijkstraSearchMapType* map_{};
    FuncT* cb_{};
  };

 public:
  //! Constructor for the graph search
  DijkstraStyleSearch(const G& graph) : graph_{graph, LeafFilter{vertices_, graph}} {}

  //! Performs the dijkstra style search by calling func whenever the shortest path for a certain vertex is
  //! discovered. Whenever func returns false, the successor edges of this vertex will not be visited.
  template <typename Func>
  void query(VertexType start, Func&& func) {
    vertices_.clear();
    vertices_.emplace(start, VertexState{start, 0., 1, 0, true, true});
    DijkstraStyleVisitor<decltype(func)> visitor{vertices_, &func};
    auto inf = std::numeric_limits<double>::infinity();
    // offering sane defaults seems to have been impossible...
    boost::dijkstra_shortest_paths_no_color_map_no_init(
        graph_, start, boost::dummy_property_map{}, DijkstraCostMap<VertexType>{&vertices_},
        boost::get(&EdgeInfo::routingCost, graph_), boost::get(boost::vertex_index, graph_), std::less<double>{},
        boost::closed_plus<double>{}, inf, 0., visitor);
  }

  //! Returns the result
  const DijkstraSearchMapType& getMap() const { return vertices_; }

 private:
  SearchGraph graph_;
  DijkstraSearchMapType vertices_;
};

}  // namespace routing
}  // namespace lanelet
