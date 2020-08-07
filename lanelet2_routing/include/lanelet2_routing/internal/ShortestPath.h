#pragma once
#include <boost/graph/dijkstra_shortest_paths_no_color_map.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/property_map/property_map.hpp>

#include "lanelet2_routing/Exceptions.h"
#include "lanelet2_routing/internal/Graph.h"
#include "lanelet2_routing/internal/GraphUtils.h"

namespace lanelet {
namespace routing {
namespace internal {

//! This object carries the required information for the graph neighbourhood search
struct VertexVisitInformation {
  RoutingGraphGraph::Vertex vertex{};
  RoutingGraphGraph::Vertex predecessor{};
  double cost{};
  size_t length{};
  size_t numLaneChanges{};
};

struct VertexState {
  RoutingGraphGraph::Vertex predecessor{};  //!< The vertex this refers to
  double cost{};                            //!< Current accumulated cost
  size_t length{};                          //!< Number of vertices to this vertex (including this one)
  size_t numLaneChanges{};                  //!< Required lane changes along the shortest path in order to get here
  bool predicate{true};                     //!< False if disabled by predicate
  bool isLeaf{true};                        //!< True if it has no successor that is on the shortest path
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
    void examine_vertex(VertexType v, const SearchGraph& /*g*/) {  // NOLINT
      auto& state = map_->at(v);
      state.predicate =
          ((*cb_)(VertexVisitInformation{v, state.predecessor, state.cost, state.length, state.numLaneChanges}));
      map_->at(state.predecessor).isLeaf = v == state.predecessor;  // necessary for the initial vertex
    }

    void edge_relaxed(EdgeType e, const SearchGraph& g) {  // NOLINT
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
  explicit DijkstraStyleSearch(const G& graph) : graph_{graph, LeafFilter{vertices_, graph}} {}

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

}  // namespace internal
}  // namespace routing
}  // namespace lanelet
