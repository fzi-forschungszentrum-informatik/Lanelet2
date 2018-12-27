#pragma once

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/property_map/property_map.hpp>
#include "Exceptions.h"
#include "Graph.h"
#include "RoutingGraph.h"

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
        boost::dijkstra_shortest_paths(graph, start,
                                       boost::predecessor_map(predecessors_.data())
                                           .distance_map(distances_.data())
                                           .weight_map(get(&EdgeInfo::routingCost, graph))
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
    if (v < 0 || v >= boost::num_vertices(graph)) {
      throw InvalidInputError("Invalid input vertex with number " + std::to_string(v));
    }
  }

  std::vector<VertexType> predecessors_;             ///< All predecessors passed to reach the goal
  std::vector<double> distances_;                    ///< Routing cost of all steps
  std::vector<boost::default_color_type> colorMap_;  ///< Color map to avoid repetitive calculations
};

}  // namespace routing
}  // namespace lanelet
