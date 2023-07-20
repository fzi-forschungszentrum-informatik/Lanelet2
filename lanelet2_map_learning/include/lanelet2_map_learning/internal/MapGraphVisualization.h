#pragma once

#include <boost/graph/graphml.hpp>
#include <boost/graph/graphviz.hpp>
#include <iostream>

#include "lanelet2_map_learning/Exceptions.h"
#include "lanelet2_map_learning/Forward.h"
#include "lanelet2_map_learning/internal/Graph.h"

namespace lanelet {

// This one needs to be in this namepace. Otherwise it's not found.
inline std::istream& operator>>(std::istream& is, ConstLaneletOrArea& /*r*/) { return is; }

namespace map_learning {
inline std::ostream& operator<<(std::ostream& os, const RelationType& r) { return os << relationToString(r); }
inline std::istream& operator>>(std::istream& is, const RelationType& /*r*/) { return is; }

namespace internal {

/** @brief Internal vertex writer for graphViz file export. */
template <class Graph>
class VertexWriterGraphViz {
 public:
  explicit VertexWriterGraphViz(const Graph* g) : graph_(g) {}
  template <class VertexOrEdge>
  void operator()(std::ostream& out, const VertexOrEdge& v) const {
    const Id id{(*graph_)[v].laneletOrArea.id()};
    out << "[label=\"" << id << "\" lanelet=\"" << id << "\"]";
  }

 private:
  const Graph* graph_;
};

/** @brief Internal edge writer for graphViz file export.
    Includes label and color. */
template <class Graph>
class EdgeWriterGraphViz {
 public:
  explicit EdgeWriterGraphViz(const Graph* g) : graph_(g) {}
  template <class VertexOrEdge>
  void operator()(std::ostream& out, const VertexOrEdge& v) const {
    const RelationType relation{(*graph_)[v].relation};
    out << "[label=\"" << relationToString(relation) << "\" color=\"" << relationToColor(relation) << "\"]";
  }

 private:
  const Graph* graph_;
};

/** @brief Implementation of graphViz export function
 *  @param filename Fully qualified file name - ideally with extension (.gv)
 *  @param g Graph to export
 *  @param edgeFilter Edge filter that will be used to create a filtered graph
 *  @param vertexFilter Vertex filter. Not used yet */
template <typename G, typename E = boost::keep_all, typename V = boost::keep_all>
inline void exportGraphVizImpl(const std::string& filename, const G& g, E edgeFilter = boost::keep_all(),
                               V vertexFilter = boost::keep_all()) {
  std::ofstream file;
  file.open(filename);
  if (!file.is_open()) {
    throw lanelet::ExportError("Could not open file at " + filename + ".");
  }

  VertexWriterGraphViz<G> vertexWriter(&g);
  EdgeWriterGraphViz<G> edgeWriter(&g);
  boost::filtered_graph<G, E, V> fg(g, edgeFilter, vertexFilter);
  boost::write_graphviz(file, fg, vertexWriter, edgeWriter);

  file.close();
}

/** @brief GraphViz export function
 *  @param filename Fully qualified file name - ideally with extension (.gv)
 *  @param g Graph to export
 *  @param relationTypes Relations that will be included in the export */
template <typename G>
inline void exportGraphVizImpl(const std::string& filename, const G& g, const RelationType& relationTypes) {
  auto edgeFilter = EdgeFilter<G>(g, relationTypes);
  exportGraphVizImpl(filename, g, edgeFilter);
}

/** @brief Implementation of graphML export function
 *  @param filename Fully qualified file name - ideally with extension (.graphml)
 *  @param g Graph to export
 *  @param eFilter Edge filter that will be used to create a filtered graph
 *  @param vFilter Vertex filter. Not used yet */
template <typename G, typename E = boost::keep_all, typename V = boost::keep_all>
inline void exportGraphMLImpl(const std::string& filename, const G& g, E eFilter = boost::keep_all(),
                              V vFilter = boost::keep_all()) {
  std::ofstream file;
  file.open(filename);
  if (!file.is_open()) {
    throw lanelet::ExportError("Could not open file at " + filename + ".");
  }

  boost::filtered_graph<G, EdgeFilter<G>> filteredGraph(g, eFilter, vFilter);

  auto pmId = boost::get(&VertexInfo::laneletOrArea, filteredGraph);  // NOLINT
  auto pmRelation = boost::get(&EdgeInfo::relation, filteredGraph);

  boost::dynamic_properties dp;
  dp.property("info", pmId);
  dp.property("relation", pmRelation);

  boost::write_graphml(file, filteredGraph, dp, false);
}

/** @brief GraphML export function
 *  @param filename Fully qualified file name - ideally with extension (.graphml)
 *  @param g Graph to export
 *  @param relationTypes Relations that will be included in the export */
template <typename G>
inline void exportGraphMLImpl(const std::string& filename, const G& g, const RelationType& relationTypes) {
  auto edgeFilter = EdgeFilter<G>(g, relationTypes);
  exportGraphMLImpl(filename, g, edgeFilter);
}
}  // namespace internal
}  // namespace map_learning
}  // namespace lanelet
