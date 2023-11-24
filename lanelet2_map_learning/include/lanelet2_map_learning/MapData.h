#pragma once
#include <lanelet2_core/Exceptions.h>
#include <lanelet2_core/Forward.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_routing/Forward.h>

#include <boost/geometry.hpp>
#include <type_traits>

#include "lanelet2_map_learning/Forward.h"
#include "lanelet2_map_learning/MapFeatures.h"
#include "lanelet2_map_learning/Types.h"
#include "lanelet2_routing/RoutingGraph.h"
#include "lanelet2_routing/internal/Graph.h"

namespace lanelet {
namespace map_learning {

struct Edge {
  Edge(Id el1, Id el2) : el1_{el1}, el2_{el2} {}
  Id el1_;
  Id el2_;
};
struct LaneEdge : Edge {
  LaneEdge(Id el1, Id el2, lanelet::routing::RelationType type) : Edge(el1, el2), type_{type} {}
  lanelet::routing::RelationType type_;
};

using Edges = std::vector<Edge>;
using LaneEdges = std::vector<LaneEdge>;

class LaneData {
 public:
  LaneData() noexcept {}
  static LaneData build(LaneletSubmapConstPtr& localSubmap, lanelet::routing::RoutingGraphConstPtr localSubmapGraph);

 private:
  LaneLineStringFeatures roadBorders_;   // auxilliary features
  LaneLineStringFeatures laneDividers_;  // auxilliary features

  CompoundLaneLineStringFeatures compoundRoadBorders_;   // auxilliary features
  CompoundLaneLineStringFeatures compoundLaneDividers_;  // auxilliary features
  CompoundLaneLineStringFeatures compoundCenterlines_;

  LaneletFeatures laneletFeatures_;  // node features
  LaneEdges edgeList_;               // edge list for centerlines
};

class TEData {
 public:
  TEData() noexcept {}

 private:
  LaneletFeatures laneletFeatures;  // node features lanelets
  TEFeatures teFeatures;            // node features traffic elements

  Edges edgeList_;  // edge list
  /// @brief adjacency matrix (sparse). Node indices are assigned as
  ///        xLane and xTE stacked, with xLane being first
};

}  // namespace map_learning
}  // namespace lanelet