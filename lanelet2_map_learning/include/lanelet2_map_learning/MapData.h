#pragma once
#include <lanelet2_core/Exceptions.h>
#include <lanelet2_core/Forward.h>
#include <lanelet2_core/geometry/LineString.h>

#include <boost/geometry.hpp>
#include <type_traits>

#include "lanelet2_map_learning/Forward.h"
#include "lanelet2_map_learning/MapFeatures.h"
#include "lanelet2_map_learning/MapGraph.h"
#include "lanelet2_map_learning/Types.h"
#include "lanelet2_map_learning/internal/Graph.h"

namespace lanelet {
namespace map_learning {

// results are given in whatever order they are in the graph object, no guarantees!
LaneLineStringFeatures extractRoadBorders(MapGraphConstPtr localSubmapGraph);
LaneLineStringFeatures extractLaneDividers(MapGraphConstPtr localSubmapGraph);

// results are given in whatever order they are in the graph object, no guarantees!
CompoundLaneLineStringFeatures extractCompoundRoadBorders(MapGraphConstPtr localSubmapGraph);
CompoundLaneLineStringFeatures extractCompoundLaneDividers(MapGraphConstPtr localSubmapGraph);
CompoundLaneLineStringFeatures extractCompoundCenterlines(MapGraphConstPtr localSubmapGraph);

class LaneData {
 public:
  LaneData() noexcept {}
  LaneData(MapGraphConstPtr localSubmapGraph);

 private:
  LaneLineStringFeatures roadBorders_;   // auxilliary features
  LaneLineStringFeatures laneDividers_;  // auxilliary features

  CompoundLaneLineStringFeatures compoundRoadBorders_;   // auxilliary features
  CompoundLaneLineStringFeatures compoundLaneDividers_;  // auxilliary features
  CompoundLaneLineStringFeatures compoundCenterlines_;

  LaneletFeatures laneletFeatures_;  // node features
  Eigen::MatrixX2i edgeList_;        // adjacency matrix (sparse) for centerlines
  Eigen::MatrixXd edgeFeatures_;     // edge features
};

class TEData {
 public:
  TEData() noexcept {}

 private:
  LaneletFeatures laneletFeatures;  // node features lanelets
  TEFeatures teFeatures;            // node features traffic elements

  /// @brief adjacency matrix (sparse). Node indices are assigned as
  ///        xLane and xTE stacked, with xLane being first
  Eigen::MatrixX2i edgeList;

  Eigen::MatrixXd edgeFeatures;  // edge features
};

}  // namespace map_learning
}  // namespace lanelet