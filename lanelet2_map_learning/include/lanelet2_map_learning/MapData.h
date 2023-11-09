#pragma once
#include <lanelet2_core/Exceptions.h>
#include <lanelet2_core/Forward.h>
#include <lanelet2_core/geometry/LineString.h>

#include <boost/geometry.hpp>
#include <type_traits>

#include "lanelet2_map_learning/Forward.h"
#include "lanelet2_map_learning/MapFeatures.h"
#include "lanelet2_map_learning/Types.h"

namespace lanelet {
namespace map_learning {

class LaneData {
 public:
  LaneData() noexcept {}
  LaneData(const ConstLanelets& lls);

 private:
  LaneLineStringFeatures roadBorders_;   // auxilliary features
  LaneLineStringFeatures laneDividers_;  // auxilliary features

  CompoundLaneLineStringFeatures compoundRoadBorders_;
  CompoundLaneLineStringFeatures compoundLaneDividers_;
  CompoundLaneLineStringFeatures compoundCenterlines_;

  LaneletFeatures laneletFeatures_;  // node features
  Eigen::MatrixX2i edgeList_;        // adjacency matrix (sparse)
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