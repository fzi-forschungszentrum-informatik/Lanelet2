#pragma once

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/utility/Optional.h>

#include "lanelet2_map_learning/Forward.h"
#include "lanelet2_map_learning/MapGraph.h"
#include "lanelet2_map_learning/internal/Graph.h"

namespace lanelet {
class LaneletLayer;

namespace map_learning {

enum class LaneletRepresentationType { Centerline, Boundaries };
enum class ParametrizationType { Bezier, BezierEndpointFixed, Polyline };

struct TensorGraphDataLaneLane {
  TensorGraphDataLaneLane() noexcept {}
  Eigen::MatrixXd x;   // node features
  Eigen::MatrixX2i a;  // adjacency matrix (sparse)
  Eigen::MatrixXd e;   // edge features
};

struct TensorGraphDataLaneTE {
  TensorGraphDataLaneTE() noexcept {}
  Eigen::MatrixXd xLane;  // node features lanelets
  Eigen::MatrixXd xTE;    // node features traffic elements

  /// @brief adjacency matrix (sparse). Node indices are assigned as
  ///        xLane and xTE stacked, with xLane being first
  Eigen::MatrixX2i a;

  Eigen::MatrixXd e;  // edge features
};

class MapGraphDataInterface {
 public:
  using FeatureBuffer = std::unordered_map<Id, Eigen::Vector3d, std::hash<Id>, std::equal_to<Id>,
                                           Eigen::aligned_allocator<std::pair<const Id, Eigen::Vector3d>>>;
  struct Configuration {
    Configuration() noexcept {}
    LaneletRepresentationType reprType{LaneletRepresentationType::Boundaries};
    ParametrizationType paramType{ParametrizationType::Polyline};
    double submapAreaX{100};
    double submapAreaY{100};
    int nPoints{11};
  };

  MapGraphDataInterface(LaneletMapConstPtr laneletMap, Configuration config = Configuration(),
                        Optional<BasicPoint2d> currPos = boost::none);

  void setCurrPosAndExtractSubmap(const BasicPoint2d& pt);
  TensorGraphDataLaneLane laneLaneTensors();
  TensorGraphDataLaneTE laneTETensors();

  TensorGraphDataLaneLane laneLaneTensorsBatch(const BasicPoints2d& pts);
  TensorGraphDataLaneTE laneTETensorsBatch(const BasicPoints2d& pts);

 private:
  TensorGraphDataLaneLane getLaneLaneData(MapGraphConstPtr localSubmapGraph);

  TensorGraphDataLaneTE getLaneTEData(MapGraphConstPtr localSubmapGraph, LaneletSubmapConstPtr localSubmap,
                                      std::unordered_map<Id, int>& teId2Index);

  LaneletMapConstPtr laneletMap_;
  LaneletSubmapConstPtr localSubmap_;
  std::unordered_map<Id, int> teId2Index_;
  MapGraphConstPtr localSubmapGraph_;
  FeatureBuffer nodeFeatureBuffer_;
  FeatureBuffer teFeatureBuffer_;
  Optional<BasicPoint2d> currPos_;  // in the map frame
  Configuration config_;
  traffic_rules::TrafficRulesPtr trafficRules_;
};

}  // namespace map_learning
}  // namespace lanelet