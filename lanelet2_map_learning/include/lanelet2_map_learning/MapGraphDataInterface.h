#pragma once

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/utility/Optional.h>

#include "lanelet2_map_learning/Forward.h"
#include "lanelet2_map_learning/MapGraph.h"
#include "lanelet2_map_learning/internal/Graph.h"

namespace lanelet {
class LaneletLayer;

namespace map_learning {

class MapGraphDataInterface {
 public:
  template <typename T>
  using FeatureBuffer =
      std::unordered_map<Id, T, std::hash<Id>, std::equal_to<Id>, Eigen::aligned_allocator<std::pair<const Id, T>>>;

  struct Configuration {
    Configuration() noexcept {}
    LaneletRepresentationType reprType{LaneletRepresentationType::Boundaries};
    ParametrizationType paramType{ParametrizationType::Polyline};
    double submapAreaLongitudinal{30};  // in driving direction
    double submapAreaLateral{15};       // in lateral direction
    int nPoints{11};
  };

  MapGraphDataInterface(LaneletMapConstPtr laneletMap, Configuration config = Configuration(),
                        Optional<BasicPoint2d> currPos = boost::none);

  void setCurrPosAndExtractSubmap(const BasicPoint2d& pt);
  LaneData laneData();
  TEData teData();

  LaneData laneDataBatch(const BasicPoints2d& pts, const std::vector<double>& yaws);
  TEData laneTEDataBatch(const BasicPoints2d& pts, const std::vector<double>& yaws);

 private:
  LaneData getLaneData(MapGraphConstPtr localSubmapGraph);

  TEData getLaneTEData(MapGraphConstPtr localSubmapGraph, LaneletSubmapConstPtr localSubmap,
                       std::unordered_map<Id, int>& teId2Index);

  LaneletMapConstPtr laneletMap_;
  LaneletSubmapConstPtr localSubmap_;
  std::unordered_map<Id, int> teId2Index_;
  MapGraphConstPtr localSubmapGraph_;
  FeatureBuffer<LaneletFeature> laneletFeatureBuffer_;
  FeatureBuffer<PolylineFeature> polylineFeatureBuffer_;
  FeatureBuffer<PolylineFeature> teFeatureBuffer_;
  Optional<BasicPoint2d> currPos_;  // in the map frame
  Optional<double> currYaw_;        // in the map frame
  Configuration config_;
  traffic_rules::TrafficRulesPtr trafficRules_;
};

}  // namespace map_learning
}  // namespace lanelet