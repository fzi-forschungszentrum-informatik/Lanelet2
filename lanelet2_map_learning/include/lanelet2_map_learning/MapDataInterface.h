#pragma once

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/utility/Optional.h>

#include "lanelet2_map_learning/Forward.h"
#include "lanelet2_map_learning/MapData.h"
#include "lanelet2_map_learning/Types.h"
#include "lanelet2_routing/RoutingGraph.h"

namespace lanelet {
class LaneletLayer;

namespace map_learning {

class MapDataInterface {
 public:
  template <typename T>
  using FeatureBuffer =
      std::unordered_map<Id, T, std::hash<Id>, std::equal_to<Id>, Eigen::aligned_allocator<std::pair<const Id, T>>>;

  struct Configuration {
    Configuration() noexcept {}
    LaneletRepresentationType reprType{LaneletRepresentationType::Boundaries};
    ParametrizationType paramType{ParametrizationType::LineString};
    bool includeLaneletBdTypes{false};
    double submapAreaLongitudinal{30};  // in driving direction
    double submapAreaLateral{15};       // in lateral direction
    int nPoints{11};
  };

  MapDataInterface(LaneletMapConstPtr laneletMap, Configuration config = Configuration(),
                   Optional<BasicPoint2d> currPos = boost::none);

  void setCurrPosAndExtractSubmap(const BasicPoint2d& pt);
  LaneData laneData();
  TEData teData();

  LaneData laneDataBatch(const BasicPoints2d& pts, const std::vector<double>& yaws);
  TEData laneTEDataBatch(const BasicPoints2d& pts, const std::vector<double>& yaws);

 private:
  LaneData getLaneData(LaneletSubmapConstPtr localSubmap, routing::RoutingGraphConstPtr localSubmapGraph,
                       bool processAll = false);

  TEData getLaneTEData(routing::RoutingGraphConstPtr localSubmapGraph, LaneletSubmapConstPtr localSubmap,
                       bool processAll);

  LaneletMapConstPtr laneletMap_;
  LaneletSubmapConstPtr localSubmap_;
  std::unordered_map<Id, int> teId2Index_;
  routing::RoutingGraphConstPtr localSubmapGraph_;
  Optional<BasicPoint2d> currPos_;  // in the map frame
  Optional<double> currYaw_;        // in the map frame
  Configuration config_;
  traffic_rules::TrafficRulesPtr trafficRules_;
};

}  // namespace map_learning
}  // namespace lanelet