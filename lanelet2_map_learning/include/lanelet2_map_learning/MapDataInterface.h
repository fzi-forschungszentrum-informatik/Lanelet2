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
  struct Configuration {
    Configuration() noexcept {}
    Configuration(LaneletRepresentationType reprType, ParametrizationType paramType, double submapExtentLongitudinal,
                  double submapExtentLateral, int nPoints) noexcept
        : reprType{reprType},
          paramType{paramType},
          submapExtentLongitudinal{submapExtentLongitudinal},
          submapExtentLateral{submapExtentLateral},
          nPoints{nPoints} {}
    LaneletRepresentationType reprType{LaneletRepresentationType::Boundaries};
    ParametrizationType paramType{ParametrizationType::LineString};
    double submapExtentLongitudinal{30};  // in driving direction
    double submapExtentLateral{15};       // in lateral direction
    int nPoints{20};
  };
  MapDataInterface(LaneletMapConstPtr laneletMap);
  MapDataInterface(LaneletMapConstPtr laneletMap, Configuration config);

  const Configuration& config() { return config_; }

  void setCurrPosAndExtractSubmap(const BasicPoint2d& pt, double yaw);
  LaneData laneData();
  TEData teData();

  std::vector<LaneData> laneDataBatch(const BasicPoints2d& pts, const std::vector<double>& yaws,
                                      bool processAll = true);
  std::vector<TEData> laneTEDataBatch(const BasicPoints2d& pts, const std::vector<double>& yaws,
                                      bool processAll = true);

 private:
  LaneData getLaneData(LaneletSubmapConstPtr localSubmap, lanelet::routing::RoutingGraphConstPtr localSubmapGraph,
                       const BasicPoint2d& pos, double yaw, bool processAll = true);

  TEData getLaneTEData(LaneletSubmapConstPtr localSubmap, lanelet::routing::RoutingGraphConstPtr localSubmapGraph,
                       const BasicPoint2d& pos, double yaw, bool processAll = true);

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