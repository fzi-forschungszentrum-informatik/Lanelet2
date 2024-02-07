#pragma once

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/utility/Optional.h>

#include "lanelet2_ml_converter/Forward.h"
#include "lanelet2_ml_converter/MapData.h"
#include "lanelet2_ml_converter/Types.h"
#include "lanelet2_routing/RoutingGraph.h"

namespace lanelet {
class LaneletLayer;

namespace ml_converter {

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

  void setCurrPosAndExtractSubmap2d(const BasicPoint2d& pt, double yaw);
  void setCurrPosAndExtractSubmap(const BasicPoint3d& pt, double yaw);
  void setCurrPosAndExtractSubmap(const BasicPoint3d& pt, double yaw, double pitch, double roll);
  LaneDataPtr laneData(bool processAll);
  TEData teData(bool processAll);

  std::vector<LaneDataPtr> laneDataBatch2d(std::vector<BasicPoint2d> pts, std::vector<double> yaws);
  std::vector<LaneDataPtr> laneDataBatch(std::vector<BasicPoint3d> pts, std::vector<double> yaws);
  std::vector<LaneDataPtr> laneDataBatch(std::vector<BasicPoint3d> pts, std::vector<double> yaws,
                                         std::vector<double> pitches, std::vector<double> rolls);
  std::vector<TEData> laneTEDataBatch(std::vector<BasicPoint2d> pts, std::vector<double> yaws,
                                      std::vector<double> pitches, std::vector<double> rolls);

 private:
  LaneDataPtr getLaneData(LaneletSubmapConstPtr localSubmap, const OrientedRect& bbox,
                          lanelet::routing::RoutingGraphConstPtr localSubmapGraph, double pitch, double roll,
                          bool processAll);

  TEData getLaneTEData(LaneletSubmapConstPtr localSubmap, const OrientedRect& bbox,
                       lanelet::routing::RoutingGraphConstPtr localSubmapGraph, double pitch, double roll,
                       bool processAll);

  LaneletMapConstPtr laneletMap_;
  LaneletSubmapConstPtr localSubmap_;
  std::unordered_map<Id, int> teId2Index_;
  routing::RoutingGraphConstPtr localSubmapGraph_;
  Optional<BasicPoint3d> currPos_;  // in the map frame
  Optional<double> currRoll_;       // in the map frame, [rad]
  Optional<double> currPitch_;      // in the map frame, [rad]
  Optional<double> currYaw_;        // in the map frame, [rad]
  Optional<OrientedRect> currBbox_;
  Configuration config_;
  traffic_rules::TrafficRulesPtr trafficRules_;
};

}  // namespace ml_converter
}  // namespace lanelet