#include "lanelet2_map_learning/MapGraphDataProvider.h"

#include <lanelet2_core/Exceptions.h>
#include <lanelet2_core/Forward.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

namespace lanelet {
namespace map_learning {

LaneletSubmapConstPtr extractSubmap(LaneletMapConstPtr laneletMap, const BasicPoint2d& center, double extentX,
                                    double extentY) {
  BasicPoint2d regionRear = {center.x() - extentX, center.y() - extentY};
  BasicPoint2d regionFront = {center.x() + extentX, center.y() + extentY};
  BoundingBox2d searchRegion{regionRear, regionFront};
  ConstLanelets inRegion = laneletMap->laneletLayer.search(searchRegion);
  return utils::createConstSubmap(inRegion, {});
}

void MapGraphDataProvider::setCurrPosAndExtractSubmap(const BasicPoint2d& pt) {
  currPos_ = pt;
  localSubmap_ = extractSubmap(laneletMap_, *currPos_, config_.submapAreaX, config_.submapAreaY);
  localSubmapGraph_ = MapGraph::build(*laneletMap_, *trafficRules_);
}

MapGraphDataProvider::MapGraphDataProvider(LaneletMapConstPtr laneletMap, Configuration config,
                                           Optional<BasicPoint2d> currPos)
    : laneletMap_{laneletMap},
      config_{config},
      currPos_{currPos},
      trafficRules_{traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Vehicle)} {}

void getLaneLaneData(MapGraphConstPtr localSubmapGraph) {}

void getLaneTEData(MapGraphConstPtr localSubmapGraph) {}

TensorGraphData MapGraphDataProvider::laneLaneTensors() {
  if (!currPos_) {
    throw InvalidObjectStateError(
        "Your current position is not set! Call setCurrPosAndExtractSubmap() before trying to get the data!");
  }
  return TensorGraphData();
}

TensorGraphData MapGraphDataProvider::laneTETensors() {
  if (!currPos_) {
    throw InvalidObjectStateError(
        "Your current position is not set! Call setCurrPosAndExtractSubmap() before trying to get the data!");
  }
  return TensorGraphData();
}

}  // namespace map_learning
}  // namespace lanelet