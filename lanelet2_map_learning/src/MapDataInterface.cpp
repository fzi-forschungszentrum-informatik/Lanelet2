#include "lanelet2_map_learning/MapDataInterface.h"

#include <lanelet2_core/Exceptions.h>
#include <lanelet2_core/Forward.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>;
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <boost/geometry.hpp>
#include <type_traits>

#include "lanelet2_map_learning/MapData.h"
#include "lanelet2_map_learning/Utils.h"

namespace lanelet {
namespace map_learning {

void MapDataInterface::setCurrPosAndExtractSubmap(const BasicPoint2d& pt) {
  currPos_ = pt;
  localSubmap_ =
      extractSubmap(laneletMap_, *currPos_, *currYaw_, config_.submapAreaLongitudinal, config_.submapAreaLateral);
  localSubmapGraph_ = lanelet::routing::RoutingGraph::build(*laneletMap_, *trafficRules_);
}

MapDataInterface::MapDataInterface(LaneletMapConstPtr laneletMap, Configuration config, Optional<BasicPoint2d> currPos)
    : laneletMap_{laneletMap},
      config_{config},
      currPos_{currPos},
      trafficRules_{traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Vehicle)} {}

LaneData MapDataInterface::getLaneData(LaneletSubmapConstPtr localSubmap,
                                       lanelet::routing::RoutingGraphConstPtr localSubmapGraph) {
  return LaneData::build(localSubmap, localSubmapGraph);
}

bool isTe(ConstLineString3d ls) {
  std::string type = ls.attribute(AttributeName::Type).value();
  return type == AttributeValueString::TrafficLight || type == AttributeValueString::TrafficSign;
}

TEData MapDataInterface::getLaneTEData(routing::RoutingGraphConstPtr localSubmapGraph,
                                       LaneletSubmapConstPtr localSubmap) {
  throw std::runtime_error("Not implemented yet!");
}

LaneData MapDataInterface::laneData() {
  if (!currPos_) {
    throw InvalidObjectStateError(
        "Your current position is not set! Call setCurrPosAndExtractSubmap() before trying to get the data!");
  }

  return getLaneData(localSubmap_, localSubmapGraph_);
}

TEData MapDataInterface::teData() {
  if (!currPos_) {
    throw InvalidObjectStateError(
        "Your current position is not set! Call setCurrPosAndExtractSubmap() before trying to get the data!");
  }
  throw std::runtime_error("Not implemented yet!");
  return TEData();
}

}  // namespace map_learning
}  // namespace lanelet