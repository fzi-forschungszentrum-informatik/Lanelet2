#include "lanelet2_map_learning/MapDataInterface.h"

#include <lanelet2_core/Exceptions.h>
#include <lanelet2_core/Forward.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <boost/geometry.hpp>
#include <type_traits>

#include "lanelet2_map_learning/MapData.h"
#include "lanelet2_map_learning/Utils.h"

namespace lanelet {
namespace map_learning {

void MapDataInterface::setCurrPosAndExtractSubmap(const BasicPoint2d& pt, double yaw) {
  currPos_ = pt;
  currYaw_ = yaw;
  localSubmap_ = extractSubmap(laneletMap_, *currPos_, config_.submapExtentLongitudinal, config_.submapExtentLateral);
  localSubmapGraph_ = lanelet::routing::RoutingGraph::build(*laneletMap_, *trafficRules_);
}

MapDataInterface::MapDataInterface(LaneletMapConstPtr laneletMap, Configuration config, Optional<BasicPoint2d> currPos)
    : laneletMap_{laneletMap},
      config_{config},
      currPos_{currPos},
      trafficRules_{traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Vehicle)} {}

LaneData MapDataInterface::getLaneData(LaneletSubmapConstPtr localSubmap,
                                       lanelet::routing::RoutingGraphConstPtr localSubmapGraph, const BasicPoint2d& pos,
                                       double yaw, bool processAll) {
  LaneData laneData = LaneData::build(localSubmap, localSubmapGraph);
  OrientedRect bbox = getRotatedRect(pos, config_.submapExtentLongitudinal, config_.submapExtentLateral, yaw);
  if (processAll) {
    laneData.processAll(bbox, config_.paramType, config_.nPoints);
  }
  return laneData;
}

std::vector<LaneData> MapDataInterface::laneDataBatch(const BasicPoints2d& pts, const std::vector<double>& yaws,
                                                      bool processAll) {
  if (pts.size() != yaws.size()) {
    throw std::runtime_error("Unequal sizes of pts and yaws!");
  }
  std::vector<LaneData> lDataVec;
  for (size_t i = 0; i < pts.size(); i++) {
    LaneletSubmapConstPtr localSubmap =
        extractSubmap(laneletMap_, pts[i], config_.submapExtentLongitudinal, config_.submapExtentLateral);
    routing::RoutingGraphConstPtr localSubmapGraph =
        lanelet::routing::RoutingGraph::build(*localSubmap, *trafficRules_);
    lDataVec.push_back(getLaneData(localSubmap, localSubmapGraph, pts[i], yaws[i], processAll));
  }
  return lDataVec;
}

std::vector<TEData> MapDataInterface::laneTEDataBatch(const BasicPoints2d& pts, const std::vector<double>& yaws,
                                                      bool processAll) {
  throw std::runtime_error("Not implemented yet!");
}

bool isTe(ConstLineString3d ls) {
  std::string type = ls.attribute(AttributeName::Type).value();
  return type == AttributeValueString::TrafficLight || type == AttributeValueString::TrafficSign;
}

TEData MapDataInterface::getLaneTEData(LaneletSubmapConstPtr localSubmap,
                                       lanelet::routing::RoutingGraphConstPtr localSubmapGraph, const BasicPoint2d& pos,
                                       double yaw, bool processAll) {
  throw std::runtime_error("Not implemented yet!");
}

LaneData MapDataInterface::laneData() {
  if (!currPos_) {
    throw InvalidObjectStateError(
        "Your current position is not set! Call setCurrPosAndExtractSubmap() before trying to get the data!");
  }
  if (!currYaw_) {
    throw InvalidObjectStateError(
        "Your current yaw angle is not set! Call setCurrPosAndExtractSubmap() before trying to get the data!");
  }
  return getLaneData(localSubmap_, localSubmapGraph_, *currPos_, *currYaw_);
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