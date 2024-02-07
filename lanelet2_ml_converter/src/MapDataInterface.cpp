#include "lanelet2_ml_converter/MapDataInterface.h"

#include <lanelet2_core/Exceptions.h>
#include <lanelet2_core/Forward.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <boost/geometry.hpp>
#include <type_traits>

#include "lanelet2_ml_converter/MapData.h"
#include "lanelet2_ml_converter/Utils.h"

namespace lanelet {
namespace ml_converter {

void MapDataInterface::setCurrPosAndExtractSubmap2d(const BasicPoint2d& pt, double yaw) {
  setCurrPosAndExtractSubmap(BasicPoint3d{pt.x(), pt.y(), 0}, yaw, 0, 0);
  currBbox_ = getRotatedRect(*currPos_, config_.submapExtentLongitudinal, config_.submapExtentLateral, *currYaw_, true);
}

void MapDataInterface::setCurrPosAndExtractSubmap(const BasicPoint3d& pt, double yaw) {
  setCurrPosAndExtractSubmap(pt, yaw, 0, 0);
  currBbox_ =
      getRotatedRect(*currPos_, config_.submapExtentLongitudinal, config_.submapExtentLateral, *currYaw_, false);
}

void MapDataInterface::setCurrPosAndExtractSubmap(const BasicPoint3d& pt, double yaw, double pitch, double roll) {
  currPos_ = pt;
  currYaw_ = yaw;
  currPitch_ = pitch;
  currRoll_ = roll;
  localSubmap_ =
      extractSubmap(laneletMap_, currPos_->head(2), config_.submapExtentLongitudinal, config_.submapExtentLateral);
  localSubmapGraph_ = lanelet::routing::RoutingGraph::build(*localSubmap_, *trafficRules_);
  currBbox_ =
      getRotatedRect(*currPos_, config_.submapExtentLongitudinal, config_.submapExtentLateral, *currYaw_, false);
}

MapDataInterface::MapDataInterface(LaneletMapConstPtr laneletMap)
    : laneletMap_{laneletMap},
      config_{},
      trafficRules_{traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Vehicle)} {}

MapDataInterface::MapDataInterface(LaneletMapConstPtr laneletMap, Configuration config)
    : laneletMap_{laneletMap},
      config_{config},
      trafficRules_{traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Vehicle)} {}

LaneDataPtr MapDataInterface::getLaneData(LaneletSubmapConstPtr localSubmap, const OrientedRect& bbox,
                                          lanelet::routing::RoutingGraphConstPtr localSubmapGraph, double pitch,
                                          double roll, bool processAll) {
  LaneDataPtr laneData = LaneData::build(localSubmap, localSubmapGraph);
  if (processAll) {
    laneData->processAll(bbox, config_.paramType, config_.nPoints, pitch, roll);
  }
  return laneData;
}

std::vector<LaneDataPtr> MapDataInterface::laneDataBatch2d(std::vector<BasicPoint2d> pts, std::vector<double> yaws) {
  if (pts.size() != yaws.size()) {
    throw std::runtime_error("Unequal sizes of pts and rotation angles!");
  }
  std::vector<LaneDataPtr> lDataVec;
  for (size_t i = 0; i < pts.size(); i++) {
    LaneletSubmapConstPtr localSubmap =
        extractSubmap(laneletMap_, pts[i], config_.submapExtentLongitudinal, config_.submapExtentLateral);
    routing::RoutingGraphConstPtr localSubmapGraph =
        lanelet::routing::RoutingGraph::build(*localSubmap, *trafficRules_);
    OrientedRect bbox = getRotatedRect(BasicPoint3d{pts[i].x(), pts[i].y(), 0}, config_.submapExtentLongitudinal,
                                       config_.submapExtentLateral, yaws[i], true);
    lDataVec.push_back(getLaneData(localSubmap, bbox, localSubmapGraph, 0, 0, true));
  }
  return lDataVec;
}

std::vector<LaneDataPtr> MapDataInterface::laneDataBatch(std::vector<BasicPoint3d> pts, std::vector<double> yaws) {
  return laneDataBatch(pts, yaws, std::vector<double>(yaws.size(), 0), std::vector<double>(yaws.size(), 0));
}

std::vector<LaneDataPtr> MapDataInterface::laneDataBatch(std::vector<BasicPoint3d> pts, std::vector<double> yaws,
                                                         std::vector<double> pitches, std::vector<double> rolls) {
  if ((pts.size() != yaws.size()) || (pts.size() != pitches.size()) || (pts.size() != rolls.size())) {
    throw std::runtime_error("Unequal sizes of pts and rotation angles!");
  }
  std::vector<LaneDataPtr> lDataVec;
  for (size_t i = 0; i < pts.size(); i++) {
    LaneletSubmapConstPtr localSubmap =
        extractSubmap(laneletMap_, pts[i].head(2), config_.submapExtentLongitudinal, config_.submapExtentLateral);
    routing::RoutingGraphConstPtr localSubmapGraph =
        lanelet::routing::RoutingGraph::build(*localSubmap, *trafficRules_);
    OrientedRect bbox =
        getRotatedRect(pts[i], config_.submapExtentLongitudinal, config_.submapExtentLateral, yaws[i], false);
    lDataVec.push_back(getLaneData(localSubmap, bbox, localSubmapGraph, pitches[i], rolls[i], true));
  }
  return lDataVec;
}

std::vector<TEData> MapDataInterface::laneTEDataBatch(std::vector<BasicPoint2d> pts, std::vector<double> yaws,
                                                      std::vector<double> pitches, std::vector<double> rolls) {
  throw std::runtime_error("Not implemented yet!");
}

bool isTe(ConstLineString3d ls) {
  std::string type = ls.attribute(AttributeName::Type).value();
  return type == AttributeValueString::TrafficLight || type == AttributeValueString::TrafficSign;
}

TEData MapDataInterface::getLaneTEData(LaneletSubmapConstPtr localSubmap, const OrientedRect& bbox,
                                       lanelet::routing::RoutingGraphConstPtr localSubmapGraph, double pitch,
                                       double roll, bool processAll) {
  throw std::runtime_error("Not implemented yet!");
}

LaneDataPtr MapDataInterface::laneData(bool processAll) {
  if (!currPos_) {
    throw InvalidObjectStateError(
        "Your current position is not set! Call setCurrPosAndExtractSubmap() before trying to get the data!");
  }
  if (!currYaw_) {
    throw InvalidObjectStateError(
        "Your current yaw angle is not set! Call setCurrPosAndExtractSubmap() before trying to get the data!");
  }
  if (!currPitch_) {
    throw InvalidObjectStateError(
        "Your current pitch angle is not set! Call setCurrPosAndExtractSubmap() before trying to get the data!");
  }
  if (!currRoll_) {
    throw InvalidObjectStateError(
        "Your current roll angle is not set! Call setCurrPosAndExtractSubmap() before trying to get the data!");
  }
  return getLaneData(localSubmap_, *currBbox_, localSubmapGraph_, *currPitch_, *currRoll_, processAll);
}

TEData MapDataInterface::teData(bool processAll) {
  if (!currPos_) {
    throw InvalidObjectStateError(
        "Your current position is not set! Call setCurrPosAndExtractSubmap() before trying to get the data!");
  }
  throw std::runtime_error("Not implemented yet!");
  return TEData();
}

}  // namespace ml_converter
}  // namespace lanelet