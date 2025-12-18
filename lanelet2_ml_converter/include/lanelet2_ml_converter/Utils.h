#pragma once
#include <lanelet2_core/Exceptions.h>
#include <lanelet2_core/Forward.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <boost/geometry.hpp>

#include "Forward.h"
#include "Serialize.h"
#include "Types.h"
namespace lanelet {
namespace ml_converter {

OrientedRect getRotatedRect(const BasicPoint3d& center, double extentLongitudinal, double extentLateral, double yaw,
                            bool from2dPos);

LaneletSubmapConstPtr extractSubmap(LaneletMapConstPtr laneletMap, const BasicPoint2d& center,
                                    double extentLongitudinal, double extentLateral);

inline bool isRoadBorder(const ConstLineString3d& lstring) {
  Attribute type = lstring.attributeOr(AttributeName::Type, "");
  return type == AttributeValueString::RoadBorder || type == AttributeValueString::Curbstone ||
         type == AttributeValueString::Fence;
}

inline LineStringType bdTypeToEnum(ConstLineString3d lString) {
  Attribute type = lString.attributeOr(AttributeName::Type, "");
  if (type == AttributeValueString::RoadBorder || type == AttributeValueString::Curbstone ||
      type == AttributeValueString::Fence) {
    return LineStringType::RoadBorder;
  } else if (type == AttributeValueString::Virtual) {
    return LineStringType::Virtual;
  }
  Attribute subtype = lString.attributeOr(AttributeName::Subtype, "");
  if (subtype == AttributeValueString::Dashed)
    return LineStringType::Dashed;
  else if (subtype == AttributeValueString::Solid)
    return LineStringType::Solid;
  else if (subtype == AttributeValueString::SolidSolid)
    return LineStringType::Solid;
  else if (subtype == AttributeValueString::SolidDashed)
    return LineStringType::Mixed;
  else if (subtype == AttributeValueString::DashedSolid)
    return LineStringType::Mixed;
  else {
    // throw std::runtime_error("Unexpected Line String Subtype!");
    return LineStringType::Unknown;
  }
}

inline TEType teTypeToEnum(const ConstLineString3d& te) {
  Attribute type = te.attributeOr(AttributeName::Type, "");
  Attribute subtype = te.attributeOr(AttributeName::Subtype, "");
  if (type == AttributeValueString::TrafficLight) {
    return TEType::TrafficLight;
  } else if (type == AttributeValueString::TrafficSign) {
    return TEType::TrafficSign;
  } else {
    // throw std::runtime_error("Unexpected Traffic Element Type!");
    return TEType::Unknown;
  }
}

BasicLineString3d resampleLineString(const BasicLineString3d& polyline, int32_t nPoints);

std::vector<BasicLineString3d> cutLineString(const OrientedRect& bbox, const BasicLineString3d& polyline);

BasicLineString3d transformLineString(const OrientedRect& bbox, const BasicLineString3d& polyline, double pitch,
                                      double roll);

void saveLaneData(const std::string& filename, const std::vector<LaneDataPtr>& lDataVec,
                  bool binary);  // saves all in one file

std::vector<LaneDataPtr> loadLaneData(const std::string& filename, bool binary);  // loads entire vector from one file

void saveLaneDataMultiFile(const std::string& path, const std::vector<std::string>& filenames,
                           const std::vector<LaneDataPtr>& lDataVec,
                           bool binary);  // saves in one file per LaneData

std::vector<LaneDataPtr> loadLaneDataMultiFile(const std::string& path, const std::vector<std::string>& filenames,
                                               bool binary);  // loads from one file per LaneData

}  // namespace ml_converter
}  // namespace lanelet