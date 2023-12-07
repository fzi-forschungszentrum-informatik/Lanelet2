#pragma once
#include <lanelet2_core/Exceptions.h>
#include <lanelet2_core/Forward.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <boost/geometry.hpp>
#include <type_traits>

#include "lanelet2_map_learning/Forward.h"
#include "lanelet2_map_learning/Types.h"

namespace lanelet {
namespace map_learning {

boost::geometry::model::polygon<BasicPoint2d> getRotatedRect(const BasicPoint2d& center, double extentLongitudinal,
                                                             double extentLateral, double yaw);

LaneletSubmapConstPtr extractSubmap(LaneletMapConstPtr laneletMap, const BasicPoint2d& center,
                                    double extentLongitudinal, double extentLateral);

inline LineStringType bdSubtypeToEnum(ConstLineString3d lString) {
  std::string subtype = lString.attribute(AttributeName::Subtype).value();
  if (subtype == AttributeValueString::Dashed)
    return LineStringType::Dashed;
  else if (subtype == AttributeValueString::Solid)
    return LineStringType::Solid;
  else if (subtype == AttributeValueString::SolidSolid)
    return LineStringType::Solid;
  else if (subtype == AttributeValueString::SolidDashed)
    return LineStringType::Solid;
  else if (subtype == AttributeValueString::DashedSolid)
    return LineStringType::Solid;
  else if (subtype == AttributeValueString::Virtual)
    return LineStringType::Virtual;
  else {
    throw std::runtime_error("Unexpected Line String Subtype!");
    return LineStringType::Unknown;
  }
}

inline TEType teTypeToEnum(const ConstLineString3d& te) {
  std::string type = te.attribute(AttributeName::Type).value();
  std::string subtype = te.attribute(AttributeName::Subtype).value();
  if (type == AttributeValueString::TrafficLight) {
    return TEType::TrafficLight;
  } else if (type == AttributeValueString::TrafficSign) {
    return TEType::TrafficSign;
  } else {
    throw std::runtime_error("Unexpected Traffic Element Type!");
    return TEType::Unknown;
  }
}

BasicLineString3d resampleLineString(const BasicLineString3d& polyline, int32_t nPoints);

BasicLineString3d cutLineString(const OrientedRect& bbox, const BasicLineString3d& polyline);

}  // namespace map_learning
}  // namespace lanelet