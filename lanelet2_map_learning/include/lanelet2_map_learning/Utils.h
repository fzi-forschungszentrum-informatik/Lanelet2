#pragma once
#include <lanelet2_core/Exceptions.h>
#include <lanelet2_core/Forward.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>;
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <boost/geometry.hpp>
#include <type_traits>

#include "lanelet2_map_learning/Forward.h"
#include "lanelet2_map_learning/Types.h"

namespace lanelet {
namespace map_learning {

boost::geometry::model::polygon<BasicPoint2d> getRotatedRect(const BasicPoint2d& center, double extentLongitudinal,
                                                             double extentLateral, double yaw);

LaneletSubmapConstPtr extractSubmap(LaneletMapConstPtr laneletMap, const BasicPoint2d& center, double yaw,
                                    double extentLongitudinal, double extentLateral);

Eigen::Vector3d getLaneletRepr(const LaneletRepresentationType& reprType, const ParametrizationType& paramType,
                               int nPoints);

inline int bdSubtypeToInt(ConstLineString3d lString);

inline int teTypeToInt(const ConstLineString3d& te);

Eigen::Vector3d getTERepr();

BasicLineString3d resamplePolyline(const BasicLineString3d& polyline, int32_t nPoints);

BasicLineString3d cutPolyline(const OrientedRect& bbox, const BasicLineString3d& polyline);

}  // namespace map_learning
}  // namespace lanelet