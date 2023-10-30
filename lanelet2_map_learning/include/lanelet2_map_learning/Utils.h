#include <lanelet2_core/Exceptions.h>
#include <lanelet2_core/Forward.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>;
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <boost/geometry.hpp>
#include <type_traits>

#include "lanelet2_map_learning/Forward.h"

namespace lanelet {
namespace map_learning {

boost::geometry::model::polygon<BasicPoint2d> getRotatedRect(const BasicPoint2d& center, double extentLongitudinal,
                                                             double extentLateral, double yaw);

LaneletSubmapConstPtr extractSubmap(LaneletMapConstPtr laneletMap, const BasicPoint2d& center, double yaw,
                                    double extentLongitudinal, double extentLateral);

int32_t getNodeFeatureLength(const LaneletRepresentationType& reprType, const ParametrizationType& paramType,
                             int nPoints);

inline int bdSubtypeToInt(ConstLineString3d lString);

inline int teTypeToInt(const ConstLineString3d& te);

Eigen::Vector3d getTEPolylineRepr(const BasicLineString3d& te);

}  // namespace map_learning
}  // namespace lanelet