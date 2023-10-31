#include "lanelet2_map_learning/Utils.h"

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>

namespace lanelet {
namespace map_learning {

OrientedRect getRotatedRect(const BasicPoint2d& center, double extentLongitudinal, double extentLateral, double yaw) {
  BasicPoints2d pts{BasicPoint2d{center.x() - extentLongitudinal, center.y() - extentLateral},
                    BasicPoint2d{center.x() - extentLongitudinal, center.y() + extentLateral},
                    BasicPoint2d{center.x() + extentLongitudinal, center.y() + extentLateral},
                    BasicPoint2d{center.x() + extentLongitudinal, center.y() - extentLateral}};
  OrientedRect axisAlignedRect;
  boost::geometry::assign_points(axisAlignedRect, pts);

  boost::geometry::strategy::transform::matrix_transformer<double, 2, 2> trans(
      cos(yaw), sin(yaw), center.x(), -sin(yaw), cos(yaw), center.y(), 0, 0, 1);
  OrientedRect rotatedRect;
  boost::geometry::transform(axisAlignedRect, rotatedRect, trans);
  return rotatedRect;
}

LaneletSubmapConstPtr extractSubmap(LaneletMapConstPtr laneletMap, const BasicPoint2d& center, double yaw,
                                    double extentLongitudinal, double extentLateral) {
  double maxExtent = std::max(extentLongitudinal, extentLateral);
  BasicPoint2d initRegionRear = {center.x() - 1.1 * maxExtent, center.y() - 1.1 * maxExtent};
  BasicPoint2d initRegionFront = {center.x() + 1.1 * maxExtent, center.y() + 1.1 * maxExtent};
  BoundingBox2d initSearchRegion{initRegionRear, initRegionFront};
  ConstLanelets initRegion = laneletMap->laneletLayer.search(initSearchRegion);
  return utils::createConstSubmap(initRegion, {});
}

Eigen::Vector3d getLaneletRepr(const LaneletRepresentationType& reprType, const ParametrizationType& paramType,
                               int nPoints) {
  int32_t nodeFeatureLength;
  if (reprType == LaneletRepresentationType::Boundaries)
    nodeFeatureLength = 2 * 3 * nPoints + 2;  // 2 boundary types
  else if (reprType == LaneletRepresentationType::Centerline)
    nodeFeatureLength = 3 * nPoints + 2;
  else
    throw std::runtime_error("Unknown LaneletRepresentationType!");

  return Eigen::Vector3d(nodeFeatureLength);
}

inline int bdSubtypeToInt(ConstLineString3d lString) {
  std::string subtype = lString.attribute(AttributeName::Subtype).value();
  if (subtype == AttributeValueString::Dashed)
    return 1;
  else if (subtype == AttributeValueString::Solid)
    return 2;
  else if (subtype == AttributeValueString::SolidSolid)
    return 2;
  else if (subtype == AttributeValueString::SolidDashed)
    return 2;
  else if (subtype == AttributeValueString::DashedSolid)
    return 2;
  else if (subtype == AttributeValueString::Virtual)
    return 3;
  else {
    throw std::runtime_error("Unexpected Line String Subtype!");
    return 0;
  }
}

inline int teTypeToInt(const ConstLineString3d& te) {
  std::string type = te.attribute(AttributeName::Type).value();
  std::string subtype = te.attribute(AttributeName::Subtype).value();
  if (type == AttributeValueString::TrafficLight) {
    return 1;
  } else if (type == AttributeValueString::TrafficSign) {
    return 2;
  } else {
    throw std::runtime_error("Unexpected Traffic Element Type!");
    return 0;
  }
}

Eigen::Vector3d getTERepr() {
  return Eigen::Vector3d(13);  // 4 points with 3 dims + type;
}

BasicLineString3d resamplePolyline(const BasicLineString3d& polyline, int32_t nPoints) {
  double length = boost::geometry::length(polyline, boost::geometry::strategy::distance::pythagoras<double>());
  double dist = length / static_cast<double>(nPoints);
  boost::geometry::model::multi_point<BasicPoint3d> bdInterp;
  boost::geometry::line_interpolate(polyline, dist, bdInterp);
  assert(bdInterp.size() == nPoints);
  return bdInterp;
}

BasicLineString3d cutPolyline(const OrientedRect& bbox, const BasicLineString3d& polyline, int32_t nPoints) {
  std::deque<BasicLineString3d> output;
  boost::geometry::intersection(bbox, polyline, output);
  assert(output.size() == 1);
  return output[0];
}

}  // namespace map_learning
}  // namespace lanelet