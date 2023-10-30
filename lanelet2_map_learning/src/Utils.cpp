#include "lanelet2_map_learning/Utils.h"

namespace lanelet {
namespace map_learning {

boost::geometry::model::polygon<BasicPoint2d> getRotatedRect(const BasicPoint2d& center, double extentLongitudinal,
                                                             double extentLateral, double yaw) {
  BasicPoints2d pts{BasicPoint2d{center.x() - extentLongitudinal, center.y() - extentLateral},
                    BasicPoint2d{center.x() - extentLongitudinal, center.y() + extentLateral},
                    BasicPoint2d{center.x() + extentLongitudinal, center.y() + extentLateral},
                    BasicPoint2d{center.x() + extentLongitudinal, center.y() - extentLateral}};
  boost::geometry::model::polygon<BasicPoint2d> axisAlignedRect;
  boost::geometry::assign_points(axisAlignedRect, pts);

  boost::geometry::strategy::transform::matrix_transformer<double, 2, 2> trans(
      cos(yaw), sin(yaw), center.x(), -sin(yaw), cos(yaw), center.y(), 0, 0, 1);
  boost::geometry::model::polygon<BasicPoint2d> rotatedRect;
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

int32_t getNodeFeatureLength(const LaneletRepresentationType& reprType, const ParametrizationType& paramType,
                             int nPoints) {
  int32_t nodeFeatureLength;
  if (reprType == LaneletRepresentationType::Boundaries)
    nodeFeatureLength = 2 * 3 * nPoints + 2;  // 2 boundary types with 2 possible types
  else if (reprType == LaneletRepresentationType::Centerline)
    nodeFeatureLength = 3 * nPoints + 2;
  else
    throw std::runtime_error("Unknown LaneletRepresentationType!");

  return nodeFeatureLength;
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

Eigen::Vector3d getTEPolylineRepr(const BasicLineString3d& te) {
  Eigen::Vector3d repr(12);
  for (size_t i = 0; i < te.size(); i++) {
    repr(Eigen::seq(i, i + 2)) = te[i](Eigen::seq(i, i + 2));
  }
  return repr;
}

}  // namespace map_learning
}  // namespace lanelet