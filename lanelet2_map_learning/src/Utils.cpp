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

BasicLineString3d resampleLineString(const BasicLineString3d& polyline, int32_t nPoints) {
  double length = boost::geometry::length(polyline, boost::geometry::strategy::distance::pythagoras<double>());
  double dist = length / static_cast<double>(nPoints);
  boost::geometry::model::multi_point<BasicPoint3d> bdInterp;
  boost::geometry::line_interpolate(polyline, dist, bdInterp);
  assert(bdInterp.size() == nPoints);
  return bdInterp;
}

BasicLineString3d cutLineString(const OrientedRect& bbox, const BasicLineString3d& polyline) {
  BasicLineString2d polyline2d;
  for (const auto& pt : polyline) {
    polyline2d.push_back(BasicPoint2d(pt.x(), pt.y()));
  }
  std::deque<BasicLineString2d> cut2d;
  boost::geometry::intersection(bbox, polyline2d, cut2d);
  assert(cut2d.size() == 1);

  // restore z value from closest point on the original linestring
  BasicLineString3d cut3d;
  for (const auto& pt2d : cut2d[0]) {
    double lastDist = std::numeric_limits<double>::max();
    for (const auto& pt : polyline) {
      double currDist = (pt2d - BasicPoint2d(pt.x(), pt.y())).norm();
      if (currDist < lastDist) {
        lastDist = currDist;
        if (pt == polyline.back()) {
          cut3d.push_back(BasicPoint3d(pt2d.x(), pt2d.y(), pt.z()));
        }
      } else {
        cut3d.push_back(BasicPoint3d(pt2d.x(), pt2d.y(), pt.z()));
      }
    }
  }

  return cut3d;
}

}  // namespace map_learning
}  // namespace lanelet