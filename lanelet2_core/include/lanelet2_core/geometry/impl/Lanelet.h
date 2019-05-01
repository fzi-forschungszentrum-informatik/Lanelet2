#pragma once
#include <boost/version.hpp>
#if BOOST_VERSION > 105800
#include <string.h>  // NOLINT
#include <boost/geometry/algorithms/relate.hpp>
#else
#include <boost/geometry/algorithms/detail/relate/relate.hpp>
#endif
#include "../../primitives/Lanelet.h"
#include "../../primitives/LineString.h"
#include "../Polygon.h"

namespace lanelet {
namespace geometry {

template <typename LaneletT>
IfLL<LaneletT, bool> inside(const LaneletT& lanelet, const BasicPoint2d& point) {
  return boost::geometry::covered_by(point, lanelet.polygon2d());
}

template <typename LaneletT>
IfLL<LaneletT, double> distance2d(const LaneletT& lanelet, const BasicPoint2d& point) {
  return distance(lanelet.polygon2d(), point);
}

template <typename LaneletT>
IfLL<LaneletT, double> distance2d(const LaneletT& lanelet1, const LaneletT& lanelet2) {
  return distance(traits::toHybrid(lanelet1.polygon2d()), traits::toHybrid(lanelet2.polygon2d()));
}

template <typename LaneletT>
IfLL<LaneletT, double> distance3d(const LaneletT& lanelet, const BasicPoint3d& point) {
  return distance(lanelet.polygon3d(), point);
}

template <typename LaneletT>
double distanceToCenterline2d(const LaneletT& lanelet, const BasicPoint2d& point) {
  return distance(ConstLineString2d(lanelet.centerline()), point);
}

template <typename LaneletT>
double distanceToCenterline3d(const LaneletT& lanelet, const BasicPoint3d& point) {
  return distance(lanelet.centerline(), point);
}

template <typename LaneletT>
IfLL<LaneletT, BoundingBox2d> boundingBox2d(const LaneletT& lanelet) {
  BoundingBox2d bb = boundingBox2d(ConstLineString2d(lanelet.leftBound()));
  bb.extend(boundingBox2d(ConstLineString2d(lanelet.rightBound())));
  return bb;
}

template <typename LaneletT>
IfLL<LaneletT, BoundingBox3d> boundingBox3d(const LaneletT& lanelet) {
  BoundingBox3d bb = boundingBox3d(lanelet.leftBound());
  bb.extend(boundingBox3d(lanelet.rightBound()));
  return bb;
}

template <typename Lanelet1T, typename Lanelet2T>
IfLL<Lanelet1T, bool> intersects2d(const Lanelet1T& lanelet, const Lanelet2T& otherLanelet) {
  if (lanelet.constData() == otherLanelet.constData()) {
    return true;
  }
  CompoundHybridPolygon2d p1(lanelet.polygon2d());
  CompoundHybridPolygon2d p2(otherLanelet.polygon2d());
  return intersects(p1, p2);
}

template <typename Lanelet1T, typename Lanelet2T>
IfLL<Lanelet1T, bool> overlaps2d(const Lanelet1T& lanelet, const Lanelet2T& otherLanelet) {
  if (lanelet.constData() == otherLanelet.constData()) {
    return true;
  }
  // check if lanelets are neighbours (share a border)
  if (lanelet.rightBound() == otherLanelet.leftBound() || lanelet.leftBound() == otherLanelet.rightBound() ||
      lanelet.leftBound().invert() == otherLanelet.leftBound() ||
      lanelet.rightBound().invert() == otherLanelet.rightBound() || follows(lanelet, otherLanelet) ||
      follows(otherLanelet, lanelet) || follows(lanelet.invert(), otherLanelet) ||
      follows(otherLanelet.invert(), lanelet)) {
    return false;
  }
  if (!intersects(boundingBox2d(lanelet), boundingBox2d(otherLanelet))) {
    return false;
  }
  CompoundHybridPolygon2d p1(lanelet.polygon2d());
  CompoundHybridPolygon2d p2(otherLanelet.polygon2d());

#if BOOST_VERSION > 105800
  using Mask = boost::geometry::de9im::static_mask<'T', '*', '*', '*', '*', '*', '*', '*', '*'>;
  return boost::geometry::relate(p1, p2, Mask());
#else
  // boost 1.58 did not officially support "relate"
  using Mask = boost::geometry::detail::relate::static_mask<'T', '*', '*', '*', '*', '*', '*', '*', '*'>;
  return boost::geometry::detail::relate::relate<Mask>(p1, p2);
#endif
}

template <typename Lanelet1T, typename Lanelet2T>
BasicPoints2d intersectCenterlines2d(const Lanelet1T& lanelet, const Lanelet2T& otherLanelet,
                                     std::vector<double>* distanceThis, std::vector<double>* distanceOther) {
  //! @todo implement intersect_centerlines
  BasicPoints2d intersections;
  const auto centerline = ConstHybridLineString2d(lanelet.centerline());
  const auto otherCenterline = ConstHybridLineString2d(otherLanelet.centerline());
  boost::geometry::intersection(centerline, otherCenterline, intersections);
  if (distanceThis != nullptr) {
    std::transform(intersections.begin(), intersections.end(), std::back_inserter(*distanceThis),
                   [&centerline](const auto& elem) { return toArcCoordinates(centerline, elem).length; });
  }
  if (distanceOther != nullptr) {
    std::transform(intersections.begin(), intersections.end(), std::back_inserter(*distanceOther),
                   [&centerline](const auto& elem) { return toArcCoordinates(centerline, elem).length; });
  }
  return intersections;
}

template <typename Lanelet1T, typename Lanelet2T>
IfLL<Lanelet1T, bool> intersects3d(const Lanelet1T& lanelet, const Lanelet2T& otherLanelet, double heightTolerance) {
  if (intersects2d(lanelet, otherLanelet)) {
    auto projPts = projectedPoint3d(lanelet.centerline(), otherLanelet.centerline());
    return std::abs(projPts.first.z() - projPts.second.z()) < heightTolerance;
  }
  return false;
}

template <typename Lanelet1T, typename Lanelet2T>
IfLL<Lanelet1T, bool> overlaps3d(const Lanelet1T& lanelet, const Lanelet2T& otherLanelet, double heightTolerance) {
  if (overlaps2d(lanelet, otherLanelet)) {
    auto projPts = projectedPoint3d(lanelet.centerline(), otherLanelet.centerline());
    return std::abs(projPts.first.z() - projPts.second.z()) < heightTolerance;
  }
  return false;
}

template <typename Lanelet1T, typename Lanelet2T>
IfLL<Lanelet1T, IfLL<Lanelet2T, bool>> leftOf(const Lanelet1T& left, const Lanelet2T& right) {
  return left.rightBound() == right.leftBound();
}

template <typename Lanelet1T, typename Lanelet2T>
IfLL<Lanelet1T, IfLL<Lanelet2T, bool>> rightOf(const Lanelet1T& right, const Lanelet2T& left) {
  return leftOf(left, right);
}

template <typename Lanelet1T, typename Lanelet2T>
IfLL<Lanelet1T, IfLL<Lanelet2T, bool>> follows(const Lanelet1T& prev, const Lanelet2T& next) {
  return !prev.leftBound().empty() && !prev.rightBound().empty() && !next.leftBound().empty() &&
         !next.rightBound().empty() && prev.leftBound().back() == next.leftBound().front() &&
         prev.rightBound().back() == next.rightBound().front();
}

template <typename LaneletT>
Velocity maxCurveSpeed(const LaneletT& /*lanelet*/, const BasicPoint2d& /*position*/,
                       const Acceleration& /*maxLateralAcceleration*/) {
  //!< @todo Implement maxCurveSpeed
  throw LaneletError("This is not yet implemented!");
}

template <typename LaneletT>
double approximatedLength2d(const LaneletT& lanelet) {
  double length = 0.;
  auto line = lanelet.leftBound();
  auto step = std::max(1ul, line.size() / 10);
  for (auto i1 = 0ul, i2 = step; i2 < line.size(); i1 += step, i2 += step) {
    length += distance2d(line[i1], line[i2]);
    if (i2 + step >= line.size()) {
      length += distance2d(line[i2], line[line.size() - 1]);
    }
  }
  return length;
}

template <typename LaneletT>
double length2d(const LaneletT& lanelet) {
  return double(length(lanelet.centerline()));
}

template <typename LaneletT>
double length3d(const LaneletT& lanelet) {
  return double(length(lanelet.centerline()));
}

}  // namespace geometry
}  // namespace lanelet
