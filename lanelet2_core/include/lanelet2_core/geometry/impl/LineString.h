#pragma once
#include <boost/geometry/algorithms/intersection.hpp>
#include "../../primitives/LineString.h"
#include "../../primitives/Traits.h"
#include "../GeometryHelper.h"
#include "../Point.h"

namespace lanelet {
namespace geometry {
namespace internal {
template <typename T>
struct GetGeometry<T, IfLS<T, void>> {
  static inline auto twoD(const T& geometry) { return traits::toHybrid(traits::to2D(geometry)); }
  static inline auto threeD(const T& geometry) { return traits::toHybrid(traits::to3D(geometry)); }
};

inline auto crossProd(const BasicPoint3d& p1, const BasicPoint3d& p2) { return p1.cross(p2).eval(); }
inline auto crossProd(const BasicPoint2d& p1, const BasicPoint2d& p2) {
  return BasicPoint3d(p1.x(), p1.y(), 0.).cross(BasicPoint3d(p2.x(), p2.y(), 0.)).eval();
}

template <typename LineStringT, typename BasicPointT>
auto findPoint(const LineStringT& ls, const BasicPointT& p) {
  return std::find_if(ls.begin(), ls.end(), [&p](const auto& elem) { return boost::geometry::equals(elem, p); });
}

template <typename PointT>
bool pointIsLeftOf(const PointT& pSeg1, const PointT& pSeg2, const PointT& p) {
  return crossProd(PointT(pSeg2 - pSeg1), PointT(p - pSeg1)).z() > 0;
}

template <typename LineStringT>
LineStringT invert(const LineStringT& ls) {
  return ls.invert();
}

template <>
inline BasicLineString2d invert(const BasicLineString2d& ls) {
  return BasicLineString2d{ls.rbegin(), ls.rend()};
}

template <>
inline BasicLineString3d invert(const BasicLineString3d& ls) {
  return BasicLineString3d{ls.rbegin(), ls.rend()};
}

template <typename LineStringT, typename BasicPointT>
bool isLeftOf(const LineStringT& ls, const BasicPointT& p, const helper::ProjectedPoint<BasicPointT>& projectedPoint) {
  BasicPointT pSeg1 = projectedPoint.result->segmentPoint1;
  BasicPointT pSeg2 = projectedPoint.result->segmentPoint2;
  BasicPointT projPoint = projectedPoint.result->projectedPoint;
  bool isLeft = pointIsLeftOf(pSeg1, pSeg2, p);
  if (pSeg2 == projPoint) {
    auto nextSegPointIt = std::next(findPoint(ls, pSeg2));
    if (nextSegPointIt != ls.end()) {
      // see stackoverflow.com/questions/10583212
      BasicPointT nextSegPoint;
      boost::geometry::convert(*nextSegPointIt, nextSegPoint);
      if (isLeft != pointIsLeftOf(pSeg2, nextSegPoint, p) && isLeft == pointIsLeftOf(pSeg1, pSeg2, nextSegPoint)) {
        return !isLeft;
      }
    }
  }
  return isLeft;
}

template <typename LineStringT, typename PointT>
std::pair<double, helper::ProjectedPoint<PointT>> signedDistanceImpl(const LineStringT lineString, const PointT& p) {
  using BasicPoint = PointT;
  helper::ProjectedPoint<BasicPoint> projectedPoint;
  const auto d = distance(lineString, p, projectedPoint);
  auto isLeft = isLeftOf(lineString, p, projectedPoint);
  return {isLeft ? d : -d, projectedPoint};
}

std::pair<BasicPoint3d, BasicPoint3d> projectedPoint3d(const ConstHybridLineString3d& l1,
                                                       const ConstHybridLineString3d& l2);

std::pair<BasicPoint3d, BasicPoint3d> projectedPoint3d(const CompoundHybridLineString3d& l1,
                                                       const CompoundHybridLineString3d& l2);
}  // namespace internal

template <typename LineStringIterator>
double rangedLength(LineStringIterator start, LineStringIterator end) {
  double l = 0.;
  helper::forEachPair(start, end, [&l](const auto& seg1, const auto& seg2) { l += distance(seg1, seg2); });
  return l;
}

template <typename LineStringT>
std::vector<double> lengthRatios(const LineStringT& lineString) {
  std::vector<double> lengths;
  if (lineString.size() <= 1) {
    return lengths;
  }
  if (lineString.size() == 2) {
    return {1.};
  }
  const auto totalLength = length(lineString);
  lengths.reserve(lineString.size() - 1);
  helper::forEachPair(lineString.begin(), lineString.end(), [&lengths, &totalLength](const auto& p1, const auto& p2) {
    lengths.push_back(distance(p1, p2) / totalLength);
  });
  return lengths;
}

template <typename LineStringT>
std::vector<double> accumulatedLengthRatios(const LineStringT& lineString) {
  auto lengths = lengthRatios(lineString);
  helper::forEachPair(lengths.begin(), lengths.end(), [](const auto& l1, auto& l2) { l2 += l1; });
  return lengths;
}

template <typename LineStringT>
traits::BasicPointT<traits::PointType<LineStringT>> interpolatedPointAtDistance(LineStringT lineString, double dist) {
  assert(!lineString.empty());
  if (dist < 0) {
    lineString = internal::invert(lineString);
    dist = -dist;
  }

  double currentCumulativeLength = 0.0;
  for (auto first = lineString.begin(), second = std::next(lineString.begin()); second != lineString.end();
       ++first, ++second) {
    const auto p1 = traits::toBasicPoint(*first);
    const auto p2 = traits::toBasicPoint(*second);
    double currentLength = distance(p1, p2);
    currentCumulativeLength += currentLength;
    if (currentCumulativeLength >= dist) {
      double remainingDistance = dist - (currentCumulativeLength - currentLength);
      if (remainingDistance < 1.e-8) {
        return p1;
      }
      return p1 + remainingDistance / currentLength * (p2 - p1);
    }
  }
  return traits::toBasicPoint(lineString.back());
}

template <typename LineStringT>
traits::PointType<LineStringT> nearestPointAtDistance(LineStringT lineString, double dist) {
  using traits::toBasicPoint;
  assert(!lineString.empty());
  if (dist < 0) {
    lineString = internal::invert(lineString);
    dist = -dist;
  }
  double currentCumulativeLength = 0.0;
  for (auto first = lineString.begin(), second = std::next(lineString.begin()); second != lineString.end();
       ++first, ++second) {
    const auto& p1 = *first;
    const auto& p2 = *second;
    double currentLength = distance(p1, p2);
    currentCumulativeLength += currentLength;
    if (currentCumulativeLength >= dist) {
      double remainingDistance = dist - (currentCumulativeLength - currentLength);
      if (remainingDistance > currentLength / 2.0) {
        return p2;
      }
      return p1;
    }
  }
  return lineString.back();
}

template <typename LineString3dT>
double signedDistance(const LineString3dT& lineString, const BasicPoint3d& p) {
  static_assert(traits::is3D<LineString3dT>(), "Please call this function with a 3D type!");
  return internal::signedDistanceImpl(lineString, p).first;
}

template <typename LineString2dT>
double signedDistance(const LineString2dT& lineString, const BasicPoint2d& p) {
  static_assert(traits::is2D<LineString2dT>(), "Please call this function with a 2D type!");
  return internal::signedDistanceImpl(lineString, p).first;
}

template <typename LineString2dT>
ArcCoordinates toArcCoordinates(const LineString2dT& lineString, const BasicPoint2d& point) {
  auto res = internal::signedDistanceImpl(lineString, point);
  auto dist = res.first;
  const auto& projectedPoint = res.second;
  // find first point in segment in linestring
  double length = 0.;
  auto accumulateLength = [&length, &point = projectedPoint.result->segmentPoint1](const auto& first,
                                                                                   const auto& second) {
    if (boost::geometry::equals(first, point)) {
      return true;
    }
    length += distance(first, second);
    return false;
  };
  helper::forEachPairUntil(lineString.begin(), lineString.end(), accumulateLength);
  length += distance(projectedPoint.result->segmentPoint1, projectedPoint.result->projectedPoint);
  return {length, dist};
}

template <typename LineString3dT>
IfLS<LineString3dT, BoundingBox3d> boundingBox3d(const LineString3dT& lineString) {
  static_assert(traits::is3D<LineString3dT>(), "Please call this function with a 3D type!");
  BoundingBox3d bb;
  for (const auto& p : lineString) {
    bb.extend(traits::toBasicPoint(p));
  }
  return bb;
}

template <typename LineString2dT>
IfLS<LineString2dT, BoundingBox2d> boundingBox2d(const LineString2dT& lineString) {
  BoundingBox2d bb;
  for (const auto& p : traits::to2D(lineString)) {
    bb.extend(traits::toBasicPoint(p));
  }
  return bb;
}

template <typename LineString3dT, typename>
BasicPoint3d project(const LineString3dT& lineString, const BasicPoint3d& pointToProject) {
  static_assert(traits::is3D<LineString3dT>(), "Please call this function with a 3D type!");
  helper::ProjectedPoint<BasicPoint3d> projectedPoint;
  distance(lineString, pointToProject, projectedPoint);
  return projectedPoint.result->projectedPoint;
}

template <typename LineString2dT, typename>
BasicPoint2d project(const LineString2dT& lineString, const BasicPoint2d& pointToProject) {
  static_assert(traits::is2D<LineString2dT>(), "Please call this function with a 2D type!");
  helper::ProjectedPoint<BasicPoint2d> projectedPoint;
  distance(lineString, pointToProject, projectedPoint);
  return projectedPoint.result->projectedPoint;
}

template <typename LineString3dT>
IfLS<LineString3dT, bool> intersects3d(const LineString3dT& linestring, const LineString3dT& otherLinestring,
                                       double heightTolerance) {
  auto ls2d(traits::toHybrid(traits::to2D(linestring)));
  auto ols2d(traits::toHybrid(traits::to2D(otherLinestring)));
  BasicPoints2d intersections;
  boost::geometry::intersection(ls2d, ols2d, intersections);
  auto distanceSmallerTolerance = [heightTolerance, &linestring, &otherLinestring](const auto& point) {
    auto pProj1 = project(linestring, BasicPoint3d(point.x(), point.y(), 0));
    auto pProj2 = project(otherLinestring, BasicPoint3d(point.x(), point.y(), 0));
    return distance(pProj1, pProj2) < heightTolerance;
  };
  return std::any_of(intersections.begin(), intersections.end(), distanceSmallerTolerance);
}

template <typename LineString3dT>
IfLS<LineString3dT, std::pair<BasicPoint3d, BasicPoint3d>> projectedPoint3d(const LineString3dT& l1,
                                                                            const LineString3dT& l2) {
  return internal::projectedPoint3d(traits::toHybrid(l1), traits::toHybrid(l2));
}

template <typename LineString3d1T, typename LineString3d2T>
IfLS<LineString3d1T, double> distance3d(const LineString3d1T& l1, const LineString3d2T& l2) {
  auto projPoint = internal::projectedPoint3d(traits::toHybrid(traits::to3D(l1)), traits::toHybrid(traits::to3D(l2)));
  return (projPoint.first - projPoint.second).norm();
}

template <typename LineString1T, typename LineString2T>
std::pair<LineString1T, LineString2T> align(LineString1T left, LineString2T right) {
  using traits::toBasicPoint;
  // degenerated case
  if ((left.size() <= 1 && right.size() <= 1) || right.empty() || left.empty()) {
    return {left, right};
  }
  auto getMiddlePoint = [](auto& ls) {
    return ls.size() > 2 ? toBasicPoint(ls[ls.size() / 2]) : (toBasicPoint(ls.front()) + toBasicPoint(ls.back())) * 0.5;
  };
  //! @todo this sadly is a bit heuristical...
  bool rightOfLeft = signedDistance(left, getMiddlePoint(right)) < 0;
  if (!rightOfLeft && left.size() > 1) {
    left = left.invert();
  }

  bool leftOfRight = signedDistance(right, getMiddlePoint(left)) > 0;
  if (!leftOfRight && right.size() > 1) {
    right = right.invert();
  }
  return {left, right};
}
}  // namespace geometry
}  // namespace lanelet
