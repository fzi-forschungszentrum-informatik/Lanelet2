#pragma once
#include "../../primitives/Polygon.h"
#include "../Point.h"
#if BOOST_VERSION > 105800
#include <boost/geometry/algorithms/relate.hpp>
#else
#include <boost/geometry/algorithms/detail/relate/relate.hpp>
#endif

namespace lanelet {
namespace geometry {
namespace internal {
std::pair<BasicPoint3d, BasicPoint3d> projectedBorderPoint3d(const ConstHybridPolygon3d& l1,
                                                             const ConstHybridPolygon3d& l2);

std::pair<BasicPoint3d, BasicPoint3d> projectedBorderPoint3d(const CompoundHybridPolygon3d& l1,
                                                             const CompoundHybridPolygon3d& l2);
}  // namespace internal

template <typename Polygon3dT>
IfPoly<Polygon3dT, std::pair<BasicPoint3d, BasicPoint3d>> projectedBorderPoint3d(const Polygon3dT& l1,
                                                                                 const Polygon3dT& l2) {
  return internal::projectedBorderPoint3d(traits::toHybrid(traits::to3D(l1)), traits::toHybrid(traits::to3D(l2)));
}

template <typename Polygon3dT>
IfPoly<Polygon3dT, double> distanceToBorder3d(const Polygon3dT& poly1, const Polygon3dT& poly2) {
  auto projPoint = internal::projectedBorderPoint3d(traits::toHybrid(poly1), traits::toHybrid(poly2));
  return (projPoint.first - projPoint.second).norm();
}

template <typename PolygonT>
IfPoly<PolygonT, bool> touches2d(const PolygonT& poly1, const PolygonT& poly2) {
  auto rotatedNext = [&](auto iter) { return iter + 1 == poly2.end() ? poly2.begin() : iter + 1; };
  for (auto i = 0u; i < poly1.numSegments(); ++i) {
    auto segment = poly1.segment(i);
    auto second = std::find(poly2.begin(), poly2.end(), segment.second);
    if (second != poly2.end() && *rotatedNext(second) == segment.first) {
      return true;
    }
  }
  return false;
}

template <typename Polygon2dT>
IfPoly<Polygon2dT, bool> overlaps2d(const Polygon2dT& poly1, const Polygon2dT& poly2) {
  if (touches2d(poly1, poly2)) {
    return false;
  }
#if BOOST_VERSION > 105800
  using Mask = boost::geometry::de9im::static_mask<'T', '*', '*', '*', '*', '*', '*', '*', '*'>;
  return boost::geometry::relate(utils::toHybrid(poly1), utils::toHybrid(poly2), Mask());
#else
  using Mask = boost::geometry::detail::relate::static_mask<'T', '*', '*', '*', '*', '*', '*', '*', '*'>;
  return boost::geometry::detail::relate::relate<Mask>(utils::toHybrid(poly1), utils::toHybrid(poly2));
#endif
}

template <typename Polygon3dT>
IfPoly<Polygon3dT, bool> overlaps3d(const Polygon3dT& poly1, const Polygon3dT& poly2, double heightTolerance) {
  if (!overlaps2d(poly1, poly2)) {
    return false;
  }
  auto points = projectedBorderPoint3d(poly1, poly2);
  return std::abs(points.first.z() - points.second.z()) < heightTolerance;
}
}  // namespace geometry
}  // namespace lanelet
