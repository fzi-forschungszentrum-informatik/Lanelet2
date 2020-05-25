#pragma once
#include "lanelet2_core/geometry/LineString.h"
#include "lanelet2_core/geometry/Point.h"
#include "lanelet2_core/primitives/Polygon.h"
#if BOOST_VERSION > 105800
#include <boost/geometry/algorithms/relate.hpp>
#else
#include <boost/geometry/algorithms/detail/relate/relate.hpp>
#endif

namespace lanelet {
namespace geometry {
namespace internal {
template <typename T>
struct GetGeometry<T, IfPoly<T, void>> {
  static inline auto twoD(const T& geometry) { return traits::toHybrid(traits::to2D(geometry)); }
  static inline auto threeD(const T& geometry) { return traits::toHybrid(traits::to3D(geometry)); }
};

std::pair<BasicPoint3d, BasicPoint3d> projectedBorderPoint3d(const ConstHybridPolygon3d& l1,
                                                             const ConstHybridPolygon3d& l2);

std::pair<BasicPoint3d, BasicPoint3d> projectedBorderPoint3d(const CompoundHybridPolygon3d& l1,
                                                             const CompoundHybridPolygon3d& l2);
BasicPolygons2d convexPartition(const BasicPolygon2d& poly);
IndexedTriangles triangulate(const BasicPolygon2d& poly);
}  // namespace internal

template <typename Polygon3dT>
IfPoly<Polygon3dT, std::pair<BasicPoint3d, BasicPoint3d>> projectedBorderPoint3d(const Polygon3dT& l1,
                                                                                 const Polygon3dT& l2) {
  static_assert(traits::is3D<Polygon3dT>(), "Please call this function with a 3D type!");
  return internal::projectedBorderPoint3d(traits::toHybrid(traits::to3D(l1)), traits::toHybrid(traits::to3D(l2)));
}

template <typename Polygon3dT>
IfPoly<Polygon3dT, double> distanceToBorder3d(const Polygon3dT& poly1, const Polygon3dT& poly2) {
  static_assert(traits::is3D<Polygon3dT>(), "Please call this function with a 3D type!");
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
  static_assert(traits::is2D<Polygon2dT>(), "Please call this function with a 2D type!");
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
  static_assert(traits::is3D<Polygon3dT>(), "Please call this function with a 3D type!");
  if (!overlaps2d(traits::to2D(poly1), traits::to2D(poly2))) {
    return false;
  }
  auto points = projectedBorderPoint3d(poly1, poly2);
  return std::abs(points.first.z() - points.second.z()) < heightTolerance;
}

template <typename Polygon2dT>
IfPoly<Polygon2dT, BasicPolygons2d> convexPartition(const Polygon2dT& poly) {
  static_assert(traits::is2D<Polygon2dT>(), "Please call this function with a 2D type!");
  return internal::convexPartition(BasicPolygon2d(poly.basicBegin(), poly.basicEnd()));
}

template <typename Polygon2dT>
IfPoly<Polygon2dT, IndexedTriangles> triangulate(const Polygon2dT& poly) {
  static_assert(traits::is2D<Polygon2dT>(), "Please call this function with a 2D type!");
  return internal::triangulate(BasicPolygon2d(poly.basicBegin(), poly.basicEnd()));
}

template <>
inline IfPoly<BasicPolygon2d, BasicPolygons2d> convexPartition<BasicPolygon2d>(const BasicPolygon2d& poly) {
  return internal::convexPartition(poly);
}

template <>
inline IfPoly<BasicPolygon2d, IndexedTriangles> triangulate<BasicPolygon2d>(const BasicPolygon2d& poly) {
  return internal::triangulate(poly);
}

}  // namespace geometry
}  // namespace lanelet
