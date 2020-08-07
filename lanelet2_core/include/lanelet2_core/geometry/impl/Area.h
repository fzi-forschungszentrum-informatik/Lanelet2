#pragma once
#include <lanelet2_core/geometry/Area.h>

#include "lanelet2_core/geometry/Point.h"
#include "lanelet2_core/geometry/Polygon.h"
#include "lanelet2_core/primitives/Area.h"
#include "lanelet2_core/primitives/Lanelet.h"

namespace lanelet {
namespace geometry {
namespace internal {
template <typename T>
struct GetGeometry<T, IfAr<T, void>> {
  static inline auto twoD(const T& geometry) { return geometry.basicPolygonWithHoles2d(); }
  static inline auto threeD(const T& geometry) { return geometry.basicPolygonWithHoles3d(); }
};
}  // namespace internal

template <typename AreaT>
IfAr<AreaT, bool> inside(const AreaT& area, const BasicPoint2d& point) {
  return boost::geometry::covered_by(point, area.basicPolygonWithHoles2d());
}

template <typename AreaT>
IfAr<AreaT, BoundingBox2d> boundingBox2d(const AreaT& area) {
  return boundingBox2d(traits::to2D(area.outerBoundPolygon()));
}

template <typename AreaT>
IfAr<AreaT, BoundingBox3d> boundingBox3d(const AreaT& area) {
  return boundingBox3d(area.outerBoundPolygon());
}

template <typename Area1T, typename Area2T>
IfAr<Area1T, bool> intersects2d(const Area1T& area, const Area2T& otherArea) {
  if (area == otherArea) {
    return true;
  }
  return intersects(area.basicPolygonWithHoles2d(), otherArea.basicPolygonWithHoles2d());
}

template <typename AreaT>
IfAr<AreaT, bool> overlaps2d(const AreaT& area, const AreaT& otherArea) {
  return overlaps2d(traits::to2D(area.outerBoundPolygon()), traits::to2D(otherArea.outerBoundPolygon()));
}

template <typename AreaT>
IfAr<AreaT, bool> overlaps3d(const AreaT& area, const AreaT& otherArea, double heightTolerance) {
  return overlaps3d(area.outerBoundPolygon(), otherArea.outerBoundPolygon(), heightTolerance);
}

template <typename AreaT, typename LaneletT>
IfAr<AreaT, IfLL<LaneletT, bool>> overlaps2d(const AreaT& area, const LaneletT& lanelet) {
  return overlaps2d(utils::to2D(area.outerBoundPolygon()), lanelet.polygon2d());
}

template <typename AreaT, typename LaneletT>
IfAr<AreaT, IfLL<LaneletT, bool>> overlaps3d(const AreaT& area, const LaneletT& lanelet, double heightTolerance) {
  return overlaps3d(area.outerBoundPolygon(), lanelet.polygon3d(), heightTolerance);
}

inline bool leftOf(const ConstLanelet& right, const ConstArea& left) {
  return utils::anyOf(left.outerBound(), [&right](auto& bound) { return bound.invert() == right.leftBound(); });
}

inline bool rightOf(const ConstLanelet& left, const ConstArea& area) { return leftOf(left.invert(), area); }

inline bool follows(const ConstLanelet& prev, const ConstArea& next) {
  auto outer = next.outerBound();
  return utils::anyOf(outer,
                      [p1 = prev.leftBound().back(), p2 = prev.rightBound().back()](const ConstLineString3d& ls) {
                        return (ls.front() == p1 && ls.back() == p2) || (ls.front() == p2 && ls.back() == p1);
                      });
}

inline bool follows(const ConstArea& prev, const ConstLanelet& next) { return follows(next.invert(), prev); }

inline bool adjacent(const ConstArea& area1, const ConstArea& area2) {
  auto outer1 = area1.outerBoundPolygon();
  auto outer2 = area2.outerBoundPolygon();
  auto rotatedNext = [&ls = outer2](auto iter) { return iter + 1 == ls.end() ? ls.begin() : iter + 1; };
  for (auto i = 0u; i < outer1.numSegments(); ++i) {
    auto segment = outer1.segment(i);
    auto second = std::find(outer2.begin(), outer2.end(), segment.second);
    if (second != outer2.end() && *rotatedNext(second) == segment.first) {
      return true;
    }
  }
  return false;
}

inline Optional<ConstLineString3d> determineCommonLinePreceding(const ConstLanelet& ll, const ConstArea& ar) {
  return utils::findIf(ar.outerBound(), [p1 = ll.leftBound().back(), p2 = ll.rightBound().back()](auto& boundLs) {
    return (boundLs.back() == p1 && boundLs.front() == p2);
  });
}

inline Optional<ConstLineString3d> determineCommonLineFollowing(const ConstArea& ar, const ConstLanelet& ll) {
  return determineCommonLinePreceding(ll.invert(), ar);
}

inline Optional<ConstLineString3d> determineCommonLineFollowingOrPreceding(const ConstArea& ar,
                                                                           const ConstLanelet& ll) {
  return utils::findIf(ar.outerBound(), [p1 = ll.leftBound().front(), p2 = ll.rightBound().front(),
                                         p3 = ll.leftBound().back(), p4 = ll.rightBound().back()](auto& boundLs) {
    return ((boundLs.back() == p2 && boundLs.front() == p1) || (boundLs.back() == p3 && boundLs.front() == p4));
  });
}

inline Optional<ConstLineString3d> determineCommonLineSideways(const ConstLanelet& ll, const ConstArea& ar) {
  auto res = determineCommonLineLeft(ll, ar);
  return res ? res : determineCommonLineRight(ll, ar);
}

inline Optional<ConstLineString3d> determineCommonLineSideways(const ConstArea& ar, const ConstLanelet& ll) {
  return utils::findIf(ar.outerBound(), [lb = ll.leftBound(), rb = ll.rightBound()](auto& boundLs) {
    return boundLs == lb.invert() || boundLs == rb;
  });
}

inline Optional<ConstLineString3d> determineCommonLine(const ConstArea& ar1, const ConstArea& ar2) {
  return utils::findIf(ar1.outerBound(), [&ar2](auto& ar1Bound) {
    return !!utils::findIf(ar2.outerBound(),
                           [ar1Bound = ar1Bound.invert()](auto& ar2Bound) { return ar2Bound == ar1Bound; });
  });
}

inline Optional<ConstLineString3d> determineCommonLine(const ConstArea& ar, const ConstLanelet& ll) {
  auto res = determineCommonLineFollowingOrPreceding(ar, ll);
  return res ? res : determineCommonLineSideways(ar, ll);
}

inline Optional<ConstLineString3d> determineCommonLineLeft(const ConstLanelet& right, const ConstArea& left) {
  auto res = determineCommonLineRight(right.invert(), left);
  if (res) {
    return res->invert();
  }
  return {};
}

inline Optional<ConstLineString3d> determineCommonLineRight(const ConstLanelet& left, const ConstArea& right) {
  return utils::findIf(right.outerBound(), [&left](auto& bound) { return bound == left.rightBound(); });
}

}  // namespace geometry
}  // namespace lanelet
