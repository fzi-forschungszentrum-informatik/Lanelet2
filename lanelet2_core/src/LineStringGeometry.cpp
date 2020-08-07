#include <boost/version.hpp>
#if BOOST_VERSION < 106300 && BOOST_VERSION >= 106200
// Boost 1.62 is missing an iostream include...
#include <iostream>
#endif
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/pointing_segment.hpp>
#include <boost/geometry/index/rtree.hpp>

#include "lanelet2_core/geometry/LineString.h"
#include "lanelet2_core/geometry/Polygon.h"

namespace lanelet {
namespace geometry {
namespace {
using V3d = BasicPoint3d;

struct LineParams {
  double sN;
  double sD;
  double tN;
  double tD;
};

constexpr double SmallNum = 1.e-10;

inline LineParams calculateLineParams(double a, double b, double c, double d, double e, double den) {
  // compute the line parameters of the two closest points
  if (den < SmallNum) {  // the lines are almost parallel
    // force using point P0 on segment S1
    // to prevent possible division by 0.0 later
    return {0.0, 1.0, e, c};
  }
  LineParams lp{};
  lp.sD = den;
  lp.tD = den;
  // get the closest points on the infinite lines
  lp.sN = (b * e - c * d);
  lp.tN = (a * e - b * d);
  if (lp.sN < 0.0) {  // sc < 0 => the s=0 edge is visible
    lp.sN = 0.0;
    lp.tN = e;
    lp.tD = c;
  } else if (lp.sN > lp.sD) {  // sc > 1  => the s=1 edge is visible
    lp.sN = lp.sD;
    lp.tN = e + b;
    lp.tD = c;
  }

  return lp;
}

std::pair<BasicPoint3d, BasicPoint3d> projectedPoint3d(const BasicPoint3d& p1, const BasicPoint3d& p2,
                                                       const BasicPoint3d& q1, const BasicPoint3d& q2) {
  // see http://geomalgorithms.com/a07-_distance.html
  V3d w = p1 - q1;
  V3d u = p2 - p1;
  V3d v = q2 - q1;
  double a = u.dot(u);
  double b = u.dot(v);
  double c = v.dot(v);
  double d = u.dot(w);
  double e = v.dot(w);
  auto den = a * c - b * b;

  auto lp = calculateLineParams(a, b, c, d, e, den);

  if (lp.tN < 0.0) {  // tc < 0 => the t=0 edge is visible
    lp.tN = 0.0;
    // recompute sc for this edge
    if (-d < 0.0) {
      lp.sN = 0.0;
    } else if (-d > a) {
      lp.sN = lp.sD;
    } else {
      lp.sN = -d;
      lp.sD = a;
    }
  } else if (lp.tN > lp.tD) {  // tc > 1  => the t=1 edge is visible
    lp.tN = lp.tD;
    // recompute sc for this edge
    if ((-d + b) < 0.0) {
      lp.sN = 0;
    } else if ((-d + b) > a) {
      lp.sN = lp.sD;
    } else {
      lp.sN = (-d + b);
      lp.sD = a;
    }
  }
  // finally do the division to get sc and tc
  double sc = (std::abs(lp.sN) < SmallNum ? 0.0 : lp.sN / lp.sD);
  double tc = (std::abs(lp.tN) < SmallNum ? 0.0 : lp.tN / lp.tD);

  return {p1 + (sc * u), q1 + (tc * v)};  // return the closest distance
}

namespace bg = boost::geometry;
namespace bgi = bg::index;
namespace bgm = boost::geometry::model;
using BasicSegment = bgm::pointing_segment<const BasicPoint3d>;
using Box = bgm::box<bgm::point<double, 3, boost::geometry::cs::cartesian>>;
using Node = std::pair<Box, BasicSegment>;
using RTree = bgi::rtree<Node, bgi::linear<8>>;

template <typename LineString1T, typename LineString2T>
std::pair<BasicPoint3d, BasicPoint3d> projectedPoint3dOrdered(const LineString1T& smallerRange,
                                                              const LineString2T& greaterRange) {
  // catch some degerated cases
  ConstHybridLineString3d duplicate;
  if (smallerRange.size() == 1 && greaterRange.size() == 1) {
    std::pair<BasicPoint3d, BasicPoint3d> ret(greaterRange.front(), smallerRange.front());
    return ret;
  }
  auto values =
      utils::transform(bg::segments_begin(greaterRange), bg::segments_end(greaterRange), [](const auto& segm) {
        Box box;
        boost::geometry::envelope(segm, box);
        return Node(box, segm);
      });
  RTree tree(values.begin(), values.end());

  bool first = true;
  double dMin{};
  std::pair<BasicPoint3d, BasicPoint3d> closestPair;
  for (auto it = bg::segments_begin(smallerRange); it != bg::segments_end(smallerRange); ++it) {
    Box queryBox;
    bg::envelope(*it, queryBox);
    for (auto qIt = tree.qbegin(bgi::nearest(queryBox, unsigned(greaterRange.size()))); qIt != tree.qend();
         ++qIt, first = false) {
      const auto& nearest = *qIt;
      auto dBox = boost::geometry::distance(nearest.first, queryBox);
      if (!first && dBox > dMin) {
        break;
      }
      auto projPair = projectedPoint3d(*nearest.second.first, *nearest.second.second, *it->first, *it->second);
      auto d = (projPair.first - projPair.second).norm();
      if (first || d < dMin) {
        closestPair = projPair;
        dMin = d;
      }
    }
  }
  return closestPair;
}

template <typename LineString1T, typename LineString2T>
std::pair<BasicPoint3d, BasicPoint3d> projectedPoint3dImpl(const LineString1T& l1, const LineString2T& l2) {
  if (l1.size() < l2.size()) {
    return projectedPoint3dOrdered(l1, l2);
  }
  auto res = projectedPoint3dOrdered(l2, l1);
  return {res.second, res.first};
}
}  // namespace

namespace internal {
std::pair<BasicPoint3d, BasicPoint3d> projectedPoint3d(const CompoundHybridLineString3d& l1,
                                                       const CompoundHybridLineString3d& l2) {
  return projectedPoint3dImpl(l1, l2);
}

std::pair<BasicPoint3d, BasicPoint3d> projectedPoint3d(const ConstHybridLineString3d& l1,
                                                       const CompoundHybridLineString3d& l2) {
  return projectedPoint3dImpl(l1, l2);
}

std::pair<BasicPoint3d, BasicPoint3d> projectedPoint3d(const CompoundHybridLineString3d& l1,
                                                       const ConstHybridLineString3d& l2) {
  return projectedPoint3dImpl(l1, l2);
}

std::pair<BasicPoint3d, BasicPoint3d> projectedPoint3d(const ConstHybridLineString3d& l1,
                                                       const ConstHybridLineString3d& l2) {
  return projectedPoint3dImpl(l1, l2);
}

std::pair<BasicPoint3d, BasicPoint3d> projectedPoint3d(const ConstHybridLineString3d& l1, const BasicLineString3d& l2) {
  return projectedPoint3dImpl(l1, l2);
}

std::pair<BasicPoint3d, BasicPoint3d> projectedPoint3d(const BasicLineString3d& l1, const ConstHybridLineString3d& l2) {
  return projectedPoint3dImpl(l1, l2);
}

std::pair<BasicPoint3d, BasicPoint3d> projectedPoint3d(const BasicLineString3d& l1, const BasicLineString3d& l2) {
  return projectedPoint3dImpl(l1, l2);
}

std::pair<BasicPoint3d, BasicPoint3d> projectedBorderPoint3d(const ConstHybridPolygon3d& l1,
                                                             const ConstHybridPolygon3d& l2) {
  return projectedPoint3dImpl(l1, l2);
}

std::pair<BasicPoint3d, BasicPoint3d> projectedBorderPoint3d(const CompoundHybridPolygon3d& l1,
                                                             const CompoundHybridPolygon3d& l2) {
  return projectedPoint3dImpl(l1, l2);  // NOLINT
}

}  // namespace internal

Segment<BasicPoint2d> closestSegment(const BasicLineString2d& lineString, const BasicPoint2d& pointToProject) {
  helper::ProjectedPoint<BasicPoint2d> projectedPoint;
  distance(lineString, pointToProject, projectedPoint);
  return Segment<BasicPoint2d>(projectedPoint.result->segmentPoint1, projectedPoint.result->segmentPoint2);
}
Segment<BasicPoint3d> closestSegment(const BasicLineString3d& lineString, const BasicPoint3d& pointToProject) {
  helper::ProjectedPoint<BasicPoint3d> projectedPoint;
  distance(lineString, pointToProject, projectedPoint);
  return Segment<BasicPoint3d>(projectedPoint.result->segmentPoint1, projectedPoint.result->segmentPoint2);
}
}  // namespace geometry
}  // namespace lanelet
