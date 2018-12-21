#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/pointing_segment.hpp>
#include <boost/geometry/index/rtree.hpp>
#include "geometry/LineString.h"
#include "geometry/Polygon.h"

namespace lanelet {
namespace geometry {
namespace internal {
using V3d = BasicPoint3d;
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
  constexpr double SmallNum = 1.e-10;

  double sc, sN, sD = den;  // sc = sN / sD, default sD = D >= 0
  double tc, tN, tD = den;  // tc = tN / tD, default tD = D >= 0

  // compute the line parameters of the two closest points
  if (den < SmallNum) {  // the lines are almost parallel
    sN = 0.0;            // force using point P0 on segment S1
    sD = 1.0;            // to prevent possible division by 0.0 later
    tN = e;
    tD = c;
  } else {  // get the closest points on the infinite lines
    sN = (b * e - c * d);
    tN = (a * e - b * d);
    if (sN < 0.0) {  // sc < 0 => the s=0 edge is visible
      sN = 0.0;
      tN = e;
      tD = c;
    } else if (sN > sD) {  // sc > 1  => the s=1 edge is visible
      sN = sD;
      tN = e + b;
      tD = c;
    }
  }

  if (tN < 0.0) {  // tc < 0 => the t=0 edge is visible
    tN = 0.0;
    // recompute sc for this edge
    if (-d < 0.0) {
      sN = 0.0;
    } else if (-d > a) {
      sN = sD;
    } else {
      sN = -d;
      sD = a;
    }
  } else if (tN > tD) {  // tc > 1  => the t=1 edge is visible
    tN = tD;
    // recompute sc for this edge
    if ((-d + b) < 0.0) {
      sN = 0;
    } else if ((-d + b) > a) {
      sN = sD;
    } else {
      sN = (-d + b);
      sD = a;
    }
  }
  // finally do the division to get sc and tc
  sc = (std::abs(sN) < SmallNum ? 0.0 : sN / sD);
  tc = (std::abs(tN) < SmallNum ? 0.0 : tN / tD);

  return {p1 + (sc * u), q1 + (tc * v)};  // return the closest distance
}

namespace bg = boost::geometry;
namespace bgi = bg::index;
namespace bgm = boost::geometry::model;
using Segment = bgm::pointing_segment<const BasicPoint3d>;
using Box = bgm::box<bgm::point<double, 3, boost::geometry::cs::cartesian>>;
using Node = std::pair<Box, Segment>;
using RTree = bgi::rtree<Node, bgi::linear<8>>;

template <typename LineStringT>
std::pair<BasicPoint3d, BasicPoint3d> projectedPoint3dImpl(const LineStringT& l1, const LineStringT& l2) {
  bool swap = l1.size() < l2.size();
  auto* smallerRange = swap ? &l1 : &l2;
  auto* greaterRange = swap ? &l2 : &l1;

  // catch some degerated cases
  ConstHybridLineString3d duplicate;
  if (smallerRange->size() == 1 && greaterRange->size() == 1) {
    std::pair<BasicPoint3d, BasicPoint3d> ret(greaterRange->front(), smallerRange->front());
    if (swap) {
      std::swap(ret.first, ret.second);
    }
    return ret;
  }
  auto values =
      utils::transform(bg::segments_begin(*greaterRange), bg::segments_end(*greaterRange), [](const auto& segm) {
        Box box;
        boost::geometry::envelope(segm, box);
        return Node(box, segm);
      });
  RTree tree(values.begin(), values.end());

  bool first = true;
  double dMin{};
  std::pair<BasicPoint3d, BasicPoint3d> closestPair;
  for (auto it = bg::segments_begin(*smallerRange); it != bg::segments_end(*smallerRange); ++it) {
    Box queryBox;
    bg::envelope(*it, queryBox);
    for (auto qIt = tree.qbegin(bgi::nearest(queryBox, greaterRange->size())); qIt != tree.qend();
         ++qIt, first = false) {
      auto& nearest = *qIt;
      auto dBox = boost::geometry::distance(nearest.first, queryBox);
      if (!first && dBox > dMin) {
        break;
      }
      auto projPair = projectedPoint3d(*nearest.second.first, *nearest.second.second, *it->first, *it->second);
      if (swap) {
        std::swap(projPair.first, projPair.second);
      }
      auto d = (projPair.first - projPair.second).norm();
      if (first || d < dMin) {
        closestPair = projPair;
        dMin = d;
      }
    }
  }
  return closestPair;
}

std::pair<BasicPoint3d, BasicPoint3d> projectedPoint3d(const CompoundHybridLineString3d& l1,
                                                       const CompoundHybridLineString3d& l2) {
  return projectedPoint3dImpl(l1, l2);
}
std::pair<BasicPoint3d, BasicPoint3d> projectedPoint3d(const ConstHybridLineString3d& l1,
                                                       const ConstHybridLineString3d& l2) {
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
}  // namespace geometry
}  // namespace lanelet
