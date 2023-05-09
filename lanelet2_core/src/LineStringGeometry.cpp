#include <boost/version.hpp>
#include <cstddef>
#include <utility>

#if BOOST_VERSION < 106300 && BOOST_VERSION >= 106200
// Boost 1.62 is missing an iostream include...
#include <iostream>
#endif
#include <boost/geometry/algorithms/disjoint.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/pointing_segment.hpp>
#include <boost/geometry/index/rtree.hpp>

#include "lanelet2_core/Exceptions.h"
#include "lanelet2_core/geometry/LineString.h"
#include "lanelet2_core/geometry/Polygon.h"

namespace lanelet {
namespace geometry {
namespace {
struct LineParams {
  double sN;
  double sD;
  double tN;
  double tD;
};

constexpr double SmallNum = 1.e-10;
constexpr std::size_t RtreeThres = 50;

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
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

template <typename PointT>
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
std::pair<PointT, PointT> projectedPoint(PointT p1, PointT p2, PointT q1, PointT q2) {
  // see http://geomalgorithms.com/a07-_distance.html
  PointT w = p1 - q1;
  PointT u = p2 - p1;
  PointT v = q2 - q1;
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

template <typename PointT>
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
PointT projectedPoint(PointT l1, PointT l2, PointT p) {
  PointT v = l2 - l1;
  PointT w = p - l1;

  const auto c1 = w.dot(v);
  if (c1 <= 0) {
    return l1;
  }
  const auto c2 = v.dot(v);
  if (c2 <= c1) {
    return l2;
  }
  const auto b = c1 / c2;
  return v * b + l1;
}

template <typename RangeT, typename Func>
void distForAllSegments(const RangeT& r, Func f) {
  if (r.size() == 1) {
    f(r.front(), r.front());
  }
  for (auto first = r.begin(), second = r.begin() + 1; second != r.end(); ++first, ++second) {
    if (f(*first, *second) == 0.) {
      break;
    }
  }
}

namespace bg = boost::geometry;
namespace bgi = bg::index;
namespace bgm = boost::geometry::model;

template <typename Point1T, typename Point2T, typename BasicPointT>
struct ProjectedPoint2L2Result {
  using Point1 = Point1T;
  using Point2 = Point2T;
  using Segm1 = Segment<Point1T>;
  using Segm2 = Segment<Point2T>;
  using BasicPoint = BasicPointT;

  bool valid() const { return !!distance; }
  double update(Segm1 segm1, Segm2 segm2) {
    auto projPair = projectedPoint(utils::toBasicPoint(segm1.first), utils::toBasicPoint(segm1.second),
                                   utils::toBasicPoint(segm2.first), utils::toBasicPoint(segm2.second));
    const auto dNew = (projPair.first - projPair.second).norm();
    if (!distance || *distance > dNew) {
      distance = dNew;
      this->segm1 = segm1;
      this->segm2 = segm2;
      this->p1 = projPair.first;
      this->p2 = projPair.second;
    }
    return *distance;
  }
  std::pair<BasicPoint, BasicPoint> projectedPoints() const { return std::make_pair(p1, p2); }

  ProjectedPoint2L2Result swap() const { return {segm2, segm1, p2, p1, distance}; }

  Segm1 segm1;
  Segm2 segm2;
  BasicPoint p1;
  BasicPoint p2;
  Optional<double> distance;
};

template <typename LineString1T, typename LineString2T>
using ProjectedPointL2LOnLinestring = ProjectedPoint2L2Result<traits::ConstPointT<traits::PointType<LineString1T>>,
                                                              traits::ConstPointT<traits::PointType<LineString2T>>,
                                                              traits::BasicPointT<traits::PointType<LineString1T>>>;

template <typename PointT, typename BasicPointT>
struct ProjectedPointL2PResult {
  using Point = PointT;
  using Segm = Segment<PointT>;
  using BasicPoint = BasicPointT;

  bool valid() const { return !!distance; }
  double update(Segm segm, BasicPoint p) {
    auto projP = projectedPoint(utils::toBasicPoint(segm.first), utils::toBasicPoint(segm.second), p);
    const auto dNew = (projP - p).norm();
    if (!distance || *distance > dNew) {
      distance = dNew;
      this->segm = segm;
      this->p = projP;
    }
    return *distance;
  }
  Segment<Point> segment() const { return segm; }

  Segm segm{};
  BasicPoint p{};
  Optional<double> distance;
};

template <typename LineStringT>
using ProjectedPointL2POnLinestring = ProjectedPointL2PResult<traits::ConstPointT<traits::PointType<LineStringT>>,
                                                              traits::BasicPointT<traits::PointType<LineStringT>>>;

template <typename PointT>
auto toSegment(const bgm::pointing_segment<const PointT>& s) {
  return std::make_pair(*s.first, *s.second);
}

template <typename PointT>
auto toBasicSegment(const bgm::pointing_segment<const PointT>& s) {
  return std::make_pair(utils::toBasicPoint(*s.first), utils::toBasicPoint(*s.second));
}

template <typename LineString1T, typename LineString2T>
ProjectedPointL2LOnLinestring<LineString1T, LineString2T> projectedPointL2LWithTree(const LineString1T& smallerRange,
                                                                                    const LineString2T& greaterRange) {
  using TreePointT = traits::ConstPointT<traits::PointType<LineString2T>>;
  using TreeSegmT = Segment<TreePointT>;
  constexpr auto Dim = traits::PointTraits<TreePointT>::Dimension;
  using Box = bgm::box<bgm::point<double, static_cast<int>(Dim), boost::geometry::cs::cartesian>>;
  using Node = std::pair<Box, TreeSegmT>;
  using RTree = bgi::rtree<Node, bgi::linear<8>>;
  const auto values =
      utils::transform(bg::segments_begin(greaterRange), bg::segments_end(greaterRange), [](const auto& segm) {
        Box box;
        boost::geometry::envelope(segm, box);
        return Node(box, toSegment(segm));
      });
  RTree tree(values.begin(), values.end());

  using PointT = traits::ConstPointT<traits::PointType<LineString1T>>;
  auto result = ProjectedPointL2LOnLinestring<LineString1T, LineString2T>{};
  distForAllSegments(smallerRange, [&](auto&& psmall1, auto&& psmall2) {
    Box queryBox;
    auto segsmall = utils::toBasicSegment(std::make_pair(psmall1, psmall2));
    bg::envelope(segsmall, queryBox);
    for (auto qIt = tree.qbegin(bgi::nearest(queryBox, unsigned(tree.size()))); qIt != tree.qend(); ++qIt) {
      const auto& nearest = *qIt;
      auto dBox = boost::geometry::distance(nearest.first, queryBox);
      if (!!result.distance && *(result.distance) < dBox) {
        break;
      }
      result.update(std::make_pair(psmall1, psmall2), nearest.second);
    }
    return *result.distance;
  });
  return result;
}

template <typename LineString1T, typename LineString2T>
ProjectedPointL2LOnLinestring<LineString1T, LineString2T> projectedPointL2LBruteForce(
    const LineString1T& smallerRange, const LineString2T& greaterRange) {
  auto result = ProjectedPointL2LOnLinestring<LineString1T, LineString2T>{};
  distForAllSegments(smallerRange, [&](auto&& psmall1, auto&& psmall2) {
    distForAllSegments(greaterRange, [&](auto&& pgreat1, auto&& pgreat2) {
      return result.update(std::make_pair(psmall1, psmall2), std::make_pair(pgreat1, pgreat2));
    });
    return *result.distance;
  });
  return result;
}

template <typename LineStringT>
ProjectedPointL2POnLinestring<LineStringT> projectedPointL2PBruteForce(
    const LineStringT& ls, const traits::BasicPointT<traits::PointType<LineStringT>>& point) {
  auto result = ProjectedPointL2POnLinestring<LineStringT>{};
  distForAllSegments(ls, [&](auto&& pl1, auto&& pl2) { return result.update(std::make_pair(pl1, pl2), point); });
  return result;
}

template <typename LineStringT>
ProjectedPointL2POnLinestring<LineStringT> projectedPointL2PWithTree(
    const LineStringT& ls, const traits::BasicPointT<traits::PointType<LineStringT>>& p) {
  using TreePointT = traits::ConstPointT<traits::PointType<LineStringT>>;
  using TreeSegmT = Segment<TreePointT>;
  constexpr auto Dim = traits::PointTraits<TreePointT>::Dimension;
  using Box = bgm::box<bgm::point<double, static_cast<int>(Dim), boost::geometry::cs::cartesian>>;
  using Node = std::pair<Box, TreeSegmT>;
  using RTree = bgi::rtree<Node, bgi::linear<8>>;

  const auto values = utils::transform(bg::segments_begin(ls), bg::segments_end(ls), [](const auto& segm) {
    Box box;
    boost::geometry::envelope(toBasicSegment(segm), box);
    return Node(box, toSegment(segm));
  });
  RTree tree(values.begin(), values.end());

  auto result = ProjectedPointL2POnLinestring<LineStringT>{};
  for (auto qIt = tree.qbegin(bgi::nearest(p, unsigned(tree.size()))); qIt != tree.qend(); ++qIt) {
    const auto& nearest = *qIt;
    auto dBox = boost::geometry::distance(nearest.first, p);
    if (!!result.distance && *(result.distance) < dBox) {
      break;
    }
    if (result.update(nearest.second, p) == 0.) {
      break;
    }
  }
  return result;
}

template <typename LineString1T, typename LineString2T>
auto projectedPointOrdered(const LineString1T& smallerRange, const LineString2T& greaterRange) {
  if (smallerRange.size() == 0) {
    throw InvalidInputError("ProjectedPoint called with empty linestring as input!");
  }
  if (greaterRange.size() < RtreeThres) {
    return projectedPointL2LBruteForce(smallerRange, greaterRange);
  }
  return projectedPointL2LWithTree(smallerRange, greaterRange);
}

template <typename LineString1T, typename LineString2T>
auto projectedPointL2LImpl(const LineString1T& l1, const LineString2T& l2) {
  if (l1.size() < l2.size()) {
    return projectedPointOrdered(l1, l2);
  }
  return projectedPointOrdered(l2, l1).swap();
}

template <typename LineStringT>
auto projectedPointL2PImpl(const LineStringT& ls, const traits::BasicPointT<traits::PointType<LineStringT>>& p) {
  if (ls.size() < RtreeThres) {
    return projectedPointL2PBruteForce(ls, p);
  }
  return projectedPointL2PWithTree(ls, p);
}
}  // namespace

namespace internal {
// 2d
std::pair<BasicPoint2d, BasicPoint2d> projectedPoint2d(const CompoundHybridLineString2d& l1,
                                                       const CompoundHybridLineString2d& l2) {
  return projectedPointL2LImpl(l1, l2).projectedPoints();
}

std::pair<BasicPoint2d, BasicPoint2d> projectedPoint2d(const ConstHybridLineString2d& l1,
                                                       const CompoundHybridLineString2d& l2) {
  return projectedPointL2LImpl(l1, l2).projectedPoints();
}

std::pair<BasicPoint2d, BasicPoint2d> projectedPoint2d(const CompoundHybridLineString2d& l1,
                                                       const ConstHybridLineString2d& l2) {
  return projectedPointL2LImpl(l1, l2).projectedPoints();
}

std::pair<BasicPoint2d, BasicPoint2d> projectedPoint2d(const ConstHybridLineString2d& l1,
                                                       const ConstHybridLineString2d& l2) {
  return projectedPointL2LImpl(l1, l2).projectedPoints();
}

std::pair<BasicPoint2d, BasicPoint2d> projectedPoint2d(const ConstHybridLineString2d& l1, const BasicLineString2d& l2) {
  return projectedPointL2LImpl(l1, l2).projectedPoints();
}

std::pair<BasicPoint2d, BasicPoint2d> projectedPoint2d(const BasicLineString2d& l1, const ConstHybridLineString2d& l2) {
  return projectedPointL2LImpl(l1, l2).projectedPoints();
}

std::pair<BasicPoint2d, BasicPoint2d> projectedPoint2d(const BasicLineString2d& l1, const BasicLineString2d& l2) {
  return projectedPointL2LImpl(l1, l2).projectedPoints();
}

std::pair<BasicPoint2d, BasicPoint2d> projectedBorderPoint2d(const ConstHybridPolygon2d& l1,
                                                             const ConstHybridPolygon2d& l2) {
  return projectedPointL2LImpl(l1, l2).projectedPoints();
}

std::pair<BasicPoint2d, BasicPoint2d> projectedBorderPoint2d(const CompoundHybridPolygon2d& l1,
                                                             const CompoundHybridPolygon2d& l2) {
  return projectedPointL2LImpl(l1, l2).projectedPoints();
}

// 3d
std::pair<BasicPoint3d, BasicPoint3d> projectedPoint3d(const CompoundHybridLineString3d& l1,
                                                       const CompoundHybridLineString3d& l2) {
  return projectedPointL2LImpl(l1, l2).projectedPoints();
}

std::pair<BasicPoint3d, BasicPoint3d> projectedPoint3d(const ConstHybridLineString3d& l1,
                                                       const CompoundHybridLineString3d& l2) {
  return projectedPointL2LImpl(l1, l2).projectedPoints();
}

std::pair<BasicPoint3d, BasicPoint3d> projectedPoint3d(const CompoundHybridLineString3d& l1,
                                                       const ConstHybridLineString3d& l2) {
  return projectedPointL2LImpl(l1, l2).projectedPoints();
}

std::pair<BasicPoint3d, BasicPoint3d> projectedPoint3d(const ConstHybridLineString3d& l1,
                                                       const ConstHybridLineString3d& l2) {
  return projectedPointL2LImpl(l1, l2).projectedPoints();
}

std::pair<BasicPoint3d, BasicPoint3d> projectedPoint3d(const ConstHybridLineString3d& l1, const BasicLineString3d& l2) {
  return projectedPointL2LImpl(l1, l2).projectedPoints();
}

std::pair<BasicPoint3d, BasicPoint3d> projectedPoint3d(const BasicLineString3d& l1, const ConstHybridLineString3d& l2) {
  return projectedPointL2LImpl(l1, l2).projectedPoints();
}

std::pair<BasicPoint3d, BasicPoint3d> projectedPoint3d(const BasicLineString3d& l1, const BasicLineString3d& l2) {
  return projectedPointL2LImpl(l1, l2).projectedPoints();
}

std::pair<BasicPoint3d, BasicPoint3d> projectedBorderPoint3d(const ConstHybridPolygon3d& l1,
                                                             const ConstHybridPolygon3d& l2) {
  return projectedPointL2LImpl(l1, l2).projectedPoints();
}

std::pair<BasicPoint3d, BasicPoint3d> projectedBorderPoint3d(const CompoundHybridPolygon3d& l1,
                                                             const CompoundHybridPolygon3d& l2) {
  return projectedPointL2LImpl(l1, l2).projectedPoints();
}

// project
BasicPoint2d project(const BasicLineString2d& lineString, const BasicPoint2d& pointToProject) {
  return projectedPointL2PImpl(lineString, pointToProject).p;
}
BasicPoint3d project(const BasicLineString3d& lineString, const BasicPoint3d& pointToProject) {
  return projectedPointL2PImpl(lineString, pointToProject).p;
}

BasicPoint2d project(const ConstHybridLineString2d& lineString, const BasicPoint2d& pointToProject) {
  return projectedPointL2PImpl(lineString, pointToProject).p;
}
BasicPoint3d project(const ConstHybridLineString3d& lineString, const BasicPoint3d& pointToProject) {
  return projectedPointL2PImpl(lineString, pointToProject).p;
}

BasicPoint2d project(const CompoundHybridLineString2d& lineString, const BasicPoint2d& pointToProject) {
  return projectedPointL2PImpl(lineString, pointToProject).p;
}
BasicPoint3d project(const CompoundHybridLineString3d& lineString, const BasicPoint3d& pointToProject) {
  return projectedPointL2PImpl(lineString, pointToProject).p;
}

}  // namespace internal

// closestsegment
Segment<BasicPoint2d> closestSegment(const BasicLineString2d& lineString, const BasicPoint2d& pointToProject) {
  return projectedPointL2PImpl(lineString, pointToProject).segment();
}
Segment<BasicPoint3d> closestSegment(const BasicLineString3d& lineString, const BasicPoint3d& pointToProject) {
  return projectedPointL2PImpl(lineString, pointToProject).segment();
}

Segment<ConstPoint2d> closestSegment(const ConstLineString2d& lineString, const BasicPoint2d& pointToProject) {
  return projectedPointL2PImpl(lineString, pointToProject).segment();
}
Segment<ConstPoint3d> closestSegment(const ConstLineString3d& lineString, const BasicPoint3d& pointToProject) {
  return projectedPointL2PImpl(lineString, pointToProject).segment();
}

Segment<BasicPoint2d> closestSegment(const ConstHybridLineString2d& lineString, const BasicPoint2d& pointToProject) {
  return projectedPointL2PImpl(lineString, pointToProject).segment();
}
Segment<BasicPoint3d> closestSegment(const ConstHybridLineString3d& lineString, const BasicPoint3d& pointToProject) {
  return projectedPointL2PImpl(lineString, pointToProject).segment();
}

Segment<ConstPoint2d> closestSegment(const CompoundLineString2d& lineString, const BasicPoint2d& pointToProject) {
  return projectedPointL2PImpl(lineString, pointToProject).segment();
}
Segment<ConstPoint3d> closestSegment(const CompoundLineString3d& lineString, const BasicPoint3d& pointToProject) {
  return projectedPointL2PImpl(lineString, pointToProject).segment();
}

Segment<BasicPoint2d> closestSegment(const CompoundHybridLineString2d& lineString, const BasicPoint2d& pointToProject) {
  return projectedPointL2PImpl(lineString, pointToProject).segment();
}
Segment<BasicPoint3d> closestSegment(const CompoundHybridLineString3d& lineString, const BasicPoint3d& pointToProject) {
  return projectedPointL2PImpl(lineString, pointToProject).segment();
}

BasicPoint2d project(const BasicSegment2d& segment, const BasicPoint2d& pointToProject) {
  return projectedPoint(segment.first, segment.second, pointToProject);
}
BasicPoint3d project(const BasicSegment3d& segment, const BasicPoint3d& pointToProject) {
  return projectedPoint(segment.first, segment.second, pointToProject);
}
}  // namespace geometry
}  // namespace lanelet
