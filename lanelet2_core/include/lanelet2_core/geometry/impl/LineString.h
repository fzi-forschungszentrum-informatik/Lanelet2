#pragma once
#include <boost/geometry/algorithms/intersection.hpp>

#include "lanelet2_core/geometry/GeometryHelper.h"
#include "lanelet2_core/geometry/Point.h"
#include "lanelet2_core/primitives/LineString.h"
#include "lanelet2_core/primitives/Traits.h"

namespace lanelet {
namespace geometry {
namespace internal {

struct SelfIntersectionLong {
  size_t idx;
  double s;  // coordinate along linestring
};

struct SelfIntersection2d {
  SelfIntersectionLong firstSegment;
  SelfIntersectionLong lastSegment;
  BasicPoint2d intersectionPoint;
};
using SelfIntersections2d = std::vector<SelfIntersection2d>;

struct PointVincinity {
  const BasicPoint2d preceding;
  const BasicPoint2d following;
};

template <typename T>
struct GetGeometry<T, IfLS<T, void>> {
  static inline auto twoD(const T& geometry) { return traits::toHybrid(traits::to2D(geometry)); }
  static inline auto threeD(const T& geometry) { return traits::toHybrid(traits::to3D(geometry)); }
};

inline auto crossProd(const BasicPoint3d& p1, const BasicPoint3d& p2) { return p1.cross(p2).eval(); }
inline auto crossProd(const BasicPoint2d& p1, const BasicPoint2d& p2) {
  return BasicPoint3d(p1.x(), p1.y(), 0.).cross(BasicPoint3d(p2.x(), p2.y(), 0.)).eval();
}
// required for Polygon triangulation
inline auto crossProd(const Eigen::Matrix<double, 2, 1>& p1, const Eigen::Matrix<double, 2, 1>& p2) {
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

std::pair<BasicPoint3d, BasicPoint3d> projectedPoint3d(const ConstHybridLineString3d& l1,
                                                       const CompoundHybridLineString3d& l2);

std::pair<BasicPoint3d, BasicPoint3d> projectedPoint3d(const CompoundHybridLineString3d& l1,
                                                       const ConstHybridLineString3d& l2);

std::pair<BasicPoint3d, BasicPoint3d> projectedPoint3d(const ConstHybridLineString3d& l1, const BasicLineString3d& l2);

std::pair<BasicPoint3d, BasicPoint3d> projectedPoint3d(const BasicLineString3d& l1, const ConstHybridLineString3d& l2);

std::pair<BasicPoint3d, BasicPoint3d> projectedPoint3d(const BasicLineString3d& l1, const BasicLineString3d& l2);

template <typename HybridLineStringT>
BasicPoint2d fromArcCoords(const HybridLineStringT& hLineString, const BasicPoint2d& projStart, const size_t startIdx,
                           const size_t endIdx, const double distance) {
  if (hLineString.size() < startIdx) {
    throw InvalidInputError(std::string("Linestring point out of bounds. Linestring size ") +
                            std::to_string(hLineString.size()) + ", startIdx " + std::to_string(startIdx));
  }
  if (hLineString.size() < endIdx) {
    throw InvalidInputError(std::string("Linestring point out of bounds. Linestring size ") +
                            std::to_string(hLineString.size()) + ", endIdx " + std::to_string(endIdx));
  }
  if (startIdx == endIdx) {
    throw InvalidInputError(
        std::string("Can't determine shift direction from two identical points on linestring. Point index ") +
        std::to_string(startIdx));
  }
  const auto dx(hLineString[endIdx](0) - hLineString[startIdx](0));
  const auto dy(hLineString[endIdx](1) - hLineString[startIdx](1));

  return projStart + Eigen::Vector2d(-dy, dx).normalized() * distance;
}

/**
 * @brief makeVincinity create pair of preceding and following point on a line string
 * @param lineString line string to evaluate
 * @param idx index of point to create vincininty around
 * @return pair of preceding and following point (zero if not applicable)
 */
template <typename LineString2dT>
PointVincinity makeVincinity(const LineString2dT& lineString, const size_t idx) {
  if (idx == 0) {
    return PointVincinity{BasicPoint2d::Zero(),
                          utils::toBasicPoint(lineString[idx + 1]) - utils::toBasicPoint(lineString[idx])};
  }
  if (idx + 1 == lineString.size()) {
    return PointVincinity{utils::toBasicPoint(lineString[idx]) - utils::toBasicPoint(lineString[idx - 1]),
                          BasicPoint2d::Zero()};
  }
  return PointVincinity{utils::toBasicPoint(lineString[idx]) - utils::toBasicPoint(lineString[idx - 1]),
                        utils::toBasicPoint(lineString[idx + 1]) - utils::toBasicPoint(lineString[idx])};
}

/**
 * @brief shiftLateral shift point along the bisectrix
 * @param lineString original line string
 * @param idx index of point to shft
 * @param offset offset distance (left is positive) to shift
 * @param pv following and preveding point on line string
 * @return shifted point
 */
template <typename LineString2dT>
BasicPoint2d shiftLateral(const LineString2dT& lineString, const size_t idx, const double offset,
                          const PointVincinity& pv) {
  Eigen::Vector2d perpendicular;
  double realOffset = offset;
  const auto epsilon{1.e-5};
  if (idx == 0) {
    perpendicular = pv.following;
  } else if (idx + 1 == lineString.size()) {
    perpendicular = pv.preceding;
  } else {
    perpendicular = pv.following.normalized() + pv.preceding.normalized();
    auto minussin2 = perpendicular.norm() / 2;
    realOffset = (minussin2 > epsilon) ? offset / minussin2 : 0;
  }

  Eigen::Vector2d direction(-perpendicular(1), perpendicular(0));
  return utils::toBasicPoint(lineString[idx]) + direction.normalized() * realOffset;
}

/**
 * @return Point and true if convex
 */
template <typename LineString2dT>
bool isConvex(const LineString2dT& lineString, const size_t idx, const bool offsetPositive) {
  if (idx != 0 && idx + 1 != lineString.size()) {
    auto isLeft = pointIsLeftOf<BasicPoint2d>(lineString[idx - 1], lineString[idx], lineString[idx + 1]);
    return offsetPositive ? !isLeft : isLeft;
  }
  return true;
}

enum class Convexity { Concave, Convex, ConvexSharp };

template <typename LineString2dT>
Convexity getConvexity(const LineString2dT& lineString, const size_t idx, const PointVincinity& pv,
                       const bool offsetPositive) {
  if (!isConvex(lineString, idx, offsetPositive)) {
    return Convexity::Concave;
  }
  if (pv.following.dot(pv.preceding) < 0) {
    return Convexity::ConvexSharp;
  }
  return Convexity::Convex;
}

template <typename LineStringT>
BasicPoints2d sortAlongS(const LineStringT& ls, const BasicPoints2d& points) {
  auto idxs = sortAlongSIdxs(ls, points);
  BasicPoints2d ret;
  ret.reserve(idxs.size());
  for (const auto& i : idxs) {
    ret.emplace_back(points.at(i));
  }
  return ret;
}

/**
 * @brief create a point by moving distance laterally from the linestring at the point idx. The direction is the
 * average of the directions orthogonal to the segments neighbouring the point.
 * @param lineString lineString to operate on
 * @param idx index of point to shift (negative means counting from back)
 * @param distance to shift (left = positive)
 */
template <typename LineString2dT>
BasicPoint2d lateralShiftPointAtIndex(const LineString2dT& lineString, const int idx, const double distance) {
  if (std::abs(idx) + 1 > lineString.size()) {
    throw InvalidInputError("Index out of bounds");
  }
  int startIdx = (idx >= 0) ? std::max(0, idx - 1) : std::max(0, static_cast<int>(lineString.size()) + idx - 1);
  int endIdx = (idx >= 0)
                   ? std::min(idx + 1, static_cast<int>(lineString.size()) - 1)
                   : std::min(static_cast<int>(lineString.size()) + idx + 1, static_cast<int>(lineString.size()) - 1);
  int pIdx = (idx >= 0) ? idx : static_cast<int>(lineString.size()) + idx;
  auto hLineString = utils::toHybrid(lineString);
  return internal::fromArcCoords(hLineString, hLineString[pIdx], startIdx, endIdx, distance);
}

namespace bgi = boost::geometry::index;
using IndexedSegment2d = std::pair<BasicSegment2d, size_t>;
using IndexedSegmentTree = bgi::rtree<IndexedSegment2d, bgi::linear<4>>;
using SegmentTree = bgi::rtree<BasicSegment2d, bgi::linear<4>>;

struct LineStringsCoordinate {
  const size_t lineStringIdx;
  const size_t segmentIdx;
};

/**
 * @brief makeIndexedSegmenTree create search tree of segments
 * @param lineString line string that the segments will be created from
 * @throws InvalidInputError if the line string size is below 2
 * @return a search tree
 */
template <typename LineString2dT>
inline SegmentTree makeSegmentTree(const LineString2dT& lineString) {
  if (lineString.size() < 2) {
    throw InvalidInputError("Need line string size of at least 2 to make tree");
  }
  SegmentTree tree;
  std::vector<BasicSegment2d> segContainer;
  segContainer.reserve(lineString.size() - 1);
  for (size_t i = 0; i < lineString.size() - 1; ++i) {
    segContainer.emplace_back(
        BasicSegment2d{utils::toBasicPoint(lineString[i]), utils::toBasicPoint(lineString[i + 1])});
  }
  tree.insert(segContainer);

  return tree;
}

/**
 * @brief makeIndexedSegmenTree create search tree of segments with indices
 * @param lineString line string that the segments will be created from
 * @throws InvalidInputError if the line string size is below 2
 * @return a search tree
 */
inline IndexedSegmentTree makeIndexedSegmenTree(const BasicLineString2d& lineString) {
  if (lineString.size() < 2) {
    throw InvalidInputError("Need line string size of at least 2 to make tree");
  }
  IndexedSegmentTree tree;
  std::vector<std::pair<BasicSegment2d, size_t>> segContainer;
  segContainer.reserve(lineString.size() - 1);
  for (size_t i = 0; i < lineString.size() - 1; ++i) {
    segContainer.emplace_back(std::make_pair(BasicSegment2d{lineString.at(i), lineString.at(i + 1)}, i));
  }
  tree.insert(segContainer);
  return tree;
}

/**
 * @brief getLowestSAbove find self-intersection nearest to the given start point along a segment
 * @param selfIntersections list of self-intersections
 * @param minS start coordinate along the segment
 * @return index of the search result in selfIntersections
 */
inline Optional<size_t> getLowestSAbove(const SelfIntersections2d& selfIntersections, const double minS) {
  double curMinS = std::numeric_limits<double>::max();
  Optional<size_t> res;
  for (size_t i = 0; i < selfIntersections.size(); ++i) {
    const auto& ci = selfIntersections.at(i);
    if (ci.firstSegment.s > minS && ci.firstSegment.s < curMinS) {
      res = i;
      curMinS = ci.firstSegment.s;
    }
  }
  return res;
}

/**
 * @brief getSelfIntersectionsAt find list of intersections based on a segment search tree
 * @param tree segment search tree
 * @param segToCheck the segment to be evaluated
 * @param seg index of the segment to be evaluated
 * @return a list of intersections involving the given segment
 */
inline SelfIntersections2d getSelfIntersectionsAt(const IndexedSegmentTree& tree, const size_t segToCheck,
                                                  const BasicSegment2d& seg) {
  SelfIntersections2d curSegIntersections;
  for (auto it = tree.qbegin(bgi::intersects(seg)); it != tree.qend(); ++it) {
    const auto& otherSegIdx = it->second;
    const auto& otherSeg = it->first;
    if (otherSeg.first != seg.second && otherSeg.second != seg.first && otherSeg.first != seg.first &&
        otherSegIdx > segToCheck) {
      BasicPoints2d intersectionPoints;
      boost::geometry::intersection(seg, otherSeg, intersectionPoints);
      const auto intersectionPoint = intersectionPoints.front();
      auto firstS = (intersectionPoint - seg.first).norm();
      auto lastS = (intersectionPoint - otherSeg.first).norm();
      curSegIntersections.emplace_back(SelfIntersection2d{SelfIntersectionLong{segToCheck, firstS},
                                                          SelfIntersectionLong{otherSegIdx, lastS}, intersectionPoint});
    }
  }

  return curSegIntersections;
}

struct PointSearchResult {
  const BasicPoint2d nextPoint;
  const size_t nextSegIdx;
  const double lastS;
};

/**
 * @brief findNextPoint find next point to walk on a line strings join operation
 * @param curSegIntersections list of intersections on the current segment
 * @param seg segment to evaluate
 * @param lastS coordinate along the segment to start the search on
 * @param i segment index
 * @return coordinate of next point on the walk
 */
inline PointSearchResult findNextPoint(const SelfIntersections2d& curSegIntersections, const BasicSegment2d& seg,
                                       const double lastS, const size_t i) {
  if (!curSegIntersections.empty()) {
    auto possibeNextIntersection = getLowestSAbove(curSegIntersections, lastS);
    if (possibeNextIntersection) {
      const auto& ni = curSegIntersections.at(*possibeNextIntersection);
      return PointSearchResult{ni.intersectionPoint, ni.lastSegment.idx, ni.lastSegment.s};
    }
  }
  return PointSearchResult{seg.second, i + 1, 0.};
}

inline BasicLineString2d removeSelfIntersections(const BasicLineString2d& lineString) {
  if (lineString.size() <= 3) {
    return lineString;
  }
  auto tree = makeIndexedSegmenTree(lineString);
  double lastS{0.};
  BasicLineString2d newLS{lineString.front()};
  size_t i = 0;
  while (i < lineString.size() - 2) {
    BasicSegment2d curSeg{lineString.at(i), lineString.at(i + 1)};
    auto curSegIntersections = getSelfIntersectionsAt(tree, i, curSeg);
    auto np = findNextPoint(curSegIntersections, curSeg, lastS, i);
    newLS.emplace_back(np.nextPoint);
    i = np.nextSegIdx;
    lastS = np.lastS;
  }
  newLS.push_back(lineString.back());
  return newLS;
}

/**
 * @brief checkForInversion asserts that a shifted line string is not inverted relative to the original line string by
 * checking the distance of those
 * @param oldLS the original line string
 * @param offsetLS the shifted line string
 * @param distance shifting distance (left is positive)
 * @param epsilon maximum allowed difference between requested and actual minimum offset
 */
template <typename LineString2dT>
inline void checkForInversion(const LineString2dT& oldLS, const BasicLineString2d& offsetLS, const double distance,
                              const double epsilon = 1.e-7) {
  auto tree = internal::makeSegmentTree(oldLS);
  for (const auto& p : offsetLS) {
    auto it = tree.qbegin(internal::bgi::nearest(p, 1));
    auto pd = geometry::distance2d(BasicLineString2d{it->first, it->second}, p);
    if (pd + epsilon < distance) {
      throw GeometryError("Possible Lanelet Inversion");
    }
  }
}

/**
 * @brief shiftPerpendicular shift point perpendicular to either the preceding or following segment
 * @param lineString original line string
 * @param idx index of point to shft
 * @param distance offset distance (left is positive) to shift
 * @param asLast use preceding segment as reference if true, else use following
 * @param pv following and preveding point on line string
 * @return shifted point
 */
template <typename LineString2dT>
inline BasicPoint2d shiftPerpendicular(const LineString2dT& lineString, const size_t idx, const double distance,
                                       const bool asLast, const PointVincinity& pv) {
  if (idx == 0 && asLast) {
    throw GeometryError("Can't shift first point of line string as endpoint of segment");
  }
  if (idx + 1 == lineString.size() && !asLast) {
    throw GeometryError("Can't shift last point of line string as start point of segment");
  }
  Eigen::Vector2d perpendicular = asLast ? pv.preceding.normalized() : pv.following.normalized();
  Eigen::Vector2d norm(-perpendicular(1), perpendicular(0));
  return utils::toBasicPoint(lineString[idx]) + norm * distance;
}

/**
 * @brief shiftConvexSharp shift corner that is convex at a more than 90 degree angle. Inserts a new point to limit
 * maximum offset
 * @param lineString original line string
 * @param idx index of point to shft
 * @param distance offset distance (left is positive) to shift
 * @param pv following and preveding point on line string
 * @return shifted points
 */
template <typename LineString2dT>
inline BasicLineString2d shiftConvexSharp(const LineString2dT& lineString, const size_t idx, const double distance,
                                          const PointVincinity& pv) {
  if (idx == 0) {
    throw GeometryError("Can't shift first point of line string as sharp convex");
  }
  if (idx + 1 == lineString.size()) {
    throw GeometryError("Can't shift last point of line string as sharp convex");
  }
  BasicPoint2d firstP = shiftPerpendicular(lineString, idx, distance, true, pv);
  BasicPoint2d lastP = shiftPerpendicular(lineString, idx, distance, false, pv);
  auto alpha = M_PI - std::acos(pv.following.dot(pv.preceding) / (pv.preceding.norm() * pv.following.norm()));
  auto overshoot = distance * std::tan(M_PI_4 - alpha / 4);
  return {firstP + pv.preceding.normalized() * overshoot, lastP - pv.following.normalized() * overshoot};
}

/**
 * @brief shiftPoint in lateral direction
 * @param lineString original line string
 * @param distance offset distance (left is positive) to shift
 * @param idx index of point to shift
 * @param pv following and preveding point on line string
 * @return shifted point(s), bool indicating insertion was still convex
 */
template <typename LineString2dT>
inline std::pair<BasicLineString2d, bool> shiftPoint(const LineString2dT& lineString, const double distance,
                                                     const size_t idx, const PointVincinity& pv) {
  auto convexity = internal::getConvexity(lineString, idx, pv, distance > 0);
  if (convexity == Convexity::Concave) {
    return std::make_pair(BasicLineString2d{shiftPerpendicular(lineString, idx, distance, true, pv)}, false);
  }
  if (convexity == Convexity::Convex) {
    return std::make_pair(BasicLineString2d{shiftLateral(lineString, idx, distance, pv)}, true);
  }
  if (convexity == Convexity::ConvexSharp) {
    return std::make_pair(shiftConvexSharp(lineString, idx, distance, pv), true);
  }
  throw InvalidInputError("Unknown Convexity status");
}

/**
 * @brief extractConvex get convex parts of line string offset by distance.
 * @param lineString original line string
 * @param distance offset distance (left is positive)
 * @return list of line strings, each one is convex. Guaranteed to intersect at non-convex parts
 */
template <typename LineString2dT>
inline std::vector<BasicLineString2d> extractConvex(const LineString2dT& lineString, const double distance) {
  std::vector<BasicLineString2d> offsets;
  size_t curIdx{0};
  while (curIdx + 1 < lineString.size()) {
    auto pv = internal::makeVincinity(lineString, curIdx);
    BasicLineString2d ls{shiftPerpendicular(lineString, curIdx, distance, false, pv)};
    for (size_t i = curIdx + 1; i < lineString.size(); ++i) {
      curIdx = i;
      auto ipv = internal::makeVincinity(lineString, i);
      auto shiftResult = shiftPoint(lineString, distance, i, ipv);
      ls.insert(ls.end(), shiftResult.first.begin(), shiftResult.first.end());
      if (!shiftResult.second) {
        break;
      }
    }
    offsets.push_back(ls);
  }
  return offsets;
}

using SegmentMap = std::map<size_t, LineStringsCoordinate>;

struct SegmentSearch {
  IndexedSegmentTree tree;
  SegmentMap map;
};

/**
 * @brief makeTree create search tree of segments from line strings
 * @param convexSubStrings list of line strings
 * @return search tree and mapping from segment index of uncut linestring to linestring and segment index of substring
 */

inline SegmentSearch makeTree(const std::vector<BasicLineString2d>& convexSubStrings) {
  IndexedSegmentTree tree;
  SegmentMap segMap;
  size_t idx{0};
  for (size_t i = 0; i < convexSubStrings.size(); ++i) {
    const auto& ss = convexSubStrings.at(i);
    for (size_t j = 0; j + 1 < ss.size(); ++j) {
      BasicSegment2d seg{utils::toBasicPoint(ss.at(j)), utils::toBasicPoint(ss.at(j + 1))};
      tree.insert(std::make_pair(seg, idx));
      segMap.emplace(idx, LineStringsCoordinate{i, j});
      ++idx;
    }
  }
  return SegmentSearch{tree, segMap};
}

/**
 * @brief joinSubStrings create one line string of several ones by walking along them and joining them at the first
 * intersection
 * @param convexSubStrings list of substrings. Walk will start at the first element and continue until the list is
 * exhausted
 * @param tree search tree
 * @param segMap mapping from segment index of uncut linestring to linestring and segment index of substring
 * @return joined line string
 */
inline BasicLineString2d joinSubStrings(const std::vector<BasicLineString2d>& convexSubStrings,
                                        const IndexedSegmentTree& tree, const SegmentMap& segMap) {
  if (convexSubStrings.empty()) {
    return BasicLineString2d();
  }
  if (convexSubStrings.size() == 1) {
    return convexSubStrings.front();
  }
  double lastS{0.};
  BasicLineString2d newLS{convexSubStrings.front().front()};
  size_t i = 0;
  size_t idx = 0;
  while (i < convexSubStrings.size()) {
    const auto& ls = convexSubStrings.at(i);
    for (size_t j = 0; j + 1 < ls.size(); ++j) {
      BasicSegment2d curSeg{ls.at(j), ls.at(j + 1)};
      auto curSegIntersections = getSelfIntersectionsAt(tree, idx, curSeg);
      auto np = findNextPoint(curSegIntersections, curSeg, lastS, idx);
      newLS.emplace_back(np.nextPoint);
      idx = np.nextSegIdx;
      if (idx >= segMap.size()) {
        i = convexSubStrings.size();
        break;
      }
      lastS = np.lastS;
      if (segMap.at(idx).lineStringIdx != i) {
        i = segMap.at(idx).lineStringIdx;
        break;
      }
    }
  }
  newLS.push_back(convexSubStrings.back().back());
  return newLS;
}

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
    auto p1 = traits::toBasicPoint(*first);
    auto p2 = traits::toBasicPoint(*second);
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

template <typename Point2dT>
double curvature2d(const Point2dT& p1, const Point2dT& p2, const Point2dT& p3) {
  // see https://en.wikipedia.org/wiki/Menger_curvature#Definition
  const double area = 0.5 * ((p2.x() - p1.x()) * (p3.y() - p1.y()) - (p2.y() - p1.y()) * (p3.x() - p1.x()));
  const double side1 = distance(p1, p2);
  const double side2 = distance(p1, p3);
  const double side3 = distance(p2, p3);
  const double product = side1 * side2 * side3;
  if (product < 1e-20) {
    return std::numeric_limits<double>::infinity();
  }
  return std::fabs(4 * area / product);
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
  static_assert(traits::is3D<LineString3dT>(), "Please call this function with a 3D type!");
  return internal::projectedPoint3d(traits::toHybrid(l1), traits::toHybrid(l2));
}

template <typename LineString3d1T, typename LineString3d2T>
IfLS2<LineString3d1T, LineString3d2T, double> distance3d(const LineString3d1T& l1, const LineString3d2T& l2) {
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

template <typename LineString2dT>
BasicPoint2d fromArcCoordinates(const LineString2dT& lineString, const ArcCoordinates& arcCoords) {
  if (lineString.size() < 2) {
    throw InvalidInputError("Can't use arc coordinates on degenerated line string");
  }
  auto hLineString = utils::toHybrid(lineString);
  auto ratios = accumulatedLengthRatios(lineString);
  const auto llength = length(lineString);
  size_t startIdx{};
  size_t endIdx{};
  for (size_t i = 0; i < ratios.size(); ++i) {
    if (ratios.at(i) * llength > arcCoords.length) {
      startIdx = i;
      endIdx = i + 1;
      break;
    }
  }
  if (endIdx == 0) {
    endIdx = lineString.size() - 1;
    startIdx = endIdx - 1;
  }
  return internal::fromArcCoords(hLineString, interpolatedPointAtDistance(utils::to2D(lineString), arcCoords.length),
                                 startIdx, endIdx, arcCoords.distance);
}

template <typename LineString2dT>
BasicLineString2d offsetNoThrow(const LineString2dT& lineString, const double distance) {
  auto convexSubStrings = internal::extractConvex(lineString, distance);
  auto segSearch = internal::makeTree(convexSubStrings);
  auto newLS = internal::joinSubStrings(convexSubStrings, segSearch.tree, segSearch.map);
  return newLS;
}

template <typename LineString2dT>
BasicLineString2d offset(const LineString2dT& lineString, const double distance) {
  auto newLS = offsetNoThrow(lineString, distance);
  internal::checkForInversion(lineString, newLS, distance);
  return newLS;
}

template <typename LineString3dT, typename>
Segment<traits::PointType<LineString3dT>> closestSegment(const LineString3dT& lineString,
                                                         const BasicPoint3d& pointToProject) {
  static_assert(traits::is3D<LineString3dT>(), "Please call this function with a 3D type!");
  helper::ProjectedPoint<traits::PointType<LineString3dT>> projectedPoint;
  distance(utils::toHybrid(lineString), pointToProject, projectedPoint);
  return Segment<traits::PointType<LineString3dT>>(projectedPoint.result->segmentPoint1,
                                                   projectedPoint.result->segmentPoint2);
}

//! Projects the given point in 2d to the LineString.
template <typename LineString2dT, typename>
Segment<traits::PointType<LineString2dT>> closestSegment(const LineString2dT& lineString,
                                                         const BasicPoint2d& pointToProject) {
  static_assert(traits::is2D<LineString2dT>(), "Please call this function with a 2D type!");
  helper::ProjectedPoint<traits::PointType<LineString2dT>> projectedPoint;
  distance(utils::toHybrid(lineString), pointToProject, projectedPoint);
  return Segment<traits::PointType<LineString2dT>>(projectedPoint.result->segmentPoint1,
                                                   projectedPoint.result->segmentPoint2);
}
}  // namespace geometry
}  // namespace lanelet
