#include "lanelet2_core/geometry/Lanelet.h"

#include <boost/geometry/algorithms/equals.hpp>

#include "lanelet2_core/geometry/LineString.h"
#include "lanelet2_core/geometry/Polygon.h"
#include "lanelet2_core/primitives/CompoundPolygon.h"
#include "lanelet2_core/primitives/Lanelet.h"
#include "lanelet2_core/primitives/RegulatoryElement.h"

namespace lanelet {
namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;
using OptDistance = boost::optional<double>;

namespace {
// gcc does not like boost optional in release builds
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
template <typename Point>
auto centerpoint(Point p1, Point p2) {
  bg::add_point(p1, p2);
  bg::multiply_value(p1, 0.5);
  return p1;
}

auto makeCenterpoint(const ConstPoint2d& p1, const ConstPoint2d& p2) {
  return Point3d(InvalId, centerpoint(utils::to3D(p1).basicPoint(), utils::to3D(p2).basicPoint()));
}

using SegmentTree = bgi::rtree<BasicSegment2d, bgi::linear<2>>;
using PointNode = std::pair<BasicPoint2d, ConstLineString2d::const_iterator>;
using PointTree = bgi::rtree<PointNode, bgi::linear<16>>;

class BoundChecker {
 public:
  BoundChecker(const ConstLineString2d& left, const ConstLineString2d& right)
      : leftSegments_{makeSegments(left)},
        rightSegments_{makeSegments(right)},
        leftPts_{makePoints(left)},
        rightPts_{makePoints(right)},
        entry_{right.front().basicPoint(), left.front().basicPoint()},
        exit_{left.back().basicPoint(), right.back().basicPoint()} {}

  bool intersects(const BasicSegment2d& seg) const {
    const bool result = intersectsLeft(seg) || intersectsRight(seg) || crossesEntry(seg) || crossesExit(seg);
    return result;
  }

  bool intersectsLeft(const BasicSegment2d& seg) const {
    for (auto it = leftSegments_.qbegin(bgi::intersects(seg)); it != leftSegments_.qend(); ++it) {
      if (!boost::geometry::equals(it->first, seg.first)) {
        return true;
      }
    }
    return false;
  }

  bool intersectsRight(const BasicSegment2d& seg) const {
    for (auto it = rightSegments_.qbegin(bgi::intersects(seg)); it != rightSegments_.qend(); ++it) {
      if (!boost::geometry::equals(it->first, seg.first)) {
        return true;
      }
    }
    return false;
  }

  bool secondCrossesBounds(const BasicSegment2d& seg, bool left) const {
    const auto& segmentTree = left ? leftSegments_ : rightSegments_;
    for (auto it = segmentTree.qbegin(bgi::intersects(seg)); it != segmentTree.qend(); ++it) {
      using boost::geometry::equals;
      if (!equals(it->first, seg.second) && !equals(it->second, seg.second)) {
        return true;
      }
    }
    return false;
  }

  bool crossesEntry(const BasicSegment2d& seg) const {
    if (geometry::intersects(seg, entry_)) {
      return geometry::internal::pointIsLeftOf(entry_.first, entry_.second, seg.second);
    }
    return false;
  }

  bool crossesExit(const BasicSegment2d& seg) const {
    if (geometry::intersects(seg, exit_)) {
      return geometry::internal::pointIsLeftOf(exit_.first, exit_.second, seg.second);
    }
    return false;
  }

  template <typename Func>
  void forEachNearestPointLeftUntil(ConstLineString2d::const_iterator iter, Func&& f) const {
    return forEachNearestPointUntilImpl(iter, leftPts_, std::forward<Func>(f));
  }
  template <typename Func>
  void forEachNearestPointRightUntil(ConstLineString2d::const_iterator iter, Func&& f) const {
    return forEachNearestPointUntilImpl(iter, rightPts_, std::forward<Func>(f));
  }

 private:
  template <typename Func>
  static void forEachNearestPointUntilImpl(ConstLineString2d::const_iterator iter, const PointTree& tree, Func&& f) {
    for (auto qIt = tree.qbegin(boost::geometry::index::nearest(iter->basicPoint(), unsigned(tree.size())));
         qIt != tree.qend(); ++qIt) {
      if (f(qIt->second)) {
        break;
      }
    }
  }
  static std::vector<PointNode> makePoints(const ConstLineString2d& ls) {
    std::vector<PointNode> nodes;
    nodes.reserve(ls.size());
    for (auto it = ls.begin(); it != ls.end(); ++it) {
      nodes.emplace_back(it->basicPoint(), it);
    }
    return nodes;
  }
  static std::vector<BasicSegment2d> makeSegments(const ConstLineString2d& line) {
    std::vector<BasicSegment2d> segments;
    segments.reserve(line.numSegments());
    auto makeSegment = [](const auto& seg) { return BasicSegment2d(seg.first.basicPoint(), seg.second.basicPoint()); };
    for (auto i = 0u; i < line.numSegments(); ++i) {
      segments.push_back(makeSegment(line.segment(i)));
    }
    return segments;
  }
  SegmentTree leftSegments_, rightSegments_;
  PointTree leftPts_, rightPts_;
  BasicSegment2d entry_, exit_;
};

std::pair<ConstLineString2d::const_iterator, OptDistance> findClosestNonintersectingPoint(
    ConstLineString2d::const_iterator currPosition, ConstLineString2d::const_iterator otherPoint,
    const BoundChecker& bounds, const Point3d& lastPoint, bool isLeft) {
  OptDistance distance;
  ConstLineString2d::const_iterator closestPosition;
  BasicPoint2d last2d = utils::to2D(lastPoint).basicPoint();
  double dLastOther = bg::distance(*otherPoint, last2d);
  // this lambda is called with points of increasing distance to otherPoint
  auto nonintersectingPointLoop = [&](ConstLineString2d::const_iterator candidate) {
    // point must be after currPosition
    if (candidate < currPosition) {
      return false;
    }
    // we use the triangle inequation to find a distance where we can not
    // expect a closer point than the current one
    if (!!distance && bg::distance(*otherPoint, *candidate) / 2 - dLastOther > *distance) {
      return true;  // stops the loop
    }
    auto candidateDistance = bg::distance(*candidate, *otherPoint) / 2;
    if (!!distance && *distance <= candidateDistance) {
      return false;
    }
    // Candidates are only valid candidates if
    // 1. their distance is minimal (at least for now) -> checked above
    // 2. the new connection does not intersect with the borders
    // 3. connection between point on one bound and point on other bound
    // does not intersect with other parts of the boundary
    BasicSegment2d boundConnection(otherPoint->basicPoint(), candidate->basicPoint());
    BasicSegment2d invBoundConnection(boundConnection.second, boundConnection.first);
    auto centerlinePointCand = centerpoint(boundConnection.first, boundConnection.second);
    BasicSegment2d centerlineCandidate{utils::to2D(lastPoint).basicPoint(), centerlinePointCand};
    if (!bounds.intersects(centerlineCandidate) && !bounds.secondCrossesBounds(boundConnection, isLeft) &&
        !bounds.secondCrossesBounds(invBoundConnection, !isLeft)) {
      distance = candidateDistance;
      closestPosition = candidate;
    }
    return false;
  };
  if (isLeft) {
    bounds.forEachNearestPointLeftUntil(otherPoint, nonintersectingPointLoop);
  } else {
    bounds.forEachNearestPointRightUntil(otherPoint, nonintersectingPointLoop);
  }
  return {closestPosition, distance};
}

std::shared_ptr<ConstLineString3d> calculateCenterline(const ConstLineString2d& leftBound,
                                                       const ConstLineString2d& rightBound) {
  LineString3d centerlinePoints(InvalId);

  // Catch degenerated case
  if (leftBound.empty() || rightBound.empty()) {
    return std::make_shared<ConstLineString3d>(centerlinePoints);
  }

  BoundChecker bounds(utils::to2D(leftBound), utils::to2D(rightBound));

  // Initial point
  centerlinePoints.push_back(makeCenterpoint(leftBound.front(), rightBound.front()));

  auto leftCurrent = leftBound.begin();
  auto rightCurrent = rightBound.begin();
  // special handling is required if the first two points are identical (i.e. the lanelet is closed at the beginning)
  if (*leftCurrent == *rightCurrent) {
    ++leftCurrent;
  }
  while (leftCurrent != leftBound.end() || rightCurrent != rightBound.end()) {
    OptDistance leftCandidateDistance;
    OptDistance rightCandidateDistance;
    ConstLineString2d::const_iterator leftCandidate;
    ConstLineString2d::const_iterator rightCandidate;

    // Determine left candidate
    std::tie(leftCandidate, leftCandidateDistance) =
        findClosestNonintersectingPoint(std::next(leftCurrent), rightCurrent, bounds, centerlinePoints.back(), true);

    // Determine right candidate
    std::tie(rightCandidate, rightCandidateDistance) =
        findClosestNonintersectingPoint(std::next(rightCurrent), leftCurrent, bounds, centerlinePoints.back(), false);
    // Choose the better one
    if (leftCandidateDistance && (!rightCandidateDistance || leftCandidateDistance <= rightCandidateDistance)) {
      assert(leftCandidate != leftBound.end());

      const auto& leftPoint = leftBound[size_t(leftCandidate - leftBound.begin())];
      const auto& rightPoint = rightBound[size_t(rightCurrent - rightBound.begin())];
      centerlinePoints.push_back(makeCenterpoint(leftPoint, rightPoint));
      leftCurrent = leftCandidate;
    } else if (rightCandidateDistance && (!leftCandidateDistance || leftCandidateDistance > rightCandidateDistance)) {
      assert(rightCandidate != rightBound.end());

      const auto& leftPoint = leftBound[size_t(leftCurrent - leftBound.begin())];
      const auto& rightPoint = rightBound[size_t(rightCandidate - rightBound.begin())];
      centerlinePoints.push_back(makeCenterpoint(leftPoint, rightPoint));
      rightCurrent = rightCandidate;
    } else {
      // no next point found. We are done here
      break;
    }
  }

  // we want the centerpoint defined by the endpoints inside in any case
  if (!(leftCurrent == std::prev(leftBound.end()) && rightCurrent == std::prev(rightBound.end()))) {
    centerlinePoints.push_back(makeCenterpoint(leftBound.back(), rightBound.back()));
  }

  return std::make_shared<ConstLineString3d>(centerlinePoints);
}
#pragma GCC diagnostic pop
}  // namespace

// =============================================================
// Class LaneletData
// =============================================================
ConstLineString3d LaneletData::centerline() const {
  //!< @todo review this (thread safe?)
  auto centerline = std::atomic_load_explicit(&centerline_, std::memory_order_acquire);
  if (!centerline) {
    centerline = calculateCenterline(utils::to2D(leftBound()), utils::to2D(rightBound()));
    std::atomic_store_explicit(&centerline_, centerline, std::memory_order_release);
  }
  return *centerline;
}

void LaneletData::setLeftBound(const LineString3d& bound) {
  if (bound != leftBound_) {
    resetCache();
    leftBound_ = bound;
  }
}

void LaneletData::setRightBound(const LineString3d& bound) {
  if (bound != rightBound_) {
    resetCache();
    rightBound_ = bound;
  }
}

void LaneletData::setCenterline(const LineString3d& centerline) {
  centerline_ = std::make_shared<ConstLineString3d>(centerline);
}

bool LaneletData::hasCustomCenterline() const {
  auto centerline = std::atomic_load_explicit(&centerline_, std::memory_order_acquire);
  return !!centerline && centerline->id() != InvalId;
}

CompoundPolygon3d LaneletData::polygon() const {
  return CompoundPolygon3d(ConstLineStrings3d{leftBound(), rightBound().invert()});
}

void lanelet::LaneletData::resetCache() const {
  if (!hasCustomCenterline()) {
    std::atomic_store_explicit(&centerline_, std::shared_ptr<ConstLineString3d>(), std::memory_order_release);
  }
}

std::ostream& operator<<(std::ostream& stream, const ConstLanelet& obj) {
  stream << "[id: " << obj.id();
  if (obj.inverted()) {
    stream << ", inverted";
  }
  stream << ", left id: " << obj.leftBound().id();
  if (obj.leftBound().inverted()) {
    stream << " (inverted)";
  }
  stream << ", right id: " << obj.rightBound().id();
  if (obj.rightBound().inverted()) {
    stream << " (inverted)";
  }
  return stream << "]";
}

namespace utils {
bool has(const ConstLanelet& ll, Id id) {
  auto regelems = ll.regulatoryElements();
  return has(ll.leftBound(), id) || has(ll.rightBound(), id) ||
         std::any_of(regelems.begin(), regelems.end(), [&id](const auto& elem) { return elem->id() == id; });
}
}  // namespace utils

CompoundPolygon3d ConstLanelet::polygon3d() const { return constData()->polygon(); }

CompoundPolygon2d ConstLanelet::polygon2d() const { return CompoundPolygon2d(polygon3d()); }

// namespace utils

}  // namespace lanelet
