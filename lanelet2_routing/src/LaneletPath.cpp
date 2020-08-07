#include "lanelet2_routing/LaneletPath.h"

#include <lanelet2_core/geometry/Area.h>
#include <lanelet2_core/geometry/Lanelet.h>

namespace lanelet {
namespace routing {

namespace {

ConstLineStrings3d extractBetween(const ConstLineStrings3d& wrappedLS, const size_t start, const size_t end) {
  auto startIt = std::next(wrappedLS.begin(), start);
  auto endIt = std::next(wrappedLS.begin(), end);
  auto size = (start >= end) ? wrappedLS.size() - (start - end) - 1 : end - start - 1;
  ConstLineStrings3d between;
  between.reserve(size);
  if (start >= end) {
    between.insert(between.end(), std::next(startIt), wrappedLS.end());
    between.insert(between.end(), wrappedLS.begin(), endIt);
  } else {
    between.insert(between.end(), std::next(startIt), endIt);
  }
  return between;
}

std::pair<ConstLineStrings3d, ConstLineStrings3d> extractBounds(const ConstArea& area,
                                                                const ConstLineString3d& rearCommon,
                                                                const ConstLineString3d& frontCommon) {
  const auto& bounds = area.outerBound();
  auto frontIt = std::find(bounds.begin(), bounds.end(), frontCommon);
  if (frontIt == bounds.end()) {
    throw InvalidInputError("line string front not found");
  }
  auto endIt = std::find(bounds.begin(), bounds.end(), rearCommon);
  if (endIt == bounds.end()) {
    throw InvalidInputError("line string rear not found");
  }
  auto start = std::distance(bounds.begin(), frontIt);
  auto end = std::distance(bounds.begin(), endIt);
  return std::make_pair(extractBetween(bounds, end, start), extractBetween(bounds, start, end));
}

ConstLineStrings3d extractExceptBorder(const ConstArea& area, const ConstLineString3d& border) {
  const auto& bounds = area.outerBound();
  auto frontIt = std::find_if(bounds.begin(), bounds.end(), [&border](auto& ls) { return ls == border; });
  if (frontIt == bounds.end()) {
    throw InvalidInputError("Given line string not in area");
  }
  auto idx = std::distance(bounds.begin(), frontIt);
  return extractBetween(bounds, idx, idx);
}

void appendLineStrings(const ConstLineStrings3d& lss, BasicLineString3d& target, const bool reversed,
                       const bool omitLast, const bool omitFirst) {
  auto compound = CompoundLineString3d(lss);
  if (compound.size() < ((omitLast || omitFirst) ? (omitFirst && omitLast) ? 3 : 2 : 1)) {
    return;
  }
  if (!reversed) {
    target.insert(target.end(), std::next(compound.basicBegin(), omitFirst ? 1 : 0),
                  std::prev(compound.basicEnd(), omitLast ? 1 : 0));
  } else {
    const auto& basicLS = compound.basicLineString();
    target.insert(target.end(), std::next(basicLS.rbegin(), omitFirst ? 1 : 0),
                  std::prev(basicLS.rend(), omitLast ? 1 : 0));
  }
}

void appendLineStringsExceptFirst(const ConstLineStrings3d& lss, BasicLineString3d& target, const bool reversed) {
  appendLineStrings(lss, target, reversed, false, true);
}

void appendLineStrings(const ConstLineStrings3d& lss, BasicLineString3d& target) {
  appendLineStrings(lss, target, false, false, false);
}

void appendLineStringsExceptFirstAndLast(const ConstLineStrings3d& lss, BasicLineString3d& target) {
  appendLineStrings(lss, target, false, true, true);
}

struct Head {
  ConstLaneletOrArea cur;
  Optional<ConstLaneletOrArea> next;
};

enum class LaneletAdjacency { Following, Preceding, Left, Right };
struct LaneletPairAdjacency {
  LaneletAdjacency ll;
  LaneletAdjacency other;
};
struct AdjacencyAndBorder {
  LaneletAdjacency adjacency{LaneletAdjacency::Preceding};
  ConstLineString3d border;
};
struct BoundsResult {
  Optional<ConstLineString3d> prevBorder;
  Optional<LaneletAdjacency> llAdjacency;
  BasicPolygon3d left;
  BasicPolygon3d right;
};

Optional<AdjacencyAndBorder> getLaneletAdjacency(const ConstLanelet& ll, const ConstArea& ar) {
  auto res = geometry::determineCommonLineFollowing(ar, ll);
  if (res) {
    return AdjacencyAndBorder{LaneletAdjacency::Following, *res};
  }
  res = geometry::determineCommonLinePreceding(ll, ar);
  if (res) {
    return AdjacencyAndBorder{LaneletAdjacency::Preceding, *res};
  }
  if (geometry::leftOf(ll, ar)) {
    return AdjacencyAndBorder{LaneletAdjacency::Right, ll.leftBound().invert()};  // must be inverted relative to area
  }
  if (geometry::rightOf(ll, ar)) {
    return AdjacencyAndBorder{LaneletAdjacency::Left, ll.rightBound()};  // must be inverted relative to area
  }
  return {};
}

LaneletPairAdjacency getLaneletAdjacency(const ConstLanelet& ll, const ConstLanelet& other) {
  if (geometry::follows(ll, other)) {
    return LaneletPairAdjacency{LaneletAdjacency::Preceding, LaneletAdjacency::Following};
  }
  if (geometry::follows(other, ll)) {
    return LaneletPairAdjacency{LaneletAdjacency::Following, LaneletAdjacency::Preceding};
  }
  if (geometry::leftOf(other, ll)) {
    return LaneletPairAdjacency{LaneletAdjacency::Right, LaneletAdjacency::Left};
  }
  if (geometry::rightOf(other, ll)) {
    return LaneletPairAdjacency{LaneletAdjacency::Left, LaneletAdjacency::Right};
  }
  return {};
}

void appendExtractedClockwise(BasicPolygon3d& target, const ConstLanelet& ll, const LaneletAdjacency& adj) {
  if (adj == LaneletAdjacency::Right) {
    target.insert(target.end(), ll.leftBound().basicBegin() + 1, ll.leftBound().basicEnd());
  } else if (adj == LaneletAdjacency::Preceding) {
    target.emplace_back(utils::toBasicPoint(ll.rightBound().back()));
  } else if (adj == LaneletAdjacency::Left) {
    target.insert(target.end(), ll.rightBound().invert().basicBegin() + 1, ll.rightBound().invert().basicEnd());
  } else if (adj == LaneletAdjacency::Following) {
    target.emplace_back(utils::toBasicPoint(ll.leftBound().front()));
  } else {
    throw InvalidInputError("Invalid adjacency");
  }
}

void appendExtractedCounterClockwise(BasicPolygon3d& target, const ConstLanelet& ll, const LaneletAdjacency& adj) {
  if (adj == LaneletAdjacency::Right) {
    target.insert(target.end(), ll.leftBound().invert().basicBegin() + 1, ll.leftBound().invert().basicEnd());
  } else if (adj == LaneletAdjacency::Preceding) {
    target.emplace_back(utils::toBasicPoint(ll.leftBound().back()));
  } else if (adj == LaneletAdjacency::Left) {
    target.insert(target.end(), ll.rightBound().basicBegin() + 1, ll.rightBound().basicEnd());
  } else if (adj == LaneletAdjacency::Following) {
    target.emplace_back(utils::toBasicPoint(ll.rightBound().front()));
  } else {
    throw InvalidInputError("Invalid adjacency");
  }
}

void appendFirst(BasicPolygon3d& poly, const ConstLanelet& ll, const LaneletAdjacency& in) {
  if (in == LaneletAdjacency::Following) {
    poly.emplace_back(utils::toBasicPoint(ll.leftBound().front()));
  } else if (in == LaneletAdjacency::Right) {
    poly.emplace_back(utils::toBasicPoint(ll.leftBound().back()));
  } else if (in == LaneletAdjacency::Preceding) {
    poly.emplace_back(utils::toBasicPoint(ll.rightBound().back()));
  } else if (in == LaneletAdjacency::Left) {
    poly.emplace_back(utils::toBasicPoint(ll.rightBound().front()));
  } else {
    throw InvalidInputError("Invalid adjacency");
  }
}

using AdjMap = std::map<LaneletAdjacency, LaneletAdjacency>;
void appendLaneletBoundsLeft(BasicPolygon3d& poly, const ConstLanelet& ll, const LaneletAdjacency& in,
                             const LaneletAdjacency& out) {
  const AdjMap cw{{LaneletAdjacency::Left, LaneletAdjacency::Following},
                  {LaneletAdjacency::Following, LaneletAdjacency::Right},
                  {LaneletAdjacency::Right, LaneletAdjacency::Preceding},
                  {LaneletAdjacency::Preceding, LaneletAdjacency::Left}};
  auto cwNext = cw.at(in);
  while (cwNext != out) {
    appendExtractedClockwise(poly, ll, cwNext);
    cwNext = cw.at(cwNext);
  }
}

void appendLaneletBoundsRight(BasicPolygon3d& poly, const ConstLanelet& ll, const LaneletAdjacency& in,
                              const LaneletAdjacency& out) {
  const AdjMap ccw{{LaneletAdjacency::Left, LaneletAdjacency::Preceding},
                   {LaneletAdjacency::Preceding, LaneletAdjacency::Right},
                   {LaneletAdjacency::Right, LaneletAdjacency::Following},
                   {LaneletAdjacency::Following, LaneletAdjacency::Left}};
  auto ccwNext = ccw.at(in);
  while (ccwNext != out) {
    appendExtractedCounterClockwise(poly, ll, ccwNext);
    ccwNext = ccw.at(ccwNext);
  }
}

void appendLaneletBounds(BoundsResult& br, const ConstLanelet& ll, const LaneletAdjacency& in,
                         const LaneletAdjacency& out) {
  appendLaneletBoundsLeft(br.left, ll, in, out);
  if (in != out) {
    appendLaneletBoundsRight(br.right, ll, in, out);
  }
}
/**
 * @brief update border between cur area and next primitive
 * @param br
 * @param head
 * @return border between cur area and next primitive in cur area
 */
ConstLineString3d getBorder(BoundsResult& br, const Head& head) {
  if (head.next->isArea()) {
    br.prevBorder = geometry::determineCommonLine(*(head.next->area()), *head.cur.area());
    if (!br.prevBorder) {
      throw GeometryError("No shared line string found between adjacent primitives");
    }
    return br.prevBorder->invert();
  }
  auto adj = getLaneletAdjacency(*head.next->lanelet(), *head.cur.area());
  if (!adj) {
    throw GeometryError("No shared line string found between adjacent primitives");
  }
  br.llAdjacency = adj->adjacency;
  return adj->border;
}

void addLaneletPair(BoundsResult& res, const Head& head, const bool notTail = true) {
  auto adj = getLaneletAdjacency(*head.cur.lanelet(), *head.next->lanelet());
  if (!notTail) {
    appendFirst(res.left, *head.cur.lanelet(), adj.ll);
  }
  appendLaneletBounds(res, *head.cur.lanelet(), notTail ? *res.llAdjacency : adj.ll, adj.ll);
  res.llAdjacency = adj.other;
}

void addLaneletAreaHead(BoundsResult& res, const Head& head, const bool notTail = true) {
  auto adj = getLaneletAdjacency(*head.cur.lanelet(), *head.next->area());
  if (!adj) {
    throw std::runtime_error("Did not find adjacency");
  }
  if (!notTail) {
    appendFirst(res.left, *head.cur.lanelet(), adj->adjacency);
  }
  appendLaneletBounds(res, *head.cur.lanelet(), notTail ? *res.llAdjacency : adj->adjacency, adj->adjacency);
  res.prevBorder = adj->border;
}

void addLanelet(BoundsResult& res, const Head& head) {
  if (!head.next) {
    appendLaneletBounds(res, *head.cur.lanelet(), *res.llAdjacency, *res.llAdjacency);
    res.left.pop_back();  // ugly AF
  } else if (head.next->isLanelet()) {
    addLaneletPair(res, head);
  } else {
    addLaneletAreaHead(res, head);
  }
}

void addFirstArea(BoundsResult& res, const Head& head) {
  auto thisBorder = getBorder(res, head);
  if (!head.cur.area()) {
    throw GeometryError("Uninitialized area");
  }
  appendLineStrings(extractExceptBorder(*head.cur.area(), thisBorder), res.left);
}

void addFirstLanelet(BoundsResult& res, const Head& head) {
  if (!head.next) {
    throw std::runtime_error("Following primitive not intialized");
  }
  if (head.next->isLanelet()) {
    addLaneletPair(res, head, false);
  } else {
    addLaneletAreaHead(res, head, false);
  }
}

void addArea(BoundsResult& br, const Head& head) {
  auto oldBorder = *br.prevBorder;
  auto border = getBorder(br, head);
  auto bounds = extractBounds(*head.cur.area(), oldBorder, border);
  appendLineStringsExceptFirst(bounds.first, br.left, false);
  appendLineStringsExceptFirst(bounds.second, br.right, true);
}

BoundsResult appendTail(const Head& head) {
  BoundsResult res;
  if (!head.next) {
    throw InvalidInputError("Tail must have successor, results must be empty");
  }
  if (head.cur.isArea()) {
    addFirstArea(res, head);
  } else {
    addFirstLanelet(res, head);
  }
  return res;
}

void appendFinalArea(const Head& head, BoundsResult& br) {
  if (!br.prevBorder) {
    throw InvalidInputError("border not given for area");
  }
  appendLineStringsExceptFirstAndLast(extractExceptBorder(*head.cur.area(), *br.prevBorder), br.left);
}

void appendBounds(const Head& head, BoundsResult& br) {
  if (head.cur.isLanelet()) {
    addLanelet(br, head);
  } else {
    if (!head.next) {
      appendFinalArea(head, br);
    } else {
      addArea(br, head);
    }
  }
}
}  // namespace

LaneletSequence LaneletPath::getRemainingLane(LaneletPath::const_iterator laneletPosition) const {
  ConstLanelets lane;
  while (laneletPosition != lanelets_.end()) {
    lane.push_back(*laneletPosition);
    if (laneletPosition + 1 == lanelets_.end() || !geometry::follows(*laneletPosition, *std::next(laneletPosition))) {
      break;
    }
    ++laneletPosition;
  }
  return lane;
}

BasicPolygon3d getEnclosingPolygon3d(const LaneletOrAreaPath& path) {
  if (path.empty()) {
    return BasicPolygon3d();
  }
  if (path.size() == 1ul) {
    return path.front().boundingPolygon().basicPolygon();
  }
  auto boundsResult = appendTail(Head{path[0], path[1]});
  for (size_t curIdx = 1; curIdx < path.size(); ++curIdx) {
    appendBounds(Head{path[curIdx], (curIdx + 1 < path.size()) ? path[curIdx + 1] : Optional<ConstLaneletOrArea>()},
                 boundsResult);
  }
  boundsResult.left.insert(boundsResult.left.end(), boundsResult.right.rbegin(), boundsResult.right.rend());
  return std::move(boundsResult.left);
}
}  // namespace routing
}  // namespace lanelet
