#include "LaneletPath.h"
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

struct BoundsResult {
  Optional<ConstLineString3d> prevBorder;
  BasicPolygon3d left;
  BasicPolygon3d right;
};

ConstLineString3d getBorder(const Head& head) {
  auto border = head.next->isArea() ? geometry::determineCommonLine(*head.cur.area(), *(head.next->area()))
                                    : geometry::determineCommonLineFollowing(*head.cur.area(), *head.next->lanelet());
  if (!border) {
    throw GeometryError("No shared line string found between adjacent primitives");
  }
  return *border;
}

void updateBorder(BoundsResult& res, const Head& head) {
  if (head.next->isArea()) {
    res.prevBorder = geometry::determineCommonLinePreceding(*head.cur.lanelet(), *head.next->area())->invert();
  }
}

void addLanelet(BoundsResult& br, const Head& head) {
  const auto& rightLS = head.cur.lanelet()->rightBound3d();
  const auto& leftLS = head.cur.lanelet()->leftBound3d();
  br.right.insert(br.right.end(), rightLS.basicBegin() + 1, rightLS.basicEnd());
  br.left.insert(br.left.end(), leftLS.basicBegin() + 1, leftLS.basicEnd());
  if (!!head.next) {
    updateBorder(br, head);
  }
}

void addFirstArea(BoundsResult& res, const Head& head) {
  res.prevBorder = getBorder(head);
  if (!head.cur.area()) {
    throw GeometryError("Uninitialized area");
  }
  appendLineStrings(extractExceptBorder(*head.cur.area(), *res.prevBorder), res.left);
}

void addFirstLanelet(BoundsResult& res, const ConstLanelet& ll) {
  const auto& rightLS = ll.rightBound3d().invert();
  const auto& leftLS = ll.leftBound3d();
  res.left.reserve(leftLS.size() + rightLS.size());
  res.left.insert(res.left.end(), rightLS.basicBegin(), rightLS.basicEnd());
  res.left.insert(res.left.end(), leftLS.basicBegin(), leftLS.basicEnd());
}

void addArea(BoundsResult& br, const Head& head) {
  auto border = getBorder(head);
  auto bounds = extractBounds(*head.cur.area(), br.prevBorder->invert(), border);
  appendLineStringsExceptFirst(bounds.first, br.left, false);
  appendLineStringsExceptFirst(bounds.second, br.right, true);
  br.prevBorder = border;
}

BoundsResult appendTail(const Head& head) {
  BoundsResult res;
  if (!head.next) {
    throw InvalidInputError("Tail must have successor, results must be empty");
  }
  if (head.cur.isArea()) {
    addFirstArea(res, head);
  } else {
    addFirstLanelet(res, *head.cur.lanelet());
    updateBorder(res, head);
  }
  return res;
}

void appendFinalArea(const Head& head, BoundsResult& br) {
  if (!br.prevBorder) {
    throw InvalidInputError("border not given for area");
  }
  appendLineStringsExceptFirstAndLast(extractExceptBorder(*head.cur.area(), br.prevBorder->invert()), br.left);
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
