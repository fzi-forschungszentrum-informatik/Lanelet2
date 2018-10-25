#include "TrafficRules.h"
#include <lanelet2_core/geometry/Area.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/RegulatoryElement.h>
#include <lanelet2_core/utility/Units.h>

namespace lanelet {
namespace {
bool canChangeToLeft(LaneChangeType t) { return t == LaneChangeType::Both || t == LaneChangeType::ToLeft; }
bool canChangeToRight(LaneChangeType t) { return t == LaneChangeType::Both || t == LaneChangeType::ToRight; }

bool isDrivingDir(const lanelet::ConstLanelet& ll) {
  return !(ll.attributeOr(AttributeName::OneWay, true) && ll.inverted());
}

template <typename T>
T sort(const T& toSort) {
  auto sorted = toSort;
  std::sort(sorted.begin(), sorted.end());
  return sorted;
}

bool isVehicleType(const std::string& participant) {
  return participant == "car" || participant == "bus" || participant == "truck" || participant == "motorcycle";
}

}  // namespace
TrafficRules::~TrafficRules() = default;

bool TrafficRules::hasDynamicRules(const ConstLanelet& lanelet) const {
  auto regelems = lanelet.regulatoryElements();
  auto isDynamic = [](const auto& elem) { return elem->attributeOr(AttributeName::Dynamic, false); };
  return std::any_of(regelems.begin(), regelems.end(), isDynamic);
}

template <typename PrimT>
bool hasMatchingParticipantAttribute(const std::string& participant, const PrimT& prim) {
  using namespace std::string_literals;
  if (!isVehicleType(participant)) {
    return prim.attributeOr(participant, false);
  }
  auto hasParticipant = prim.attributeOr("vehicle:"s + participant, Optional<bool>());
  if (!hasParticipant) {
    return prim.attributeOr(AttributeName::Vehicle, false);
  }
  return *hasParticipant;
}

bool TrafficRules::canPass(const lanelet::ConstLanelet& lanelet) const {
  if (!isDrivingDir(lanelet)) {
    return false;
  }
  return hasMatchingParticipantAttribute(participant(), lanelet);
}

bool TrafficRules::canPass(const ConstArea& area) const { return hasMatchingParticipantAttribute(participant(), area); }

bool TrafficRules::canPass(const ConstLanelet& from, const ConstLanelet& to) const {
  return geometry::follows(from, to) && canPass(from) && canPass(to) && canPassImpl(from, to);
}

Optional<ConstLineString3d> determineCommonLine(const ConstLanelet& ll, const ConstArea& ar) {
  return utils::findIf(ar.outerBound(), [ p1 = ll.leftBound().back(), p2 = ll.rightBound().back() ](auto& boundLs) {
    return (boundLs.back() == p1 && boundLs.front() == p2);
  });
}
Optional<ConstLineString3d> determineCommonLine(const ConstArea& ar1, const ConstArea& ar2) {
  return utils::findIf(ar1.outerBound(), [&ar2](auto& ar1Bound) {
    return !!utils::findIf(
        ar2.outerBound(), [ar1Bound = ar1Bound.invert()](auto& ar2Bound) { return ar2Bound == ar1Bound; });
  });
}

bool TrafficRules::canPass(const ConstLanelet& from, const ConstArea& to) const {
  if (!canPass(from) || !canPass(to)) {
    return false;
  }
  if (geometry::leftOf(from, to)) {
    return canChangeToLeft(laneChangeType(from.leftBound()));
  }
  if (geometry::rightOf(from, to)) {
    return canChangeToRight(laneChangeType(from.rightBound()));
  }
  auto line = determineCommonLine(from, to);
  if (!!line) {
    return canChangeToRight(laneChangeType(*line));
  }
  return false;
}

bool TrafficRules::canPass(const ConstArea& from, const ConstLanelet& to) const {
  if (!canPass(from) || !canPass(to)) {
    return false;
  }
  if (geometry::leftOf(to, from)) {
    return canChangeToRight(laneChangeType(to.leftBound()));
  }
  if (geometry::rightOf(to, from)) {
    return canChangeToLeft(laneChangeType(to.rightBound()));
  }
  auto line = determineCommonLine(to.invert(), from);
  if (!!line) {
    return canChangeToLeft(laneChangeType(*line));
  }
  return false;
}

bool TrafficRules::canPass(const ConstArea& from, const ConstArea& to) const {
  if (!canPass(from) && canPass(to)) {
    return false;
  }
  auto line = determineCommonLine(from, to);
  if (!line) {
    return false;
  }
  return canChangeToLeft(laneChangeType(*line));
}

bool TrafficRules::canChangeLane(const ConstLanelet& from, const ConstLanelet& to) const {
  if (!canPass(from) || !canPass(to)) {
    return false;
  }
  bool isLeft = false;
  if (geometry::leftOf(from, to)) {
    isLeft = true;
  } else if (!geometry::rightOf(from, to)) {
    return false;
  }
  auto type = laneChangeType(isLeft ? from.rightBound() : from.leftBound());
  return isLeft ? canChangeToRight(type) : canChangeToLeft(type);
}

bool TrafficRules::hasSameTrafficRules(const ConstLanelet& lanelet, const ConstLanelet& other) const {
  return lanelet.attributes() == other.attributes() &&
         sort(lanelet.regulatoryElements()) == sort(other.regulatoryElements());
}

std::string TrafficRules::participant() const { return config_.at("participant").value(); }

std::string TrafficRules::location() const { return config_.at("location").value(); }

bool lanelet::TrafficRules::isOneWay(const ConstLanelet& lanelet) const {
  return lanelet.attributeOr(AttributeName::OneWay, true);
}

std::ostream& operator<<(std::ostream& stream, const SpeedLimitInformation& obj) {
  return stream << "speedLimit: " << units::KmHQuantity(obj.speedLimit).value()
                << "km/h, mandatory: " << (obj.isMandatory ? "yes" : "no");
}

std::ostream& operator<<(std::ostream& stream, const TrafficRules& obj) {
  return stream << "location: " << obj.location() << ", participant: " << obj.participant();
}

}  // namespace lanelet
