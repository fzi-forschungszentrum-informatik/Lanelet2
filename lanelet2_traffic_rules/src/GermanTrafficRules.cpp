#include "GermanTrafficRules.h"
#include <lanelet2_core/Forward.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/utility/Units.h>
#include "Exceptions.h"
#include "TrafficRulesFactory.h"

using namespace std::string_literals;

namespace lanelet {
namespace {
RegisterTrafficRules<GermanVehicle> gvRules(Locations::Germany, Participants::Vehicle);
RegisterTrafficRules<GermanPedestrian> gpRules(Locations::Germany, Participants::Pedestrian);

template <typename Key, typename Value>
Value getMapOrDefault(const std::map<Key, Value>& map, Key key, Value defaultVal) {
  auto elem = map.find(key);
  if (elem == map.end()) {
    return defaultVal;
  }
  return elem->second;
}

LaneChangeType getChangeType(const std::string& type, const std::string& subtype) {
  // clang-format off
  const static std::map<std::pair<std::string, std::string>, LaneChangeType>
      ChangeType{
        {{AttributeValueString::LineThin, AttributeValueString::Dashed}, LaneChangeType::Both},
        {{AttributeValueString::LineThick, AttributeValueString::Dashed}, LaneChangeType::Both},
        {{AttributeValueString::LineThin, AttributeValueString::DashedStraight}, LaneChangeType::ToRight},
        {{AttributeValueString::LineThick, AttributeValueString::DashedStraight}, LaneChangeType::ToRight},
        {{AttributeValueString::LineThin, AttributeValueString::StraightDashed}, LaneChangeType::ToLeft},
        {{AttributeValueString::LineThick, AttributeValueString::StraightDashed}, LaneChangeType::ToLeft}};
  // clang-format on
  return getMapOrDefault(ChangeType, std::make_pair(type, subtype), LaneChangeType::None);
}

Velocity trafficSignToVelocity(const std::string& typeString) {
  using namespace lanelet::units::literals;
  const static std::map<std::string, Velocity> StrToVelocity{
      {"de274", 30_kmh},      {"de274-5", 5_kmh},     {"de274-10", 10_kmh},   {"de274-20", 20_kmh},
      {"de274-30", 30_kmh},   {"de274-40", 40_kmh},   {"de274-50", 50_kmh},   {"de274-60", 60_kmh},
      {"de274-70", 70_kmh},   {"de274-80", 80_kmh},   {"de274-90", 90_kmh},   {"de274-100", 100_kmh},
      {"de274-110", 110_kmh}, {"de274-120", 120_kmh}, {"de274-130", 130_kmh}, {"de310", 50_kmh}};
  try {
    return StrToVelocity.at(typeString);
  } catch (std::out_of_range&) {
    // not really standard conforming: try to interpret typeString as velocity
    // directly
    Attribute asAttribute(typeString);
    auto velocity = asAttribute.asVelocity();
    if (!!velocity) {
      return *velocity;
    }
    throw lanelet::InterpretationError("Unabe to interpret the velocity information from " + typeString);
  }
}

Optional<LaneChangeType> getHardcodedChangeType(const ConstLineString3d& boundary) {
  if (boundary.hasAttribute(AttributeName::LaneChange)) {
    if (boundary.attributeOr(AttributeName::LaneChange, false)) {
      return {true, LaneChangeType::Both};
    }
    return {true, LaneChangeType::None};
  }
  if (boundary.hasAttribute(AttributeName::LaneChangeLeft)) {
    if (boundary.attributeOr(AttributeName::LaneChangeLeft, false)) {
      if (boundary.attributeOr(AttributeName::LaneChangeRight, false)) {
        return {true, LaneChangeType::Both};
      }
      return {true, LaneChangeType::ToLeft};
    }
    return {true, LaneChangeType::None};
  }
  if (boundary.hasAttribute(AttributeName::LaneChangeRight)) {
    if (boundary.attributeOr(AttributeName::LaneChangeRight, false)) {
      return {true, LaneChangeType::ToRight};
    }
    return {true, LaneChangeType::None};
  }
  return {false, LaneChangeType::None};
}
}  // namespace

SpeedLimitInformation GermanVehicle::speedLimit(const ConstLanelet& lanelet) const {
  auto speedLimits = lanelet.regulatoryElementsAs<SpeedLimit>();
  if (!speedLimits.empty()) {
    return {trafficSignToVelocity(speedLimits.front()->type())};
  }
  using namespace lanelet::units::literals;
  std::string subtype = lanelet.attributeOr(AttributeName::Subtype, AttributeValueString::Normal);
  if (subtype == AttributeValueString::Highway || subtype == AttributeValueString::EmergencyLane) {
    return {130_kmh, false};
  }
  if (subtype == AttributeValueString::PlayStreet) {
    return {7_kmh};  //!< @todo this is not really defined
  }
  std::string location = lanelet.attributeOr(AttributeName::Location, AttributeValueString::Urban);
  if (location == AttributeValueString::Urban) {
    return {50_kmh};
  }
  if (location == AttributeValueString::Nonurban) {
    return {100_kmh};
  }
  // location is garbage
  return {Velocity(50_kmh)};
}

SpeedLimitInformation GermanVehicle::speedLimit(const ConstArea& /*area*/) const {
  using namespace lanelet::units::literals;
  return {1_kmh, false};
}

bool GermanVehicle::canPassImpl(const ConstLanelet& from, const ConstLanelet& to) const { return true; }

LaneChangeType GermanVehicle::laneChangeType(const ConstLineString3d& boundary) const {
  LaneChangeType type;
  auto result = getHardcodedChangeType(boundary);
  if (!!result) {
    type = *result;
  } else {
    type = getChangeType(boundary.attributeOr(AttributeName::Type, ""s),
                         boundary.attributeOr(AttributeName::Subtype, ""s));
  }
  // handle inverted ls
  if (boundary.inverted()) {
    if (type == LaneChangeType::ToLeft) {
      return LaneChangeType::ToRight;
    }
    if (type == LaneChangeType::ToRight) {
      return LaneChangeType::ToLeft;
    }
  }
  return type;
}

bool GermanVehicle::canOvertakeLeft(const ConstLanelet& /*lanelet*/) const { return true; }

bool GermanVehicle::canOvertakeRight(const ConstLanelet& lanelet) const {
  return lanelet.attributeOr(AttributeName::Location, AttributeValueString::Urban) != AttributeValueString::Urban;
}

SpeedLimitInformation GermanPedestrian::speedLimit(const ConstLanelet& /*lanelet*/) const {
  using namespace lanelet::units::literals;
  return SpeedLimitInformation{4_kmh, false};
}

SpeedLimitInformation GermanPedestrian::speedLimit(const ConstArea& /*area*/) const {
  using namespace lanelet::units::literals;
  return SpeedLimitInformation{4_kmh, false};
}

LaneChangeType GermanPedestrian::laneChangeType(const ConstLineString3d& boundary) const {
  auto result = getHardcodedChangeType(boundary);
  if (!result) {
    auto type = boundary.attributeOr(AttributeName::Type, ""s);
    auto subtype = boundary.attributeOr(AttributeName::Subtype, ""s);
    if ((type == AttributeValueString::Curbstone && subtype == AttributeValueString::Low) ||
        type == AttributeValueString::Virtual) {
      result = LaneChangeType::Both;
    } else {
      result = LaneChangeType::None;
    }
  }
  if (boundary.inverted()) {
    if (result == LaneChangeType::ToLeft) {
      return LaneChangeType::ToRight;
    }
    if (result == LaneChangeType::ToRight) {
      return LaneChangeType::ToLeft;
    }
  }
  return *result;
}

}  // namespace lanelet
