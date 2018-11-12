#include "primitives/BasicRegulatoryElements.h"
#include <vector>
#include "Exceptions.h"
#include "primitives/LineString.h"
#include "primitives/Point.h"
#include "primitives/RegulatoryElement.h"
#include "utility/Utilities.h"

namespace std {
// namespace {
bool operator==(const lanelet::LaneletDataConstWptr& lhs, const lanelet::LaneletDataConstWptr& rhs) {
  return !lhs.expired() && !rhs.expired() && lhs.lock() == rhs.lock();
}
//}  // namespace
}  // namespace std
namespace lanelet {
namespace {
template <typename T>
bool findAndErase(const T& primitive, RuleParameters& member) {
  auto it = std::find(member.begin(), member.end(), RuleParameter(primitive));
  if (it == member.end()) {
    return false;
  }
  member.erase(it);
  return true;
}

template <typename T>
Optional<T> tryGetFront(const std::vector<T>& vec) {
  if (vec.empty()) {
    return {};
  }
  return vec.front();
}

template <typename T>
RuleParameters toRuleParameters(const std::vector<T>& primitives) {
  return utils::transform(primitives, [](const auto& elem) { return RuleParameter(elem); });
}

void updateTrafficSigns(TrafficSignsWithType trafficSigns) {
  if (!trafficSigns.type.empty()) {
    for (auto& trafficSign : trafficSigns.trafficSigns) {
      trafficSign.setAttribute(AttributeName::Type, AttributeValueString::TrafficSign);
      trafficSign.setAttribute(AttributeName::Subtype, trafficSigns.type);
    }
  }
}

RegulatoryElementDataPtr constructTrafficLightData(Id id, const AttributeMap& attributes,
                                                   const LineStrings3d& trafficLights,
                                                   const Optional<LineString3d>& stopLine) {
  RuleParameterMap rpm = {{RoleNameString::Refers, toRuleParameters(trafficLights)}};
  if (!!stopLine) {
    rpm.insert({RoleNameString::RefLine, {*stopLine}});
  }
  auto data = std::make_shared<RegulatoryElementData>(id, rpm, attributes);
  data->attributes[AttributeName::Type] = AttributeValueString::RegulatoryElement;
  data->attributes[AttributeName::Subtype] = AttributeValueString::TrafficLight;
  return data;
}
RegulatoryElementDataPtr constructTrafficSignData(Id id, const AttributeMap& attributes,
                                                  const TrafficSignsWithType& trafficSigns,
                                                  const TrafficSignsWithType& cancellingTrafficSigns,
                                                  const LineStrings3d& refLines, const LineStrings3d& cancelLines) {
  updateTrafficSigns(trafficSigns);
  updateTrafficSigns(cancellingTrafficSigns);
  RuleParameterMap rpm = {{RoleNameString::Refers, toRuleParameters(trafficSigns.trafficSigns)},
                          {RoleNameString::Cancels, toRuleParameters(cancellingTrafficSigns.trafficSigns)},
                          {RoleNameString::RefLine, toRuleParameters(refLines)},
                          {RoleNameString::CancelLine, toRuleParameters(cancelLines)}};
  auto data = std::make_shared<RegulatoryElementData>(id, rpm, attributes);
  data->attributes[AttributeName::Type] = AttributeValueString::RegulatoryElement;
  data->attributes[AttributeName::Subtype] = AttributeValueString::TrafficSign;
  return data;
}
RegulatoryElementDataPtr constructSpeedLimitData(Id id, const AttributeMap& attributes,
                                                 const TrafficSignsWithType& trafficSigns,
                                                 const TrafficSignsWithType& cancellingTrafficSigns,
                                                 const LineStrings3d& refLines, const LineStrings3d& cancelLines) {
  auto data = constructTrafficSignData(id, attributes, trafficSigns, cancellingTrafficSigns, refLines, cancelLines);
  data->attributes[AttributeName::Subtype] = AttributeValueString::SpeedLimit;
  return data;
}
RegulatoryElementDataPtr constructRightOfWayData(Id id, const AttributeMap& attributes, const Lanelets& rightOfWay,
                                                 const Lanelets& yield, const Optional<LineString3d>& stopLine) {
  RuleParameterMap rpm = {{RoleNameString::RightOfWay, toRuleParameters(rightOfWay)},
                          {RoleNameString::Yield, toRuleParameters(yield)}};
  auto data = std::make_shared<RegulatoryElementData>(id, rpm, attributes);
  data->attributes[AttributeName::Type] = AttributeValueString::RegulatoryElement;
  data->attributes[AttributeName::Subtype] = AttributeValueString::RightOfWay;
  if (stopLine) {
    data->parameters[RoleName::RefLine] = {*stopLine};
  }
  return data;
}
}  // namespace

static RegisterRegulatoryElement<TrafficLight> regTraffic;
static RegisterRegulatoryElement<RightOfWay> regRightOfWay;
static RegisterRegulatoryElement<TrafficSign> regTrafficSign;
static RegisterRegulatoryElement<SpeedLimit> regSpeedLimit;
constexpr char TrafficLight::RuleName[];
constexpr char RightOfWay::RuleName[];
constexpr char TrafficSign::RuleName[];
constexpr char SpeedLimit::RuleName[];

TrafficLight::TrafficLight(const RegulatoryElementDataPtr& data) : RegulatoryElement(data) {
  if (getParameters<ConstLineString3d>(RoleName::Refers).empty()) {
    throw InvalidInputError("No traffic light defined!");
  }
  if (getParameters<ConstLineString3d>(RoleName::RefLine).size() > 1) {
    throw InvalidInputError("There not exist more than one stop line!");
  }
}

TrafficLight::TrafficLight(Id id, const AttributeMap& attributes, const LineStrings3d& trafficLights,
                           const Optional<LineString3d>& stopLine)
    : TrafficLight(constructTrafficLightData(id, attributes, trafficLights, stopLine)) {}

Optional<ConstLineString3d> TrafficLight::stopLine() const {
  return tryGetFront(getParameters<ConstLineString3d>(RoleName::RefLine));
}

Optional<LineString3d> TrafficLight::stopLine() { return tryGetFront(getParameters<LineString3d>(RoleName::RefLine)); }

ConstLineStrings3d TrafficLight::trafficLights() const { return getParameters<ConstLineString3d>(RoleName::Refers); }

LineStrings3d TrafficLight::trafficLights() { return getParameters<LineString3d>(RoleName::Refers); }

void TrafficLight::addTrafficLight(const LineString3d& primitive) {
  parameters()[RoleName::Refers].emplace_back(primitive);
}

bool TrafficLight::removeTrafficLight(const LineString3d& primitive) {
  return findAndErase(primitive, parameters().find(RoleName::Refers)->second);
}

void TrafficLight::setStopLine(const LineString3d& stopLine) { parameters()[RoleName::RefLine] = {stopLine}; }

void TrafficLight::removeStopLine() { parameters()[RoleName::RefLine] = {}; }

RightOfWay::RightOfWay(const RegulatoryElementDataPtr& data) : RegulatoryElement(data) {
  if (getParameters<WeakLanelet>(RoleName::RightOfWay).empty()) {
    throw InvalidInputError("A maneuver must refer to at least one lanelet that has right of way!");
  }
  if (getParameters<WeakLanelet>(RoleName::Yield).empty()) {
    throw InvalidInputError("A maneuver must refer to at least one lanelet that has to yield!");
  }
}

RightOfWay::RightOfWay(Id id, const AttributeMap& attributes, const Lanelets& rightOfWay, const Lanelets& yield,
                       const Optional<LineString3d>& stopLine)
    : RightOfWay(constructRightOfWayData(id, attributes, rightOfWay, yield, stopLine)) {}

ManeuverType RightOfWay::getManeuver(const ConstLanelet& lanelet) const {
  if (utils::contains(rightOfWayLanelets(), lanelet)) {
    return ManeuverType::RightOfWay;
  }
  if (utils::contains(yieldLanelets(), lanelet)) {
    return ManeuverType::Yield;
  }
  return ManeuverType::Unknown;
}

ConstLanelets RightOfWay::rightOfWayLanelets() const { return getParameters<ConstLanelet>(RoleName::RightOfWay); }

Lanelets RightOfWay::rightOfWayLanelets() { return utils::strong(getParameters<WeakLanelet>(RoleName::RightOfWay)); }

ConstLanelets RightOfWay::yieldLanelets() const { return getParameters<ConstLanelet>(RoleName::Yield); }

Lanelets RightOfWay::yieldLanelets() { return utils::strong(getParameters<WeakLanelet>(RoleName::Yield)); }

Optional<ConstLineString3d> RightOfWay::stopLine() const {
  auto stopLine = getParameters<ConstLineString3d>(RoleName::RefLine);
  if (!stopLine.empty()) {
    return stopLine.front();
  }
  return {};
}

Optional<LineString3d> RightOfWay::stopLine() {
  auto stopLine = getParameters<LineString3d>(RoleName::RefLine);
  if (!stopLine.empty()) {
    return stopLine.front();
  }
  return {};
}

void RightOfWay::setStopLine(const LineString3d& stopLine) { parameters()[RoleName::RefLine] = {stopLine}; }

void RightOfWay::addRightOfWayLanelet(const Lanelet& lanelet) {
  parameters()[RoleName::RightOfWay].emplace_back(lanelet);
}

void RightOfWay::addYieldLanelet(const Lanelet& lanelet) { parameters()[RoleName::Yield].emplace_back(lanelet); }

bool RightOfWay::removeRightOfWayLanelet(const Lanelet& lanelet) {
  return findAndErase(lanelet, parameters().find(RoleName::RightOfWay)->second);
}

bool RightOfWay::removeYieldLanelet(const Lanelet& lanelet) {
  return findAndErase(lanelet, parameters().find(RoleName::Yield)->second);
}

void RightOfWay::removeStopLine() { parameters()[RoleName::RefLine] = {}; }

TrafficSign::TrafficSign(const RegulatoryElementDataPtr& data) : RegulatoryElement{data} {
  type();  // will throw if type is invalid
}

TrafficSign::TrafficSign(Id id, const AttributeMap& attributes, const TrafficSignsWithType& trafficSigns,
                         const TrafficSignsWithType& cancellingTrafficSigns, const LineStrings3d& refLines,
                         const LineStrings3d& cancelLines)
    : TrafficSign(
          constructTrafficSignData(id, attributes, trafficSigns, cancellingTrafficSigns, refLines, cancelLines)) {}

ConstLineStrings3d TrafficSign::trafficSigns() const { return getParameters<ConstLineString3d>(RoleName::Refers); }

LineStrings3d TrafficSign::trafficSigns() { return getParameters<LineString3d>(RoleName::Refers); }

std::string TrafficSign::type() const {
  auto signs = trafficSigns();
  if (signs.empty()) {
    throw InvalidInputError("Regulatory element can not determine the type of the traffic sign!");
  }
  if (signs.front().hasAttribute(AttributeName::Subtype)) {
    const auto& attr = signs.front().attribute(AttributeName::Subtype);
    return attr.value();
  }
  throw InvalidInputError("Regulatory element has a traffic sign without subtype attribute!");
}

ConstLineStrings3d TrafficSign::refLines() const { return getParameters<ConstLineString3d>(RoleName::RefLine); }

LineStrings3d TrafficSign::refLines() { return getParameters<LineString3d>(RoleName::RefLine); }

void TrafficSign::addTrafficSign(const LineString3d& sign) { parameters()[RoleName::Refers].emplace_back(sign); }

bool TrafficSign::removeTrafficSign(const LineString3d& sign) {
  return findAndErase(sign, parameters().find(RoleName::Refers)->second);
}

void TrafficSign::addRefLine(const LineString3d& line) { parameters()[RoleName::RefLine].emplace_back(line); }

bool TrafficSign::removeRefLine(const LineString3d& line) {
  return findAndErase(line, parameters().find(RoleName::RefLine)->second);
}

void TrafficSign::addCancellingRefLine(const LineString3d& line) {
  parameters()[RoleName::CancelLine].emplace_back(line);
}

bool TrafficSign::removeCancellingRefLine(const LineString3d& line) {
  return findAndErase(line, parameters().find(RoleName::CancelLine)->second);
}

SpeedLimit::SpeedLimit(Id id, const AttributeMap& attributes, const TrafficSignsWithType& trafficSigns,
                       const TrafficSignsWithType& cancellingTrafficSigns, const LineStrings3d& refLines,
                       const LineStrings3d& cancelLines)
    : TrafficSign(
          constructSpeedLimitData(id, attributes, trafficSigns, cancellingTrafficSigns, refLines, cancelLines)) {}

SpeedLimit::SpeedLimit(const RegulatoryElementDataPtr& data) : TrafficSign(data) {}

void TrafficSign::addCancellingTrafficSign(const LineString3d& sign) {
  parameters()[RoleName::Cancels].emplace_back(sign);
}

bool TrafficSign::removeCancellingTrafficSign(const LineString3d& sign) {
  return findAndErase(sign, parameters().find(RoleName::Cancels)->second);
}

ConstLineStrings3d TrafficSign::cancellingTrafficSigns() const {
  return getParameters<ConstLineString3d>(RoleName::Cancels);
}

LineStrings3d TrafficSign::cancellingTrafficSigns() { return getParameters<LineString3d>(RoleName::Cancels); }

std::string TrafficSign::cancelType() const {
  auto signs = cancellingTrafficSigns();
  if (signs.front().hasAttribute(AttributeName::Subtype)) {
    const auto& attr = signs.front().attribute(AttributeName::Subtype);
    return attr.value();
  }
  throw InvalidInputError(
      "Regulatory element has a cancelling traffic sign without subtype "
      "attribute!");
}

ConstLineStrings3d TrafficSign::cancelLines() const { return getParameters<ConstLineString3d>(RoleName::CancelLine); }

LineStrings3d TrafficSign::cancelLines() { return getParameters<LineString3d>(RoleName::CancelLine); }

}  // namespace lanelet
