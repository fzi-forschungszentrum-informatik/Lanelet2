#include "lanelet2_validation/ValidatorFactory.h"
#include "lanelet2_validation/validators/mapping/BoolTags.h"
#include "lanelet2_validation/validators/mapping/MandatoryTags.h"
#include "lanelet2_validation/validators/mapping/UnknownTagValue.h"
#include "lanelet2_validation/validators/mapping/UnknownTags.h"

namespace lanelet {
namespace validation {

using Values = std::vector<std::string>;
using ValueMap = std::map<std::string, Values>;

namespace {
RegisterMapValidator<BoolTags> regBool;
RegisterMapValidator<UnknownTags> regUnknown;
RegisterMapValidator<UnknownTagValue> regUnknownValue;
RegisterMapValidator<MandatoryTags> regMandatoryTags;

bool startsWith(const std::string& str, const std::string& substr) {  // NOLINT
  return str.compare(0, substr.size(), substr) == 0;
}

const std::vector<std::string>& knownPointTags() {
  using Attr = AttributeNamesString;
  const static std::vector<std::string> Tags{Attr::Ele, Attr::Type, Attr::Subtype};
  return Tags;
}

const ValueMap& knownPointValues() {
  using Attr = AttributeNamesString;
  using Value = AttributeValueString;
  const static ValueMap Tags{{Attr::Type, {Value::Dot, Value::End, Value::Pole, Value::Post, Value::Start}}};
  return Tags;
}

const std::vector<std::string>& knownLineStringTags() {
  using Attr = AttributeNamesString;
  const static std::vector<std::string> Tags{Attr::Type,   Attr::Subtype,   Attr::LaneChange, Attr::Width,
                                             Attr::Height, Attr::Temporary, Attr::Color};
  return Tags;
}

const std::vector<std::string>& mandatoryLineStringTags() {
  using Attr = AttributeNamesString;
  const static std::vector<std::string> Tags{Attr::Type};
  return Tags;
}

const ValueMap& knownLineStringValues() {
  using Attr = AttributeNamesString;
  using Value = AttributeValueString;
  using Str = std::string;
  const static ValueMap Tags{
      {Attr::Type, {Value::Arrow,         Value::BikeMarking,
                    Value::Bump,          Value::Curbstone,
                    Value::Door,          Value::Fence,
                    Value::Gate,          Value::GuardRail,
                    Value::JerseyBarrier, Value::Keepout,
                    Value::LiftGate,      Value::LineThick,
                    Value::LineThin,      Value::PedestrianMarking,
                    Value::Rail,          Value::RoadBorder,
                    Value::StopLine,      Value::Symbol,
                    Value::TrafficLight,  Value::TrafficSign,
                    Value::Trajectory,    Value::Virtual,
                    Value::Visualization, Value::Wall,
                    Value::Zebra,         Value::ZigZag}},
      {Str(Value::LineThick) + Attr::Subtype,
       {Value::Dashed, Value::DashedSolid, Value::Solid, Value::SolidDashed, Value::SolidSolid}},
      {Str(Value::LineThin) + Attr::Subtype,
       {Value::Dashed, Value::DashedSolid, Value::Solid, Value::SolidDashed, Value::SolidSolid}},
      {Str(Value::Curbstone) + Attr::Subtype, {Value::High, Value::Low}},
      {Str(Value::Arrow) + Attr::Subtype,
       {Value::Left, Value::LeftRight, Value::Right, Value::Straight, Value::StraightLeft, Value::StraightRight}},
      {Str(Value::BikeMarking) + Attr::Subtype, {Value::Dashed, Value::Solid}},
      {Str(Value::PedestrianMarking) + Attr::Subtype, {Value::Dashed, Value::Solid}},
      {Str(Value::StopLine) + Attr::Subtype, {Value::Dashed, Value::Solid}},
      {Str(Value::Bump) + Attr::Subtype, {}},
      {Str(Value::TrafficLight) + Attr::Subtype,
       {Value::Red, Value::RedGreen, Value::RedYellow, Value::RedYellowGreen}},
      {Str(Value::Virtual) + Attr::Subtype, {}}};
  return Tags;
}

const std::vector<std::string>& knownLaneletTags() {
  using Attr = AttributeNamesString;
  const static std::vector<std::string> Tags{Attr::Type,   Attr::Subtype,    Attr::OneWay,   Attr::Name,
                                             Attr::Region, Attr::SpeedLimit, Attr::Location, Attr::Participant};
  return Tags;
}

const std::vector<std::string>& mandatoryLaneletTags() {
  using Attr = AttributeNamesString;
  const static std::vector<std::string> Tags{Attr::Type, Attr::Subtype, Attr::Location};
  return Tags;
}

const ValueMap& knownLaneletValues() {
  using Attr = AttributeNamesString;
  using Value = AttributeValueString;
  const static ValueMap Tags{
      {Attr::Type, {Value::Lanelet}},
      {Attr::Subtype,
       {Value::BicycleLane, Value::BusLane, Value::Crosswalk, Value::EmergencyLane, Value::Highway, Value::PlayStreet,
        Value::Rail, Value::Road, Value::SharedWalkway, Value::Stairs, Value::Walkway}},
      {Attr::Location, {Value::Nonurban, Value::Private, Value::Urban}}};
  return Tags;
}

const std::vector<std::string>& knownAreaTags() {
  using Attr = AttributeNamesString;
  const static std::vector<std::string> Tags{Attr::Type,     Attr::Subtype, Attr::SpeedLimit,
                                             Attr::Location, Attr::Region,  Attr::Name};
  return Tags;
}

const ValueMap& knownAreaValues() {
  using Attr = AttributeNamesString;
  using Value = AttributeValueString;
  const static ValueMap Tags{
      {Attr::Type, {Value::Multipolygon}},
      {Attr::Subtype,
       {Value::BicycleLane, Value::Building, Value::BusLane, Value::Crosswalk, Value::EmergencyLane, Value::Exit,
        Value::Freespace, Value::Highway, Value::Keepout, Value::Parking, Value::PlayStreet, Value::Road,
        Value::SharedWalkway, Value::Stairs, Value::TrafficIsland, Value::Vegetation, Value::Walkway}},
      {Attr::Location, {Value::Nonurban, Value::Private, Value::Urban}}};
  return Tags;
}

const std::vector<std::string>& mandatoryAreaTags() {
  using Attr = AttributeNamesString;
  const static std::vector<std::string> Tags{Attr::Type, Attr::Subtype, Attr::Location};
  return Tags;
}

const std::vector<std::string>& knownRegelemTags() {
  using Attr = AttributeNamesString;
  const static std::vector<std::string> Tags{Attr::Dynamic,    Attr::Fallback, Attr::SignType,
                                             Attr::SpeedLimit, Attr::Subtype,  Attr::Type};
  return Tags;
}

const std::vector<std::string>& mandatoryRegelemTags() {
  using Attr = AttributeNamesString;
  const static std::vector<std::string> Tags{Attr::Type, Attr::Subtype};
  return Tags;
}

Issues checkKnownTags(const AttributeMap& attrs, const std::vector<std::string>& knownTags, Id id,
                      Primitive primitive) {
  Issues issues;
  for (const auto& attr : attrs) {
    bool isKnown = utils::anyOf(knownTags, [&attr](auto& knownTag) { return startsWith(attr.first, knownTag); });
    if (!isKnown) {
      issues.push_back(
          Issue(Severity::Warning, primitive, id, "attribute " + attr.first + " is not a known lanelet2 tag."));
    }
  }
  return issues;
}

Issues checkMandatoryTags(const AttributeMap& attrs, const std::vector<std::string>& mandatoryTags, Id id,
                          Primitive primitive) {
  Issues issues;
  for (const auto& tag : mandatoryTags) {
    if (attrs.find(tag) == attrs.end()) {
      issues.push_back(Issue(Severity::Warning, primitive, id, "should have a " + tag + " attribute."));
    }
  }
  return issues;
}

Issues checkKnownTagValues(const AttributeMap& attrs, const ValueMap& knownTagValues, Id id, Primitive primitive,
                           bool combineSubtypes = false) {
  using Attr = AttributeNamesString;
  Issues issues;
  auto type = attrs.find(AttributeName::Type);
  for (const auto& attr : attrs) {
    auto key = attr.first;
    bool isSubtype = combineSubtypes && key == Attr::Subtype && type != attrs.end();
    if (isSubtype) {
      key = type->first + key;  // NOLINT
    }
    auto values = knownTagValues.find(key);
    if (values == knownTagValues.end()) {
      continue;
    }
    assert(std::is_sorted(values->second.begin(), values->second.end()));
    bool isKnown = std::binary_search(values->second.begin(), values->second.end(), attr.second.value());
    if (!isKnown) {
      auto issue = "value " + attr.second.value() + " is not a known value for " + attr.first;
      if (isSubtype) {
        issue += " and type=" + type->second.value();
      }
      issues.push_back(Issue(Severity::Warning, primitive, id,
                             "value " + attr.second.value() + " is not a known value for " + attr.first));
    }
  }
  return issues;
}

const ValueMap& knownRegelemValues() {
  using Attr = AttributeNamesString;
  using Value = AttributeValueString;
  static ValueMap tags{{Attr::Type, {Value::RegulatoryElement}}};
  return tags;
}

Issues checkAttribute(const AttributeMap& map, Id id, Primitive primitive) {
  using Attr = AttributeNamesString;
  static std::vector<std::string> boolAttributes{Attr::OneWay,      Attr::Dynamic,  Attr::LaneChange,
                                                 Attr::Participant, Attr::Fallback, Attr::Temporary};
  Issues issues;
  for (const auto& attr : map) {
    bool isBoolAttr =
        utils::anyOf(boolAttributes, [&attr](auto& boolAttr) { return startsWith(attr.first, boolAttr); });
    if (isBoolAttr && !attr.second.asBool()) {
      issues.push_back(
          Issue(Severity::Warning, primitive, id,
                "attribute " + attr.first + ": " + attr.second.value() + " should be convertible to bool, but isn't."));
    }
  }
  return issues;
}
template <typename T>
void append(T& container, const T& toAppend) {
  container.insert(container.end(), toAppend.begin(), toAppend.end());
}
}  // namespace

Issues BoolTags::operator()(const lanelet::LaneletMap& map) {
  Issues issues;
  append(issues, utils::concatenate(map.lineStringLayer, [](auto& elem) {
           return checkAttribute(elem.attributes(), elem.id(), Primitive::LineString);
         }));
  append(issues, utils::concatenate(map.laneletLayer, [](auto& elem) {
           return checkAttribute(elem.attributes(), elem.id(), Primitive::Lanelet);
         }));
  append(issues, utils::concatenate(map.areaLayer, [](auto& elem) {
           return checkAttribute(elem.attributes(), elem.id(), Primitive::Area);
         }));
  append(issues, utils::concatenate(map.regulatoryElementLayer, [](auto& elem) {
           return checkAttribute(elem->attributes(), elem->id(), Primitive::RegulatoryElement);
         }));
  return issues;
}

Issues UnknownTags::operator()(const lanelet::LaneletMap& map) {
  Issues issues;
  append(issues, utils::concatenate(map.pointLayer, [](auto& elem) {
           return checkKnownTags(elem.attributes(), knownPointTags(), elem.id(), Primitive::Point);
         }));
  append(issues, utils::concatenate(map.lineStringLayer, [](auto& elem) {
           return checkKnownTags(elem.attributes(), knownLineStringTags(), elem.id(), Primitive::LineString);
         }));
  append(issues, utils::concatenate(map.laneletLayer, [](auto& elem) {
           return checkKnownTags(elem.attributes(), knownLaneletTags(), elem.id(), Primitive::Lanelet);
         }));
  append(issues, utils::concatenate(map.areaLayer, [](auto& elem) {
           return checkKnownTags(elem.attributes(), knownAreaTags(), elem.id(), Primitive::Area);
         }));
  append(issues, utils::concatenate(map.regulatoryElementLayer, [](auto& elem) {
           return checkKnownTags(elem->attributes(), knownRegelemTags(), elem->id(), Primitive::RegulatoryElement);
         }));
  return issues;
}

Issues UnknownTagValue::operator()(const lanelet::LaneletMap& map) {
  Issues issues;
  append(issues, utils::concatenate(map.pointLayer, [](auto& elem) {
           return checkKnownTagValues(elem.attributes(), knownPointValues(), elem.id(), Primitive::Point);
         }));
  append(issues, utils::concatenate(map.lineStringLayer, [](auto& elem) {
           return checkKnownTagValues(elem.attributes(), knownLineStringValues(), elem.id(), Primitive::LineString,
                                      true);
         }));
  append(issues, utils::concatenate(map.laneletLayer, [](auto& elem) {
           return checkKnownTagValues(elem.attributes(), knownLaneletValues(), elem.id(), Primitive::Lanelet);
         }));
  append(issues, utils::concatenate(map.areaLayer, [](auto& elem) {
           return checkKnownTagValues(elem.attributes(), knownAreaValues(), elem.id(), Primitive::Area);
         }));
  append(issues, utils::concatenate(map.regulatoryElementLayer, [](auto& elem) {
           return checkKnownTagValues(elem->attributes(), knownRegelemValues(), elem->id(),
                                      Primitive::RegulatoryElement);
         }));
  return issues;
}

Issues MandatoryTags::operator()(const lanelet::LaneletMap& map) {
  Issues issues;
  append(issues, utils::concatenate(map.lineStringLayer, [](auto& elem) {
           return checkMandatoryTags(elem.attributes(), mandatoryLineStringTags(), elem.id(), Primitive::LineString);
         }));
  append(issues, utils::concatenate(map.laneletLayer, [](auto& elem) {
           return checkMandatoryTags(elem.attributes(), mandatoryLaneletTags(), elem.id(), Primitive::Lanelet);
         }));
  append(issues, utils::concatenate(map.areaLayer, [](auto& elem) {
           return checkMandatoryTags(elem.attributes(), mandatoryAreaTags(), elem.id(), Primitive::Area);
         }));
  append(issues, utils::concatenate(map.regulatoryElementLayer, [](auto& elem) {
           return checkMandatoryTags(elem->attributes(), mandatoryRegelemTags(), elem->id(),
                                     Primitive::RegulatoryElement);
         }));
  return issues;
}

}  // namespace validation
}  // namespace lanelet
