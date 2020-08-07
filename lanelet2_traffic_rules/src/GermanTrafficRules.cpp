#include "lanelet2_traffic_rules/GermanTrafficRules.h"

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/utility/Units.h>

#include "lanelet2_traffic_rules/Exceptions.h"
#include "lanelet2_traffic_rules/TrafficRulesFactory.h"

using namespace std::string_literals;

namespace lanelet {
namespace traffic_rules {

namespace {
RegisterTrafficRules<GermanVehicle> gvRules(Locations::Germany, Participants::Vehicle);
RegisterTrafficRules<GermanPedestrian> gpRules(Locations::Germany, Participants::Pedestrian);
RegisterTrafficRules<GermanBicycle> gbRules(Locations::Germany, Participants::Bicycle);

Velocity trafficSignToVelocity(const std::string& typeString) {
  using namespace lanelet::units::literals;
  const static std::map<std::string, Velocity> StrToVelocity{
      {"de274", 30_kmh},      {"de274-5", 5_kmh},     {"de274-10", 10_kmh},   {"de274-15", 15_kmh},
      {"de274-20", 20_kmh},   {"de274-30", 30_kmh},   {"de274-40", 40_kmh},   {"de274-50", 50_kmh},
      {"de274-60", 60_kmh},   {"de274-70", 70_kmh},   {"de274-80", 80_kmh},   {"de274-90", 90_kmh},
      {"de274-100", 100_kmh}, {"de274-110", 110_kmh}, {"de274-120", 120_kmh}, {"de274-130", 130_kmh},
      {"de310", 50_kmh}};
  try {
    return StrToVelocity.at(typeString);
  } catch (std::out_of_range&) {
    // try to interpret typeString directly as velocity
    Attribute asAttribute(typeString);
    auto velocity = asAttribute.asVelocity();
    if (!!velocity) {
      return *velocity;
    }
    throw lanelet::InterpretationError("Unable to interpret the velocity information from " + typeString);
  }
}
}  // namespace

Optional<SpeedLimitInformation> GermanVehicle::speedLimit(const RegulatoryElementConstPtrs& regelems) const {
  for (const auto& regelem : regelems) {
    auto speedLimit = std::dynamic_pointer_cast<const SpeedLimit>(regelem);
    if (!!speedLimit) {
      return SpeedLimitInformation{trafficSignToVelocity(speedLimit->type()), true};
    }
  }
  return {};
}

CountrySpeedLimits germanSpeedLimits() {
  using namespace units::literals;
  return {{50_kmh}, {100_kmh}, {130_kmh, false}, {130_kmh, false}, {7_kmh}, {5_kmh}, {20_kmh}};
}

}  // namespace traffic_rules
}  // namespace lanelet
