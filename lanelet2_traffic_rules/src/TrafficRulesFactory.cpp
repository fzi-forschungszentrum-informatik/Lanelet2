#include "TrafficRulesFactory.h"

namespace lanelet {

constexpr char Locations::Germany[];

constexpr char Participants::Vehicle[];
constexpr char Participants::Car[];
constexpr char Participants::Bus[];
constexpr char Participants::Truck[];
constexpr char Participants::Bicycle[];
constexpr char Participants::Pedestrian[];
constexpr const char* Participants::VehicleTypes[];

namespace {
bool isVehicleType(const std::string& t) {
  return std::find(std::begin(Participants::VehicleTypes), std::end(Participants::VehicleTypes), t);
}
}  // namespace

TrafficRulesUPtr TrafficRulesFactory::create(const std::string& location, const std::string& participant,
                                             TrafficRules::Configuration configuration) {
  auto& registry = instance().registry_;
  auto elem = registry.find(std::make_pair(location, participant));
  if (elem == registry.end() && isVehicleType(participant)) {
    // second try for vehicle types
    elem = registry.find(std::make_pair(location, std::string(Participants::Vehicle)));
  }
  if (elem != registry.end()) {
    configuration["location"] = location;
    configuration["participant"] = participant;
    return elem->second(configuration);
  }
  throw InvalidInputError("No matching traffic rules found for location " + location + ", participant " + participant);
}

std::vector<std::pair<std::string, std::string>> TrafficRulesFactory::availableTrafficRules() {
  std::vector<std::string> rules;
  auto& registry = TrafficRulesFactory::instance().registry_;
  return utils::transform(registry, [](const auto& elem) { return elem.first; });
}

TrafficRulesFactory& TrafficRulesFactory::instance() {
  static TrafficRulesFactory factory;
  return factory;
}
}  // namespace lanelet
