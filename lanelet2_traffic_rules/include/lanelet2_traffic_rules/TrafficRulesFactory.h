#pragma once
#include "lanelet2_traffic_rules/TrafficRules.h"

namespace lanelet {
struct Locations {
  static constexpr char Germany[] = "de";
};

// participants are defined in lanelet2_core/Attributes.h

namespace traffic_rules {

class TrafficRulesFactory {
 public:
  using FactoryFcn = std::function<TrafficRulesUPtr(const TrafficRules::Configuration&)>;
  void registerStrategy(const std::string& location, const std::string& participant,
                        const FactoryFcn& factoryFunction) {
    registry_[std::make_pair(location, participant)] = factoryFunction;
  }

  /** create a traffic rule object based on location and participant
   * @throws InvalidInputError if no traffic rules are available for a
   * location/participant combination
   */
  static TrafficRulesUPtr create(const std::string& location, const std::string& participant,
                                 TrafficRules::Configuration configuration = TrafficRules::Configuration());

  /**
   * @brief returns registered traffic rules by location and participant
   * @return first member of pair is location, second is participant
   */
  static std::vector<std::pair<std::string, std::string>> availableTrafficRules();

  static TrafficRulesFactory& instance();

 private:
  TrafficRulesFactory() = default;
  std::map<std::pair<std::string, std::string>, FactoryFcn> registry_;
};

/**
 * @brief template class for registering new TrafficRules for a certain location
 * and type.
 *
 * To register a class, put
 * RegisterTrafficRules<MyClass> regMyClass(<location>, <participant>);
 * somewhere in your cpp file.
 *
 * Your class is required to have a constructor that takes a
 * TrafficRules::Configuration as argument.
 */
template <class T>
class RegisterTrafficRules {
 public:
  RegisterTrafficRules(const std::string& location, const std::string& participant) {
    // initialize
    TrafficRulesFactory::instance().registerStrategy(
        location, participant,
        +[](const TrafficRules::Configuration& config) -> TrafficRulesUPtr { return std::make_unique<T>(config); });
  }
};
}  // namespace traffic_rules
}  // namespace lanelet
