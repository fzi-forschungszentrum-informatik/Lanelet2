#include "lanelet2_validation/ValidatorFactory.h"

namespace lanelet {
namespace validation {
namespace {

std::vector<std::string> matchSingleRegex(const std::regex& regex, const std::vector<std::string>& toMatch) {
  std::vector<std::string> matches;
  for (const auto& str : toMatch) {
    if (std::regex_match(str, regex)) {
      matches.push_back(str);
    }
  }
  return matches;
}

template <typename RegistryT>
std::vector<std::string> matchValidators(const Regexes& regexes, const RegistryT& registry) {
  auto getKeys = [](auto& map) { return utils::transform(map, [](auto elem) { return elem.first; }); };
  auto toMatch = getKeys(registry);
  if (regexes.empty()) {
    return toMatch;
  }
  auto matches = utils::concatenate(regexes, [&toMatch](auto& regex) { return matchSingleRegex(regex, toMatch); });
  std::sort(matches.begin(), matches.end());
  auto eraseIt = std::unique(matches.begin(), matches.end());
  matches.erase(eraseIt, matches.end());
  return matches;
}

template <typename ValidatorT>
ValidatorsWithName<ValidatorT> createValidators(
    const Regexes& regexes, const std::map<std::string, ValidatorFactory::CreationFcn<ValidatorT>>& registry) {
  auto matches = matchValidators(regexes, registry);
  return utils::transform(matches, [&registry](auto& match) {
    return std::make_pair(match, std::unique_ptr<ValidatorT>(registry.at(match)()));
  });
}
}  // namespace

ValidatorFactory& ValidatorFactory::instance() {
  static ValidatorFactory validatorFactory;
  return validatorFactory;
}

ValidatorsWithName<MapValidator> ValidatorFactory::createMapValidators(const Regexes& regexes) {
  return createValidators(regexes, mapValidatorRegistry_);
}

ValidatorsWithName<TrafficRuleValidator> ValidatorFactory::createTrafficRuleValidators(const Regexes& regexes) {
  return createValidators(regexes, trafficRuleValidatorRegistry_);
}

ValidatorsWithName<RoutingGraphValidator> ValidatorFactory::createRoutingGraphValidators(const Regexes& regexes) {
  return createValidators(regexes, routingGraphValidatorRegistry_);
}

std::vector<std::string> ValidatorFactory::availableValidators(const Regexes& regexes) {
  auto mapValidators = matchValidators(regexes, mapValidatorRegistry_);
  auto ruleValidators = matchValidators(regexes, trafficRuleValidatorRegistry_);
  auto routingValidators = matchValidators(regexes, routingGraphValidatorRegistry_);
  return utils::concatenate({mapValidators, ruleValidators, routingValidators});
}

void ValidatorFactory::registerMapValidator(const std::string& name, const CreationFcn<MapValidator>& creator) {
  mapValidatorRegistry_.emplace(name, creator);
}

void ValidatorFactory::registerTrafficRuleValidator(const std::string& name,
                                                    const CreationFcn<TrafficRuleValidator>& creator) {
  trafficRuleValidatorRegistry_.emplace(name, creator);
}

void ValidatorFactory::registerRoutingGraphValidator(const std::string& name,
                                                     const CreationFcn<RoutingGraphValidator>& creator) {
  routingGraphValidatorRegistry_.emplace(name, creator);
}

}  // namespace validation
}  // namespace lanelet
