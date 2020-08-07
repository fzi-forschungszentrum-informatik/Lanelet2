#pragma once
#include <lanelet2_core/utility/Utilities.h>

#include <functional>
#include <regex>

#include "lanelet2_validation/BasicValidator.h"

namespace lanelet {
namespace validation {

using Regexes = std::vector<std::regex>;

template <typename ValidatorT>
using ValidatorWithName = std::pair<std::string, std::unique_ptr<ValidatorT>>;

template <typename ValidatorT>
using ValidatorsWithName = std::vector<ValidatorWithName<ValidatorT>>;

class ValidatorFactory {
 public:
  template <typename ValidatorT>
  using CreationFcn = std::function<ValidatorT*()>;
  static ValidatorFactory& instance();

  ValidatorsWithName<MapValidator> createMapValidators(const Regexes& regexes);
  ValidatorsWithName<TrafficRuleValidator> createTrafficRuleValidators(const Regexes& regexes);
  ValidatorsWithName<RoutingGraphValidator> createRoutingGraphValidators(const Regexes& regexes);

  /**
   * @brief returns all available parsers as vector
   * @return vector of parser names that match the regex, or all if empty
   */
  std::vector<std::string> availableValidators(const Regexes& regexes = {});

  template <typename ValidatorT>
  friend class RegisterMapValidator;

  template <typename ValidatorT>
  friend class RegisterTrafficRuleValidator;

  template <typename ValidatorT>
  friend class RegisterRoutingGraphValidator;

 private:
  void registerMapValidator(const std::string& name, const CreationFcn<MapValidator>& creator);
  void registerTrafficRuleValidator(const std::string& name, const CreationFcn<TrafficRuleValidator>& creator);
  void registerRoutingGraphValidator(const std::string& name, const CreationFcn<RoutingGraphValidator>& creator);

  ValidatorFactory() = default;
  std::map<std::string, CreationFcn<MapValidator>> mapValidatorRegistry_;
  std::map<std::string, CreationFcn<TrafficRuleValidator>> trafficRuleValidatorRegistry_;
  std::map<std::string, CreationFcn<RoutingGraphValidator>> routingGraphValidatorRegistry_;
};

/**
 * @brief Registration object for a map validator. Needs to be instanciated as static
 * object once to register a writer. Registration might look like this:
 *   static RegisterWriter<Mywriter> register;
 */
template <class T>
class RegisterMapValidator {
 public:
  RegisterMapValidator() {
    static_assert(!utils::strequal(T::name(), ""), "You did not overload the name() function!");
    ValidatorFactory::instance().registerMapValidator(T::name(), []() { return new T(); });
  }
};

//! Registration object for traffic rule validators
template <class T>
class RegisterTrafficRuleValidator {
 public:
  RegisterTrafficRuleValidator() {
    static_assert(!utils::strequal(T::name(), ""), "You did not overload the name() function!");
    ValidatorFactory::instance().registerTrafficRuleValidator(T::name(), []() { return new T(); });
  }
};

//! Registration object for routing graph validators
template <class T>
class RegisterRoutingGraphValidator {
 public:
  RegisterRoutingGraphValidator() {
    static_assert(!utils::strequal(T::name(), ""), "You did not overload the name() function!");
    ValidatorFactory::instance().registerRoutingGraphValidator(T::name(), []() { return new T(); });
  }
};
}  // namespace validation
}  // namespace lanelet
