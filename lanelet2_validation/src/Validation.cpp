#include "lanelet2_validation/Validation.h"

#include <lanelet2_core/primitives/GPSPoint.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include "lanelet2_validation/ValidatorFactory.h"

namespace lanelet {
namespace validation {
namespace {
Regexes parseFilterString(const std::string& str) {
  Regexes regexes;
  std::stringstream ss(str);

  while (ss.good()) {
    std::string substr;
    getline(ss, substr, ',');
    if (substr.empty()) {
      continue;
    }
    regexes.emplace_back(substr, std::regex::basic | std::regex::icase);
  }
  return regexes;
}

template <typename Container1T, typename Container2T>
void append(Container1T& to, Container2T&& from) {
  to.insert(to.end(), std::make_move_iterator(from.begin()), std::make_move_iterator(from.end()));
}

template <typename ValidatorT, typename Func>
std::vector<DetectedIssues> runValidators(const ValidatorsWithName<ValidatorT>& validators, Func f) {
  std::vector<DetectedIssues> issues;
  issues.reserve(validators.size());
  for (auto& validator : validators) {
    auto foundIssues = f(*validator.second);
    if (!foundIssues.empty()) {
      issues.emplace_back(validator.first, std::move(foundIssues));
    }
  }
  return issues;
}

void runMapValidators(std::vector<DetectedIssues>& issues, const Regexes& regexes, const LaneletMap& map) {
  auto validators = ValidatorFactory::instance().createMapValidators(regexes);
  append(issues, runValidators(validators, [&map](auto& validator) { return validator(map); }));
}

void runRuleValidators(std::vector<DetectedIssues>& issues, const Regexes& regexes, const LaneletMap& map,
                       const std::vector<traffic_rules::TrafficRulesUPtr>& rules) {
  auto validators = ValidatorFactory::instance().createTrafficRuleValidators(regexes);
  append(issues, runValidators(validators, [&map, &rules](auto& validator) { return validator(map, rules); }));
}

void runRoutingGraphValidators(std::vector<DetectedIssues>& issues, const Regexes& regexes, LaneletMap& map,
                               const std::vector<traffic_rules::TrafficRulesUPtr>& rules) {
  auto routingGraphValidators = ValidatorFactory::instance().createRoutingGraphValidators(regexes);
  if (routingGraphValidators.empty()) {
    return;
  }
  for (const auto& rule : rules) {
    routing::RoutingGraphPtr routingGraph;
    try {
      routingGraph = routing::RoutingGraph::build(map, *rule);
    } catch (LaneletError& err) {
      std::stringstream msg;
      msg << "Failed to create routing graph for " << *rule << ": " << err.what();
      issues.emplace_back("general", Issues{Issue(Severity::Error, msg.str())});
      continue;
    }
    append(issues, runValidators(routingGraphValidators,
                                 [&routingGraph, &rule](auto& validator) { return validator(*routingGraph, *rule); }));
  }
}
}  // namespace

Issues DetectedIssues::errors() const {
  Issues errors;
  for (const auto& issue : issues) {
    if (issue.severity == Severity::Error) {
      errors.push_back(issue);
    }
  }
  return errors;
}

Issues DetectedIssues::warnings() const {
  Issues warning;
  for (const auto& issue : issues) {
    if (issue.severity == Severity::Warning) {
      warning.push_back(issue);
    }
  }
  return warning;
}

Strings availabeChecks(const std::string& filterString) {
  return ValidatorFactory::instance().availableValidators(parseFilterString(filterString));
}

IssueReport buildReport(std::vector<DetectedIssues> issues) {
  IssueReport report;
  for (auto& issue : issues) {
    auto buildReports = [&check = issue.checkName](auto& issue) { return issue.buildReport() + " [" + check + "]"; };
    auto errorsFromCheck = utils::transform(issue.errors(), buildReports);
    if (!errorsFromCheck.empty()) {
      report.errors.insert(report.errors.end(), errorsFromCheck.begin(), errorsFromCheck.end());
    }
    auto warningsFromCheck = utils::transform(issue.warnings(), buildReports);
    if (!warningsFromCheck.empty()) {
      report.warnings.insert(report.warnings.end(), warningsFromCheck.begin(), warningsFromCheck.end());
    }
  }
  return report;
}

std::vector<DetectedIssues> validateMap(LaneletMap& map, const ValidationConfig& config) {
  std::vector<DetectedIssues> issues;
  Regexes regexes = parseFilterString(config.checksFilter);

  runMapValidators(issues, regexes, map);

  auto trafficRules = utils::transform(config.participants, [&config](auto& participant) {
    return traffic_rules::TrafficRulesFactory::create(config.location, participant);
  });

  runRuleValidators(issues, regexes, map, trafficRules);
  runRoutingGraphValidators(issues, regexes, map, trafficRules);
  return issues;
}

std::vector<DetectedIssues> validateMap(const std::string& mapFilename, const ValidationConfig& config) {
  using namespace std::string_literals;
  auto checks = parseFilterString(config.checksFilter);
  auto projector = lanelet::projection::UtmProjector(Origin{config.origin});
  std::vector<DetectedIssues> issues;
  LaneletMapPtr map;
  try {
    Strings errors;
    map = lanelet::load(mapFilename, projector, &errors);
    if (!errors.empty()) {
      issues.emplace_back("general",
                          utils::transform(errors, [](auto& error) { return Issue(Severity::Error, error); }));
    }
  } catch (lanelet::LaneletError& err) {
    return {{"general", {Issue(Severity::Error, "Failed to load map: "s + err.what())}}};
  }
  append(issues, validateMap(*map, config));
  return issues;
}

}  // namespace validation
}  // namespace lanelet
