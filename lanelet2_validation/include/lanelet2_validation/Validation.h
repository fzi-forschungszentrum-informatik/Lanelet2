#pragma once
#include <lanelet2_io/Projection.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include "lanelet2_validation/Issue.h"

namespace lanelet {
namespace validation {

using Strings = std::vector<std::string>;

//! Struct for the detected issues that were discovered by a specific check
struct DetectedIssues {
  DetectedIssues() = default;
  DetectedIssues(std::string checkName, Issues issues) : checkName{std::move(checkName)}, issues{std::move(issues)} {}
  Issues errors() const;
  Issues warnings() const;
  std::string checkName;
  Issues issues;
};

//! Configuration object for the validation
struct ValidationConfig {
  std::string checksFilter;
  std::string location{Locations::Germany};
  Strings participants{Participants::Vehicle};
  GPSPoint origin;
};

//! Contains each warning/error as formatted strings
struct IssueReport {
  Strings warnings;
  Strings errors;
};

//! Generates the issue report
IssueReport buildReport(std::vector<DetectedIssues> issues);

//! Reports the available checks for the given filter. Empty will return all.
Strings availabeChecks(const std::string& filterString);

std::vector<DetectedIssues> validateMap(LaneletMap& map, const ValidationConfig& config);

//! Central function here. Loads the map, runs the checks and returns the issues.
std::vector<DetectedIssues> validateMap(const std::string& mapFilename, const ValidationConfig& config);
}  // namespace validation
}  // namespace lanelet
