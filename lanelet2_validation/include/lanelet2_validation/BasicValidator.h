#pragma once
#include <lanelet2_core/Forward.h>
#include <lanelet2_traffic_rules/TrafficRules.h>

#include <memory>

#include "lanelet2_validation/Issue.h"

namespace lanelet {
namespace routing {
class RoutingGraph;
}  // namespace routing

namespace validation {

//! Most simple form of a validator. It gets a map once and reports errors.
class MapValidator {  // NOLINT
 public:
  virtual ~MapValidator() = default;
  constexpr static const char* name() { return ""; }
  virtual Issues operator()(const LaneletMap& map) = 0;
};
using MapValidatorUPtr = std::unique_ptr<MapValidator>;
using MapValidatorUPtrs = std::vector<MapValidatorUPtr>;

//! A traffic rule validator gets a part of the map and the current traffic rules and tries to detect issues there.
//! The object ist not destroyed between subseqent calls with different traffic rules, so that information about already
//! reported issues can be stored.
class TrafficRuleValidator {  // NOLINT
 public:
  constexpr static const char* name() { return ""; }
  virtual Issues operator()(const LaneletMap& map, const std::vector<traffic_rules::TrafficRulesUPtr>& rules) = 0;
  virtual ~TrafficRuleValidator() = default;
};
using TrafficRuleValidatorUPtr = std::unique_ptr<TrafficRuleValidator>;
using TrafficRuleValidatorUPtrs = std::vector<TrafficRuleValidatorUPtr>;

//! A routing graph validator works similar, but instead uses the routing graph of a map to detect issues
class RoutingGraphValidator {  // NOLINTs
 public:
  constexpr static const char* name() { return ""; }
  //! The RoutingGraphValidator is called together with the rules with which it was created.
  virtual Issues operator()(const routing::RoutingGraph& graph, const traffic_rules::TrafficRules& rules) = 0;
  virtual ~RoutingGraphValidator() = default;
};
using RoutingGraphValidatorUPtr = std::unique_ptr<RoutingGraphValidator>;
using RoutingGraphValidatorUPtrs = std::vector<RoutingGraphValidatorUPtr>;

}  // namespace validation
}  // namespace lanelet
