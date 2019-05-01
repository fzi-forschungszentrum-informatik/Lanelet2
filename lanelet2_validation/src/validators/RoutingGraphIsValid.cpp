#include "validators/routing/RoutingGraphIsValid.h"
#include "ValidatorFactory.h"
#include "lanelet2_routing/RoutingGraph.h"

namespace lanelet {
namespace validation {
namespace {
RegisterRoutingGraphValidator<RoutingGraphIsValid> reg;
}  // namespace

Issues RoutingGraphIsValid::operator()(const routing::RoutingGraph& graph,
                                       const traffic_rules::TrafficRules& /*rules*/) {
  auto errors = graph.checkValidity(false);
  return utils::transform(errors, [](auto& error) { return Issue(Severity::Error, error); });
}

}  // namespace validation
}  // namespace lanelet
