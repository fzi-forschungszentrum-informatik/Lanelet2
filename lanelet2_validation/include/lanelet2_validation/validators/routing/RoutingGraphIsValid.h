#include "lanelet2_validation/BasicValidator.h"

namespace lanelet {
namespace validation {

//! This check basically calls the RoutingGraph self-check to ensure the graph is valid.
class RoutingGraphIsValid : public RoutingGraphValidator {
 public:
  constexpr static const char* name() { return "routing.graph_is_valid"; }
  Issues operator()(const routing::RoutingGraph& graph, const traffic_rules::TrafficRules& /*rules*/) override;
};

}  // namespace validation
}  // namespace lanelet
