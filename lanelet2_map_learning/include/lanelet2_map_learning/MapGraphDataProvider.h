#pragma once

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/utility/Optional.h>

#include "lanelet2_map_learning/Forward.h"
#include "lanelet2_map_learning/MapGraph.h"
#include "lanelet2_map_learning/internal/Graph.h"

namespace lanelet {
class LaneletLayer;

namespace map_learning {

struct TensorGraphData {
  Eigen::MatrixXd x;   // node features
  Eigen::MatrixX2i a;  // adjacency matrix (sparse)
  Eigen::MatrixXd e;   // edge features
};

class MapGraphDataProvider {
 public:
  enum class LaneletRepresentationType { Centerline, Boundaries };
  enum class ParametrizationType { Bezier, BezierEndpointFixed, Polyline8Pts, Polyline11Pts };

  struct Configuration {
    Configuration() noexcept {}
    LaneletRepresentationType reprType{LaneletRepresentationType::Boundaries};
    ParametrizationType paramType{ParametrizationType::Polyline11Pts};
    double submapAreaX{100};
    double submapAreaY{100};
  };

  MapGraphDataProvider(LaneletMapConstPtr laneletMap, Configuration config = Configuration(),
                       Optional<BasicPoint2d> currPos = boost::none);

  void setCurrPosAndExtractSubmap(const BasicPoint2d& pt);
  TensorGraphData laneLaneTensors();
  TensorGraphData laneTETensors();

  TensorGraphData laneLaneTensorsBatch(const BasicPoints2d& pts);
  TensorGraphData laneTETensorsBatch(const BasicPoints2d& pts);

 private:
  LaneletMapConstPtr laneletMap_;
  LaneletSubmapConstPtr localSubmap_;
  MapGraphConstPtr localSubmapGraph_;
  Optional<BasicPoint2d> currPos_;  // in the map frame
  Configuration config_;
  traffic_rules::TrafficRulesPtr trafficRules_;
};

}  // namespace map_learning
}  // namespace lanelet