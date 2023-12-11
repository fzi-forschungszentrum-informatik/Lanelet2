#pragma once
#include <lanelet2_core/Exceptions.h>
#include <lanelet2_core/Forward.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_routing/Forward.h>

#include <boost/geometry.hpp>
#include <type_traits>

#include "lanelet2_map_learning/Forward.h"
#include "lanelet2_map_learning/MapFeatures.h"
#include "lanelet2_map_learning/Types.h"
#include "lanelet2_routing/RoutingGraph.h"
#include "lanelet2_routing/internal/Graph.h"

namespace lanelet {
namespace map_learning {

struct Edge {
  Edge(Id el1, Id el2) : el1_{el1}, el2_{el2} {}
  Id el1_;
  Id el2_;
};

namespace internal {
struct CompoundElsList {
  CompoundElsList(const Id& start, LineStringType type) : ids{start}, type{type} {}
  CompoundElsList(const std::vector<Id>& ids, LineStringType type) : ids{ids}, type{type} {}
  std::vector<Id> ids;
  LineStringType type;
};
}  // namespace internal

using Edges = std::vector<Edge>;

class LaneData {
 public:
  LaneData() noexcept {}
  static LaneData build(LaneletSubmapConstPtr& localSubmap, lanelet::routing::RoutingGraphConstPtr localSubmapGraph);
  bool processAll(const OrientedRect& bbox, const ParametrizationType& paramType, int32_t nPoints);

  const LaneLineStringFeatures& roadBorders() { return roadBorders_; }
  const LaneLineStringFeatures& laneDividers() { return laneDividers_; }

  const CompoundLaneLineStringFeatureList& compoundRoadBorders() { return compoundRoadBorders_; }
  const CompoundLaneLineStringFeatureList& compoundLaneDividers() { return compoundLaneDividers_; }
  const CompoundLaneLineStringFeatureList& compoundCenterlines() { return compoundCenterlines_; }

  const LaneletFeatures& laneletFeatures() { return laneletFeatures_; }
  const Edges& edgeList() { return edgeList_; }

 private:
  void initLeftBoundaries(LaneletSubmapConstPtr& localSubmap, lanelet::routing::RoutingGraphConstPtr localSubmapGraph);
  void initRightBoundaries(LaneletSubmapConstPtr& localSubmap, lanelet::routing::RoutingGraphConstPtr localSubmapGraph);
  void initLaneletFeatures(LaneletSubmapConstPtr& localSubmap, lanelet::routing::RoutingGraphConstPtr localSubmapGraph);
  void initCompoundFeatures(LaneletSubmapConstPtr& localSubmap,
                            lanelet::routing::RoutingGraphConstPtr localSubmapGraph);

  LineStringType getLineStringTypeFromId(Id id);
  LaneLineStringFeature getLineStringFeatFromId(Id id);
  std::vector<internal::CompoundElsList> computeCompoundLeftBorders(const ConstLanelets& path);
  std::vector<internal::CompoundElsList> computeCompoundRightBorders(const ConstLanelets& path);
  CompoundLaneLineStringFeature computeCompoundCenterline(const ConstLanelets& path);

  LaneLineStringFeatures roadBorders_;   // auxilliary features
  LaneLineStringFeatures laneDividers_;  // auxilliary features

  CompoundLaneLineStringFeatureList compoundRoadBorders_;   // auxilliary features
  CompoundLaneLineStringFeatureList compoundLaneDividers_;  // auxilliary features
  CompoundLaneLineStringFeatureList compoundCenterlines_;

  LaneletFeatures laneletFeatures_;  // node features
  Edges edgeList_;                   // edge list for centerlines
};

class TEData {
 public:
  TEData() noexcept {}

 private:
  LaneletFeatures laneletFeatures;  // node features lanelets
  TEFeatures teFeatures;            // node features traffic elements

  Edges edgeList_;  // edge list
  /// @brief adjacency matrix (sparse). Node indices are assigned as
  ///        xLane and xTE stacked, with xLane being first
};

}  // namespace map_learning
}  // namespace lanelet