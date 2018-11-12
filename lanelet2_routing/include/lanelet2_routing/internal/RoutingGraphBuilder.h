#pragma once
#include <lanelet2_core/LaneletMap.h>
#include "Graph.h"
#include "RoutingGraph.h"

namespace lanelet {
class LaneletLayer;

namespace routing {

class LaneChangeLaneletsCollector;

class RoutingGraphBuilder {
 public:
  RoutingGraphBuilder(const traffic_rules::TrafficRules& trafficRules, const RoutingCostPtrs& routingCosts,
                      const RoutingGraph::Configuration& config);

  RoutingGraphUPtr build(LaneletMap& laneletMap);

 private:
  using PointsLaneletMap = std::multimap<IdPair, ConstLanelet>;
  using PointsLaneletMapIt = PointsLaneletMap::iterator;
  using PointsLaneletMapResult = std::pair<PointsLaneletMapIt, PointsLaneletMapIt>;

  Lanelets getPassableLanelets(LaneletLayer& lanelets, const traffic_rules::TrafficRules& trafficRules);
  Areas getPassableAreas(AreaLayer& areas, const traffic_rules::TrafficRules& trafficRules);
  void appendBidirectionalLanelets(Lanelets& llts);
  void addLaneletsToGraph(Lanelets& llts);
  void addAreasToGraph(Areas& areas);
  void addEdges(const Lanelets& lanelets, const LaneletLayer& passableLanelets);
  void addEdges(const Areas& areas, const LaneletLayer& passableLanelets, const AreaLayer& passableAreas);
  void addFollowingEdges(const Lanelet& ll);
  void addSidewayEdge(LaneChangeLaneletsCollector& laneChangeLanelets, const Lanelet& ll,
                      const ConstLineString3d& bound, const RelationType& relation);
  void addConflictingEdge(const Lanelet& ll, const LaneletLayer& passableLanelets);
  void addLaneChangeEdges(LaneChangeLaneletsCollector& laneChanges, const RelationType& relation);
  void addAreaEdge(const Area& area, const LaneletLayer& passableLanelets);
  void addAreaEdge(const Area& area, const AreaLayer& passableAreas);

  //! Helper function to read the participant height from the configuration
  Optional<double> participantHeight() const;

  //! Adds the first and last points of a lanelet to the search index
  void addPointsToSearchIndex(const ConstLanelet& ll);
  bool hasEdge(const ConstLanelet& from, const ConstLanelet& to);
  void assignLaneChangeCosts(const ConstLanelets& froms, const ConstLanelets& tos, const RelationType& relation);

  /** @brief Assigns routing costs of each routing cost module to a relation between two lanelets
   *  @param from Start lanelet
   *  @param to Goal lanelet
   *  @param relation Relation between the two lanelets */
  void assignCosts(const ConstLaneletOrArea& from, const ConstLaneletOrArea& to, const RelationType& relation);
  std::unique_ptr<Graph> graph_;
  PointsLaneletMap pointsToLanelets_;  ///< A map of tuples (first or last left and right boundary points) to lanelets
  std::set<Id> bothWaysLaneletIds_;
  const traffic_rules::TrafficRules& trafficRules_;
  const RoutingCostPtrs& routingCosts_;
  const RoutingGraph::Configuration& config_;
};
}  // namespace routing
}  // namespace lanelet
