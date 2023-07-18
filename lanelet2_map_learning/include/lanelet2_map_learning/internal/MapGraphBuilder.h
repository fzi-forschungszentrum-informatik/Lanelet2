#pragma once
#include <lanelet2_core/LaneletMap.h>

#include "lanelet2_map_learning/MapGraph.h"
#include "lanelet2_map_learning/internal/Graph.h"

namespace lanelet {
class LaneletLayer;

namespace map_learning {
namespace internal {

class LaneChangeLaneletsCollector;

class MapGraphBuilder {
 public:
  MapGraphBuilder(const traffic_rules::TrafficRules& trafficRules, const MapGraph::Configuration& config);

  MapGraphUPtr build(const LaneletMapLayers& laneletMapLayers);

 private:
  using PointsLaneletMap = std::multimap<IdPair, ConstLanelet>;
  using PointsLaneletMapIt = PointsLaneletMap::iterator;
  using PointsLaneletMapResult = std::pair<PointsLaneletMapIt, PointsLaneletMapIt>;

  static ConstLanelets getPassableLanelets(const LaneletLayer& lanelets,
                                           const traffic_rules::TrafficRules& trafficRules);
  static ConstAreas getPassableAreas(const AreaLayer& areas, const traffic_rules::TrafficRules& trafficRules);
  void appendBidirectionalLanelets(ConstLanelets& llts);
  void addLaneletsToGraph(ConstLanelets& llts);
  void addAreasToGraph(ConstAreas& areas);
  void addEdges(const ConstLanelets& lanelets, const LaneletLayer& passableLanelets);
  void addEdges(const ConstAreas& areas, const LaneletLayer& passableLanelets, const AreaLayer& passableAreas);
  void addFollowingEdges(const ConstLanelet& ll);
  void addSidewayEdge(LaneChangeLaneletsCollector& laneChangeLanelets, const ConstLanelet& ll,
                      const ConstLineString3d& bound, const RelationType& relation);
  void addConflictingEdge(const ConstLanelet& ll, const LaneletLayer& passableLanelets);
  void addLaneChangeEdges(LaneChangeLaneletsCollector& laneChanges, const RelationType& relation);
  void addAreaEdge(const ConstArea& area, const LaneletLayer& passableLanelets);
  void addAreaEdge(const ConstArea& area, const AreaLayer& passableAreas);

  //! Helper function to read the participant height from the configuration
  Optional<double> participantHeight() const;

  //! Adds the first and last points of a lanelet to the search index
  void addPointsToSearchIndex(const ConstLanelet& ll);
  bool hasEdge(const ConstLanelet& from, const ConstLanelet& to);

  void assignLaneChange(ConstLanelets froms, ConstLanelets tos, const RelationType& relation);
  /** @brief Assigns edge to a relation between two lanelets
   *  @param from Start lanelet
   *  @param to Goal lanelet
   *  @param relation Relation between the two lanelets */
  void assignEdge(const ConstLaneletOrArea& from, const ConstLaneletOrArea& to, const RelationType& relation);

  std::unique_ptr<MapGraphGraph> graph_;
  PointsLaneletMap pointsToLanelets_;  ///< A map of tuples (first or last left and right boundary points) to lanelets
  std::set<Id> bothWaysLaneletIds_;
  const traffic_rules::TrafficRules& trafficRules_;
  const MapGraph::Configuration& config_;
};
}  // namespace internal
}  // namespace map_learning
}  // namespace lanelet
