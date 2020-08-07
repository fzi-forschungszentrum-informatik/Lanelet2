#pragma once

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <algorithm>
#include <unordered_set>

#include "lanelet2_routing/Route.h"
#include "lanelet2_routing/RoutingGraph.h"

namespace lanelet {
namespace routing {

/** @brief Container to associate multiple routing graphs to allow queries on multiple graphs
 *  @note We cannot use the 'conflicting' relations that have been determined when creating the individual graphs
 * because they used their respective height in 3D (e.g. 2m for a pedestrian), but the participant we want to query for
 * could be taller (e.g. 4m truck). Therefore we can't rely on that. */
class RoutingGraphContainer {
 public:
  using ConflictingInGraph = std::pair<size_t, ConstLanelets>;  //!< id of conflicing graph, lanelets in conflict there
  using ConflictingInGraphs = std::vector<ConflictingInGraph>;

  /** @brief Constructor of routing graph container
   *  @param routingGraphs The routing graphs that should be used in the container */
  explicit RoutingGraphContainer(std::vector<RoutingGraphConstPtr> routingGraphs) : graphs_{std::move(routingGraphs)} {}

  /** @brief Constructor of routing graph container
   *  @param routingGraphs The routing graphs that should be used in the container */
  explicit RoutingGraphContainer(const std::vector<RoutingGraphPtr>& routingGraphs)
      : graphs_{utils::transform(routingGraphs, [](auto& g) { return RoutingGraphConstPtr(g); })} {}

  /** @brief Find the conflicting lanelets of a given lanelet within a specified graph
   *  @param lanelet Find conflicting ones for this lanelet
   *  @param routingGraphId ID/position in vector of the routing graph
   *  @param participantHeight Optional height of the participant
   *  @return Conflicting lanelets within that graph
   *  @throws InvalidInputError if the routingGraphId is too high */
  ConstLanelets conflictingInGraph(const ConstLanelet& lanelet, size_t routingGraphId,
                                   double participantHeight = .0) const {
    if (routingGraphId >= graphs_.size()) {
      throw InvalidInputError("Routing Graph ID is higher than the number of graphs.");
    }
    auto overlaps = [lanelet, participantHeight](const ConstLanelet& ll) {
      return participantHeight != .0 ? !geometry::overlaps3d(lanelet, ll, participantHeight)
                                     : !geometry::overlaps2d(lanelet, ll);
    };
    const auto map{graphs_[routingGraphId]->passableSubmap()};
    ConstLanelets conflicting{map->laneletLayer.search(geometry::boundingBox2d(lanelet))};
    auto begin = conflicting.begin();
    auto end = conflicting.end();
    end = std::remove(begin, end, lanelet);
    end = std::remove_if(begin, end, overlaps);
    conflicting.erase(end, conflicting.end());
    return conflicting;
  }

  /** @brief Find the conflicting lanelets of a given lanelet within all graphs
   *  @param lanelet Find conflicting ones for this lanelet
   *  @param participantHeight Optional height of the participant
   *  @return Conflicting lanelets within the graphs */
  ConflictingInGraphs conflictingInGraphs(const ConstLanelet& lanelet, double participantHeight = .0) const {
    ConflictingInGraphs result;
    for (size_t it = 0; it < graphs_.size(); it++) {
      result.emplace_back(std::make_pair(it, conflictingInGraph(lanelet, it, participantHeight)));
    }
    return result;
  }

  /** @brief Find the conflicting lanelets of a given route within a specified graph
   *  @param route Find conflicting lanelets for all lanelets within this route
   *  @param routingGraphId ID/position in vector of the routing graph
   *  @param participantHeight Optional height of the participant
   *  @return Conflicting lanelets within that graph to any lanelet within the route
   *  @throws InvalidInputError if the routingGraphId is too high */
  ConstLanelets conflictingOfRouteInGraph(const Route* route, size_t routingGraphId,
                                          double participantHeight = .0) const {
    std::unordered_set<ConstLanelet> conflicting;
    for (const auto& it : route->laneletSubmap()->laneletLayer) {
      ConstLanelets tempConf{conflictingInGraph(it, routingGraphId, participantHeight)};
      conflicting.insert(tempConf.begin(), tempConf.end());
    }
    return ConstLanelets(conflicting.begin(), conflicting.end());
  }

  /** @brief Find the conflicting lanelets of a given route within all graphs
   *  @param route Find conflicting lanelets for all lanelets within this route
   *  @param participantHeight Optional height of the participant
   *  @return Conflicting lanelets of each graph to any lanelet within the route */
  ConflictingInGraphs conflictingOfRouteInGraphs(const Route* route, double participantHeight = .0) const {
    ConflictingInGraphs result;
    for (size_t it = 0; it < graphs_.size(); it++) {
      result.emplace_back(std::make_pair(it, conflictingOfRouteInGraph(route, it, participantHeight)));
    }
    return result;
  }

  //! Returns the routing graphs stored in the container
  const std::vector<RoutingGraphConstPtr>& routingGraphs() const { return graphs_; }

 private:
  std::vector<RoutingGraphConstPtr> graphs_;  ///< Routing graphs of the container.
};

}  // namespace routing
}  // namespace lanelet
