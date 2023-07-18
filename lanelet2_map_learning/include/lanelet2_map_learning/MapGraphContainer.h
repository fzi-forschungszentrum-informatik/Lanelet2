#pragma once

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <algorithm>
#include <unordered_set>

#include "lanelet2_map_learning/MapGraph.h"

namespace lanelet {
namespace map_learning {

/** @brief Container to associate multiple graphs to allow queries on multiple graphs
 *  @note We cannot use the 'conflicting' relations that have been determined when creating the individual graphs
 * because they used their respective height in 3D (e.g. 2m for a pedestrian), but the participant we want to query for
 * could be taller (e.g. 4m truck). Therefore we can't rely on that. */
class MapGraphContainer {
 public:
  using ConflictingInGraph = std::pair<size_t, ConstLanelets>;  //!< id of conflicing graph, lanelets in conflict there
  using ConflictingInGraphs = std::vector<ConflictingInGraph>;

  /** @brief Constructor of graph container
   *  @param MapGraphs The graphs that should be used in the container */
  explicit MapGraphContainer(std::vector<MapGraphConstPtr> MapGraphs) : graphs_{std::move(MapGraphs)} {}

  /** @brief Constructor of graph container
   *  @param MapGraphs The graphs that should be used in the container */
  explicit MapGraphContainer(const std::vector<MapGraphPtr>& MapGraphs)
      : graphs_{utils::transform(MapGraphs, [](auto& g) { return MapGraphConstPtr(g); })} {}

  /** @brief Find the conflicting lanelets of a given lanelet within a specified graph
   *  @param lanelet Find conflicting ones for this lanelet
   *  @param MapGraphId ID/position in vector of the graph
   *  @param participantHeight Optional height of the participant
   *  @return Conflicting lanelets within that graph
   *  @throws InvalidInputError if the MapGraphId is too high */
  ConstLanelets conflictingInGraph(const ConstLanelet& lanelet, size_t MapGraphId,
                                   double participantHeight = .0) const {
    if (MapGraphId >= graphs_.size()) {
      throw InvalidInputError("Graph ID is higher than the number of graphs.");
    }
    auto overlaps = [lanelet, participantHeight](const ConstLanelet& ll) {
      return participantHeight != .0 ? !geometry::overlaps3d(lanelet, ll, participantHeight)
                                     : !geometry::overlaps2d(lanelet, ll);
    };
    const auto map{graphs_[MapGraphId]->passableSubmap()};
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

  //! Returns the graphs stored in the container
  const std::vector<MapGraphConstPtr>& MapGraphs() const { return graphs_; }

 private:
  std::vector<MapGraphConstPtr> graphs_;  ///< Map graphs of the container.
};

}  // namespace map_learning
}  // namespace lanelet
