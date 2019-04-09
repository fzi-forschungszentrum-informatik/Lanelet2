#pragma once

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/utility/Optional.h>
#include <map>
#include <set>
#include "Forward.h"
#include "LaneletPath.h"
#include "RouteElement.h"
#include "RoutingCost.h"

namespace lanelet {
namespace routing {
struct EdgeInfo;

/** @brief Main class of the routing module that holds routing information and can be queried.
 *  The RoutingGraph class is the central object of this module and is initialized with a LaneletMap, TrafficRules and
 * RoutingCost.
 *  A routing graph with all interesting relations will be created for the traffic participant featured in the provided
 * TrafficRules module. Routing costs will be calculated for each provided module. The routing graph can answer queries
 * like "left", "following", "conflicting" lanelets, but also provide a shortest route or a Route.
 *
 * @note The direction of lanelets matters! 'lanelet' and 'lanelet.invert()' are differentiated since this matters when
 * lanelets are passable in both directions.
 * @note 'adjacent_left' and 'adjacent_right' means that there is a passable lanelet left/right of another passable
 * lanelet, but a lane change is not allowed. */
class RoutingGraph {
  using PointsLaneletMap = std::multimap<IdPair, ConstLanelet>;
  using PointsLaneletMapIt = PointsLaneletMap::iterator;
  using PointsLaneletMapResult = std::pair<PointsLaneletMapIt, PointsLaneletMapIt>;

 public:
  using Errors = std::vector<std::string>;                 ///< For the checkValidity function
  using Configuration = std::map<std::string, Attribute>;  ///< Used to provide a configuration
  //! Defined configuration attributes
  static constexpr const char ParticipantHeight[] = "participant_height";

  /** @brief Main constructor with optional configuration.
   *  @param laneletMap Map that should be used to build the graph
   *  @param trafficRules Traffic rules that apply to find passable lanelets
   *  @param routingCosts One or more ways to calculate routing costs
   *  @param config Optional configuration */
  static RoutingGraphUPtr build(const LaneletMap& laneletMap, const traffic_rules::TrafficRules& trafficRules,
                                const RoutingCostPtrs& routingCosts = defaultRoutingCosts(),
                                const Configuration& config = Configuration());

  //! The graph can not be copied, only moved
  RoutingGraph() = delete;
  RoutingGraph(const RoutingGraph&) = delete;
  RoutingGraph& operator=(const RoutingGraph&) = delete;
  RoutingGraph(RoutingGraph&& /*other*/) noexcept;
  RoutingGraph& operator=(RoutingGraph&& /*other*/) noexcept;
  ~RoutingGraph();

  /** @brief Get a driving route from 'start' to 'end' lanelet
   *  @param from Start lanelet to find a shortest path
   *  @param to End lanelet to find a shortest path
   *  @param routingCostId ID of RoutingCost module to determine shortest route
   *  @return A route of all adjacent lanelets of the shortest route that lead to 'end'. Nothing if there's not
   * route.
   *  @see Route */
  Optional<Route> getRoute(const ConstLanelet& from, const ConstLanelet& to, RoutingCostId routingCostId = {}) const;

  /** @brief Get a driving route from 'start' to 'end' lanelets while going via other lanelets
   *  @param from Start lanelet to find a shortest path
   *  @param via Other lanelets to visit (in this order)
   *  @param to End lanelet to find a shortest path
   *  @param routingCostId ID of RoutingCost module to determine shortest route
   *  @return A route of all adjacent lanelets of the shortest route that lead to 'end'. Nothing if there's not
   * route.
   *  @see Route */
  Optional<Route> getRouteVia(const ConstLanelet& from, const ConstLanelets& via, const ConstLanelet& to,
                              RoutingCostId routingCostId = {}) const;

  /** @brief Retrieve a shortest path between 'start' and 'end'.
   *
   *  Will find a shortest path using Djikstra's shortest path algorithm and the routing cost calculated by the
   * routing cost module with the respective ID. Be aware that the shortest path may contain lane changes,
   * i.e. lanelets that are parallel and not only adjacent.
   *  @param from Start lanelet to find a shortest path
   *  @param to End lanelet to find a shortest path
   *  @param routingCostId ID of RoutingCost module to determine shortest path */
  Optional<LaneletPath> shortestPath(const ConstLanelet& from, const ConstLanelet& to,
                                     RoutingCostId routingCostId = {}) const;

  /** @brief Retrieve a shortest path between 'start' and 'end' using intermediate points.
   *  Will find a shortest path using Djikstra's shortest path algorithm and the routing cost calculated by the
   * routing cost module with the respective ID. Be aware that the shortest path may contain lane changes,
   * i.e. lanelets that are parallel and not only adjacent.
   *  @param start Start lanelet to find a shortest path
   *  @param via Intermediate lanelets that have to be passed
   *  @param end End lanelet to find a shortest path
   *  @param routingCostId ID of RoutingCost module to determine shortest path
   *  @see shortestPath */
  Optional<LaneletPath> shortestPathVia(const ConstLanelet& start, const ConstLanelets& via, const ConstLanelet& end,
                                        RoutingCostId routingCostId = {}) const;

  /** @brief Determines the relation between two lanelets
   *  @param from Start lanelet
   *  @param to Goal lanelet
   *  @param includeConflicting if false, conflicting lanelets are not considered as relations
   *  @return Relation between the lanelets or nothing if no relation exists. Nothing if at least one of the lanelets
   * is not passable. */
  Optional<RelationType> routingRelation(const ConstLanelet& from, const ConstLanelet& to,
                                         bool includeConflicting = false) const;

  /** @brief Returns the lanelets that can be reached from this lanelet.
   *  @param lanelet Start lanelet
   *  @param withLaneChanges Include left and right lanes or not
   *  @return Lanelets that can be directly reached
   *  @see followingRelations */
  ConstLanelets following(const ConstLanelet& lanelet, bool withLaneChanges = false) const;

  /** @brief Returns the lanelets that can be reached from this lanelet and the relation.
   *  @param lanelet Start lanelet
   *  @param withLaneChanges Include left and right lanes or not
   *  @return Lanelets can be directly reached
   *  @see following */
  LaneletRelations followingRelations(const ConstLanelet& lanelet, bool withLaneChanges = false) const;

  /** @brief Returns the possible previous lanelets of this lanelet.
   *  @param lanelet Start lanelet
   *  @param withLaneChanges Include left and right lanes or not
   *  @return All previous lanelets
   *  @see previousRelations */
  ConstLanelets previous(const ConstLanelet& lanelet, bool withLaneChanges = false) const;

  /** @brief Returns the possible previous lanelets of this lanelet and the relation.
   *  @param lanelet Start lanelet
   *  @param withLaneChanges Include left and right lanes or not
   *  @return Lanelets that could be used to reach this lanelet
   *  @see previous */
  LaneletRelations previousRelations(const ConstLanelet& lanelet, bool withLaneChanges = false) const;

  /** @brief Retrieve all reachable left and right lanelets
   *  @param lanelet Start lanelet
   *  @return All left and right lanelets that can be reached, including lanelet, ordered left to right. */
  ConstLanelets besides(const ConstLanelet& lanelet) const;

  /** @brief Get left (routable) lanelet of a given lanelet if it exists.
   *  @param lanelet Start lanelet
   *  @return Left lanelet if it exists. Nothing if it doesn't.
   *  @see adjacentLeft, lefts, adjacentLefts */
  Optional<ConstLanelet> left(const ConstLanelet& lanelet) const;

  /** @brief Get adjacent left (non-routable) lanelet of a given lanelet if it exists.
   *  @param lanelet Start lanelet
   *  @return Adjacent left lanelet if it exists. Nothing if it doesn't.
   *  @see left, lefts, adjacentLefts */
  Optional<ConstLanelet> adjacentLeft(const ConstLanelet& lanelet) const;

  /** @brief Get right (routable) lanelet of a given lanelet if it exists.
   *  @param lanelet Start lanelet
   *  @return Right lanelet if it exists. Nothing if it doesn't.
   *  @see adjacentRight, rights, adjacentRights */
  Optional<ConstLanelet> right(const ConstLanelet& lanelet) const;

  /** @brief Get adjacent right (non-routable) lanelet of a given lanelet if it exists.
   *  @param lanelet Start lanelet
   *  @return Adjacent right lanelet if it exists. Nothing if it doesn't.
   *  @see right, rights, adjacentRights */
  Optional<ConstLanelet> adjacentRight(const ConstLanelet& lanelet) const;

  /** @brief Get all left (routable) lanelets of a given lanelet if they exist.
   *  @param lanelet Start lanelet
   *  @return Left lanelets if they exists. Empty if they don't.
   *  @see adjacentLeft, left, adjacentLefts */
  ConstLanelets lefts(const ConstLanelet& lanelet) const;

  /** @brief Get all adjacent left (non-routable) lanelets of a given lanelet if they exist.
   *  @param lanelet Start lanelet
   *  @return Adjacent left lanelets if they exists. Empty if they don't.
   *  @see adjacentLeft, left, lefts */
  ConstLanelets adjacentLefts(const ConstLanelet& lanelet) const;

  /** @brief Retrieve all lanelets and relations left of a given lanelet.
   *  @param lanelet Start lanelet
   *  @return All lanelets and relations left of a given lanelet.
   *  @see lefts, adjacentLefts */
  LaneletRelations leftRelations(const ConstLanelet& lanelet) const;

  /** @brief Get all right (routable) lanelets of a given lanelet if they exist.
   *  @param lanelet Start lanelet
   *  @return Right lanelets if they exists. Empty if they don't.
   *  @see adjacentRight, right, adjacentRights */
  ConstLanelets rights(const ConstLanelet& lanelet) const;

  /** @brief Get all adjacent right (non-routable) lanelets of a given lanelet if they exist.
   *  @param lanelet Start lanelet
   *  @return Adjacent right lanelets if they exists. Empty if they don't.
   *  @see adjacentRight, right, rights */
  ConstLanelets adjacentRights(const ConstLanelet& lanelet) const;

  /** @brief Retrieve all lanelets and relations right of a given lanelet.
   *  @param lanelet Start lanelet
   *  @return All lanelets and relations right of a given lanelet.
   *  @see rights, adjacentRights */
  LaneletRelations rightRelations(const ConstLanelet& lanelet) const;

  /** @brief Retrieve all lanelets that are conflicting with the given lanelet.
   *
   *  Conflicting means that their bounding boxes overlap and the height clearance is smaller than the specified
   * "participant_height".
   *  @param laneletOrArea Lanelet/Area to get conflicting lanelets for.
   *  @return All conflicting lanelets. */
  ConstLaneletOrAreas conflicting(const ConstLaneletOrArea& laneletOrArea) const;

  /** @brief Retrieve a set of lanelets that can be reached from a given lanelet
   *
   *  Determines which lanelets can be reached from a give start lanelets within a given amount of routing cost.
   *  @param lanelet Start lanelet
   *  @param maxRoutingCost Maximum amount of routing cost allowed to reach other lanelets
   *  @param routingCostId ID of the routing cost module used for routing cost.
   *  @return all lanelets that are reachable in no particular orders. "lanelet" itself is always included. */
  ConstLanelets reachableSet(const ConstLanelet& lanelet, double maxRoutingCost,
                             RoutingCostId routingCostId = {}) const;

  //! Retrieve set of lanelet or areas that are reachable without exceeding routing cost.
  ConstLaneletOrAreas reachableSetIncludingAreas(const ConstLaneletOrArea& llOrAr, double maxRoutingCost,
                                                 RoutingCostId routingCostId = {}) const;

  /** @brief Determines possible routes from a given start lanelet that are "minRoutingCost"-long.
   *
   *  @return possible paths that are at least as long as specified in 'minRoutingCost'. If a lanelet can be reached
   * using different paths, only the one is included that requires the least number of lane changes and has minimum
   * routing costs. "lanelet" itself is always included.
   *  @param startPoint Start lanelet
   *  @param minRoutingCost Costs that must be reached by a route.
   *  @param routingCostId ID of the routing cost module used
   *  @param allowLaneChanges Allow or forbid lane changes */
  LaneletPaths possiblePaths(const ConstLanelet& startPoint, double minRoutingCost, RoutingCostId routingCostId = {},
                             bool allowLaneChanges = false) const;

  /** @brief Determines possible paths from a given start lanelet that are "minLanelets"-long.
   *  Returns possible routes that are at least as long as specified number of lanelets. It is possible to forbid
   * lane changes in order to avoid routes that alternate between two neighboring lanelets.
   *  @param startPoint Start lanelet
   *  @param minLanelets Number of lanelets a route must be long
   *  @param allowLaneChanges Allow or forbid lane changes. Note that "lane changes" from a lanelet to an area do not
   * count. */
  LaneletPaths possiblePaths(const ConstLanelet& startPoint, uint32_t minLanelets, bool allowLaneChanges = false) const;

  //! Similar to RoutingGraph::possiblePaths, but also considers areas.
  LaneletOrAreaPaths possiblePathsIncludingAreas(const ConstLaneletOrArea& startPoint, double minRoutingCost,
                                                 RoutingCostId routingCostId = {}, bool allowLaneChanges = false) const;

  //! Similar to RoutingGraph::possiblePaths, but also considers areas.
  LaneletOrAreaPaths possiblePathsIncludingAreas(const ConstLaneletOrArea& startPoint, uint32_t minElements,
                                                 bool allowLaneChanges = false) const;

  /** @brief Export the internal graph to graphML (xml-based) file format.
   *  @param filename Fully qualified file name - ideally with extension (.graphml)
   *  @param edgeTypesToExclude Exclude the specified relations. E.g. conflicting
   *  @param routingCostId ID of the routing cost module
      @see exportGraphViz */
  void exportGraphML(const std::string& filename, const RelationTypes& edgeTypesToExclude = RelationTypes(),
                     RoutingCostId routingCostId = {}) const;

  /** @brief Export the internal graph to graphViz (DOT) file format.
   *  This format includes coloring of the edges in the graph and bears little more information than graphML export.
   *  @param filename Fully qualified file name - ideally with extension (.gv)
   *  @param edgeTypesToExclude Exclude the specified relations. E.g. conflicting
   *  @param routingCostId ID of the routing cost module */
  void exportGraphViz(const std::string& filename, const RelationTypes& edgeTypesToExclude = RelationTypes(),
                      RoutingCostId routingCostId = {}) const;

  /** @brief An abstract lanelet map holding the information of the routing graph.
   *  A good way to view the routing graph since it can be exported using the lanelet2_io module and there can be
   * viewed in tools like JOSM. Each lanelet is represented by a point at the center of gravity of the lanelet.
   * Relations are linestrings between points representing lanelets.
   *  @param routingCostId ID of the routing cost module used for the cost assignment
   *  @param includeAdjacent Also include adjacent (non-routable) relations
   *  @param includeConflicting Also include conflicting relations
   *  @return LaneletMap with the requested information */
  LaneletMapPtr getDebugLaneletMap(RoutingCostId routingCostId = {}, bool includeAdjacent = false,
                                   bool includeConflicting = false) const;

  /** @brief LaneletMap that includes all passable lanelets and areas.
   *  This map contains all passable lanelets and areas with all primitives (linestrings, points), but
   *  no regulatory elements. It can be used to perform spacial queries e.g. for localization.
   *  When selecting a lanelet from this map please be aware that the routing graph may also contain the inverted
   * lanelet.
   *  @return LaneletMap with all passable lanelets and areas */
  inline LaneletMapConstPtr passableMap() const noexcept { return passableLaneletMap_; }

  /** @brief Performs some basic sanity checks.
   *  It is recommended to call this function after the routing graph has been generated since it can point out some
   * mapping errors.
   *  @throws RoutingGraphError if an error is found an 'throwOnError' is true
   *  @param throwOnError Decide wheter to throw an exception or just return the errors
   *  @return Possible errors if 'throwOnError' is false. */
  Errors checkValidity(bool throwOnError = true) const;

  /**
   * Constructs the routing graph. Don't call this directly, use RoutingGraph::make instead.
   */
  RoutingGraph(std::unique_ptr<Graph>&& graph, lanelet::LaneletMapConstPtr&& passableMap);

 private:
  //! Documentation to be found in the cpp file.
  std::unique_ptr<Graph> graph_;           ///< Wrapper of the routing graph
  LaneletMapConstPtr passableLaneletMap_;  ///< Lanelet map of all passable lanelets
};

}  // namespace routing
}  // namespace lanelet
