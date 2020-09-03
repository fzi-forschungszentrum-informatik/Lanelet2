#pragma once
#include <lanelet2_core/Forward.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LaneletOrArea.h>
#include <lanelet2_core/utility/Optional.h>

#include <map>

#include "lanelet2_routing/Forward.h"
#include "lanelet2_routing/LaneletPath.h"
#include "lanelet2_routing/RoutingCost.h"
#include "lanelet2_routing/Types.h"

namespace lanelet {
namespace routing {

/**
 * @brief Controls the behaviour of the different possible path algorithms in RoutingGraph
 *
 * Consider the following graph (- or \ connects followers, | is a lane change):
 * ```
 * 1-2-3-4-5
 *  \   /
 *   6-7
 *   | |
 *   8-9
 * ```
 * Assuming we are using lanelet 1 as start of the search you can get the following results:
 * - routingCostLimit=50 (and the rest the default): Will give you all optimal paths that are at least 50
 * long (wrt to routing cost module zero, i.e. the first routing cost object from the "routingCosts"
 * parameter that the routing graph was crated with), does not include lane changes. The paths end
 * with the first lanelet that exceeds the 50 cost limit. Depending on the costs, the returned paths
 * might be only the path 1-2-3 (assuming 1-6-7 is too short).
 * - routingCostLimit=70: Will give you only 1-2-3-4, because 1-6-7-4 is a suboptimal path and 1-6-7 is too short.
 * - routingCostLimit=70, includeShorterPaths=True: Will give you 1-2-3-4 and 1-6-7. 1-6-7-4 is not included because the
 * path to 4 is suboptimal
 * - elementLimit=3: Will give you all optimal paths with exactly 3 lanelets (excluding lane changes). The result
 * will be 1-2-3 and 1-6-7
 * - elementLimit=4: Will give you only 1-2-3-4, also because 1-6-7-4 is suboptimal
 * - elementLimit=5, includeShorterPaths=true, includeLaneChanges=true: Will give you all optimal paths that
 * are at most 5 lanelets/areas long, including lane changes. The result might be 1-2-3-4-5, 1-6-7,
 * 1-8-9.
 * - routingCostLimit=50, elementLimit=3, includeLaneChanges=true:  Will give you all optimal paths (including lane
 * changes) where the last lanelet/area exceeds the 50 cost limit or that are 3 lanelets/areas long (whatever occurs
 * first). The result might be 1-2-3, 1-6-7 and 1-8-9
 * - routingCostLimit=70, elementLimit=4, includeShorterPaths=true: Will give you all optimal paths as above but also
 * include shorter/cheaper paths. Result: 1-2-3-4, 1-6-7, 1-8-9.
 */
struct PossiblePathsParams {
  Optional<double> routingCostLimit;  //!< cost limit for every path. Either that or maxElements must be valid.
  Optional<uint32_t> elementLimit;    //!< element limit for every path. Effect depends on includeShorterPaths
  RoutingCostId routingCostId{};      //!< the routing cost module to be used for the costs
  bool includeLaneChanges{false};     //!< if true, returned paths will include lane changes
  bool includeShorterPaths{false};    //!< also return paths that do not reach the limits
};

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

  //! Similar to the above but for a LaneletSubmap
  static RoutingGraphUPtr build(const LaneletSubmap& laneletSubmap, const traffic_rules::TrafficRules& trafficRules,
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
   *  @param withLaneChanges if false, the shortest path will not contain lane changes and the route will only contain
   *  lanelets that are reachable without lane changes
   *  @return A route of all adjacent lanelets of the shortest route that lead to 'end'. Nothing if there's no route.
   *  @see Route */
  Optional<Route> getRoute(const ConstLanelet& from, const ConstLanelet& to, RoutingCostId routingCostId = {},
                           bool withLaneChanges = true) const;

  /** @brief Get a driving route from 'start' to 'end' lanelets while going via other lanelets
   *  @param from Start lanelet to find a shortest path
   *  @param via Other lanelets to visit (in this order)
   *  @param to End lanelet to find a shortest path
   *  @param routingCostId ID of RoutingCost module to determine shortest route
   *  @param withLaneChanges if false, the shortest path will not contain lane changes and the route will only contain
   *  lanelets that are reachable without lane changes
   *  @return A route of all adjacent lanelets of the shortest route that lead to 'end'. Nothing if there's not
   * route.
   *
   * If from and to are the same lanelet, the resulting route will form a loop.
   *  @see Route */
  Optional<Route> getRouteVia(const ConstLanelet& from, const ConstLanelets& via, const ConstLanelet& to,
                              RoutingCostId routingCostId = {}, bool withLaneChanges = true) const;

  /** @brief Retrieve a shortest path between 'start' and 'end'.
   *
   *  Will find a shortest path using Djikstra's shortest path algorithm and the routing cost calculated by the
   * routing cost module with the respective ID. Be aware that the shortest path may contain lane changes,
   * i.e. lanelets that are parallel and not only adjacent.
   *  @param from Start lanelet to find a shortest path
   *  @param to End lanelet to find a shortest path
   *  @param routingCostId ID of RoutingCost module to determine shortest path
   *  @param withLaneChanges if false, the shortest path will not contain lane changes */
  Optional<LaneletPath> shortestPath(const ConstLanelet& from, const ConstLanelet& to, RoutingCostId routingCostId = {},
                                     bool withLaneChanges = true) const;

  /** @brief Retrieve a shortest path between 'start' and 'end' that may contain Areas.
   *
   *  Will find a shortest path using Djikstra's shortest path algorithm and the routing cost calculated by the
   * routing cost module with the respective ID. Be aware that the shortest path may contain lane changes,
   * i.e. lanelets that are parallel and not only adjacent.
   *  @param from Start lanelet or area to find a shortest path
   *  @param to End lanelet or area to find a shortest path to
   *  @param routingCostId ID of RoutingCost module to determine shortest path
   *  @param withLaneChanges if false, the shortest path will not contain lane changes */
  Optional<LaneletOrAreaPath> shortestPathIncludingAreas(const ConstLaneletOrArea& from, const ConstLaneletOrArea& to,
                                                         RoutingCostId routingCostId = {},
                                                         bool withLaneChanges = true) const;

  /** @brief Retrieve a shortest path between 'start' and 'end' using intermediate points.
   *  Will find a shortest path using Djikstra's shortest path algorithm and the routing cost calculated by the
   * routing cost module with the respective ID. Be aware that the shortest path may contain lane changes,
   * i.e. lanelets that are parallel and not only adjacent.
   *  @param start Start lanelet to find a shortest path
   *  @param via Intermediate lanelets that have to be passed
   *  @param end End lanelet to find a shortest path
   *  @param routingCostId ID of RoutingCost module to determine shortest path
   *  @param withLaneChanges if false, the shortest path will not contain lane changes
   *  @see shortestPath */
  Optional<LaneletPath> shortestPathVia(const ConstLanelet& start, const ConstLanelets& via, const ConstLanelet& end,
                                        RoutingCostId routingCostId = {}, bool withLaneChanges = true) const;

  /** @brief Retrieve a shortest path between 'start' and 'end' using intermediate points.
   *  Will find a shortest path using Djikstra's shortest path algorithm and the routing cost calculated by the
   * routing cost module with the respective ID. Be aware that the shortest path may contain lane changes,
   * i.e. lanelets that are parallel and not only adjacent.
   *  @param start Start lanelet or area to find a shortest path
   *  @param via Intermediate lanelets or areas that have to be passed
   *  @param end End lanelet or area to find a shortest path
   *  @param routingCostId ID of RoutingCost module to determine shortest path
   *  @param withLaneChanges if false, the shortest path will not contain lane changes
   *  @see shortestPath */
  Optional<LaneletOrAreaPath> shortestPathIncludingAreasVia(const ConstLaneletOrArea& start,
                                                            const ConstLaneletOrAreas& via,
                                                            const ConstLaneletOrArea& end,
                                                            RoutingCostId routingCostId = {},
                                                            bool withLaneChanges = true) const;

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
   *  @param routingCostId the routing cost module to be used. Can make a difference if one allows a lane change but the
   * other one doesn't.
   *  @return All left and right lanelets that can be reached, including lanelet, ordered left to right. */
  ConstLanelets besides(const ConstLanelet& lanelet, RoutingCostId routingCostId = {}) const;

  /** @brief Get left (routable) lanelet of a given lanelet if it exists.
   *  @param lanelet Start lanelet
   *  @param routingCostId the routing cost module to be used. Can make a difference if one allows a lane change but the
   * other one doesn't.
   *  @return Left lanelet if it exists. Nothing if it doesn't.
   *  @see adjacentLeft, lefts, adjacentLefts */
  Optional<ConstLanelet> left(const ConstLanelet& lanelet, RoutingCostId routingCostId = {}) const;

  /** @brief Get adjacent left (non-routable) lanelet of a given lanelet if it exists.
   *  @param lanelet Start lanelet
   *  @param routingCostId the routing cost module to be used. Can make a difference if one allows a lane change but the
   * other one doesn't.
   *  @return Adjacent left lanelet if it exists. Nothing if it doesn't.
   *  @see left, lefts, adjacentLefts */
  Optional<ConstLanelet> adjacentLeft(const ConstLanelet& lanelet, RoutingCostId routingCostId = {}) const;

  /** @brief Get right (routable) lanelet of a given lanelet if it exists.
   *  @param lanelet Start lanelet
   *  @param routingCostId the routing cost module to be used. Can make a difference if one allows a lane change but the
   * other one doesn't.
   *  @return Right lanelet if it exists. Nothing if it doesn't.
   *  @see adjacentRight, rights, adjacentRights */
  Optional<ConstLanelet> right(const ConstLanelet& lanelet, RoutingCostId routingCostId = {}) const;

  /** @brief Get adjacent right (non-routable) lanelet of a given lanelet if it exists.
   *  @param lanelet Start lanelet
   *  @param routingCostId the routing cost module to be used. Can make a difference if one allows a lane change but the
   * other one doesn't.
   *  @return Adjacent right lanelet if it exists. Nothing if it doesn't.
   *  @see right, rights, adjacentRights */
  Optional<ConstLanelet> adjacentRight(const ConstLanelet& lanelet, RoutingCostId routingCostId = {}) const;

  /** @brief Get all left (routable) lanelets of a given lanelet if they exist.
   *  @param lanelet Start lanelet
   *  @param routingCostId the routing cost module to be used. Can make a difference if one allows a lane change but the
   * other one doesn't.
   *  @return Left lanelets if they exists. Empty if they don't.
   *  @see adjacentLeft, left, adjacentLefts */
  ConstLanelets lefts(const ConstLanelet& lanelet, RoutingCostId routingCostId = {}) const;

  /** @brief Get all adjacent left (non-routable) lanelets of a given lanelet if they exist.
   *  @param lanelet Start lanelet
   *  @param routingCostId the routing cost module to be used. Can make a difference if one allows a lane change but the
   * other one doesn't.
   *  @return Adjacent left lanelets if they exists. Empty if they don't.
   *  @see adjacentLeft, left, lefts */
  ConstLanelets adjacentLefts(const ConstLanelet& lanelet, RoutingCostId routingCostId = {}) const;

  /** @brief Retrieve all lanelets and relations left of a given lanelet.
   *  @param lanelet Start lanelet
   *  @param routingCostId the routing cost module to be used. Can make a difference if one allows a lane change but the
   * other one doesn't.
   *  @return All lanelets and relations left of a given lanelet.
   *  @see lefts, adjacentLefts */
  LaneletRelations leftRelations(const ConstLanelet& lanelet, RoutingCostId routingCostId = {}) const;

  /** @brief Get all right (routable) lanelets of a given lanelet if they exist.
   *  @param lanelet Start lanelet
   *  @param routingCostId the routing cost module to be used. Can make a difference if one allows a lane change but the
   * other one doesn't.
   *  @return Right lanelets if they exists. Empty if they don't.
   *  @see adjacentRight, right, adjacentRights */
  ConstLanelets rights(const ConstLanelet& lanelet, RoutingCostId routingCostId = {}) const;

  /** @brief Get all adjacent right (non-routable) lanelets of a given lanelet if they exist.
   *  @param lanelet Start lanelet
   *  @param routingCostId the routing cost module to be used. Can make a difference if one allows a lane change but the
   * other one doesn't.
   *  @return Adjacent right lanelets if they exists. Empty if they don't.
   *  @see adjacentRight, right, rights */
  ConstLanelets adjacentRights(const ConstLanelet& lanelet, RoutingCostId routingCostId = {}) const;

  /** @brief Retrieve all lanelets and relations right of a given lanelet.
   *  @param lanelet Start lanelet
   *  @param routingCostId the routing cost module to be used. Can make a difference if one allows a lane change but the
   * other one doesn't.
   *  @return All lanelets and relations right of a given lanelet.
   *  @see rights, adjacentRights */
  LaneletRelations rightRelations(const ConstLanelet& lanelet, RoutingCostId routingCostId = {}) const;

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
   *  @param allowLaneChanges Allow or forbid lane changes
   *  @return all lanelets that are reachable in no particular orders. "lanelet" itself is always included. */
  ConstLanelets reachableSet(const ConstLanelet& lanelet, double maxRoutingCost, RoutingCostId routingCostId = {},
                             bool allowLaneChanges = true) const;

  //! Retrieve set of lanelet or areas that are reachable without exceeding routing cost.
  ConstLaneletOrAreas reachableSetIncludingAreas(const ConstLaneletOrArea& llOrAr, double maxRoutingCost,
                                                 RoutingCostId routingCostId = {}) const;

  /** @brief Retrieve a set of lanelets that can reach a given lanelet
   *
   *  Determines which reach the given lanelet with a given amount of routing cost.
   *  @param lanelet destination lanelet
   *  @param maxRoutingCost Maximum amount of routing cost allowed to reach other lanelets
   *  @param routingCostId ID of the routing cost module used for routing cost.
   *  @param allowLaneChanges Allow or forbid lane changes
   *  @return all lanelets in range in no particular order. "lanelet" itself is always included. */
  ConstLanelets reachableSetTowards(const ConstLanelet& lanelet, double maxRoutingCost,
                                    RoutingCostId routingCostId = {}, bool allowLaneChanges = true) const;

  /**
   * @brief Determines possible routes from a given start lanelet that satisfy the configuration in PossiblePathsParams
   * @param startPoint Start lanelet
   * @param params Parameters that configure the behaviour of the algorithm; see doc on PossiblePathsParams for details.
   * @return all valid possible paths. If a lanelet can be reached
   * using different paths, only the one is included that requires the least number of lane changes and has minimum
   * routing costs. "startPoint" itself is always included if it is in the routing graph.
   * @throws InvalidInputError if neither elementLimit nor costLimit is valid in params
   */
  LaneletPaths possiblePaths(const ConstLanelet& startPoint, const PossiblePathsParams& params) const;

  /** @brief Determines possible routes from a given start lanelet that are "minRoutingCost"-long.
   *
   * This behaves exactly as the PossiblePathsParams version with params.costLimit=minRoutingCost,
   * params.includeLaneChanges=allowLaneChanges and params.routingCostId=routingCostId (and the rest as default).
   */
  LaneletPaths possiblePaths(const ConstLanelet& startPoint, double minRoutingCost, RoutingCostId routingCostId = {},
                             bool allowLaneChanges = false) const;

  /** @brief Determines possible paths from a given start lanelet that are "minLanelets"-long.
   *
   * This behaves exactly as the PossiblePathsParams version with params.elementLimit=minLanelets,
   * params.includeLaneChanges=allowLaneChanges and params.routingCostId=routingCostId (and the rest as default).
   */
  LaneletPaths possiblePaths(const ConstLanelet& startPoint, uint32_t minLanelets, bool allowLaneChanges = false,
                             RoutingCostId routingCostId = {}) const;

  /**
   * @brief Determines possible routes to reach the given lanelet which satisfy the configuration in PossiblePathsParams
   * @return possible paths that are at least as long as specified in 'minRoutingCost'. "targetLanelet" itself is
   * always included if it is in the routing graph.
   * @param targetLanelet Destination lanelet
   * @param params Parameters that configure the behaviour of the algorithm; see doc on PossiblePathsParams for details.
   * @throws InvalidInputError if neither elementLimit nor costLimit is valid in params */
  LaneletPaths possiblePathsTowards(const ConstLanelet& targetLanelet, const PossiblePathsParams& params) const;

  /** @brief Determines possible routes that reach the given lanelet and are "minRoutingCost" long.
   *
   * This behaves exactly as the PossiblePathsParams version with params.costLimit=minRoutingCost,
   * params.includeLaneChanges=allowLaneChanges and params.routingCostId=routingCostId (and the rest as default). */
  LaneletPaths possiblePathsTowards(const ConstLanelet& targetLanelet, double minRoutingCost,
                                    RoutingCostId routingCostId = {}, bool allowLaneChanges = false) const;

  /** @brief Determines possible paths towards a destination lanelet that are "minLanelets"-long.
   *
   * This behaves exactly as the PossiblePathsParams version with params.elementLimit=minLanelets,
   * params.includeLaneChanges=allowLaneChanges and params.routingCostId=routingCostId (and the rest as default). */
  LaneletPaths possiblePathsTowards(const ConstLanelet& targetLanelet, uint32_t minLanelets,
                                    bool allowLaneChanges = false, RoutingCostId routingCostId = {}) const;

  //! Similar to RoutingGraph::possiblePaths, but also considers areas.
  LaneletOrAreaPaths possiblePathsIncludingAreas(const ConstLaneletOrArea& startPoint,
                                                 const PossiblePathsParams& params) const;

  //! Similar to RoutingGraph::possiblePaths, but also considers areas.
  LaneletOrAreaPaths possiblePathsIncludingAreas(const ConstLaneletOrArea& startPoint, double minRoutingCost,
                                                 RoutingCostId routingCostId = {}, bool allowLaneChanges = false) const;

  //! Similar to RoutingGraph::possiblePaths, but also considers areas.
  LaneletOrAreaPaths possiblePathsIncludingAreas(const ConstLaneletOrArea& startPoint, uint32_t minElements,
                                                 bool allowLaneChanges = false, RoutingCostId routingCostId = {}) const;

  /** @brief Calls a function on every successor of lanelet, optionally including lane changes
   *
   * This function can be used to query the routing graph on a more direct level. The function will be called on
   * lanelets with monotonically increasing cost from the start lanelet, including the start lanelet itself. If the
   * function returns "false" on an input, its followers will not be visited through this lanelet, as if it did not
   * exist.
   *
   * The search internally uses the dijkstra algorithm in order to discover the shortest path to the successors of this
   * lanelet and calls the provided function once it is known.
   *
   * In order to abort the query early, an exception can be thrown. If the lanelet is not part of the graph, nothing
   * will be called.
   * @param lanelet the lanelet where the search starts
   * @param f the function to be called on lanelet and its successors
   * @param allowLaneChanges also consider lane changes
   * @param routingCostId id for the routing cost module that is used to calculate the shortest path
   */
  void forEachSuccessor(const ConstLanelet& lanelet, const LaneletVisitFunction& f, bool allowLaneChanges = true,
                        RoutingCostId routingCostId = {}) const;

  //! Similar to RoutingGraph::forEachSuccessor but also includes areas into the search
  void forEachSuccessorIncludingAreas(const ConstLaneletOrArea& lanelet, const LaneletOrAreaVisitFunction& f,
                                      bool allowLaneChanges = true, RoutingCostId routingCostId = {}) const;

  //! Similar to RoutingGraph::forEachSuccessor but goes backwards in the routing graph instead of forward. The
  //! LaneletVisitInformation::cost will still be positive, despite going backwards.
  void forEachPredecessor(const ConstLanelet& lanelet, const LaneletVisitFunction& f, bool allowLaneChanges = true,
                          RoutingCostId routingCostId = {}) const;

  //! Similar to RoutingGraph::forEachPredecessor but also includes areas into the search
  void forEachPredecessorIncludingAreas(const ConstLaneletOrArea& lanelet, const LaneletOrAreaVisitFunction& f,
                                        bool allowLaneChanges = true, RoutingCostId routingCostId = {}) const;

  /** @brief Export the internal graph to graphML (xml-based) file format.
   *  @param filename Fully qualified file name - ideally with extension (.graphml)
   *  @param edgeTypesToExclude Exclude the specified relations. E.g. conflicting. Combine them with "|".
   *  @param routingCostId ID of the routing cost module
      @see exportGraphViz */
  void exportGraphML(const std::string& filename, const RelationType& edgeTypesToExclude = RelationType::None,
                     RoutingCostId routingCostId = {}) const;

  /** @brief Export the internal graph to graphViz (DOT) file format.
   *  This format includes coloring of the edges in the graph and bears little more information than graphML export.
   *  @param filename Fully qualified file name - ideally with extension (.gv)
   *  @param edgeTypesToExclude Exclude the specified relations. E.g. conflicting. Combine them with "|".
   *  @param routingCostId ID of the routing cost module */
  void exportGraphViz(const std::string& filename, const RelationType& edgeTypesToExclude = RelationType::None,
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

  /**
   * @brief Returns a submap that contains all lanelets and areas within this routing graph, and nothing else.
   * You can obtain a full map of the routing graph by calling passabelSubmap()->laneletMap(), which ist a potentially
   * costly operation.
   */
  inline LaneletSubmapConstPtr passableSubmap() const noexcept { return passableLaneletSubmap_; }

  /** @brief LaneletSubmap that includes all passable lanelets and areas.
   *  This map contains all passable lanelets and areas with all primitives (linestrings, points), including regulatory
   * elements and lanelets referenced by them. It can be used to perform spacial queries e.g. for localization. When
   * selecting a lanelet from this map please be aware that the routing graph may also contain the inverted lanelet.
   *  @return LaneletMap with all passable lanelets and areas
   *
   * This function is deprecated because it was misleading that the map also contained lanelets referenced by regulatory
   * elements and not only the lanelets from the routing graph.
   */
  [[deprecated(
      "Use passableSubmap to obtain the lanelets and areas within the routing graph!")]] inline LaneletMapConstPtr
  passableMap() const noexcept {
    return passableSubmap()->laneletMap();
  }

  /** @brief Performs some basic sanity checks.
   *  It is recommended to call this function after the routing graph has been generated since it can point out some
   * mapping errors.
   *  @throws RoutingGraphError if an error is found an 'throwOnError' is true
   *  @param throwOnError Decide wheter to throw an exception or just return the errors
   *  @return Possible errors if 'throwOnError' is false. */
  Errors checkValidity(bool throwOnError = true) const;

  /**
   * Constructs the routing graph. Don't call this directly, use RoutingGraph::build instead.
   */
  RoutingGraph(std::unique_ptr<internal::RoutingGraphGraph>&& graph, lanelet::LaneletSubmapConstPtr&& passableMap);

 private:
  //! Documentation to be found in the cpp file.
  std::unique_ptr<internal::RoutingGraphGraph> graph_;  ///< Wrapper of the routing graph
  LaneletSubmapConstPtr passableLaneletSubmap_;         ///< Lanelet map of all passable lanelets
};

}  // namespace routing
}  // namespace lanelet
