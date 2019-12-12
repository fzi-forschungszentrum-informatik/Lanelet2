#pragma once
#include <lanelet2_core/Forward.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/LaneletSequence.h>
#include <memory>
#include <string>
#include <vector>
#include "Forward.h"
#include "LaneletPath.h"

namespace lanelet {
namespace routing {

/** @brief The famous route object that marks a route from A to B.
 *
 * - A Route includes all Lanelets that are adjacent to the Lanelets that represent the shortest path and lead
 * towards the goal. Any Lanelet that is close, but not directly attached, e.g. the other way around a channeling island
 * is not included. Any adjacent Lanelet which does not lead to the goal is also not included.
 * - A Route does not have any relations to Lanelets that are not part of the Route (the only exception are
 * conflicting Lanelets)
 * - A Route is usually obtained from a RoutingGraph via the "getRoute" function.
 * - A route is self-sustained in terms of that there is no relation to the RoutingGraph anymore
 * - RelationTypes within a Route refer to relations of the Elements that are part of the Route. A lanelet can have
 * fewer successors than in the routing graph if some are not part of the route
 * - The route is divided into lanes with individual Ids. The exact value of the id for a lane is random. Ids are not
 * continuous.
 * - A new lane begins at every lanelet that has not exactly one predecessor or that is part of a diverging situation.
 * Simple routes often just have one lane leading across multiple intersections.
 * - It is recommended to check a couple of basic things with the "checkValidity" function once the Route is created
 * - Routes can also be circular (e.g. if start and end lanelet are the same). The "last" lanelet of the route will then
 * have the first lanelet of the route as successor.
 * */
class Route {
 public:
  using Errors = std::vector<std::string>;
  Route();
  Route(const Route& other) = delete;
  Route& operator=(const Route& other) = delete;
  Route& operator=(Route&& other) noexcept;
  Route(Route&& other) noexcept;
  ~Route() noexcept;

  Route(LaneletPath shortestPath, std::unique_ptr<RouteGraph> graph, LaneletMapConstPtr laneletMap) noexcept;

  /** @brief Returns the shortest path that was the base of this route */
  inline const LaneletPath& shortestPath() const noexcept { return shortestPath_; }

  /**
   * @brief Obtains the remaining shortest path to the destination. If the route is circular, the result will always
   * have the same length and end before the lanelet passed as input argument
   * @param ll a lanelet on the shortest path
   * @return shortest path where lanelet is the first element. Nothing if the lanelet is not on the shortest path.
   */
  LaneletPath remainingShortestPath(const ConstLanelet& ll) const;

  /** @brief Returns the complete lane a Lanelet belongs to. Circular lanes will always have 'll' as the first element.
   *  @param ll Lanelet to get lane for
   *  @return All lanelets of that route. Nothing if Lanelet is not part of the route.
   *
   * If
   */
  LaneletSequence fullLane(const ConstLanelet& ll) const;

  /** @brief Returns that remaining lane a Lanelet belongs to
   *  @param ll Lanelet to get remaining lane for
   *  @return All remaining lanelets of that lane. Nothing if Lanelet is not part of the route.*/
  LaneletSequence remainingLane(const ConstLanelet& ll) const;

  /** @brief Get the 2d length of the shortest path of this route */
  double length2d() const;

  /** @brief Returns the number of individual lanes
   *  @return Number of lanes */
  size_t numLanes() const;

  /** @brief A laneletMap with all lanelets that are part of the route.
   *  @return A laneletMap with all lanelets of the route, excluding regulatory elements.
   *  Can be used to do spatial lookups like 'which Lanelets of the route are close to my position'.
   *  Note that not all lanelets in the map automatically belong to the route. They can also belong to one of the
   * regulatory elements of the lanelet. Use Route::contains for that.
   */
  inline const LaneletMapConstPtr& laneletMap() const noexcept { return laneletMap_; }

  /** @brief Get a laneletMap that represents the Lanelets of the Route and their relationship.
   *  @return The laneletMap
   *  Note that this laneletMap won't contain any Lanelets but rather just points and line string that represent
   * the lanelets and their relationship. */
  LaneletMapPtr getDebugLaneletMap() const;

  /** @brief Number of Lanelets in the route
   *  @return Number of lanelets.
   */
  size_t size() const;

  /** @brief Provides information of the following lanelets within the Route
   *  @param lanelet Lanelet to get information about.
   * @return Relations to following lanelets in the route. Nothing if 'lanelet' is not part of the route. The relation
   * will always be "succesor"
   */
  LaneletRelations followingRelations(const ConstLanelet& lanelet) const;

  //! Similar to followingRelations but directly provides the following lanelets within the Route
  ConstLanelets following(const ConstLanelet& lanelet) const;

  /** @brief Provides information of the previous lanelets within the Route
   *  @param lanelet Lanelet to get information about.
   * @return Relations to previous lanelets in the route. Nothing if 'lanelet' is not part of the route. The type
   * will always be "successor".
   */
  LaneletRelations previousRelations(const ConstLanelet& lanelet) const;

  //! Similar to followingRelations but directly provides the following lanelets within the Route
  ConstLanelets previous(const ConstLanelet& lanelet) const;

  /** @brief Provides information of the lanelet left of a given lanelet within the Route
   *  @note The returned lanelet can include a lanelet that can be switched or not-switched to. The relation will be
   * "left" and "adjacent_left" accordingly.
   *  @param lanelet Lanelet to get information about.
   * @return Relation to lanelet left of the input lanelet in the route. Nothing if 'lanelet' is not part of the
   * route.*/
  Optional<LaneletRelation> leftRelation(const ConstLanelet& lanelet) const;

  /** @brief Provides information of the *all* lanelets left of a given lanelet within the Route.
   *  @note The returned lanelets include lanelets that can be switched or not-switched to. The relations will be
   * "left" and "adjacent_left" accordingly.
   *  @param lanelet Lanelet to get information about.
   * @return Relations to lanelets left of the input lanelet in the route. Nothing if 'lanelet' is not part of the
   * route.*/
  LaneletRelations leftRelations(const ConstLanelet& lanelet) const;

  /** @brief Provides information of the lanelet right of a given lanelet within the Route
   *  @note The returned lanelet can include a lanelet that can be switched or not-switched to. The relation will be
   * "right" and "adjacent_right" accordingly.
   *  @param lanelet Lanelet to get information about.
   * @return Relation to lanelet right of the input lanelet in the route. Nothing if 'lanelet' is not part of the
   * route.*/
  Optional<LaneletRelation> rightRelation(const ConstLanelet& lanelet) const;

  /** @brief Provides information of the *all* lanelets right of a given lanelet within the Route.
   *  @note The returned lanelets include lanelets that can be switched or not-switched to. The relations will be
   * "right" and "adjacent_right" accordingly.
   *  @param lanelet Lanelet to get information about.
   * @return Relations to lanelets right of the input lanelet in the route. Nothing if 'lanelet' is not part of the
   * route.*/
  LaneletRelations rightRelations(const ConstLanelet& lanelet) const;

  /** @brief Information about conflicting lanelets of a lanelet within the route
   *  @param lanelet Lanelet to find conflicting lanelets to
   *  @return Vector of conflicting lanelets. Empty vector if input lanelet is not part of the route.
   *  @see conflictingInMap */
  ConstLanelets conflictingInRoute(const ConstLanelet& lanelet) const;

  /** @brief Information about conflicting lanelets of a lanelet within all passable lanelets in the laneletMap
   *  @param lanelet Lanelet to find conflicting lanelets to
   *  @return Vector of conflicting lanelets. Empty vector if input lanelet is not part of the route. Lanelets that are
   * also conflicting in route are returned as well.
   *  @see conflictingInRoute
   */
  ConstLaneletOrAreas conflictingInMap(const ConstLanelet& lanelet) const;

  /**
   * @brief Provides all lanelets in the map that conflict with any lanelet in the route.
   */
  ConstLaneletOrAreas allConflictingInMap() const;

  //! Checks if a specific lanelet is part of the route
  bool contains(const ConstLanelet& lanelet) const;

  /** @brief Perform some sanity checks on the route
   *  @param throwOnError Throw or not-throw an exception if errors are found
   *  @throw Throws if errors are encountered and throwOnError is true
   *  @return Vector of errors if no-throw is chosen and errors are found */
  Errors checkValidity(bool throwOnError = false) const;

 private:
  std::unique_ptr<RouteGraph> graph_;  ///< The internal graph
  LaneletPath shortestPath_;           ///< The underlying shortest path used to create the route
  LaneletMapConstPtr laneletMap_;      ///< LaneletMap with all lanelets that are part of the route
};
};  // namespace routing
};  // namespace lanelet
