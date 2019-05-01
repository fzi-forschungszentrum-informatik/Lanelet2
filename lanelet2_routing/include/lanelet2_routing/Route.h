#pragma once

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/LaneletSequence.h>
#include <algorithm>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include "Forward.h"
#include "LaneletPath.h"
#include "RouteElement.h"

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
 * - RelationTypes within a Route refer to relations of the Elements that are part of the Route. E.g. in a
 * route a relation can be "SUCCEEDING" while in the complete graph it is "DIVERGING". If you want to know the
 * relation outside the route query the routing graph.
 * - Lanes have unique ids
 * - A lane goes all the way until a merging or diverging situation or if there is no succeeding lanelet. Simple
 * routes often just have one lane leading across multiple intersections.
 * - It is recommended to check a couple of basic things with the "checkValidity" function once the Route is created
 * */

class Route {
 public:
  using Errors = std::vector<std::string>;
  Route() = delete;
  Route(const Route& other) = delete;
  Route& operator=(const Route& other) = delete;
  Route& operator=(Route&& other) = delete;
  Route(Route&& other) = default;
  ~Route() = default;

  inline Route(LaneletPath shortestPath, ConstLaneletRouteElementMap& elements,
               std::map<RouteElement::LaneId, RouteElement*> firstElementsInInitLane, LaneletMapConstPtr laneletMap,
               RouteElement::LaneId numLanes)
      : elements_{std::move(elements)},
        shortestPath_{std::move(shortestPath)},
        firstElementsInInitLane_{std::move(firstElementsInInitLane)},
        laneletMap_{std::move(laneletMap)},
        numLanes_{numLanes} {
    lanes_.reserve(numLanes_);
  }

  /** @brief Returns the shortest path that was the base of this route */
  inline const LaneletPath& shortestPath() const noexcept { return shortestPath_; }

  /** @brief Returns the complete lane a Lanelet belongs to
   *  Circular lanes will always have 'll' as the first element.
   *  @param ll Lanelet to get lane for
   *  @return All lanelets of that route. Nothing if Lanelet is not part of the route.*/
  LaneletSequence fullLane(const ConstLanelet& ll);

  /** @brief Returns that remaining lane a Lanelet belongs to
   *  @param ll Lanelet to get remaining lane for
   *  @return All remaining lanelets of that lane. Nothing if Lanelet is not part of the route.*/
  inline LaneletSequence remainingLane(const ConstLanelet& ll) {
    auto lane = fullLane(ll);
    if (lane.empty()) {
      return lane;
    }
    auto llts = lane.lanelets();
    auto element = std::find(llts.begin(), llts.end(), ll);
    return ConstLanelets(element, llts.end());
  }

  /** @brief Get the 2d length of the shortest path of this route */
  double length2d() const;

  /** @brief Returns the number of individual lanes
   *  @return Number of lanes */
  inline size_t numLanes() const noexcept { return numLanes_; }

  /** @brief A laneletMap with all lanelets that are part of the route.
   *  @return A laneletMap with all lanelets of the route, excluding regulatory elements.
   *  Can be used to do spatial lookups like 'which Lanelets of the route are close to my position'.
   *  Note that not all lanelets in the map automatically belong to the route. They can also belong to one of the
   * regulatory elements of the lanelet. Use Route::contains for that.
   */
  inline LaneletMapConstPtr laneletMap() const noexcept { return laneletMap_; }

  /** @brief Get a laneletMap that represents the Lanelets of the Route and their relationship.
   *  @return The laneletMap
   *  Note that this laneletMap won't contain any Lanelets but rather just points and line string that represent
   * the lanelets and their relationship. */
  LaneletMapPtr getDebugLaneletMap() const;

  /** @brief Number of Lanelets in the route
   *  @return Number of lanelets. */
  inline size_t size() const { return elements_.size(); }

  /** @brief Provides information of the following lanelets within the Route
   *  @param lanelet Lanelet to get information about.
   * @return Relations to following lanelets in the route. Nothing if 'lanelet' is not part of the route.*/
  LaneletRelations followingRelations(const ConstLanelet& lanelet) const;

  /** @brief Provides information of the previous lanelets within the Route
   *  @param lanelet Lanelet to get information about.
   * @return Relations to previous lanelets in the route. Nothing if 'lanelet' is not part of the route.*/
  LaneletRelations previousRelations(const ConstLanelet& lanelet) const;

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
   *  @return Vector of conflicting lanelets. Empty vector if input lanelet is not part of the route.
   *  @see conflictingInRoute */
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
  ConstLaneletRouteElementMap elements_;  ///< All elements that are part of the map
  // NOLINTNEXTLINE
  const LaneletPath shortestPath_;  ///< The underlying shortest path used to create the route
  std::unordered_map<RouteElement::LaneId, ConstLanelets> lanes_;  ///< Map of lanes (lazy initialization)
  // NOLINTNEXTLINE
  const std::map<RouteElement::LaneId, RouteElement*> firstElementsInInitLane_;  ///< First elements of the lanes
  // NOLINTNEXTLINE
  const LaneletMapConstPtr laneletMap_;  ///< LaneletMap with all lanelets that are part of the route
  // NOLINTNEXTLINE
  const RouteElement::LaneId numLanes_{};  ///< Number of lanes

  /** @brief Checks if a lane is circular (e.g. roundabout) */
  inline bool checkCircularLanes(const ConstLanelets& lane) const {
    assert(!lane.empty());
    if (lane.empty()) {
      return false;
    }
    auto following = elements_.find(lane.back())->second->following();
    return !following.empty() && following.front().routeElement == elements_.find(lane.front())->second.get();
  }
};
};  // namespace routing
};  // namespace lanelet
