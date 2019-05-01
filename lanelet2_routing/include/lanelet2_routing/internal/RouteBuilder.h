#pragma once
#include "LaneletPath.h"
#include "Route.h"

namespace lanelet {
namespace routing {

class RoutingGraph;

//! Builder class to create a route from a routing graph and the shortest path
class RouteBuilder {
 public:
  /** @note Important things about pending elements:
   *  Pending elements are lanelets that can be reached from the shortest path (e.g. a lanelet right of a shortest
   * path lanelet), but we don't know yet if we can reach the lanelet of the shortest path using that lanelet. This
   * could happen if one is allowed to change the lane to the right, but not back. It could happen that we could
   * switch to the left a couple of lanelets later, but we don't know yet. Pending elements hold relations the left or
   * right lanelet that is certainly part of the route, but a lanelet that is part of the route *never* has a
   * relation to a pending element. These are added once we know that a pending element will become a part of the
   * route.
   * @note Important things about lanes:
   * @anchor laneIdNotes
   *  During the creation of a route we assign lane IDs. A lane is a number of consecutive lanelets that follow each
   * other and have unambiguous relations. Relations are unambiugous when two or more lanelets are merging or if they
   * are diverging. Judging which of the lanes is going to continue and which one ends or starts is difficult. So in
   * both those situations we're starting new lanes. Every so often - for example in diverging situations - we don't
   * know if both new lanes are going to be a part of the final route, but lane IDs are issued anyway. As soon as the
   * route is determined we can replace the lane IDs with IDs that make more sense in the context of the route. For
   * example: there's a diverging situation and we issue new lane IDs. It turns out that the left lanelet doesn't lead
   * to the goal. So in the context of the route this is not a diverging situation but rather it's just one lanelet
   * (the right one) is following its predecessor. We would say that they are part of the same lane since there's no
   * other way to go. */
  explicit RouteBuilder(const RoutingGraph& g) : graph_{g} {}
  Optional<Route> getRouteFromShortestPath(const LaneletPath& path);

 private:
  void addPendingToElements(std::vector<RouteElement*>& elementsQueue);
  std::vector<RouteElement*> addPreviousRelations(RouteElement* newElement, bool directlyConnected);
  Optional<Route> createRoutefromElements(const LaneletPath& path);
  std::vector<ConstLanelets> determineDivergingLanes(const ConstLanelets& diverging);
  std::vector<ConstLanelets> determineMergingLanes(const ConstLanelets& merging);
  std::vector<ConstLanelets> determineLanesImpl(
      const ConstLanelets& initialSplit,
      const std::function<ConstLanelets(const RoutingGraph&, ConstLanelet, bool)>& nextLanelets) const;
  std::vector<RouteElementUPtrs> divergingToPending(RouteElement* element, RouteElement::LaneId& initLaneId,
                                                    const RouteElementUPtrs& currentCandidates = RouteElementUPtrs());
  RouteElement::LaneId initlaneIdForRoute(const ConstLanelet& lanelet, RouteElement::LaneId& initLaneId);
  void processMergingLanelets(std::vector<RouteElement*>& elementsQueue, RouteElement::LaneId& initLaneId,
                              RouteElement* thisElement, RouteElement* previousElement,
                              const Optional<RelationType>& relation);
  void processLeftSide(RouteElement::LaneId& initLaneId, RouteElement* thisElement, bool directRelation = true,
                       bool preferPendingToElements = false);
  void processRightSide(RouteElement::LaneId& initLaneId, RouteElement* thisElement, bool directRelation = true,
                        bool preferPendingToElements = false);
  void recursiveDivergingToPending(RouteElement::LaneId& initLaneId, RouteElement* thisElement);

  const RoutingGraph& graph_;
  std::vector<RouteElementUPtrs> pending_;
  std::unordered_map<ConstLanelet, RouteElementUPtr> elements_;  // These are going to part of the route
  std::map<RouteElement::LaneId, RouteElement*> firstInLane_;    // First elements of each lane
};

}  // namespace routing
}  // namespace lanelet
