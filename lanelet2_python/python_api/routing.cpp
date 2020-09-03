#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <boost/make_shared.hpp>
#include <boost/python.hpp>

#include "lanelet2_python/internal/converter.h"

using namespace boost::python;
using namespace lanelet;

Optional<std::shared_ptr<lanelet::routing::Route>> getRouteWrapper(const lanelet::routing::RoutingGraph& self,
                                                                   const ConstLanelet& from, const ConstLanelet& to,
                                                                   lanelet::routing::RoutingCostId costId,
                                                                   bool withLaneChange) {
  auto route = self.getRoute(from, to, costId, withLaneChange);
  if (!route) {
    return {};
  }
  return std::make_shared<lanelet::routing::Route>(std::move(*route));
}

Optional<std::shared_ptr<lanelet::routing::Route>> getRouteViaWrapper(const lanelet::routing::RoutingGraph& self,
                                                                      const ConstLanelet& from,
                                                                      const ConstLanelets& via, const ConstLanelet& to,
                                                                      lanelet::routing::RoutingCostId costId,
                                                                      bool withLaneChange) {
  auto route = self.getRouteVia(from, via, to, costId, withLaneChange);
  if (!route) {
    return {};
  }
  return std::make_shared<lanelet::routing::Route>(std::move(*route));
}

routing::RoutingGraphPtr makeRoutingGraph(LaneletMap& laneletMap, const traffic_rules::TrafficRules& trafficRules,
                                          const routing::RoutingCostPtrs& routingCosts) {
  return routing::RoutingGraph::build(laneletMap, trafficRules, routingCosts);
}

routing::RoutingGraphPtr makeRoutingGraphSubmap(LaneletSubmap& laneletMap,
                                                const traffic_rules::TrafficRules& trafficRules,
                                                const routing::RoutingCostPtrs& routingCosts) {
  return routing::RoutingGraph::build(laneletMap, trafficRules, routingCosts);
}

template <typename T>
object optionalToObject(const Optional<T>& v) {
  return v ? object(*v) : object();
}

template <typename T>
Optional<T> objectToOptional(const object& o) {
  return o == object() ? Optional<T>{} : Optional<T>{extract<T>(o)()};
}

BOOST_PYTHON_MODULE(PYTHON_API_MODULE_NAME) {  // NOLINT
  auto trafficRules = import("lanelet2.traffic_rules");
  using namespace lanelet::routing;

  using converters::IterableConverter;
  using converters::OptionalConverter;
  using converters::VectorToListConverter;
  OptionalConverter<Route>();
  OptionalConverter<std::shared_ptr<Route>>();
  OptionalConverter<RelationType>();
  OptionalConverter<LaneletRelation>();
  OptionalConverter<LaneletPath>();

  VectorToListConverter<LaneletRelations>();
  VectorToListConverter<RoutingCostPtrs>();
  VectorToListConverter<LaneletPaths>();

  // Register interable conversions.
  IterableConverter().fromPython<RoutingCostPtrs>();

  implicitly_convertible<std::shared_ptr<RoutingCostDistance>, RoutingCostPtr>();
  implicitly_convertible<std::shared_ptr<RoutingCostTravelTime>, RoutingCostPtr>();
  implicitly_convertible<LaneletMapPtr, LaneletMapConstPtr>();

  // Routing costs
  class_<RoutingCost, boost::noncopyable, std::shared_ptr<RoutingCost>>(  // NOLINT
      "RoutingCost", "Object for calculating routing costs between lanelets", no_init);

  class_<RoutingCostDistance, bases<RoutingCost>, std::shared_ptr<RoutingCostDistance>>(  // NOLINT
      "RoutingCostDistance", "Distance based routing cost calculation object",
      init<double, double>((arg("laneChangeCost"), arg("minLaneChangeDistance") = 0)));

  class_<RoutingCostTravelTime, bases<RoutingCost>, std::shared_ptr<RoutingCostTravelTime>>(  // NOLINT
      "RoutingCostTravelTime", "Travel time based routing cost calculation object",
      init<double, double>((arg("laneChangeCost"), arg("minLaneChangeTime") = 0)));

  auto possPCost = static_cast<LaneletPaths (RoutingGraph::*)(const ConstLanelet&, double, RoutingCostId, bool) const>(
      &RoutingGraph::possiblePaths);
  auto possPLen = static_cast<LaneletPaths (RoutingGraph::*)(const ConstLanelet&, uint32_t, bool, RoutingCostId) const>(
      &RoutingGraph::possiblePaths);
  auto possPParam = static_cast<LaneletPaths (RoutingGraph::*)(const ConstLanelet&, const PossiblePathsParams&) const>(
      &RoutingGraph::possiblePaths);
  auto possPToCost =
      static_cast<LaneletPaths (RoutingGraph::*)(const ConstLanelet&, double, RoutingCostId, bool) const>(
          &RoutingGraph::possiblePathsTowards);
  auto possPToLen =
      static_cast<LaneletPaths (RoutingGraph::*)(const ConstLanelet&, uint32_t, bool, RoutingCostId) const>(
          &RoutingGraph::possiblePathsTowards);
  auto possPToParam =
      static_cast<LaneletPaths (RoutingGraph::*)(const ConstLanelet&, const PossiblePathsParams&) const>(
          &RoutingGraph::possiblePathsTowards);

  class_<LaneletPath>("LaneletPath",
                      "A set of consecutive lanelets connected in straight "
                      "directon or by lane changes",
                      init<ConstLanelets>(arg("lanelets")))
      .def("__iter__", iterator<LaneletPath>())
      .def("__len__", &LaneletPath::size)
      .def("__getitem__", wrappers::getItem<LaneletPath>, return_internal_reference<>())
      .def(
          "getRemainingLane",
          +[](const LaneletPath& self, const ConstLanelet& llt) { return self.getRemainingLane(llt); },
          "get the sequence of remaining lanelets without a lane change")
      .def(self == self)   // NOLINT
      .def(self != self);  // NOLINT

  class_<LaneletVisitInformation>("LaneletVisitInformation",
                                  "Object passed as input for the forEachSuccessor function of the routing graph")
      .def_readwrite("lanelet", &LaneletVisitInformation::lanelet, "the currently visited lanelet")
      .def_readwrite("predecessor", &LaneletVisitInformation::predecessor, "the predecessor within the shortest path")
      .def_readwrite("length", &LaneletVisitInformation::length,
                     "The length of the shortest path to this lanelet (including lanelet")
      .def_readwrite("cost", &LaneletVisitInformation::cost, "The cost along the shortest path")
      .def_readwrite("numLaneChanges", &LaneletVisitInformation::numLaneChanges,
                     "The number of lane changes necessary along the shortest path");

  class_<LaneletOrAreaVisitInformation>(
      "LaneletOrAreaVisitInformation",
      "Object passed as input for the forEachSuccessorIncludingAreas function of the routing graph")
      .def_readwrite("laneletOrArea", &LaneletOrAreaVisitInformation::laneletOrArea,
                     "the currently visited lanelet/area")
      .def_readwrite("predecessor", &LaneletOrAreaVisitInformation::predecessor,
                     "the predecessor within the shortest path")
      .def_readwrite("length", &LaneletOrAreaVisitInformation::length,
                     "The length of the shortest path to this lanelet (including lanelet")
      .def_readwrite("cost", &LaneletOrAreaVisitInformation::cost, "The cost along the shortest path")
      .def_readwrite("numLaneChanges", &LaneletOrAreaVisitInformation::numLaneChanges,
                     "The number of lane changes necessary along the shortest path");

  class_<PossiblePathsParams>(
      "PossiblePathsParams", "Parameters for configuring the behaviour of the possible path algorithms of RoutingGraph")
      .def("__init__",
           make_constructor(
               +[](object costLimit, object elemLimit, RoutingCostId costId, bool includeLc, bool includeShorter) {
                 return boost::make_shared<PossiblePathsParams>(PossiblePathsParams{
                     objectToOptional<double>(costLimit), objectToOptional<std::uint32_t>(elemLimit), costId, includeLc,
                     includeShorter});
               },
               default_call_policies{},
               (arg("routingCostLimit") = object(), arg("elementLimit") = object(), arg("routingCostId") = 0,
                arg("includeLaneChanges") = false, arg("includeShorterPaths") = false)))
      .add_property(
          "routingCostLimit", +[](const PossiblePathsParams& self) { return optionalToObject(self.routingCostLimit); },
          +[](PossiblePathsParams& self, object value) { self.routingCostLimit = objectToOptional<double>(value); })
      .add_property(
          "elementLimit", +[](const PossiblePathsParams& self) { return optionalToObject(self.elementLimit); },
          +[](PossiblePathsParams& self, object value) { self.elementLimit = objectToOptional<std::uint32_t>(value); })
      .def_readwrite("routingCostId", &PossiblePathsParams::routingCostId)
      .def_readwrite("includeLaneChanges", &PossiblePathsParams::includeLaneChanges)
      .def_readwrite("includeShorterPaths", &PossiblePathsParams::includeShorterPaths);

  auto lltAndLc = (arg("lanelet"), arg("withLaneChanges") = false);
  auto lltAndRoutingCost = (arg("lanelet"), arg("routingCostId") = 0);
  class_<RoutingGraph, boost::noncopyable, RoutingGraphPtr>(
      "RoutingGraph",
      "Main class of the routing module that holds routing information and can "
      "be queried",
      no_init)
      .def("__init__",
           make_constructor(makeRoutingGraph, default_call_policies(),
                            (arg("laneletMap"), arg("trafficRules"), arg("routingCost") = defaultRoutingCosts())),
           "Initialization with default routing costs")
      .def("__init__",
           make_constructor(makeRoutingGraphSubmap, default_call_policies(),
                            (arg("laneletSubmap"), arg("trafficRules"), arg("routingCost") = defaultRoutingCosts())),
           "Initialization from a submap")
      .def("getRoute", getRouteWrapper, "driving route from 'start' to 'end' lanelet",
           (arg("from"), arg("to"), arg("routingCostId") = 0, arg("withLaneChanges") = true))
      .def("getRouteVia", getRouteViaWrapper, "driving route from 'start' to 'end' lanelet using the 'via' lanelets",
           (arg("from"), arg("via"), arg("to"), arg("routingCostId") = 0, arg("withLaneChanges") = true))
      .def("shortestPath", &RoutingGraph::shortestPath, "shortest path between 'start' and 'end'",
           (arg("from"), arg("to"), arg("routingCostId") = 0, arg("withLaneChanges") = true))
      .def("shortestPathWithVia", &RoutingGraph::shortestPathVia,
           "shortest path between 'start' and 'end' using intermediate points",
           (arg("start"), arg("via"), arg("end"), arg("routingCostId") = 0, arg("withLaneChanges") = true))
      .def("routingRelation", &RoutingGraph::routingRelation, "relation between two lanelets excluding 'conflicting'",
           (arg("from"), arg("to")))
      .def("following", &RoutingGraph::following, "lanelets that can be reached from this lanelet", lltAndLc)
      .def("followingRelations", &RoutingGraph::followingRelations, "relations to following lanelets", lltAndLc)
      .def("previous", &RoutingGraph::previous, "previous lanelets of this lanelet", lltAndLc)
      .def("previousRelations", &RoutingGraph::previousRelations, "relations to preceding lanelets", lltAndLc)
      .def("besides", &RoutingGraph::besides,
           "all reachable left and right lanelets, including lanelet, from "
           "left to right",
           lltAndRoutingCost)
      .def("left", &RoutingGraph::left, "left (routable)lanelet, if exists", lltAndRoutingCost)
      .def("right", &RoutingGraph::right, "right (routable)lanelet, if it exists", lltAndRoutingCost)
      .def("adjacentLeft", &RoutingGraph::adjacentLeft, "left non-routable lanelet", lltAndRoutingCost)
      .def("adjacentRight", &RoutingGraph::adjacentRight, "right non-routable lanelet", lltAndRoutingCost)
      .def("lefts", &RoutingGraph::lefts, "all left (routable) lanelets", lltAndRoutingCost)
      .def("rights", &RoutingGraph::rights, "all right (routable) lanelets", lltAndRoutingCost)
      .def("adjacentLefts", &RoutingGraph::adjacentLefts, "all left (non-routable) lanelets", lltAndRoutingCost)
      .def("adjacentRights", &RoutingGraph::adjacentRights, "all right (non-routable) lanelets", lltAndRoutingCost)
      .def("leftRelations", &RoutingGraph::leftRelations, "relations to left lanelets", lltAndRoutingCost)
      .def("rightRelations", &RoutingGraph::rightRelations, "relations to right lanelets", lltAndRoutingCost)
      .def("conflicting", &RoutingGraph::conflicting, "Conflicting lanelets", arg("lanelet"))
      .def("reachableSet", &RoutingGraph::reachableSet, "set of lanelets that can be reached from a given lanelet",
           (arg("lanelet"), arg("maxRoutingCost"), arg("RoutingCostId") = 0, arg("allowLaneChanges") = true))
      .def("reachableSetTowards", &RoutingGraph::reachableSetTowards, "set of lanelets that can reach a given lanelet",
           (arg("lanelet"), arg("maxRoutingCost"), arg("RoutingCostId") = 0, arg("allowLaneChanges") = true))
      .def("possiblePaths", possPCost, "possible paths from a given start lanelet that are 'minRoutingCost'-long",
           (arg("lanelet"), arg("minRoutingCost"), arg("RoutingCostId") = 0, arg("allowLaneChanges") = false,
            arg("routingCostId") = 0))
      .def("possiblePaths", possPParam, "possible paths from a given start lanelet as configured in parameters",
           (arg("lanelet"), arg("parameters")))
      .def("possiblePathsTowards", possPToCost,
           "possible paths to a given start lanelet that are 'minRoutingCost'-long",
           (arg("lanelet"), arg("minRoutingCost"), arg("RoutingCostId") = 0, arg("allowLaneChanges") = false,
            arg("routingCostId") = 0))
      .def("possiblePathsTowards", possPToParam, "possible paths to a given lanelet as configured in parameters",
           (arg("lanelet"), arg("parameters")))
      .def("possiblePathsMinLen", possPLen, "possible routes from a given start lanelet that are 'minLanelets'-long",
           (arg("lanelet"), arg("minLanelets"), arg("allowLaneChanges") = false, arg("routingCostId") = 0))
      .def("possiblePathsTowardsMinLen", possPToLen,
           "possible routes from a given start lanelet that are 'minLanelets'-long",
           (arg("lanelet"), arg("minLanelets"), arg("allowLaneChanges") = false, arg("routingCostId") = 0))
      .def(
          "forEachSuccessor",
          +[](RoutingGraph& self, const ConstLanelet& from, object func, bool lc, RoutingCostId costId) {
            self.forEachSuccessor(from, std::move(func), lc, costId);
          },
          "calls a function on each successor of lanelet with increasing cost. The function must receives a "
          "LaneletVisitInformation object as input and must return a bool whether followers of the current lanelet "
          "should be visited as well. The function can raise an exception if an early exit is desired",
          (arg("lanelet"), arg("func"), arg("allowLaneChanges") = true, arg("routingCostId") = 0))
      .def(
          "forEachSuccessorIncludingAreas",
          +[](RoutingGraph& self, const ConstLaneletOrArea& from, object func, bool lc, RoutingCostId costId) {
            self.forEachSuccessorIncludingAreas(from, std::move(func), lc, costId);
          },
          "similar to forEachSuccessor but also includes areas into the search",
          (arg("lanelet"), arg("func"), arg("allowLaneChanges") = true, arg("routingCostId") = 0))
      .def(
          "forEachPredecessor",
          +[](RoutingGraph& self, const ConstLanelet& from, object func, bool lc, RoutingCostId costId) {
            self.forEachPredecessor(from, std::move(func), lc, costId);
          },
          "similar to forEachSuccessor but instead goes backwards along the routing graph",
          (arg("lanelet"), arg("func"), arg("allowLaneChanges") = true, arg("routingCostId") = 0))
      .def(
          "forEachPredecessorIncludingAreas",
          +[](RoutingGraph& self, const ConstLaneletOrArea& from, object func, bool lc, RoutingCostId costId) {
            self.forEachPredecessorIncludingAreas(from, std::move(func), lc, costId);
          },
          "calls a function on each successor of lanelet. The function must receives a LaneletVisitInformation object "
          "as input and must return a bool whether followers of the current lanelet should be visited as well. The "
          "function can throw an exception if an early exit is desired",
          (arg("lanelet"), arg("func"), arg("allowLaneChanges") = true, arg("routingCostId") = 0))
      .def(
          "exportGraphML", +[](RoutingGraph& self, const std::string& path) { self.exportGraphML(path); },
          "Export the internal graph to graphML (xml-based) file format")
      .def(
          "exportGraphViz", +[](RoutingGraph& self, const std::string& path) { self.exportGraphViz(path); },
          "Export the internal graph to graphViz (DOT) file format")
      .def("getDebugLaneletMap", &RoutingGraph::getDebugLaneletMap,
           "abstract lanelet map holding the information of the routing graph",
           (arg("routingCostId") = 0, arg("includeAdjacent") = false, arg("includeConflicting") = false))
      .def("passableLaneletSubmap", &RoutingGraph::passableSubmap, "LaneletMap that includes all passable lanelets")
      .def("checkValidity", &RoutingGraph::checkValidity, "Performs some basic sanity checks",
           (arg("throwOnError") = true));

  class_<LaneletRelation>("LaneletRelation")
      .def_readwrite("lanelet", &LaneletRelation::lanelet)
      .def_readwrite("relationType", &LaneletRelation::relationType);

  enum_<RelationType>("RelationType")
      .value("Successor", RelationType::Successor)
      .value("Left", RelationType::Left)
      .value("Right", RelationType::Right)
      .value("Conflicting", RelationType::Conflicting)
      .value("AdjacentLeft", RelationType::AdjacentLeft)
      .value("AdjacentRight", RelationType::AdjacentRight)
      .export_values();

  class_<Route, boost::noncopyable, std::shared_ptr<Route>>("Route",
                                                            "The famous route object that marks a route from A to B, "
                                                            "including all lanelets that can be used",
                                                            no_init)
      .def("shortestPath", &Route::shortestPath, "Returns the shortest path along this route",
           return_internal_reference<>())
      .def("fullLane", &Route::fullLane, "Returns the complete lane a Lanelet belongs to")
      .def("remainingLane", &Route::remainingLane, "Returns that remaining lane a Lanelet belongs to")
      .def("remainingShortestPath", &Route::remainingShortestPath,
           "Returns all lanelets on the shortest path that follow the input lanelet")
      .def("length2d", &Route::length2d, "2d length of shortest path")
      .def("numLanes", &Route::numLanes, "Number of inidividual lanes")
      .def(
          "laneletSubmap", +[](const Route& r) { return std::const_pointer_cast<LaneletSubmap>(r.laneletSubmap()); },
          "laneletSubmap with all lanelets that are part of the route")
      .def("getDebugLaneletMap", &Route::getDebugLaneletMap,
           "laneletMap that represents the Lanelets of the Route and their relationship")
      .def("size", &Route::size, "Number of lanelets")
      .def("followingRelations", &Route::followingRelations, "Provides the following lanelets within the Route")
      .def("previousRelations", &Route::previousRelations, "Provides the previous lanelets within the Route")
      .def("leftRelation", &Route::leftRelation, "Provides the lanelet left of a given lanelet within the Route")
      .def("leftRelations", &Route::leftRelations, "*all* lanelets left of a given lanelet within the Route")
      .def("rightRelation", &Route::rightRelation, "Provides the lanelet right of a given lanelet within the Route")
      .def("rightRelations", &Route::rightRelations, "*all* lanelets right of a given lanelet within the Route")
      .def("conflictingInRoute", &Route::conflictingInRoute, "conflicting lanelets of a lanelet within the route")
      .def("conflictingInMap", &Route::conflictingInMap,
           "conflicting lanelets of a lanelet within all passable lanelets in the laneletMap")
      .def("allConflictingInMap", &Route::allConflictingInMap,
           "all lanelets in the map that conflict with any lanelet in the route")
      .def(
          "forEachSuccessor",
          +[](Route& self, const ConstLanelet& from, object func) { self.forEachSuccessor(from, std::move(func)); },
          "calls a function on each successor of lanelet with increasing cost. The function must receives a "
          "LaneletVisitInformation object as input and must return a bool whether followers of the current lanelet "
          "should be visited as well. The function can raise an exception if an early exit is desired",
          (arg("lanelet"), arg("func")))
      .def(
          "forEachPredecessor",
          +[](Route& self, const ConstLanelet& from, object func) { self.forEachPredecessor(from, std::move(func)); },
          "similar to forEachSuccessor but instead goes backwards along the routing graph",
          (arg("lanelet"), arg("func")))
      .def("checkValidity", &Route::checkValidity, "perform sanity check on the route");
}
