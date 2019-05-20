#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <boost/python.hpp>
#include "converter.h"

using namespace boost::python;
using namespace lanelet;

Optional<std::shared_ptr<lanelet::routing::Route>> getRouteWrapper(const lanelet::routing::RoutingGraph& self,
                                                                   const ConstLanelet& from, const ConstLanelet& to,
                                                                   lanelet::routing::RoutingCostId costId) {
  auto route = self.getRoute(from, to, costId);
  if (!route) {
    return {};
  }
  return std::make_shared<lanelet::routing::Route>(std::move(*route));
}

Optional<std::shared_ptr<lanelet::routing::Route>> getRouteViaWrapper(const lanelet::routing::RoutingGraph& self,
                                                                      const ConstLanelet& from,
                                                                      const ConstLanelets& via, const ConstLanelet& to,
                                                                      lanelet::routing::RoutingCostId costId) {
  auto route = self.getRouteVia(from, via, to, costId);
  if (!route) {
    return {};
  }
  return std::make_shared<lanelet::routing::Route>(std::move(*route));
}

routing::RoutingGraphPtr makeRoutingGraph(LaneletMap& laneletMap, const traffic_rules::TrafficRules& trafficRules,
                                          const routing::RoutingCostPtrs& routingCosts) {
  return routing::RoutingGraph::build(laneletMap, trafficRules, routingCosts);
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
  OptionalConverter<RouteElementRelation>();
  OptionalConverter<LaneletPath>();

  VectorToListConverter<LaneletRelations>();
  VectorToListConverter<RelationTypes>();
  VectorToListConverter<RouteElementRelations>();
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
      "RoutingCostDistance", "Travel time based routing cost calculation object",
      init<double, double>((arg("laneChangeCost"), arg("minLaneChangeTime") = 0)));

  auto possR1 = static_cast<LaneletPaths (RoutingGraph::*)(const ConstLanelet&, double, RoutingCostId, bool) const>(
      &RoutingGraph::possiblePaths);
  auto possR2 = static_cast<LaneletPaths (RoutingGraph::*)(const ConstLanelet&, uint32_t, bool) const>(
      &RoutingGraph::possiblePaths);

  class_<LaneletPath>("LaneletPath",
                      "A set of consecutive lanelets connected in straight "
                      "directon or by lane changes",
                      init<ConstLanelets>(arg("lanelets")))
      .def("__iter__", iterator<LaneletPath>())
      .def("__len__", &LaneletPath::size)
      .def("__getitem__", wrappers::getItem<LaneletPath>, return_internal_reference<>())
      .def("getRemainingLane", +[](const LaneletPath& self, ConstLanelet llt) { return self.getRemainingLane(llt); },
           "get the sequence of remaining lanelets without a lane change")
      .def(self == self)   // NOLINT
      .def(self != self);  // NOLINT

  class_<RoutingGraph, boost::noncopyable, RoutingGraphPtr>(
      "RoutingGraph",
      "Main class of the routing module that holds routing information and can "
      "be queried",
      no_init)
      .def("__init__",
           make_constructor(makeRoutingGraph, default_call_policies(),
                            (arg("laneletMap"), arg("trafficRules"), arg("routingCost") = defaultRoutingCosts())),
           "Initialization with default routing costs")
      .def("getRoute", getRouteWrapper, "driving route from 'start' to 'end' lanelet",
           (arg("from"), arg("to"), arg("routingCostId") = 0))
      .def("getRouteVia", getRouteWrapper,
           "driving route from 'start' to 'end' lanelet using the 'via' "
           "lanelets",
           (arg("from"), arg("via"), arg("to"), arg("routingCostId") = 0))
      .def("shortestPath", &RoutingGraph::shortestPath, "shortest path between 'start' and 'end'",
           (arg("from"), arg("to"), arg("routingCostId") = 0))
      .def("shortestPathWithVia", &RoutingGraph::shortestPathVia,
           "shortest path between 'start' and 'end' using intermediate points",
           (arg("start"), arg("via"), arg("end"), arg("routingCostId") = 0))
      .def("routingRelation", &RoutingGraph::routingRelation, "relation between two lanelets excluding 'conflicting'",
           (arg("from"), arg("to")))
      .def("following", &RoutingGraph::following, "lanelets that can be reached from this lanelet",
           (arg("lanelet"), arg("withLaneChanges") = false))
      .def("followingRelations", &RoutingGraph::followingRelations, "relations to following lanelets",
           (arg("lanelet"), arg("withLaneChanges") = false))
      .def("previous", &RoutingGraph::previous, "previous lanelets of this lanelet",
           (arg("lanelet"), arg("withLaneChanges") = false))
      .def("previousRelations", &RoutingGraph::previousRelations, "relations to preceding lanelets",
           (arg("lanelet"), arg("withLaneChanges") = false))
      .def("besides", &RoutingGraph::besides,
           "all reachable left and right lanelets, including lanelet, from "
           "left to right",
           (arg("lanelet")))
      .def("left", &RoutingGraph::left, "left (routable)lanelet, if exists", (arg("lanelet")))
      .def("right", &RoutingGraph::right, "right (routable)lanelet, if it exists", (arg("lanelet")))
      .def("adjacentLeft", &RoutingGraph::adjacentLeft, "left non-routable lanelet", arg("lanelet"))
      .def("adjacentRight", &RoutingGraph::adjacentRight, "right non-routable lanelet", arg("lanelet"))
      .def("lefts", &RoutingGraph::lefts, "all left (routable) lanelets", arg("lanelet"))
      .def("rights", &RoutingGraph::rights, "all right (routable) lanelets", arg("lanelet"))
      .def("adjacentLefts", &RoutingGraph::adjacentLefts, "all left (non-routable) lanelets", arg("lanelet"))
      .def("adjacentRights", &RoutingGraph::adjacentRights, "all right (non-routable) lanelets", arg("lanelet"))
      .def("leftRelations", &RoutingGraph::leftRelations, "relations to left lanelets", arg("lanelet"))
      .def("rightRelations", &RoutingGraph::rightRelations, "relations to right lanelets", arg("lanelet"))
      .def("conflicting", &RoutingGraph::conflicting, "Conflicting lanelets", arg("lanelet"))
      .def("reachableSet", &RoutingGraph::reachableSet, "set of lanelets that can be reached from a given lanelet",
           (arg("lanelet"), arg("maxRoutingCost"), arg("RoutingCostId") = 0))
      .def("possiblePaths", possR1,
           "possible routes from a given start lanelet that are "
           "'minRoutingCost'-long",
           (arg("lanelet"), arg("minRoutingCost"), arg("RoutingCostId") = 0, arg("allowLaneChanges") = false))
      .def("possiblePaths", possR2,
           "possible routes from a given start lanelet that are "
           "'minLanelets'-long",
           (arg("lanelet"), arg("minLanelets"), arg("allowLaneChanges")))
      .def("exportGraphML", +[](RoutingGraph& self, const std::string& path) { self.exportGraphML(path); },
           "Export the internal graph to graphML (xml-based) file format")
      .def("exportGraphViz", +[](RoutingGraph& self, const std::string& path) { self.exportGraphViz(path); },
           "Export the internal graph to graphViz (DOT) file format")
      .def("getDebugLaneletMap", &RoutingGraph::getDebugLaneletMap,
           "abstract lanelet map holding the information of the routing graph",
           (arg("routingCostId") = 0, arg("includeAdjacent") = false, arg("includeConflicting") = false))
      .def("passableLaneletMap", &RoutingGraph::passableMap, "LaneletMap that includes all passable lanelets")
      .def("checkValidity", &RoutingGraph::checkValidity, "Performs some basic sanity checks",
           (arg("throwOnError") = true));

  class_<RouteElementRelation>("RouteElementRelation", "Represents relation of a lanelet to another lanelet")
      .add_property("routeElement", &RouteElementRelation::routeElement)
      .add_property("relationType", &RouteElementRelation::relationType);

  class_<LaneletRelation>("LaneletRelation")
      .add_property("lanelet", &LaneletRelation::lanelet)
      .add_property("relationType", &LaneletRelation::relationType);

  enum_<RelationType>("RelationType")
      .value("Successor", RelationType::Successor)
      .value("Left", RelationType::Left)
      .value("Right", RelationType::Right)
      .value("Conflicting", RelationType::Conflicting)
      .value("Merging", RelationType::Merging)
      .value("Diverging", RelationType::Diverging)
      .value("AdjacentLeft", RelationType::AdjacentLeft)
      .value("AdjacentRight", RelationType::AdjacentRight)
      .export_values();

  class_<RouteElement, boost::noncopyable, std::shared_ptr<RouteElement>>("RouteElement",
                                                                          "fundamental building block of a route "
                                                                          "that represents the relations of one "
                                                                          "single lanelet",
                                                                          init<ConstLanelet, RouteElement::LaneId>())
      .def("initLaneId", &RouteElement::initLaneId, "temporary laneID used when creating the route")
      .def("laneId", &RouteElement::laneId, "laneID of the route element")
      .add_property("left", make_function(&RouteElement::left, return_internal_reference<>()), &RouteElement::setLeft,
                    "the relation to the left lanelet if it exists")
      .add_property("right", make_function(&RouteElement::right, return_internal_reference<>()),
                    &RouteElement::setRight, "the relation to the right lanelet if it exists")
      .add_property("lanelet", make_function(&RouteElement::lanelet, return_internal_reference<>()),
                    "The lanelet this is all about")
      .add_property("previous", make_function(&RouteElement::previous, return_internal_reference<>()),
                    "relations to the previous lanelets if they exist")
      .add_property("following", make_function(&RouteElement::following, return_internal_reference<>()),
                    "relations to the following lanelets if they exist")
      .add_property("conflictingInRoute",
                    make_function(&RouteElement::conflictingInRoute, return_internal_reference<>()),
                    "conflicting lanelets within the route if they exist")
      .add_property("conflictingInMap", make_function(&RouteElement::conflictingInMap, return_internal_reference<>()),
                    "conflicting lanelets within the passable lanelets in the "
                    "base laneletMap if they exist")
      .def("id", &RouteElement::id, "ID of the referenced lanelet");

  class_<Route, boost::noncopyable, std::shared_ptr<Route>>("Route",
                                                            "The famous route object that marks a route from A to B, "
                                                            "including all lanelets that can be used",
                                                            no_init)
      .def("shortestPath", &Route::shortestPath, "Returns the shortest path along this route",
           return_internal_reference<>())
      .def("fullLane", &Route::fullLane, "Returns the complete lane a Lanelet belongs to")
      .def("remainingLane", &Route::remainingLane, "Returns that remaining lane a Lanelet belongs to")
      .def("length2d", &Route::length2d, "2d length of shortest path")
      .def("numLanes", &Route::numLanes, "Number of inidividual lanes")
      .def("laneletMap", &Route::laneletMap, "laneletMap with all lanelets that are part of the route")
      .def("getDebugLaneletMap", &Route::getDebugLaneletMap,
           "laneletMap that represents the Lanelets of the Route "
           "and their relationship")
      .def("size", &Route::size, "Number of lanelets")
      .def("followingRelations", &Route::followingRelations, "Provides the following lanelets within the Route")
      .def("previousRelations", &Route::previousRelations, "Provides the previous lanelets within the Route")
      .def("leftRelation", &Route::leftRelation,
           "Provides the lanelet left of a given lanelet within the "
           "Route")
      .def("leftRelations", &Route::leftRelations, "*all* lanelets left of a given lanelet within the Route")
      .def("rightRelation", &Route::rightRelation,
           "Provides the lanelet right of a given lanelet within the "
           "Route")
      .def("rightRelations", &Route::rightRelations, "*all* lanelets right of a given lanelet within the Route")
      .def("conflictingInRoute", &Route::conflictingInRoute, "conflicting lanelets of a lanelet within the route")
      .def("conflictingInMap", &Route::conflictingInMap,
           "conflicting lanelets of a lanelet within all passable "
           "lanelets in the laneletMap")
      .def("allConflictingInMap", &Route::allConflictingInMap,
           "all lanelets in the map that conflict with any lanelet in the "
           "route")
      .def("checkValidity", &Route::checkValidity, "perform sanity check on the route");
}
