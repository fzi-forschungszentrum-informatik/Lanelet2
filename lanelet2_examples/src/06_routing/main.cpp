#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

// we want assert statements to work in release mode
#undef NDEBUG

namespace {
std::string exampleMapPath = std::string(PKG_DIR) + "/../lanelet2_maps/res/mapping_example.osm";
}  // namespace

void part1CreatingAndUsingRoutingGraphs();
void part2UsingRoutes();
void part3UsingRoutingGraphContainers();

int main() {
  // this tutorial finally shows you how to use routing graphs.
  part1CreatingAndUsingRoutingGraphs();
  part2UsingRoutes();
  part3UsingRoutingGraphContainers();
  return 0;
}

void part1CreatingAndUsingRoutingGraphs() {
  using namespace lanelet;
  // a routing graph is created from a map, traffic rules and routing cost. Each routing graph can be created with
  // multiple different routing cost calculations. By default distance and time (using the speed limit of each
  // lanelet) are used.
  // The choice of the traffic object influences from which perspective the routing graph will be generated. We will
  // make a routing graph for vehicles:
  LaneletMapPtr map = load(exampleMapPath, projection::UtmProjector(Origin({49, 8.4})));
  traffic_rules::TrafficRulesPtr trafficRules =
      traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Vehicle);
  routing::RoutingGraphUPtr routingGraph = routing::RoutingGraph::build(*map, *trafficRules);

  // the routing graph can be used to query thre different kinds of information: Neighbourhood relations for a
  // specific lanelet/area, possible routes from a lanelet/area and shortest routes between two lanelets.

  // we will do the following queries from the perspective of lanelet 4984315 (if you search it in josm: it is the
  // leftmost lanelet on the bottom right intersection arm leading into the roundabout.
  // When working with the routing graph, it is a good idea to use Const objects because the routing graph will not
  // notice (unwanted) changes to the map.
  ConstLanelet lanelet = map->laneletLayer.get(4984315);

  // lets inspect the neighbourhood
  assert(!routingGraph->adjacentLeft(lanelet));
  assert(!routingGraph->adjacentRight(lanelet));  // adjacent lanelets are neighbours but not reachable by lane change
  assert(!!routingGraph->right(lanelet));         // right lanelets are lane-changable neighbours
  assert(routingGraph->besides(lanelet).size() == 3);  // besides gives us the "slice" of the road where the lanelet is
  assert(routingGraph->following(lanelet).size() == 1);
  assert(routingGraph->conflicting(lanelet).empty());  // we can also search for conflicting lanelets/areas, but this
                                                       // will only return conflicts with other lanelets of the same
                                                       // participant type. E.g. this will never return a crosswalk
                                                       // when this graph is for vehicles.

  // we can also find out what paths are possible from here. We chose routing cost id 0, this will give us routes that
  // are at least 100m long.
  routing::LaneletPaths paths = routingGraph->possiblePaths(lanelet, 100, 0, false);
  assert(paths.size() == 1);  // there is just one possible path if lane changes are excluded
  paths = routingGraph->possiblePaths(lanelet, 100, 0, true);
  assert(paths.size() == 4);  // with lane changes, we have more options

  // you can also obtain the reachable set, this is basically the same as possible paths, but the lanelets are in an
  // unsorted order and contain no duplicates. Also, possiblePaths discards paths that are below the cost threshold
  // while reachable set keeps them all.
  ConstLanelets reachableSet = routingGraph->reachableSet(lanelet, 100, 0);
  assert(reachableSet.size() > 10);

  // finally the routing stuff. Here we only concentrate on shortest path and will cover the more mighty Route type in
  // the next part.
  ConstLanelet toLanelet = map->laneletLayer.get(2925017);
  Optional<routing::LaneletPath> shortestPath = routingGraph->shortestPath(lanelet, toLanelet, 1);
  assert(!!shortestPath);

  // the shortest path can contain (sudden) lane changes. It is more a guideline for the vehicle which lanelets should
  // be preferred than a full route recommendation. You can query the path for the sequence of lanelets that you can
  // follow until you must make a lane change:
  LaneletSequence lane = shortestPath->getRemainingLane(shortestPath->begin());
  assert(!lane.empty());

  // routing graph also have a self-check mechanism that validates if all parts of the graph are in a sane state.
  routingGraph->checkValidity();
}

void part2UsingRoutes() {
  using namespace lanelet;
  LaneletMapPtr map = load(exampleMapPath, projection::UtmProjector(Origin({49, 8.4})));
  traffic_rules::TrafficRulesPtr trafficRules =
      traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Vehicle);
  routing::RoutingGraphUPtr routingGraph = routing::RoutingGraph::build(*map, *trafficRules);
  ConstLanelet lanelet = map->laneletLayer.get(4984315);
  ConstLanelet toLanelet = map->laneletLayer.get(2925017);

  // the Route object is built upon the shortest path between start and destination. But instead of only containing
  // the lanelets along the shortest path, it contains all lanelets that can be used to the destination without
  // leaving the chosen road. When you want to know all your options of getting to the destination (including lane
  // changes), this is the object to go for.
  Optional<routing::Route> route = routingGraph->getRoute(lanelet, toLanelet, 0);
  assert(!!route);

  // if we want the actual underlying shortest path, we can now ask the route object
  routing::LaneletPath shortestPath = route->shortestPath();
  assert(!shortestPath.empty());

  // now we can get individual lanes from the route. They are as long as possible before either the destination is
  // reached or we have to make a lane change:
  LaneletSequence fullLane = route->fullLane(lanelet);
  assert(!fullLane.empty());

  // but we can also check for earlier possibilites of a lane change or query other relations, similar to the routing
  // graph.
  auto right = route->rightRelation(lanelet);
  assert(!!right);

  // finally, we can also create a lanelet submap from the route that only contains the relevant primitives for the
  // chosen route:
  LaneletSubmapConstPtr routeMap = route->laneletSubmap();
  assert(!routeMap->laneletLayer.empty());
}

void part3UsingRoutingGraphContainers() {
  // since routing graphs only contain primitives that are usable for one specific participant, we can not use them to
  // query possible conflicts between different participants. This problem is solved by the routing graph container:
  // It finds conflicts between individual participants by comparing the topology of the different graphs.
  using namespace lanelet;
  LaneletMapPtr map = load(exampleMapPath, projection::UtmProjector(Origin({49, 8.4})));
  traffic_rules::TrafficRulesPtr trafficRules =
      traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Vehicle);
  traffic_rules::TrafficRulesPtr pedestrianRules =
      traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Pedestrian);
  routing::RoutingGraphConstPtr vehicleGraph = routing::RoutingGraph::build(*map, *trafficRules);
  routing::RoutingGraphConstPtr pedestrGraph = routing::RoutingGraph::build(*map, *pedestrianRules);
  routing::RoutingGraphContainer graphs({vehicleGraph, pedestrGraph});
  ConstLanelet lanelet = map->laneletLayer.get(4984315);
  ConstLanelet intersectLanelet = map->laneletLayer.get(185265);
  assert(graphs.conflictingInGraph(lanelet, 1).empty());
  assert(graphs.conflictingInGraph(intersectLanelet, 1).size() == 1);
}
