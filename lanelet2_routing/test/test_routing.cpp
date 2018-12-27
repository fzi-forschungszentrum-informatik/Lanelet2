#include <gtest/gtest.h>
#include <lanelet2_core/primitives/LaneletSequence.h>
#include <algorithm>
#include "RoutingGraph.h"
#include "test_routing_map.h"

using namespace lanelet;
using namespace lanelet::routing;
using namespace lanelet::routing::tests;

TEST_F(GermanPedestrianGraph, NumberOfLanelets) {  // NOLINT
  EXPECT_EQ(graph->passableMap()->laneletLayer.size(), 5ul);
  EXPECT_TRUE(graph->passableMap()->laneletLayer.exists(2031));
  EXPECT_TRUE(graph->passableMap()->laneletLayer.exists(2050));
  EXPECT_EQ(graph->passableMap()->areaLayer.size(), 2ul);
  EXPECT_TRUE(graph->passableMap()->areaLayer.exists(3000));
  EXPECT_TRUE(graph->passableMap()->areaLayer.exists(3001));
}

TEST_F(GermanBicycleGraph, NumberOfLanelets) {  // NOLINT
  EXPECT_TRUE(graph->passableMap()->laneletLayer.exists(2013));
  EXPECT_FALSE(graph->passableMap()->laneletLayer.exists(2022));
}

TEST_F(GermanVehicleGraph, GetShortestPath) {  // NOLINT
  // Multiple1
  auto shortestPath = graph->shortestPath(lanelets.at(2001), lanelets.at(2004), 0);
  ASSERT_TRUE(!!shortestPath);
  EXPECT_EQ(shortestPath->size(), 3ul);
  EXPECT_EQ((*shortestPath)[0], lanelets.at(2001));
  EXPECT_EQ((*shortestPath)[1], lanelets.at(2003));
  EXPECT_EQ((*shortestPath)[2], lanelets.at(2004));

  shortestPath = graph->shortestPath(lanelets.at(2003), lanelets.at(2002), 0);
  ASSERT_TRUE(!!shortestPath);
  EXPECT_EQ(shortestPath->size(), 3ul);
  EXPECT_EQ((*shortestPath)[0], lanelets.at(2003));
  EXPECT_EQ((*shortestPath)[1], lanelets.at(2001));
  EXPECT_EQ((*shortestPath)[2], lanelets.at(2002));

  shortestPath = graph->shortestPath(lanelets.at(2003), lanelets.at(2001), 0);
  ASSERT_TRUE(!!shortestPath);
  EXPECT_EQ(shortestPath->size(), 2ul);
  EXPECT_EQ((*shortestPath)[0], lanelets.at(2003));
  EXPECT_EQ((*shortestPath)[1], lanelets.at(2001));

  shortestPath = graph->shortestPath(lanelets.at(2003), lanelets.at(2004), 0);
  ASSERT_TRUE(!!shortestPath);
  EXPECT_EQ(shortestPath->size(), 2ul);
  EXPECT_EQ((*shortestPath)[0], lanelets.at(2003));
  EXPECT_EQ((*shortestPath)[1], lanelets.at(2004));

  shortestPath = graph->shortestPath(lanelets.at(2001), lanelets.at(2002), 0);
  ASSERT_TRUE(!!shortestPath);
  EXPECT_EQ(shortestPath->size(), 2ul);
  EXPECT_EQ((*shortestPath)[0], lanelets.at(2001));
  EXPECT_EQ((*shortestPath)[1], lanelets.at(2002));

  shortestPath = graph->shortestPath(lanelets.at(2004), lanelets.at(2002), 0);
  EXPECT_FALSE(!!shortestPath);

  shortestPath = graph->shortestPath(lanelets.at(2002), lanelets.at(2004), 0);
  EXPECT_FALSE(!!shortestPath);
}

TEST_F(GermanVehicleGraph, GetShortestPathMaxHose) {  // NOLINT
  auto shortestPath = graph->shortestPath(lanelets.at(2019), lanelets.at(2022), 0);
  ASSERT_TRUE(!!shortestPath);
  EXPECT_EQ(shortestPath->size(), 3ul);
  EXPECT_EQ((*shortestPath)[0], lanelets.at(2019));
  EXPECT_EQ((*shortestPath)[1], lanelets.at(2020));
  EXPECT_EQ((*shortestPath)[2], lanelets.at(2022));

  shortestPath = graph->shortestPath(lanelets.at(2021), lanelets.at(2018), 0);
  ASSERT_TRUE(!!shortestPath);
  EXPECT_EQ(shortestPath->size(), 3ul);
  EXPECT_EQ((*shortestPath)[0], lanelets.at(2021));
  EXPECT_EQ((*shortestPath)[1], lanelets.at(2020).invert());
  EXPECT_EQ((*shortestPath)[2], lanelets.at(2018));

  shortestPath = graph->shortestPath(lanelets.at(2019), lanelets.at(2018), 0);
  EXPECT_FALSE(!!shortestPath);
}

TEST_F(GermanVehicleGraph, GetShortestPathInvalid) {  // NOLINT
  ConstLanelet invalidLanelet;
  auto shortestPath = graph->shortestPath(invalidLanelet, lanelets.at(2004), 0);
  EXPECT_FALSE(!!shortestPath);

  EXPECT_THROW(graph->shortestPath(lanelets.at(2001), lanelets.at(2004), numCostModules), InvalidInputError);  // NOLINT
}

TEST_F(GermanVehicleGraph, GetShortestPathVia1) {  // NOLINT
  // Multiple1
  ConstLanelets interm;
  interm.push_back(static_cast<ConstLanelet>(lanelets.at(2003)));
  auto shortestPath = graph->shortestPathVia(lanelets.at(2001), interm, lanelets.at(2004), 0);
  ASSERT_TRUE(!!shortestPath);
  EXPECT_EQ(shortestPath->size(), 3ul);
  EXPECT_EQ((*shortestPath)[0], lanelets.at(2001));
  EXPECT_EQ((*shortestPath)[1], lanelets.at(2003));
  EXPECT_EQ((*shortestPath)[2], lanelets.at(2004));

  shortestPath = graph->shortestPathVia(lanelets.at(2001), interm, lanelets.at(2002), 0);
  ASSERT_TRUE(!!shortestPath);
  EXPECT_EQ(shortestPath->size(), 4ul);
  EXPECT_EQ((*shortestPath)[0], lanelets.at(2001));
  EXPECT_EQ((*shortestPath)[1], lanelets.at(2003));
  EXPECT_EQ((*shortestPath)[2], lanelets.at(2001));
  EXPECT_EQ((*shortestPath)[3], lanelets.at(2002));

  shortestPath = graph->shortestPathVia(lanelets.at(2004), interm, lanelets.at(2002), 0);
  ASSERT_FALSE(!!shortestPath);

  shortestPath = graph->shortestPathVia(lanelets.at(2002), interm, lanelets.at(2004), 0);
  ASSERT_FALSE(!!shortestPath);
}

TEST_F(GermanVehicleGraph, GetShortestPathVia2) {  // NOLINT
  // Multiple1 -> Multiple2
  ConstLanelets interm;
  interm.push_back(static_cast<ConstLanelet>(lanelets.at(2003)));
  interm.push_back(static_cast<ConstLanelet>(lanelets.at(2007)));
  auto shortestPath = graph->shortestPathVia(lanelets.at(2001), interm, lanelets.at(2013), 0);
  ASSERT_TRUE(!!shortestPath);
  EXPECT_EQ(shortestPath->size(), 8ul);
  auto pathIt = shortestPath->begin();
  EXPECT_EQ(*pathIt, lanelets.at(2001));
  EXPECT_EQ(*++pathIt, lanelets.at(2003));
  EXPECT_EQ(*++pathIt, lanelets.at(2004));
  EXPECT_EQ(*++pathIt, lanelets.at(2005));
  EXPECT_EQ(*++pathIt, lanelets.at(2007));
  EXPECT_EQ(*++pathIt, lanelets.at(2008));
  EXPECT_EQ(*++pathIt, lanelets.at(2010));
  EXPECT_EQ(*++pathIt, lanelets.at(2013));
}

TEST_F(GermanVehicleGraph, GetShortestPathViaInvalid) {  // NOLINT
  ConstLanelets interm;
  interm.push_back(static_cast<ConstLanelet>(lanelets.at(2003)));
  interm.push_back(static_cast<ConstLanelet>(lanelets.at(2007)));
  // NOLINTNEXTLINE
  EXPECT_THROW(graph->shortestPathVia(lanelets.at(2001), interm, lanelets.at(2002), numCostModules), InvalidInputError);
}

template <typename LaneletsT>
bool containsLanelet(const LaneletsT& reachableSet, Id lltId) {
  return !!utils::findIf(reachableSet, [lltId](auto& llt) { return llt.id() == lltId; });
}

TEST_F(GermanVehicleGraph, reachableSet) {  // NOLINT
  auto reachable = graph->reachableSet(lanelets.at(2001), 0., 0);
  EXPECT_EQ(reachable.size(), 1ul);
  EXPECT_TRUE(containsLanelet(reachable, 2001));

  reachable = graph->reachableSet(lanelets.at(2001), 2.1, 0);
  EXPECT_EQ(reachable.size(), 3ul);
  EXPECT_TRUE(containsLanelet(reachable, 2002));
  EXPECT_TRUE(containsLanelet(reachable, 2003));

  reachable = graph->reachableSet(lanelets.at(2001), 100, 0);
  EXPECT_EQ(reachable.size(), 15ul);  // Will fail if people extend the map

  reachable = graph->reachableSet(lanelets.at(2002), 100, 0);
  EXPECT_EQ(reachable.size(), 11ul);  // Will fail if people extend the map
}

TEST_F(GermanVehicleGraph, reachableSetMaxHose) {  // NOLINT
  auto reachable = graph->reachableSet(lanelets.at(2017), 100, 0);
  EXPECT_EQ(reachable.size(), 17ul);  // Will fail if people extend the map

  reachable = graph->reachableSet(lanelets.at(2021), 100, 0);
  EXPECT_EQ(reachable.size(), 4ul);
}

TEST_F(GermanVehicleGraph, reachableSetInvalid) {                                                // NOLINT
  EXPECT_THROW(graph->reachableSet(lanelets.at(2021), 0.0, numCostModules), InvalidInputError);  // NOLINT
  ConstLanelet invalid;
  auto reachable = graph->reachableSet(invalid, 0, 0);
  EXPECT_TRUE(reachable.empty());
}

TEST_F(GermanPedestrianGraph, reachableSetCrossingWithArea) {  // NOLINT
  auto reachable = graph->reachableSetIncludingAreas(lanelets.at(2050), 100);
  EXPECT_EQ(reachable.size(), 5ul);
  EXPECT_TRUE(containsLanelet(reachable, 2050));
  EXPECT_TRUE(containsLanelet(reachable, 2053));
  EXPECT_TRUE(containsLanelet(reachable, 2052));
  EXPECT_TRUE(containsLanelet(reachable, 3000));
  EXPECT_TRUE(containsLanelet(reachable, 3001));
}
TEST_F(GermanPedestrianGraph, reachableSetStartingFromArea) {  // NOLINT
  auto reachable = graph->reachableSetIncludingAreas(areas.at(3000), 100);
  EXPECT_EQ(reachable.size(), 4ul);
}
TEST_F(GermanPedestrianGraph, reachableSetWithAreaFromTwoWayLanelet) {  // NOLINT
  auto reachable = graph->reachableSetIncludingAreas(lanelets.at(2053).invert(), 100);
  EXPECT_EQ(reachable.size(), 4ul);
}
TEST_F(GermanPedestrianGraph, reachableSetWithAreaFromUnconnectedLanelet) {  // NOLINT
  auto reachable = graph->reachableSetIncludingAreas(lanelets.at(2051), 100);
  EXPECT_EQ(reachable.size(), 1ul);
}

TEST_F(GermanPedestrianGraph, possiblePathsWithAreaFromLanelet) {  // NOLINT
  auto reachable = graph->possiblePathsIncludingAreas(lanelets.at(2050), 10, 0, false);
  ASSERT_EQ(reachable.size(), 2ul);
  EXPECT_EQ(reachable[0].size(), 3ul);
  EXPECT_EQ(reachable[1].size(), 3ul);
}

TEST_F(GermanPedestrianGraph, possiblePathsWithAreaFromUnconnectedLanelet) {  // NOLINT
  auto reachable = graph->possiblePathsIncludingAreas(lanelets.at(2050), 3, false);
  ASSERT_EQ(reachable.size(), 2ul);
  EXPECT_EQ(reachable[0].size(), 3ul);
  EXPECT_EQ(reachable[1].size(), 3ul);
}

TEST_F(GermanVehicleGraph, possiblePathsMinRoutingCosts) {  // NOLINT
  // MIN ROUTING COST - With lane changes
  // Multiple 1
  auto routes = graph->possiblePaths(lanelets.at(2001), 2.2, 0, true);
  EXPECT_EQ(routes.size(), 2ul);

  routes = graph->possiblePaths(lanelets.at(2002), 4.0, 0, true);
  ASSERT_EQ(routes.size(), 1ul);
  auto& firstRoute = *routes.begin();
  EXPECT_EQ(firstRoute.size(), 3ul);
  EXPECT_TRUE(containsLanelet(firstRoute, 2006));
}

TEST_F(GermanVehicleGraph, possiblePathsMaxHose) {  // NOLINT
  // Max Hose Problem
  auto routes = graph->possiblePaths(lanelets.at(2017), 10.0, 0, true);
  ASSERT_EQ(routes.size(), 1ul);
  auto& firstRoute = *routes.begin();
  EXPECT_EQ(firstRoute.size(), 5ul);
  EXPECT_TRUE(containsLanelet(firstRoute, 2024));
}

TEST_F(GermanVehicleGraph, possiblePathsMinLanelets) {  // NOLINT
  // MIN NUMBER LANELETS - With lane changes
  auto routes = graph->possiblePaths(lanelets.at(2001), 2, false);
  EXPECT_EQ(routes.size(), 1ul);
  auto& firstRoute = *routes.begin();
  ASSERT_EQ(firstRoute.size(), 2ul);
  EXPECT_TRUE(containsLanelet(firstRoute, 2002));
  EXPECT_EQ(firstRoute.getRemainingLane(firstRoute.begin()).size(), firstRoute.size());

  routes = graph->possiblePaths(lanelets.at(2001), 30, false);
  EXPECT_EQ(routes.size(), 0ul);

  routes = graph->possiblePaths(lanelets.at(2001), 10, true);
  EXPECT_EQ(routes.size(), 0ul);

  routes = graph->possiblePaths(lanelets.at(2001), 7, true);
  EXPECT_EQ(routes.size(), 3ul);
}

TEST_F(GermanVehicleGraph, possiblePathsInvalid) {  // NOLINT
  // Invalid min length
  EXPECT_THROW(graph->possiblePaths(lanelets.at(2002), 0., numCostModules, true), InvalidInputError);  // NOLINT
  ConstLanelet invalid;
  auto routes = graph->possiblePaths(invalid, 10.0, 0, true);
  EXPECT_EQ(routes.size(), 0ul);

  // Invalid min number lanelets
  EXPECT_THROW(graph->possiblePaths(lanelets.at(2002), 1, numCostModules, true), InvalidInputError);  // NOLINT
  routes = graph->possiblePaths(invalid, 10, 0, true);
  EXPECT_EQ(routes.size(), 0ul);
}

TEST(RoutingCostInitialization, NegativeLaneChangeCost) {    // NOLINT
  EXPECT_NO_THROW(RoutingCostDistance(1));                   // NOLINT
  EXPECT_THROW(RoutingCostDistance(-1), InvalidInputError);  // NOLINT
}

class ConfigurationInitialization : public ::testing::Test {
 public:
  LaneletMapPtr emptyMap = std::make_shared<LaneletMap>();

  // Initialize traffic rules and routing cost calculation module
  traffic_rules::TrafficRulesPtr trafficRules{traffic_rules::TrafficRulesFactory::create(
      Locations::Germany, Participants::Vehicle, traffic_rules::TrafficRules::Configuration())};

  RoutingCostPtrs costPtrs{std::make_shared<RoutingCostDistance>(RoutingCostDistance{2.})};

  // Optional: Initialize config for routing graph:
  RoutingGraph::Configuration routingGraphConf;
};

TEST_F(ConfigurationInitialization, EmptyConfig) {                                             // NOLINT
  EXPECT_NO_THROW(RoutingGraph::build(*emptyMap, *trafficRules, costPtrs, routingGraphConf));  // NOLINT
}

TEST_F(ConfigurationInitialization, Config) {                                                  // NOLINT
  routingGraphConf.emplace(std::make_pair(RoutingGraph::ParticipantHeight, Attribute("2.")));  // NOLINT
  EXPECT_NO_THROW(RoutingGraph::build(*emptyMap, *trafficRules, costPtrs, routingGraphConf));  // NOLINT
}

TEST_F(ConfigurationInitialization, NoConfig) {                              // NOLINT
  EXPECT_NO_THROW(RoutingGraph::build(*emptyMap, *trafficRules, costPtrs));  // NOLINT
}
