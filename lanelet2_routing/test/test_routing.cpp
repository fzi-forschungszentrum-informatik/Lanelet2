#include <gtest/gtest.h>
#include <lanelet2_core/primitives/LaneletSequence.h>

#include <algorithm>

#include "lanelet2_routing/RoutingGraph.h"
#include "lanelet2_routing/internal/Graph.h"
#include "lanelet2_routing/internal/ShortestPath.h"
#include "test_routing_map.h"

using namespace lanelet;
using namespace lanelet::routing;
using namespace lanelet::routing::internal;
using namespace lanelet::routing::tests;

GraphType getSimpleGraph() {
  /*        3
   *    (1)---(3)
   *    /1 \1 /  \3
   *  (0)   X     (5)
   *    \2 /1 \  /1
   *    (2)----(4)
   *         3
   */
  GraphType g;
  auto v0 = boost::add_vertex(g);
  auto v1 = boost::add_vertex(g);
  auto v2 = boost::add_vertex(g);
  auto v3 = boost::add_vertex(g);
  auto v4 = boost::add_vertex(g);
  auto v5 = boost::add_vertex(g);
  auto addEdge = [](auto v0, auto v1, auto& g, double c) {
    auto e = boost::add_edge(v0, v1, g);
    g[e.first].routingCost = c;
  };
  addEdge(v0, v1, g, 1);
  addEdge(v0, v2, g, 2);
  addEdge(v1, v3, g, 3);
  addEdge(v1, v4, g, 1);
  addEdge(v2, v3, g, 1);
  addEdge(v2, v4, g, 3);
  addEdge(v3, v5, g, 3);
  addEdge(v4, v5, g, 1);
  return g;
}

TEST(DijkstraSearch, onSimpleGraph) {
  auto g = getSimpleGraph();
  DijkstraStyleSearch<GraphType> searcher(g);
  std::vector<double> expCost{0, 1, 2, 3, 2, 6};
  std::vector<size_t> length{1, 2, 2, 3, 3, 4};
  std::vector<GraphType::vertex_descriptor> predecessors{0, 0, 0, 2, 1, 3};
  searcher.query(0, [&](const VertexVisitInformation& v) -> bool {
    EXPECT_DOUBLE_EQ(expCost[v.vertex], v.cost) << v.vertex;
    EXPECT_EQ(length[v.vertex], v.length) << v.vertex;
    EXPECT_EQ(predecessors[v.vertex], v.predecessor) << v.vertex;
    EXPECT_EQ(v.length, v.numLaneChanges + 1);
    return v.vertex != 4;
  });
  EXPECT_EQ(searcher.getMap().size(), boost::num_vertices(g));
  for (const auto& v : searcher.getMap()) {
    EXPECT_EQ(v.second.predicate, v.first != 4) << v.first;
    EXPECT_EQ(v.second.isLeaf, v.first == 5 || v.first == 4) << v.first;
  }
}

TEST_F(GermanPedestrianGraph, NumberOfLanelets) {  // NOLINT
  EXPECT_EQ(graph->passableSubmap()->laneletLayer.size(), 5ul);
  EXPECT_TRUE(graph->passableSubmap()->laneletLayer.exists(2031));
  EXPECT_TRUE(graph->passableSubmap()->laneletLayer.exists(2050));
  EXPECT_EQ(graph->passableSubmap()->areaLayer.size(), 3ul);
  EXPECT_TRUE(graph->passableSubmap()->areaLayer.exists(3000));
  EXPECT_TRUE(graph->passableSubmap()->areaLayer.exists(3001));
  EXPECT_TRUE(graph->passableSubmap()->areaLayer.exists(3002));
}

TEST_F(GermanBicycleGraph, NumberOfLanelets) {  // NOLINT
  EXPECT_TRUE(graph->passableSubmap()->laneletLayer.exists(2013));
  EXPECT_FALSE(graph->passableSubmap()->laneletLayer.exists(2022));
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
  EXPECT_EQ(reachable.size(), 22ul);  // Will fail if people extend the map

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
  EXPECT_EQ(reachable.size(), 6ul);
  EXPECT_TRUE(containsLanelet(reachable, 2050));
  EXPECT_TRUE(containsLanelet(reachable, 2053));
  EXPECT_TRUE(containsLanelet(reachable, 2052));
  EXPECT_TRUE(containsLanelet(reachable, 3000));
  EXPECT_TRUE(containsLanelet(reachable, 3001));
  EXPECT_TRUE(containsLanelet(reachable, 3002));
}
TEST_F(GermanPedestrianGraph, reachableSetStartingFromArea) {  // NOLINT
  auto reachable = graph->reachableSetIncludingAreas(areas.at(3000), 100);
  EXPECT_EQ(reachable.size(), 5ul);
}
TEST_F(GermanPedestrianGraph, reachableSetWithAreaFromTwoWayLanelet) {  // NOLINT
  auto reachable = graph->reachableSetIncludingAreas(lanelets.at(2053).invert(), 100);
  EXPECT_TRUE(containsLanelet(reachable, 2053));
  EXPECT_EQ(reachable.size(), 6ul);
}
TEST_F(GermanPedestrianGraph, reachableSetWithAreaFromUnconnectedLanelet) {  // NOLINT
  auto reachable = graph->reachableSetIncludingAreas(lanelets.at(2051), 100);
  EXPECT_EQ(reachable.size(), 1ul);
}

TEST_F(GermanPedestrianGraph, possiblePathsWithAreaFromLanelet) {  // NOLINT
  auto reachable = graph->possiblePathsIncludingAreas(lanelets.at(2050), 10, 0, false);
  ASSERT_EQ(reachable.size(), 3ul);
  EXPECT_EQ(reachable[0].size(), 3ul);
  EXPECT_EQ(reachable[1].size(), 3ul);
  EXPECT_EQ(reachable[2].size(), 3ul);
}

TEST_F(GermanPedestrianGraph, possiblePathsWithAreaFromUnconnectedLanelet) {  // NOLINT
  auto reachable = graph->possiblePathsIncludingAreas(lanelets.at(2050), 3, false);
  ASSERT_EQ(reachable.size(), 3ul);
  EXPECT_EQ(reachable[0].size(), 3ul);
  EXPECT_EQ(reachable[1].size(), 3ul);
  EXPECT_EQ(reachable[2].size(), 3ul);
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

TEST_F(GermanVehicleGraph, possiblePathsIncludeShorterLc) {  // NOLINT
  auto routes = graph->possiblePaths(lanelets.at(2041), PossiblePathsParams{1000, {}, 0, true, true});
  EXPECT_EQ(routes.size(), 3);
  auto lastLLts = utils::transform(routes, [](auto& route) { return route.back(); });
  EXPECT_TRUE(has(lastLLts, lanelets.at(2062)));
  EXPECT_TRUE(has(lastLLts, lanelets.at(2049)));
  EXPECT_TRUE(has(lastLLts, lanelets.at(2048)));
}

TEST_F(GermanVehicleGraph, possiblePathsIncludeShorterAllLimitsLc) {  // NOLINT
  auto routes = graph->possiblePaths(lanelets.at(2041), PossiblePathsParams{1000, 100, 0, true, true});
  EXPECT_EQ(routes.size(), 3);
  auto lastLLts = utils::transform(routes, [](auto& route) { return route.back(); });
  EXPECT_TRUE(has(lastLLts, lanelets.at(2062)));
  EXPECT_TRUE(has(lastLLts, lanelets.at(2049)));
  EXPECT_TRUE(has(lastLLts, lanelets.at(2048)));
}

TEST_F(GermanVehicleGraph, possiblePathsIncludeShorterNoLc) {  // NOLINT
  auto routes = graph->possiblePaths(lanelets.at(2041), PossiblePathsParams{1000, {}, 0, false, true});
  EXPECT_EQ(routes.size(), 3);
  auto lastLLts = utils::transform(routes, [](auto& route) { return route.back(); });
  EXPECT_TRUE(has(lastLLts, lanelets.at(2063)));
  EXPECT_TRUE(has(lastLLts, lanelets.at(2049)));
  EXPECT_TRUE(has(lastLLts, lanelets.at(2047)));
}

TEST_F(GermanVehicleGraph, possiblePathsIncludeShorterAllLimitsNoLc) {  // NOLINT
  auto routes = graph->possiblePaths(lanelets.at(2041), PossiblePathsParams{1000, 100, 0, false, true});
  EXPECT_EQ(routes.size(), 3);
  auto lastLLts = utils::transform(routes, [](auto& route) { return route.back(); });
  EXPECT_TRUE(has(lastLLts, lanelets.at(2063)));
  EXPECT_TRUE(has(lastLLts, lanelets.at(2049)));
  EXPECT_TRUE(has(lastLLts, lanelets.at(2047)));
}

TEST_F(GermanVehicleGraph, possiblePathsLimitLengthNoLc) {  // NOLINT
  auto routes = graph->possiblePaths(lanelets.at(2041), PossiblePathsParams{2.5, 3, 0, false, false});
  EXPECT_TRUE(std::all_of(routes.begin(), routes.end(), [](auto& r) { return r.size() <= 3; }));
  EXPECT_EQ(routes.size(), 3);
  auto lastLLts = utils::transform(routes, [](auto& route) { return route.back(); });
  EXPECT_TRUE(has(lastLLts, lanelets.at(2042)));
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
  // Invalid num costs length
  EXPECT_THROW(graph->possiblePaths(lanelets.at(2002), 0., numCostModules, true), InvalidInputError);  // NOLINT
  auto routes = graph->possiblePaths(lanelets.at(2002), 0.);
  ASSERT_EQ(routes.size(), 1ul);
  EXPECT_EQ(routes[0].size(), 1ul);
  ConstLanelet invalid;
  routes = graph->possiblePaths(invalid, 10.0, 0, true);
  EXPECT_EQ(routes.size(), 0ul);

  // Invalid min number lanelets
  EXPECT_THROW(graph->possiblePaths(lanelets.at(2002), 1, numCostModules, true), InvalidInputError);  // NOLINT
  routes = graph->possiblePaths(invalid, 10, 0, true);
  EXPECT_EQ(routes.size(), 0ul);
}

TEST_F(GermanVehicleGraph, possiblePathsTowardsWithoutLc) {  // NOLINT
  auto routes = graph->possiblePathsTowards(lanelets.at(2024), 9, 0, false);
  ASSERT_EQ(routes.size(), 1UL);
  EXPECT_EQ(routes[0].front().id(), 2017);
  EXPECT_EQ(routes[0].back().id(), 2024);
}

TEST_F(GermanVehicleGraph, possiblePathsTowardsWithLc) {  // NOLINT
  auto routes = graph->possiblePathsTowards(lanelets.at(2015), 7, 0, true);
  ASSERT_EQ(routes.size(), 2UL);
  EXPECT_TRUE(containsLanelet(routes[0], 2009) || containsLanelet(routes[0], 2008));
  EXPECT_TRUE(containsLanelet(routes[1], 2009) || containsLanelet(routes[1], 2008));
}

TEST_F(GermanVehicleGraph, possiblePathsTowardsMinLanelets) {  // NOLINT
  auto routes = graph->possiblePathsTowards(lanelets.at(2015), 5, true);
  ASSERT_EQ(routes.size(), 2UL);
  EXPECT_TRUE(containsLanelet(routes[0], 2009) || containsLanelet(routes[0], 2008));
  EXPECT_TRUE(containsLanelet(routes[1], 2009) || containsLanelet(routes[1], 2008));
}

TEST_F(GermanVehicleGraph, forEachSuccessorIsMonotonic) {  // NOLINT
  double lastVal = 0.;
  bool lanelet2010Seen = false;
  bool lanelet2013Seen = false;
  graph->forEachSuccessor(lanelets.at(2007), [&](const LaneletVisitInformation& i) {
    if (i.cost == 0.) {
      EXPECT_EQ(i.lanelet.id(), 2007) << "First lanelet must be 2007";
    }
    EXPECT_LE(lastVal, i.cost);
    lastVal = i.cost;
    lanelet2010Seen |= i.lanelet.id() == 2010;
    lanelet2013Seen |= i.lanelet.id() == 2013;
    return i.lanelet.id() != 2010 && i.lanelet.id() != 2014;
  });
  EXPECT_TRUE(lanelet2010Seen);
  EXPECT_FALSE(lanelet2013Seen);
}

TEST_F(GermanPedestrianGraph, forEachSuccessorIncludingAreasReachesLanelet) {  // NOLINT
  class TargetFound {};
  auto throwIfTarget = [&](const LaneletOrAreaVisitInformation& i) {
    if (i.laneletOrArea.id() == 2053) {
      throw TargetFound{};
    }
    return true;
  };
  EXPECT_THROW(graph->forEachSuccessorIncludingAreas(lanelets.at(2050), throwIfTarget), TargetFound);  // NOLINT
}

TEST_F(GermanPedestrianGraph, forEachPredecessorIncludingAreasReachesLanelet) {  // NOLINT
  class TargetFound {};
  auto throwIfTarget = [&](const LaneletOrAreaVisitInformation& i) {
    if (i.laneletOrArea.id() == 2050) {
      throw TargetFound{};
    }
    return true;
  };
  EXPECT_THROW(graph->forEachPredecessorIncludingAreas(lanelets.at(2053), throwIfTarget), TargetFound);  // NOLINT
}

TEST_F(GermanVehicleGraph, forEachPredecessorIncludingAreasReachesLanelet) {  // NOLINT
  class TargetFound {};
  auto throwIfTarget = [&](const LaneletOrAreaVisitInformation& i) {
    if (i.laneletOrArea.id() == 2004) {
      throw TargetFound{};
    }
    return true;
  };
  EXPECT_THROW(graph->forEachPredecessorIncludingAreas(lanelets.at(2007), throwIfTarget), TargetFound);  // NOLINT
}

TEST_F(GermanVehicleGraph, forEachPredecessorReachesLanelet) {  // NOLINT
  class TargetFound {};
  auto throwIfTarget = [&](const LaneletVisitInformation& i) {
    if (i.lanelet.id() == 2004) {
      throw TargetFound{};
    }
    return true;
  };
  EXPECT_THROW(graph->forEachPredecessor(lanelets.at(2007), throwIfTarget), TargetFound);  // NOLINT
}

TEST_F(GermanPedestrianGraph, shortestPathIncludingAreasFromArea) {  // NOLINT
  auto path =
      graph->shortestPathIncludingAreas(ConstLaneletOrArea(areas.at(3001)), ConstLaneletOrArea(lanelets.at(2053)));
  ASSERT_TRUE(!!path);
  EXPECT_EQ(path->size(), 2ul);
}

TEST_F(GermanPedestrianGraph, shortestPathIncludingAreasThroughAreas) {
  auto path =
      graph->shortestPathIncludingAreas(ConstLaneletOrArea(lanelets.at(2050)), ConstLaneletOrArea(lanelets.at(2053)));
  ASSERT_TRUE(!!path);
  EXPECT_EQ(path->size(), 4ul);
}

TEST_F(GermanPedestrianGraph, shortestPathIncludingAreasViaThroughAreas) {
  auto path =
      graph->shortestPathIncludingAreasVia(ConstLaneletOrArea(lanelets.at(2050)), {ConstLaneletOrArea(areas.at(3002))},
                                           ConstLaneletOrArea(lanelets.at(2053)));
  ASSERT_TRUE(!!path);
  EXPECT_EQ(path->size(), 6ul);
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
