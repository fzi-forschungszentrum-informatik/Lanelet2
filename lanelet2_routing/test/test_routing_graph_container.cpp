#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>

#include "lanelet2_routing/Forward.h"
#include "lanelet2_routing/RoutingGraph.h"
#include "lanelet2_routing/RoutingGraphContainer.h"
#include "test_routing_map.h"

using namespace lanelet;
using namespace lanelet::routing;
using namespace lanelet::routing::tests;

class RoutingGraphContainerTest : public RoutingGraphTest {
 public:
  RoutingGraphContainerTest() {
    std::vector<RoutingGraphConstPtr> graphs{testData.vehicleGraph, testData.pedestrianGraph};
    container = std::make_unique<RoutingGraphContainer>(graphs);
  }

  RoutingGraphContainerUPtr container;
};

class RouteRoutingGraphContainerTest : public RoutingGraphContainerTest {
 public:
  void getAndCheckRoute(const RoutingGraphConstPtr& graph, Id from, Id to, routing::RoutingCostId routingCostId = 0) {
    ASSERT_NE(graph->passableSubmap()->laneletLayer.find(from), graph->passableSubmap()->laneletLayer.end());
    ConstLanelet fromLanelet{*graph->passableSubmap()->laneletLayer.find(from)};
    ASSERT_NE(graph->passableSubmap()->laneletLayer.find(to), graph->passableSubmap()->laneletLayer.end());
    ConstLanelet toLanelet{*graph->passableSubmap()->laneletLayer.find(to)};
    Optional<Route> tempRoute = graph->getRoute(fromLanelet, toLanelet, routingCostId);
    ASSERT_TRUE(!!tempRoute);
    route = std::make_unique<Route>(std::move(*tempRoute));
  }

  std::unique_ptr<Route> route;
};

TEST_F(RoutingGraphContainerTest, ConflictingInGraph) {  // NOLINT
  ConstLanelet pedestrianLanelet{*laneletMap->laneletLayer.find(2031)};
  ConstLanelets conflictingVehicle{container->conflictingInGraph(pedestrianLanelet, 0)};
  ASSERT_EQ(conflictingVehicle.size(), 1ul);
  EXPECT_EQ(conflictingVehicle[0], *laneletMap->laneletLayer.find(2020));

  ConstLanelets conflictingPedestrian{container->conflictingInGraph(pedestrianLanelet, 1)};
  EXPECT_EQ(conflictingPedestrian.size(), 0ul);
}

TEST_F(RoutingGraphContainerTest, ConflictingInGraphs) {  // NOLINT
  ConstLanelet pedestrianLanelet{*laneletMap->laneletLayer.find(2031)};
  RoutingGraphContainer::ConflictingInGraphs result{container->conflictingInGraphs(pedestrianLanelet)};
  ASSERT_EQ(result.size(), 2ul);

  ConstLanelets conflictingVehicle{result[0].second};
  ASSERT_EQ(conflictingVehicle.size(), 1ul);
  EXPECT_EQ(conflictingVehicle[0], *laneletMap->laneletLayer.find(2020));

  ConstLanelets conflictingPedestrian{result[1].second};
  EXPECT_EQ(conflictingPedestrian.size(), 0ul);
}

TEST_F(RouteRoutingGraphContainerTest, ConflictingOfRouteInGraph) {  // NOLINT
  getAndCheckRoute(container->routingGraphs()[0], 2017, 2024);
  ConstLanelets conflictingVehicle{container->conflictingOfRouteInGraph(route.get(), 0)};
  EXPECT_EQ(conflictingVehicle.size(), 2ul);

  ConstLanelets conflictingPedestrian{container->conflictingOfRouteInGraph(route.get(), 1)};
  EXPECT_EQ(conflictingPedestrian.size(), 1ul);
}

TEST_F(RouteRoutingGraphContainerTest, ConflictingOfRouteInGraphs) {  // NOLINT
  getAndCheckRoute(container->routingGraphs()[0], 2017, 2024);
  RoutingGraphContainer::ConflictingInGraphs result{container->conflictingOfRouteInGraphs(route.get(), 0)};
  ASSERT_EQ(result.size(), 2ul);

  ConstLanelets conflictingVehicle{result[0].second};
  EXPECT_EQ(conflictingVehicle.size(), 2ul);

  ConstLanelets conflictingPedestrian{result[1].second};
  EXPECT_EQ(conflictingPedestrian.size(), 1ul);
}

TEST_F(RoutingGraphContainerTest, ConflictingInGraph3dFits) {  // NOLINT
  ConstLanelet bridgeLanelet{*laneletMap->laneletLayer.find(2032)};
  ConstLanelets conflictingVehicle{container->conflictingInGraph(bridgeLanelet, 0, 2.)};
  EXPECT_EQ(conflictingVehicle.size(), 0ul);

  conflictingVehicle = container->conflictingInGraph(lanelets.find(2005)->second, 0, 2.);
  ASSERT_EQ(conflictingVehicle.size(), 1ul);
  EXPECT_TRUE(conflictingVehicle.front() == lanelets.find(2006)->second);

  conflictingVehicle = container->conflictingInGraph(lanelets.find(2006)->second, 0, 2.);
  ASSERT_EQ(conflictingVehicle.size(), 1ul);
  EXPECT_TRUE(conflictingVehicle.front() == lanelets.find(2005)->second);

  ConstLanelets conflictingPedestrian{container->conflictingInGraph(bridgeLanelet, 1, 2.)};
  EXPECT_EQ(conflictingPedestrian.size(), 0ul);
}

TEST_F(RoutingGraphContainerTest, ConflictingInGraph3dDoesntFit) {  // NOLINT
  ConstLanelet bridgeLanelet{*laneletMap->laneletLayer.find(2032)};
  ConstLanelets conflictingVehicle{container->conflictingInGraph(bridgeLanelet, 0, 4.)};
  EXPECT_EQ(conflictingVehicle.size(), 2ul);

  conflictingVehicle = container->conflictingInGraph(lanelets.find(2005)->second, 0, 4.);
  EXPECT_EQ(conflictingVehicle.size(), 2ul);

  conflictingVehicle = container->conflictingInGraph(lanelets.find(2006)->second, 0, 4.);
  EXPECT_EQ(conflictingVehicle.size(), 2ul);

  ConstLanelets conflictingPedestrian{container->conflictingInGraph(bridgeLanelet, 1, 4.)};
  EXPECT_EQ(conflictingPedestrian.size(), 0ul);
}
