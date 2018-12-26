#include <gtest/gtest.h>
#include "Route.h"
#include "RoutingGraph.h"
#include "test_routing_map.h"

using namespace lanelet;
using namespace lanelet::routing;
using namespace lanelet::routing::tests;

class TestRoute : public GermanVehicleGraph {
 public:
  void setUpRoute(Id from, Id to, uint16_t routingCostId = 0) {
    ASSERT_TRUE(lanelets.find(from) != lanelets.end());
    start = lanelets.find(from)->second;
    ASSERT_TRUE(lanelets.find(to) != lanelets.end());
    end = lanelets.find(to)->second;
    Optional<Route> optRoute{graph->getRoute(start, end, routingCostId)};
    ASSERT_TRUE(!!optRoute);
    route = std::make_unique<Route>(std::move(*optRoute));
  }

  ConstLanelet start;
  ConstLanelet end;
  RouteUPtr route;
};

class Route1 : public TestRoute {
 public:
  Route1() { setUpRoute(from, to); }

  Id from{2001};
  Id to{2014};
};

class Route2 : public TestRoute {
 public:
  Route2() { setUpRoute(from, to); }

  Id from{2001};
  Id to{2004};
};

class Route3 : public TestRoute {
 public:
  void SetUp() override { setUpRoute(from, to); }

  Id from{2003};
  Id to{2002};
};

class Route4 : public TestRoute {
 public:
  void SetUp() override { setUpRoute(from, to); }

  Id from{2003};
  Id to{2013};
};

class RouteMaxHoseLeftRight : public TestRoute {
 public:
  void SetUp() override { setUpRoute(from, to); }

  Id from{2017};
  Id to{2024};
};

class RouteMaxHoseRightLeft : public TestRoute {
 public:
  void SetUp() override { setUpRoute(from, to); }

  Id from{2023};
  Id to{2016};
};

class RouteMaxHoseLeftRightDashedSolid : public TestRoute {
 public:
  void SetUp() override { setUpRoute(from, to); }

  Id from{2017};
  Id to{2025};
};

class RouteMaxHoseLeftRightDashedSolidFurther : public TestRoute {
 public:
  void SetUp() override { setUpRoute(from, to); }

  Id from{2017};
  Id to{2027};
};

class RouteSolidDashed : public TestRoute {
 public:
  void SetUp() override { setUpRoute(from, to); }

  Id from{2024};
  Id to{2026};
};

class RouteSolidDashedWithAdjacent : public TestRoute {
 public:
  void SetUp() override { setUpRoute(from, to); }

  Id from{2024};
  Id to{2029};
};

class RouteSplittedDiverging : public TestRoute {
 public:
  void SetUp() override { setUpRoute(from, to); }

  Id from{2029};
  Id to{2038};
};

class RouteSplittedDivergingAndMerging : public TestRoute {
 public:
  void SetUp() override { setUpRoute(from, to); }

  Id from{2041};
  Id to{2049};
};

template <typename T>
class AllRoutesTest : public T {};

using AllRoutes =
    testing::Types<Route1, Route2, Route3, Route4, RouteMaxHoseLeftRight, RouteMaxHoseRightLeft,
                   RouteMaxHoseLeftRightDashedSolid, RouteMaxHoseLeftRightDashedSolidFurther, RouteSolidDashed,
                   RouteSolidDashedWithAdjacent, RouteSplittedDiverging, RouteSplittedDivergingAndMerging>;

TYPED_TEST_CASE(AllRoutesTest, AllRoutes);

TYPED_TEST(AllRoutesTest, CheckValidity) { EXPECT_NO_THROW(this->route->checkValidity()); }

TYPED_TEST(AllRoutesTest, ShortestMapInDebugMap) {
  const auto map{this->route->getDebugLaneletMap()};
  for (const auto& el : this->route->shortestPath()) {
    ASSERT_NE(map->pointLayer.find(el.id()), map->pointLayer.end());
    const Point3d point{*map->pointLayer.find(el.id())};
    EXPECT_TRUE(*point.attribute("shortest_path").asBool());
  }
}

TYPED_TEST(AllRoutesTest, DebugMapCompleteness) {
  const LaneletMapPtr debugMap{this->route->getDebugLaneletMap()};
  const LaneletMapConstPtr laneletMap{this->route->laneletMap()};

  for (const auto& it : laneletMap->laneletLayer) {
    ASSERT_NE(debugMap->pointLayer.find(it.id()), debugMap->pointLayer.end());
    Point3d point{*debugMap->pointLayer.find(it.id())};
    EXPECT_NO_THROW(point.attribute("id"));
    EXPECT_NO_THROW(point.attribute("lane_id"));
    EXPECT_NO_THROW(point.attribute("init_lane_id"));
  }

  for (const auto& it : debugMap->lineStringLayer) {
    EXPECT_NO_THROW(it.attribute("relation_1"));
  }
}

TEST_F(Route1, CreateRoute1) {     // NOLINT
  EXPECT_EQ(route->size(), 15ul);  // NOLINT
  EXPECT_GT(route->length2d(), 10.);
  EXPECT_EQ(route->shortestPath(), *graph->shortestPath(start, end, 0));  // NOLINT
  EXPECT_NE(route->getDebugLaneletMap(), nullptr);
  EXPECT_EQ(route->getDebugLaneletMap()->pointLayer.size(), route->size());  // NOLINT
  EXPECT_NE(route->laneletMap(), nullptr);
  EXPECT_EQ(route->laneletMap()->laneletLayer.size(), route->size());  // NOLINT
}

bool hasRelation(const LaneletRelations& relations, const ConstLanelet& llt, RelationType relationType) {
  LaneletRelation rel{llt, relationType};
  return utils::anyOf(relations, [&rel](auto& relation) { return rel == relation; });
}

template <typename ContainerT>
bool hasLanelet(const ContainerT& llts, const ConstLanelet& llt) {
  return utils::anyOf(llts, [&llt](auto& curr) { return curr == llt; });
}

TEST_F(Route1, Relations) {  // NOLINT
  LaneletRelations previous{route->previousRelations(lanelets.find(2007)->second)};
  EXPECT_EQ(previous.size(), 2ul);  // NOLINT
  EXPECT_TRUE(hasRelation(previous, ConstLanelet(lanelets.find(2005)->second), RelationType::Merging));
  EXPECT_TRUE(hasRelation(previous, ConstLanelet(lanelets.find(2006)->second), RelationType::Merging));

  LaneletRelations following{route->followingRelations(lanelets.find(2007)->second)};
  EXPECT_EQ(following.size(), 2ul);  // NOLINT
  EXPECT_TRUE(hasRelation(following, ConstLanelet(lanelets.find(2008)->second), RelationType::Diverging));
  EXPECT_TRUE(hasRelation(following, ConstLanelet(lanelets.find(2009)->second), RelationType::Diverging));
}

TEST_F(Route1, Conflicting) {  // NOLINT
  auto conflictingInMap{route->conflictingInMap(lanelets.find(2009)->second)};
  EXPECT_EQ(conflictingInMap.size(), 1ul);  // NOLINT
  EXPECT_TRUE(hasLanelet(conflictingInMap, lanelets.find(2008)->second));

  ConstLanelets conflictingInRoute{route->conflictingInRoute(lanelets.find(2009)->second)};
  EXPECT_EQ(conflictingInRoute.size(), 1ul);  // NOLINT
  EXPECT_TRUE(hasLanelet(conflictingInRoute, lanelets.find(2008)->second));

  EXPECT_TRUE(route->conflictingInRoute(lanelets.find(2007)->second).empty());
  EXPECT_TRUE(route->conflictingInMap(lanelets.find(2007)->second).empty());
}

TEST_F(Route1, Lanes) {  // NOLINT
  auto remainingLane{route->remainingLane(lanelets.find(2002)->second)};
  EXPECT_EQ(remainingLane.size(), 2ul);  // NOLINT
  EXPECT_TRUE(hasLanelet(remainingLane, lanelets.find(2002)->second));

  auto fullLane{route->fullLane(lanelets.find(2001)->second)};
  EXPECT_EQ(fullLane.size(), 3ul);  // NOLINT
  EXPECT_TRUE(hasLanelet(fullLane, lanelets.find(2002)->second));
  EXPECT_EQ(route->numLanes(), 7ul);
}

TEST_F(Route2, CreateRoute2) {                                            // NOLINT
  EXPECT_EQ(route->size(), 3ul);                                          // NOLINT
  EXPECT_EQ(route->shortestPath(), *graph->shortestPath(start, end, 0));  // NOLINT
  EXPECT_NE(route->getDebugLaneletMap(), nullptr);
  EXPECT_EQ(route->getDebugLaneletMap()->pointLayer.size(), route->size());  // NOLINT
  EXPECT_NE(route->laneletMap(), nullptr);
  EXPECT_EQ(route->laneletMap()->laneletLayer.size(), route->size());  // NOLINT
}

TEST_F(Route3, CreateRoute3) {                                            // NOLINT
  EXPECT_EQ(route->size(), 3ul);                                          // NOLINT
  EXPECT_EQ(route->shortestPath(), *graph->shortestPath(start, end, 0));  // NOLINT
  EXPECT_NE(route->getDebugLaneletMap(), nullptr);
  EXPECT_EQ(route->getDebugLaneletMap()->pointLayer.size(), route->size());  // NOLINT
  EXPECT_NE(route->laneletMap(), nullptr);
  EXPECT_EQ(route->laneletMap()->laneletLayer.size(), route->size());  // NOLINT
}

TEST_F(Route4, CreateRoute4) {                                            // NOLINT
  EXPECT_EQ(route->size(), 15ul);                                         // NOLINT
  EXPECT_EQ(route->shortestPath(), *graph->shortestPath(start, end, 0));  // NOLINT
  EXPECT_NE(route->getDebugLaneletMap(), nullptr);
  EXPECT_EQ(route->getDebugLaneletMap()->pointLayer.size(), route->size());  // NOLINT
  EXPECT_NE(route->laneletMap(), nullptr);
  EXPECT_EQ(route->laneletMap()->laneletLayer.size(), route->size());  // NOLINT
}

TEST_F(RouteMaxHoseLeftRight, CreateRouteMaxHose1) {                      // NOLINT
  EXPECT_EQ(route->size(), 5ul);                                          // NOLINT
  EXPECT_EQ(route->shortestPath(), *graph->shortestPath(start, end, 0));  // NOLINT
  EXPECT_NE(route->getDebugLaneletMap(), nullptr);
  EXPECT_EQ(route->getDebugLaneletMap()->pointLayer.size(), route->size());  // NOLINT
  EXPECT_NE(route->laneletMap(), nullptr);
  EXPECT_EQ(route->laneletMap()->laneletLayer.size(), route->size());  // NOLINT
}

TEST_F(RouteMaxHoseLeftRight, Lanes) {  // NOLINT
  auto remainingLane{route->remainingLane(lanelets.find(2020)->second)};
  EXPECT_EQ(remainingLane.size(), 3ul);  // NOLINT
  EXPECT_TRUE(std::find(std::begin(remainingLane), std::end(remainingLane),
                        ConstLanelet(lanelets.find(2022)->second)) != std::end(remainingLane));

  auto fullLane{route->fullLane(lanelets.find(2020)->second)};
  EXPECT_EQ(fullLane.size(), 5ul);  // NOLINT
  EXPECT_TRUE(std::find(std::begin(fullLane), std::end(fullLane), ConstLanelet(lanelets.find(2024)->second)) !=
              std::end(fullLane));
  EXPECT_EQ(route->numLanes(), 1ul);
}

TEST_F(RouteMaxHoseLeftRight, InvalidLane) {  // NOLINT
  auto remainingLane{route->remainingLane(lanelets.find(2020)->second.invert())};
  EXPECT_TRUE(remainingLane.empty());
}

TEST_F(RouteMaxHoseRightLeft, CreateRouteMaxHose2) {                      // NOLINT
  EXPECT_EQ(route->size(), 5ul);                                          // NOLINT
  EXPECT_EQ(route->shortestPath(), *graph->shortestPath(start, end, 0));  // NOLINT
  EXPECT_NE(route->getDebugLaneletMap(), nullptr);
  EXPECT_EQ(route->getDebugLaneletMap()->pointLayer.size(), route->size());  // NOLINT
  EXPECT_NE(route->laneletMap(), nullptr);
  EXPECT_EQ(route->laneletMap()->laneletLayer.size(), route->size());  // NOLINT
}

TEST_F(RouteMaxHoseRightLeft, Lanes) {  // NOLINT
  auto remainingLane{route->remainingLane(lanelets.find(2020)->second.invert())};
  EXPECT_EQ(remainingLane.size(), 3ul);  // NOLINT
  EXPECT_TRUE(std::find(std::begin(remainingLane), std::end(remainingLane),
                        ConstLanelet(lanelets.find(2018)->second)) != std::end(remainingLane));

  auto fullLane{route->fullLane(lanelets.find(2020)->second.invert())};
  EXPECT_EQ(fullLane.size(), 5ul);  // NOLINT
  EXPECT_TRUE(std::find(std::begin(fullLane), std::end(fullLane), ConstLanelet(lanelets.find(2021)->second)) !=
              std::end(fullLane));
  EXPECT_EQ(route->numLanes(), 1ul);  // NOLINT
}

TEST_F(RouteMaxHoseLeftRightDashedSolid, Lanes) {  // NOLINT
  auto remainingLane{route->remainingLane(lanelets.find(2020)->second)};
  EXPECT_EQ(remainingLane.size(), 4ul);  // NOLINT
  EXPECT_FALSE(std::find(std::begin(remainingLane), std::end(remainingLane),
                         ConstLanelet(lanelets.find(2018)->second)) != std::end(remainingLane));

  auto fullLane{route->fullLane(lanelets.find(2020)->second)};
  EXPECT_EQ(fullLane.size(), 6ul);  // NOLINT
  EXPECT_TRUE(std::find(std::begin(fullLane), std::end(fullLane), ConstLanelet(lanelets.find(2021)->second)) ==
              std::end(fullLane));
  EXPECT_EQ(route->numLanes(), 1ul);  // NOLINT
}

TEST_F(RouteMaxHoseLeftRightDashedSolid, DashedSolidLineRegarded) {  // NOLINT
  EXPECT_FALSE(!!route->rightRelation(lanelets.find(2025)->second));
}

TEST_F(RouteMaxHoseLeftRightDashedSolidFurther, Lanes) {  // NOLINT
  EXPECT_EQ(route->numLanes(), 1ul);                      // NOLINT
  EXPECT_TRUE(route->remainingLane(lanelets.find(2026)->second).empty());
}

TEST_F(RouteMaxHoseLeftRightDashedSolidFurther, Elements) {  // NOLINT
  EXPECT_EQ(route->size(), 7ul);                             // NOLINT
}

TEST_F(RouteMaxHoseLeftRightDashedSolidFurther, DashedSolidLineRegarded) {  // NOLINT
  EXPECT_FALSE(!!route->rightRelation(lanelets.find(2025)->second));
}

TEST_F(RouteSolidDashed, Lanes) {     // NOLINT
  EXPECT_EQ(route->numLanes(), 2ul);  // NOLINT
}

TEST_F(RouteSolidDashedWithAdjacent, Lanes) {  // NOLINT
  EXPECT_EQ(route->numLanes(), 2ul);           // NOLINT
}

TEST_F(RouteSolidDashedWithAdjacent, AdjacentLaneletInRoute) {  // NOLINT
  ASSERT_TRUE(!!route->rightRelation(lanelets.find(2025)->second));
  EXPECT_EQ(*route->rightRelation(lanelets.find(2025)->second),  // NOLINT
            (LaneletRelation{lanelets.find(2026)->second, RelationType::Right}));
  ASSERT_TRUE(!!route->leftRelation(lanelets.find(2026)->second));
  EXPECT_EQ(*route->leftRelation(lanelets.find(2026)->second),  // NOLINT
            (LaneletRelation{lanelets.find(2025)->second, RelationType::AdjacentLeft}));
  ASSERT_TRUE(!!route->rightRelation(lanelets.find(2027)->second));
  EXPECT_EQ(*route->rightRelation(lanelets.find(2027)->second),  // NOLINT
            (LaneletRelation{lanelets.find(2028)->second, RelationType::AdjacentRight}));
  ASSERT_TRUE(!!route->leftRelation(lanelets.find(2028)->second));
  EXPECT_EQ(*route->leftRelation(lanelets.find(2028)->second),  // NOLINT
            (LaneletRelation{lanelets.find(2027)->second, RelationType::AdjacentLeft}));
  ASSERT_TRUE(!!route->rightRelation(lanelets.find(2029)->second));
  EXPECT_EQ(*route->rightRelation(lanelets.find(2029)->second),  // NOLINT
            (LaneletRelation{lanelets.find(2030)->second, RelationType::AdjacentRight}));
  ASSERT_TRUE(!!route->leftRelation(lanelets.find(2030)->second));
  EXPECT_EQ(*route->leftRelation(lanelets.find(2030)->second),  // NOLINT
            (LaneletRelation{lanelets.find(2029)->second, RelationType::Left}));
}

TEST_F(GermanVehicleGraph, CreateRouteVia) {  // NOLINT
  auto start{lanelets.find(2003)->second};
  auto via{lanelets.find(2009)->second};
  auto end{lanelets.find(2015)->second};
  Optional<Route> route{graph->getRouteVia(start, {via}, end, 0)};
  ASSERT_TRUE(!!route);
  EXPECT_EQ(route->size(), 15ul);                                         // NOLINT
  EXPECT_EQ(route->shortestPath(), *graph->shortestPath(start, end, 0));  // NOLINT
}

TEST_F(GermanVehicleGraph, CreateRouteMissingOneLanelet) {  // NOLINT
  auto start{lanelets.find(2003)->second};
  auto end{lanelets.find(2015)->second};
  Optional<Route> route{graph->getRoute(start, end, 0)};
  ASSERT_TRUE(!!route);
  EXPECT_EQ(route->size(), 15ul);                                         // NOLINT
  EXPECT_EQ(route->shortestPath(), *graph->shortestPath(start, end, 0));  // NOLINT
}

TEST_F(GermanVehicleGraph, CannotCreateRoute) {  // NOLINT
  Optional<Route> route{graph->getRoute(lanelets.find(2014)->second, lanelets.find(2007)->second, 0)};
  EXPECT_FALSE(!!route);

  EXPECT_THROW(graph->getRoute(lanelets.find(2001)->second, lanelets.find(2004)->second, numCostModules),
               lanelet::InvalidInputError);
}

TEST_F(RouteSplittedDiverging, Completeness) {               // NOLINT
  EXPECT_EQ(route->laneletMap()->laneletLayer.size(), 7ul);  // NOLINT
  EXPECT_EQ(route->numLanes(), 3ul);                         // NOLINT
}

TEST_F(RouteSplittedDivergingAndMerging, Completeness) {      // NOLINT
  EXPECT_EQ(route->laneletMap()->laneletLayer.size(), 11ul);  // NOLINT
  EXPECT_EQ(route->numLanes(), 5ul);                          // NOLINT
}
