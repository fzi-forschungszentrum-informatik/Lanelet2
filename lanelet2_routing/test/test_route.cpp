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
    start = lanelets.at(from);
    ASSERT_TRUE(lanelets.find(to) != lanelets.end());
    end = lanelets.at(to);
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

TYPED_TEST(AllRoutesTest, CheckValidity) { EXPECT_NO_THROW(this->route->checkValidity()); }  // NOLINT

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
    EXPECT_NO_THROW(point.attribute("id"));            // NOLINT
    EXPECT_NO_THROW(point.attribute("lane_id"));       // NOLINT
    EXPECT_NO_THROW(point.attribute("init_lane_id"));  // NOLINT
  }

  for (const auto& it : debugMap->lineStringLayer) {
    EXPECT_NO_THROW(it.attribute("relation_1"));  // NOLINT
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
  LaneletRelations previous{route->previousRelations(lanelets.at(2007))};
  EXPECT_EQ(previous.size(), 2ul);  // NOLINT
  EXPECT_TRUE(hasRelation(previous, ConstLanelet(lanelets.at(2005)), RelationType::Merging));
  EXPECT_TRUE(hasRelation(previous, ConstLanelet(lanelets.at(2006)), RelationType::Merging));

  LaneletRelations following{route->followingRelations(lanelets.at(2007))};
  EXPECT_EQ(following.size(), 2ul);  // NOLINT
  EXPECT_TRUE(hasRelation(following, ConstLanelet(lanelets.at(2008)), RelationType::Diverging));
  EXPECT_TRUE(hasRelation(following, ConstLanelet(lanelets.at(2009)), RelationType::Diverging));
}

TEST_F(Route1, Conflicting) {  // NOLINT
  auto conflictingInMap{route->conflictingInMap(lanelets.at(2009))};
  EXPECT_EQ(conflictingInMap.size(), 1ul);  // NOLINT
  EXPECT_TRUE(hasLanelet(conflictingInMap, lanelets.at(2008)));

  ConstLanelets conflictingInRoute{route->conflictingInRoute(lanelets.at(2009))};
  EXPECT_EQ(conflictingInRoute.size(), 1ul);  // NOLINT
  EXPECT_TRUE(hasLanelet(conflictingInRoute, lanelets.at(2008)));

  EXPECT_TRUE(route->conflictingInRoute(lanelets.at(2007)).empty());
  EXPECT_TRUE(route->conflictingInMap(lanelets.at(2007)).empty());
}

TEST_F(Route1, Lanes) {  // NOLINT
  auto remainingLane{route->remainingLane(lanelets.at(2002))};
  EXPECT_EQ(remainingLane.size(), 2ul);  // NOLINT
  EXPECT_TRUE(hasLanelet(remainingLane, lanelets.at(2002)));

  auto fullLane{route->fullLane(lanelets.at(2001))};
  EXPECT_EQ(fullLane.size(), 3ul);  // NOLINT
  EXPECT_TRUE(hasLanelet(fullLane, lanelets.at(2002)));
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
  auto remainingLane{route->remainingLane(lanelets.at(2020))};
  EXPECT_EQ(remainingLane.size(), 3ul);  // NOLINT
  EXPECT_TRUE(std::find(std::begin(remainingLane), std::end(remainingLane), ConstLanelet(lanelets.at(2022))) !=
              std::end(remainingLane));

  auto fullLane{route->fullLane(lanelets.at(2020))};
  EXPECT_EQ(fullLane.size(), 5ul);  // NOLINT
  EXPECT_TRUE(std::find(std::begin(fullLane), std::end(fullLane), ConstLanelet(lanelets.at(2024))) !=
              std::end(fullLane));
  EXPECT_EQ(route->numLanes(), 1ul);
}

TEST_F(RouteMaxHoseLeftRight, InvalidLane) {  // NOLINT
  auto remainingLane{route->remainingLane(lanelets.at(2020).invert())};
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
  auto remainingLane{route->remainingLane(lanelets.at(2020).invert())};
  EXPECT_EQ(remainingLane.size(), 3ul);  // NOLINT
  EXPECT_TRUE(std::find(std::begin(remainingLane), std::end(remainingLane), ConstLanelet(lanelets.at(2018))) !=
              std::end(remainingLane));

  auto fullLane{route->fullLane(lanelets.at(2020).invert())};
  EXPECT_EQ(fullLane.size(), 5ul);  // NOLINT
  EXPECT_TRUE(std::find(std::begin(fullLane), std::end(fullLane), ConstLanelet(lanelets.at(2021))) !=
              std::end(fullLane));
  EXPECT_EQ(route->numLanes(), 1ul);  // NOLINT
}

TEST_F(RouteMaxHoseLeftRightDashedSolid, Lanes) {  // NOLINT
  auto remainingLane{route->remainingLane(lanelets.at(2020))};
  EXPECT_EQ(remainingLane.size(), 4ul);  // NOLINT
  EXPECT_FALSE(std::find(std::begin(remainingLane), std::end(remainingLane), ConstLanelet(lanelets.at(2018))) !=
               std::end(remainingLane));

  auto fullLane{route->fullLane(lanelets.at(2020))};
  EXPECT_EQ(fullLane.size(), 6ul);  // NOLINT
  EXPECT_TRUE(std::find(std::begin(fullLane), std::end(fullLane), ConstLanelet(lanelets.at(2021))) ==
              std::end(fullLane));
  EXPECT_EQ(route->numLanes(), 1ul);  // NOLINT
}

TEST_F(RouteMaxHoseLeftRightDashedSolid, DashedSolidLineRegarded) {  // NOLINT
  EXPECT_FALSE(!!route->rightRelation(lanelets.at(2025)));
}

TEST_F(RouteMaxHoseLeftRightDashedSolidFurther, Lanes) {  // NOLINT
  EXPECT_EQ(route->numLanes(), 1ul);                      // NOLINT
  EXPECT_TRUE(route->remainingLane(lanelets.at(2026)).empty());
}

TEST_F(RouteMaxHoseLeftRightDashedSolidFurther, Elements) {  // NOLINT
  EXPECT_EQ(route->size(), 7ul);                             // NOLINT
}

TEST_F(RouteMaxHoseLeftRightDashedSolidFurther, DashedSolidLineRegarded) {  // NOLINT
  EXPECT_FALSE(!!route->rightRelation(lanelets.at(2025)));
}

TEST_F(RouteSolidDashed, Lanes) {     // NOLINT
  EXPECT_EQ(route->numLanes(), 2ul);  // NOLINT
}

TEST_F(RouteSolidDashedWithAdjacent, Lanes) {  // NOLINT
  EXPECT_EQ(route->numLanes(), 2ul);           // NOLINT
}

TEST_F(RouteSolidDashedWithAdjacent, AdjacentLaneletInRoute) {  // NOLINT
  ASSERT_TRUE(!!route->rightRelation(lanelets.at(2025)));
  EXPECT_EQ(*route->rightRelation(lanelets.at(2025)),  // NOLINT
            (LaneletRelation{lanelets.at(2026), RelationType::Right}));
  ASSERT_TRUE(!!route->leftRelation(lanelets.at(2026)));
  EXPECT_EQ(*route->leftRelation(lanelets.at(2026)),  // NOLINT
            (LaneletRelation{lanelets.at(2025), RelationType::AdjacentLeft}));
  ASSERT_TRUE(!!route->rightRelation(lanelets.at(2027)));
  EXPECT_EQ(*route->rightRelation(lanelets.at(2027)),  // NOLINT
            (LaneletRelation{lanelets.at(2028), RelationType::AdjacentRight}));
  ASSERT_TRUE(!!route->leftRelation(lanelets.at(2028)));
  EXPECT_EQ(*route->leftRelation(lanelets.at(2028)),  // NOLINT
            (LaneletRelation{lanelets.at(2027), RelationType::AdjacentLeft}));
  ASSERT_TRUE(!!route->rightRelation(lanelets.at(2029)));
  EXPECT_EQ(*route->rightRelation(lanelets.at(2029)),  // NOLINT
            (LaneletRelation{lanelets.at(2030), RelationType::AdjacentRight}));
  ASSERT_TRUE(!!route->leftRelation(lanelets.at(2030)));
  EXPECT_EQ(*route->leftRelation(lanelets.at(2030)),  // NOLINT
            (LaneletRelation{lanelets.at(2029), RelationType::Left}));
}

TEST_F(GermanVehicleGraph, CreateRouteVia) {  // NOLINT
  auto start{lanelets.at(2003)};
  auto via{lanelets.at(2009)};
  auto end{lanelets.at(2015)};
  Optional<Route> route{graph->getRouteVia(start, {via}, end, 0)};
  ASSERT_TRUE(!!route);
  EXPECT_EQ(route->size(), 15ul);                                         // NOLINT
  EXPECT_EQ(route->shortestPath(), *graph->shortestPath(start, end, 0));  // NOLINT
}

TEST_F(GermanVehicleGraph, CreateRouteMissingOneLanelet) {  // NOLINT
  auto start{lanelets.at(2003)};
  auto end{lanelets.at(2015)};
  Optional<Route> route{graph->getRoute(start, end, 0)};
  ASSERT_TRUE(!!route);
  EXPECT_EQ(route->size(), 15ul);                                         // NOLINT
  EXPECT_EQ(route->shortestPath(), *graph->shortestPath(start, end, 0));  // NOLINT
}

TEST_F(GermanVehicleGraph, CannotCreateRoute) {  // NOLINT
  Optional<Route> route{graph->getRoute(lanelets.at(2014), lanelets.at(2007), 0)};
  EXPECT_FALSE(!!route);
  // NOLINTNEXTLINE
  EXPECT_THROW(graph->getRoute(lanelets.at(2001), lanelets.at(2004), numCostModules), lanelet::InvalidInputError);
}

TEST_F(RouteSplittedDiverging, Completeness) {               // NOLINT
  EXPECT_EQ(route->laneletMap()->laneletLayer.size(), 7ul);  // NOLINT
  EXPECT_EQ(route->numLanes(), 3ul);                         // NOLINT
}

TEST_F(RouteSplittedDivergingAndMerging, Completeness) {      // NOLINT
  EXPECT_EQ(route->laneletMap()->laneletLayer.size(), 11ul);  // NOLINT
  EXPECT_EQ(route->numLanes(), 5ul);                          // NOLINT
}
