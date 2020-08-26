#include <gtest/gtest.h>

#include "lanelet2_routing/Route.h"
#include "lanelet2_routing/RoutingGraph.h"
#include "test_routing_map.h"

using namespace lanelet;
using namespace lanelet::routing;
using namespace lanelet::routing::tests;

template <Id FromId, Id ToId, Id... ViaIds>
class TestRoute : public GermanVehicleGraph {
 public:
  explicit TestRoute(uint16_t routingCostId = 0, bool withLaneChanges = true) {
    Ids viaIds{ViaIds...};
    start = lanelets.at(From);
    end = lanelets.at(To);
    via = utils::transform(viaIds, [&](auto id) -> ConstLanelet { return lanelets.at(id); });
    auto optRoute = via.empty() ? graph->getRoute(start, end, routingCostId, withLaneChanges)
                                : graph->getRouteVia(start, via, end, routingCostId, withLaneChanges);
    EXPECT_TRUE(!!optRoute);
    route = std::move(*optRoute);
  }
  ConstLanelet start;
  ConstLanelet end;
  ConstLanelets via;
  Route route;
  // these would better be defined as static constexpr, but c++14 doesnt support it well
  const Id From{FromId};  // NOLINT
  const Id To{ToId};      // NOLINT
};

class Route1 : public TestRoute<2001, 2014> {};
class Route1NoLc : public TestRoute<2001, 2014> {
 public:
  Route1NoLc() : TestRoute(0, false) {}
};
class Route2 : public TestRoute<2001, 2004> {};
class Route3 : public TestRoute<2003, 2002> {};
class Route4 : public TestRoute<2003, 2013> {};
class Route5 : public TestRoute<2066, 2064> {};
class RouteMaxHoseLeftRight : public TestRoute<2017, 2024> {};
class RouteMaxHoseRightLeft : public TestRoute<2023, 2016> {};
class RouteMaxHoseLeftRightDashedSolid : public TestRoute<2017, 2025> {};
class RouteMaxHoseLeftRightDashedSolidFurther : public TestRoute<2017, 2027> {};
class RouteSolidDashed : public TestRoute<2024, 2026> {};
class RouteSolidDashedWithAdjacent : public TestRoute<2024, 2029> {};
class RouteSplittedDiverging : public TestRoute<2029, 2038> {};
class RouteSplittedDivergingAndMerging : public TestRoute<2041, 2049> {};
class RouteViaSimple : public TestRoute<2003, 2015, 2009> {};
class RouteMissingLanelet : public TestRoute<2003, 2015> {};
class RouteInCircle : public TestRoute<2029, 2068> {};
class RouteCircular : public TestRoute<2037, 2037, 2065> {};
class RouteCircularNoLc : public TestRoute<2037, 2037, 2065> {
 public:
  RouteCircularNoLc() : TestRoute(0, false) {}
};

template <typename T>
class AllRoutesTest : public T {};

using AllRoutes =
    testing::Types<Route1, Route1NoLc, Route2, Route3, Route4, Route5, RouteMaxHoseLeftRight, RouteMaxHoseRightLeft,
                   RouteMaxHoseLeftRightDashedSolid, RouteMaxHoseLeftRightDashedSolidFurther, RouteSolidDashed,
                   RouteSolidDashedWithAdjacent, RouteSplittedDiverging, RouteSplittedDivergingAndMerging,
                   RouteViaSimple, RouteMissingLanelet, RouteInCircle, RouteCircular, RouteCircularNoLc>;

TYPED_TEST_CASE(AllRoutesTest, AllRoutes);

TYPED_TEST(AllRoutesTest, CheckValidity) { EXPECT_NO_THROW(this->route.checkValidity()); }  // NOLINT

TYPED_TEST(AllRoutesTest, ShortestMapInDebugMap) {
  const auto map{this->route.getDebugLaneletMap()};
  for (const auto& el : this->route.shortestPath()) {
    ASSERT_NE(map->pointLayer.find(el.id()), map->pointLayer.end());
    const Point3d point{*map->pointLayer.find(el.id())};
    EXPECT_TRUE(*point.attribute("shortest_path").asBool());
  }
}

TYPED_TEST(AllRoutesTest, DebugMapCompleteness) {
  const LaneletMapPtr debugMap{this->route.getDebugLaneletMap()};
  const LaneletSubmapConstPtr laneletMap{this->route.laneletSubmap()};

  for (const auto& it : laneletMap->laneletLayer) {
    ASSERT_NE(debugMap->pointLayer.find(it.id()), debugMap->pointLayer.end());
    Point3d point{*debugMap->pointLayer.find(it.id())};
    EXPECT_NO_THROW(point.attribute("id"));       // NOLINT
    EXPECT_NO_THROW(point.attribute("lane_id"));  // NOLINT
  }

  for (const auto& it : debugMap->lineStringLayer) {
    EXPECT_NO_THROW(it.attribute("relation_1"));  // NOLINT
  }
}

TEST_F(Route1, CreateRoute1) {    // NOLINT
  EXPECT_EQ(route.size(), 15ul);  // NOLINT
  EXPECT_GT(route.length2d(), 10.);
  EXPECT_EQ(route.shortestPath(), *graph->shortestPath(start, end, 0));  // NOLINT
  EXPECT_EQ(route.remainingShortestPath(start), route.shortestPath());   // NOLINT
  EXPECT_TRUE(route.remainingShortestPath(lanelets.at(2003)).empty());   // NOLINT
  EXPECT_NE(route.getDebugLaneletMap(), nullptr);
  EXPECT_EQ(route.getDebugLaneletMap()->pointLayer.size(), route.size());  // NOLINT
  EXPECT_NE(route.laneletSubmap(), nullptr);
  EXPECT_EQ(route.laneletSubmap()->laneletLayer.size(), route.size());  // NOLINT
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
  LaneletRelations previous{route.previousRelations(lanelets.at(2007))};
  EXPECT_EQ(previous.size(), 2ul);  // NOLINT
  EXPECT_TRUE(hasRelation(previous, ConstLanelet(lanelets.at(2005)), RelationType::Successor));
  EXPECT_TRUE(hasRelation(previous, ConstLanelet(lanelets.at(2006)), RelationType::Successor));

  LaneletRelations following{route.followingRelations(lanelets.at(2007))};
  EXPECT_EQ(following.size(), 2ul);  // NOLINT
  EXPECT_TRUE(hasRelation(following, ConstLanelet(lanelets.at(2008)), RelationType::Successor));
  EXPECT_TRUE(hasRelation(following, ConstLanelet(lanelets.at(2009)), RelationType::Successor));
}

TEST_F(Route1, Conflicting) {  // NOLINT
  auto conflictingInMap{route.conflictingInMap(lanelets.at(2009))};
  EXPECT_EQ(conflictingInMap.size(), 1ul);  // NOLINT
  EXPECT_TRUE(hasLanelet(conflictingInMap, lanelets.at(2008)));

  ConstLanelets conflictingInRoute{route.conflictingInRoute(lanelets.at(2009))};
  EXPECT_EQ(conflictingInRoute.size(), 1ul);  // NOLINT
  EXPECT_TRUE(hasLanelet(conflictingInRoute, lanelets.at(2008)));

  EXPECT_TRUE(route.conflictingInRoute(lanelets.at(2007)).empty());
  EXPECT_TRUE(route.conflictingInMap(lanelets.at(2007)).empty());
}

TEST_F(Route1, forEachSuccessor) {  // NOLINT
  double cLast = 0.;
  route.forEachSuccessor(lanelets.at(2001), [&](const LaneletVisitInformation& i) {
    EXPECT_TRUE(route.contains(i.lanelet));
    EXPECT_LE(cLast, i.cost);
    cLast = i.cost;
    return true;
  });
}

TEST_F(Route1, Lanes) {  // NOLINT
  auto remainingLane{route.remainingLane(lanelets.at(2002))};
  EXPECT_EQ(remainingLane.size(), 2ul);  // NOLINT
  EXPECT_TRUE(hasLanelet(remainingLane, lanelets.at(2002)));

  auto fullLane{route.fullLane(lanelets.at(2001))};
  EXPECT_EQ(fullLane.size(), 3ul);  // NOLINT
  EXPECT_TRUE(hasLanelet(fullLane, lanelets.at(2002)));
  EXPECT_EQ(route.numLanes(), 7ul);
}

TEST_F(Route1NoLc, CreateRoute) {                                               // NOLINT
  EXPECT_EQ(route.size(), 7ul);                                                 // NOLINT
  EXPECT_EQ(route.shortestPath(), *graph->shortestPath(start, end, 0, false));  // NOLINT
  EXPECT_NE(route.getDebugLaneletMap(), nullptr);
  EXPECT_EQ(route.getDebugLaneletMap()->pointLayer.size(), route.size());  // NOLINT
  EXPECT_NE(route.laneletSubmap(), nullptr);
  EXPECT_EQ(route.laneletSubmap()->laneletLayer.size(), route.size());  // NOLINT
}
TEST_F(Route1NoLc, Lanes) {  // NOLINT
  auto remainingLane{route.remainingLane(lanelets.at(2002))};
  EXPECT_EQ(remainingLane.size(), 6ul);  // NOLINT
  EXPECT_TRUE(hasLanelet(remainingLane, lanelets.at(2002)));

  auto fullLane{route.fullLane(lanelets.at(2002))};
  EXPECT_EQ(fullLane.size(), 7ul);  // NOLINT

  EXPECT_TRUE(route.remainingLane(lanelets.at(2010)).empty());
  EXPECT_EQ(route.numLanes(), 1ul);
}

TEST_F(Route1NoLc, forEachPredecessor) {  // NOLINT
  route.forEachPredecessor(lanelets.at(2001), [&](const LaneletVisitInformation& i) {
    EXPECT_EQ(i.lanelet, lanelets.at(2001));
    return true;
  });
}

TEST_F(Route2, CreateRoute2) {                                           // NOLINT
  EXPECT_EQ(route.size(), 3ul);                                          // NOLINT
  EXPECT_EQ(route.shortestPath(), *graph->shortestPath(start, end, 0));  // NOLINT
  EXPECT_NE(route.getDebugLaneletMap(), nullptr);
  EXPECT_EQ(route.getDebugLaneletMap()->pointLayer.size(), route.size());  // NOLINT
  EXPECT_NE(route.laneletSubmap(), nullptr);
  EXPECT_EQ(route.laneletSubmap()->laneletLayer.size(), route.size());  // NOLINT
}

TEST_F(Route3, CreateRoute3) {                                           // NOLINT
  EXPECT_EQ(route.size(), 3ul);                                          // NOLINT
  EXPECT_EQ(route.shortestPath(), *graph->shortestPath(start, end, 0));  // NOLINT
  EXPECT_NE(route.getDebugLaneletMap(), nullptr);
  EXPECT_EQ(route.getDebugLaneletMap()->pointLayer.size(), route.size());  // NOLINT
  EXPECT_NE(route.laneletSubmap(), nullptr);
  EXPECT_EQ(route.laneletSubmap()->laneletLayer.size(), route.size());  // NOLINT
}

TEST_F(Route4, CreateRoute4) {                                           // NOLINT
  EXPECT_EQ(route.size(), 15ul);                                         // NOLINT
  EXPECT_EQ(route.shortestPath(), *graph->shortestPath(start, end, 0));  // NOLINT
  EXPECT_NE(route.getDebugLaneletMap(), nullptr);
  EXPECT_EQ(route.getDebugLaneletMap()->pointLayer.size(), route.size());  // NOLINT
  EXPECT_NE(route.laneletSubmap(), nullptr);
  EXPECT_EQ(route.laneletSubmap()->laneletLayer.size(), route.size());  // NOLINT
}

TEST_F(Route5, NoCircle) {
  EXPECT_EQ(route.size(), 9);
  EXPECT_FALSE(hasLanelet(route.laneletSubmap()->laneletLayer, lanelets.at(2065)));
}

TEST_F(RouteMaxHoseLeftRight, CreateRouteMaxHose1) {                     // NOLINT
  EXPECT_EQ(route.size(), 5ul);                                          // NOLINT
  EXPECT_EQ(route.shortestPath(), *graph->shortestPath(start, end, 0));  // NOLINT
  EXPECT_NE(route.getDebugLaneletMap(), nullptr);
  EXPECT_EQ(route.getDebugLaneletMap()->pointLayer.size(), route.size());  // NOLINT
  EXPECT_NE(route.laneletSubmap(), nullptr);
  EXPECT_EQ(route.laneletSubmap()->laneletLayer.size(), route.size());  // NOLINT
}

TEST_F(RouteMaxHoseLeftRight, Lanes) {  // NOLINT
  auto remainingLane{route.remainingLane(lanelets.at(2020))};
  EXPECT_EQ(remainingLane.size(), 3ul);  // NOLINT
  EXPECT_TRUE(std::find(std::begin(remainingLane), std::end(remainingLane), ConstLanelet(lanelets.at(2022))) !=
              std::end(remainingLane));

  auto fullLane{route.fullLane(lanelets.at(2020))};
  EXPECT_EQ(fullLane.size(), 5ul);  // NOLINT
  EXPECT_TRUE(std::find(std::begin(fullLane), std::end(fullLane), ConstLanelet(lanelets.at(2024))) !=
              std::end(fullLane));
  EXPECT_EQ(route.numLanes(), 1ul);
}

TEST_F(RouteMaxHoseLeftRight, InvalidLane) {  // NOLINT
  auto remainingLane{route.remainingLane(lanelets.at(2020).invert())};
  EXPECT_TRUE(remainingLane.empty());
}

TEST_F(RouteMaxHoseRightLeft, CreateRouteMaxHose2) {                     // NOLINT
  EXPECT_EQ(route.size(), 5ul);                                          // NOLINT
  EXPECT_EQ(route.shortestPath(), *graph->shortestPath(start, end, 0));  // NOLINT
  EXPECT_NE(route.getDebugLaneletMap(), nullptr);
  EXPECT_EQ(route.getDebugLaneletMap()->pointLayer.size(), route.size());  // NOLINT
  EXPECT_NE(route.laneletSubmap(), nullptr);
  EXPECT_EQ(route.laneletSubmap()->laneletLayer.size(), route.size());  // NOLINT
}

TEST_F(RouteMaxHoseRightLeft, Lanes) {  // NOLINT
  auto remainingLane{route.remainingLane(lanelets.at(2020).invert())};
  EXPECT_EQ(remainingLane.size(), 3ul);  // NOLINT
  EXPECT_TRUE(std::find(std::begin(remainingLane), std::end(remainingLane), ConstLanelet(lanelets.at(2018))) !=
              std::end(remainingLane));

  auto fullLane{route.fullLane(lanelets.at(2020).invert())};
  EXPECT_EQ(fullLane.size(), 5ul);  // NOLINT
  EXPECT_TRUE(std::find(std::begin(fullLane), std::end(fullLane), ConstLanelet(lanelets.at(2021))) !=
              std::end(fullLane));
  EXPECT_EQ(route.numLanes(), 1ul);  // NOLINT
}

TEST_F(RouteMaxHoseLeftRightDashedSolid, Lanes) {  // NOLINT
  auto remainingLane{route.remainingLane(lanelets.at(2020))};
  EXPECT_EQ(remainingLane.size(), 4ul);  // NOLINT
  EXPECT_FALSE(std::find(std::begin(remainingLane), std::end(remainingLane), ConstLanelet(lanelets.at(2018))) !=
               std::end(remainingLane));

  auto fullLane{route.fullLane(lanelets.at(2020))};
  EXPECT_EQ(fullLane.size(), 6ul);  // NOLINT
  EXPECT_TRUE(std::find(std::begin(fullLane), std::end(fullLane), ConstLanelet(lanelets.at(2021))) ==
              std::end(fullLane));
  EXPECT_EQ(route.numLanes(), 1ul);  // NOLINT
}

TEST_F(RouteMaxHoseLeftRightDashedSolid, DashedSolidLineRegarded) {  // NOLINT
  EXPECT_FALSE(!!route.rightRelation(lanelets.at(2025)));
}

TEST_F(RouteMaxHoseLeftRightDashedSolidFurther, Lanes) {  // NOLINT
  EXPECT_EQ(route.numLanes(), 1ul);                       // NOLINT
  EXPECT_TRUE(route.remainingLane(lanelets.at(2026)).empty());
}

TEST_F(RouteMaxHoseLeftRightDashedSolidFurther, Elements) {  // NOLINT
  EXPECT_EQ(route.size(), 7ul);                              // NOLINT
}

TEST_F(RouteMaxHoseLeftRightDashedSolidFurther, DashedSolidLineRegarded) {  // NOLINT
  EXPECT_FALSE(!!route.rightRelation(lanelets.at(2025)));
}

TEST_F(RouteSolidDashed, Lanes) {    // NOLINT
  EXPECT_EQ(route.numLanes(), 2ul);  // NOLINT
}

TEST_F(RouteSolidDashedWithAdjacent, Lanes) {  // NOLINT
  EXPECT_EQ(route.numLanes(), 2ul);            // NOLINT
}

TEST_F(RouteSolidDashedWithAdjacent, AdjacentLaneletInRoute) {  // NOLINT
  ASSERT_TRUE(!!route.rightRelation(lanelets.at(2025)));
  EXPECT_EQ(*route.rightRelation(lanelets.at(2025)),  // NOLINT
            (LaneletRelation{lanelets.at(2026), RelationType::Right}));
  ASSERT_TRUE(!!route.leftRelation(lanelets.at(2026)));
  EXPECT_EQ(*route.leftRelation(lanelets.at(2026)),  // NOLINT
            (LaneletRelation{lanelets.at(2025), RelationType::AdjacentLeft}));
  ASSERT_TRUE(!!route.rightRelation(lanelets.at(2027)));
  EXPECT_EQ(*route.rightRelation(lanelets.at(2027)),  // NOLINT
            (LaneletRelation{lanelets.at(2028), RelationType::AdjacentRight}));
  ASSERT_TRUE(!!route.leftRelation(lanelets.at(2028)));
  EXPECT_EQ(*route.leftRelation(lanelets.at(2028)),  // NOLINT
            (LaneletRelation{lanelets.at(2027), RelationType::AdjacentLeft}));
  ASSERT_TRUE(!!route.rightRelation(lanelets.at(2029)));
  EXPECT_EQ(*route.rightRelation(lanelets.at(2029)),  // NOLINT
            (LaneletRelation{lanelets.at(2030), RelationType::AdjacentRight}));
  ASSERT_TRUE(!!route.leftRelation(lanelets.at(2030)));
  EXPECT_EQ(*route.leftRelation(lanelets.at(2030)),  // NOLINT
            (LaneletRelation{lanelets.at(2029), RelationType::Left}));
}

TEST_F(RouteViaSimple, create) {  // NOLINT
  EXPECT_EQ(route.size(), 15ul);
  EXPECT_EQ(route.shortestPath(), *graph->shortestPath(start, end, 0));
}

TEST_F(RouteMissingLanelet, create) {                                    // NOLINT
  EXPECT_EQ(route.size(), 15ul);                                         // NOLINT
  EXPECT_EQ(route.shortestPath(), *graph->shortestPath(start, end, 0));  // NOLINT
}

TEST_F(GermanVehicleGraph, CannotCreateRoute) {  // NOLINT
  Optional<Route> route{graph->getRoute(lanelets.at(2014), lanelets.at(2007), 0)};
  EXPECT_FALSE(!!route);
  // NOLINTNEXTLINE
  EXPECT_THROW(graph->getRoute(lanelets.at(2001), lanelets.at(2004), numCostModules), lanelet::InvalidInputError);
}

TEST_F(RouteSplittedDiverging, Completeness) {                 // NOLINT
  EXPECT_EQ(route.laneletSubmap()->laneletLayer.size(), 7ul);  // NOLINT
  EXPECT_EQ(route.numLanes(), 3ul);                            // NOLINT
}

TEST_F(RouteSplittedDivergingAndMerging, Completeness) {        // NOLINT
  EXPECT_EQ(route.laneletSubmap()->laneletLayer.size(), 11ul);  // NOLINT
  EXPECT_EQ(route.numLanes(), 5ul);                             // NOLINT
}

TEST_F(RouteInCircle, create) {                                   // NOLINT
  EXPECT_EQ(route.laneletSubmap()->laneletLayer.size(), 11ul);    // NOLINT
  EXPECT_EQ(route.numLanes(), 3ul);                               // NOLINT
  EXPECT_EQ(route.remainingLane(lanelets.at(2067)).size(), 0ul);  // NOLINT
}

TEST_F(RouteCircular, Circularity) {  // NOLINT
  EXPECT_EQ(route.size(), 10ul);
  // at any point of the circular route, the remaining lane should cross the circle
  EXPECT_EQ(route.remainingLane(start).size(), 6ul);
  EXPECT_EQ(*route.remainingLane(start).begin(), start);
  EXPECT_EQ(route.remainingLane(lanelets.at(2036)).size(), 7ul);
  EXPECT_EQ(route.remainingShortestPath(lanelets.at(2067)).size(), 7ul);
  EXPECT_EQ(route.remainingLane(lanelets.at(2067)).size(), 1ul);
  EXPECT_EQ(route.remainingShortestPath(lanelets.at(2067)).front(), lanelets.at(2067));
  EXPECT_EQ(route.remainingLane(lanelets.at(2033)).size(), 3ul);
}

TEST_F(RouteCircularNoLc, Circularity) {  // NOLINT
  EXPECT_EQ(route.size(), 7ul);
  EXPECT_EQ(route.numLanes(), 1ul);
  EXPECT_EQ(route.remainingLane(start).size(), 7ul);
  EXPECT_EQ(route.remainingShortestPath(lanelets.at(2036)).size(), 7ul);
  EXPECT_EQ(route.remainingLane(lanelets.at(2067)).size(), 7ul);
}
