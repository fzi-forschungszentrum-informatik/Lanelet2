#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <algorithm>
#include <string>

#include "lanelet2_routing/Route.h"
#include "lanelet2_routing/RoutingGraph.h"

using namespace lanelet;
using namespace lanelet::routing;

namespace {

struct BoundSpec {
  std::string type{AttributeValueString::BikeMarking};
  std::string subtype{AttributeValueString::Dashed};
  Optional<bool> laneChange;
};

struct SkipOverMap {
  Lanelet v1;
  Lanelet bike;
  Lanelet v2;
  bool hasV2{true};
  LaneletMapUPtr map;
};

void tagBound(LineString3d* ls, const BoundSpec& spec) {
  ls->setAttribute(AttributeName::Type, spec.type);
  if (!spec.subtype.empty()) {
    ls->setAttribute(AttributeName::Subtype, spec.subtype);
  }
  if (!!spec.laneChange) {
    ls->setAttribute(AttributeNamesString::LaneChange, *spec.laneChange);
  }
}

SkipOverMap makeSkipOverMap(const BoundSpec& bikeRight, const BoundSpec& bikeLeft, bool includeV2 = true) {
  // Driving +x, y increases to the left:
  //   V2 | bike.leftBound | Bike | bike.rightBound | V1
  Point3d p1{1, 0., 0., 0.}, p2{2, 10., 0., 0.};
  Point3d p3{3, 0., 2., 0.}, p4{4, 10., 2., 0.};
  Point3d p5{5, 0., 4., 0.}, p6{6, 10., 4., 0.};
  Point3d p7{7, 0., 6., 0.}, p8{8, 10., 6., 0.};
  LineString3d v1Right{10, {p1, p2}};
  LineString3d sharedV1Bike{11, {p3, p4}};
  LineString3d sharedBikeV2{12, {p5, p6}};
  LineString3d v2Left{13, {p7, p8}};
  tagBound(&sharedV1Bike, bikeRight);
  tagBound(&sharedBikeV2, bikeLeft);

  AttributeMap road{{AttributeNamesString::Subtype, AttributeValueString::Road}};
  AttributeMap bikeAttr{{AttributeNamesString::Subtype, AttributeValueString::BicycleLane}};
  SkipOverMap result;
  result.v1 = Lanelet{21, sharedV1Bike, v1Right, road};
  result.bike = Lanelet{22, sharedBikeV2, sharedV1Bike, bikeAttr};
  result.hasV2 = includeV2;
  Lanelets lanelets{result.v1, result.bike};
  if (includeV2) {
    result.v2 = Lanelet{23, v2Left, sharedBikeV2, road};
    lanelets.push_back(result.v2);
  }
  result.map = utils::createMap(lanelets);
  return result;
}

RoutingGraphUPtr buildVehicleGraph(const LaneletMap& map, bool allowSkipOver = true) {
  auto rules = traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Vehicle);
  RoutingGraph::Configuration config;
  if (allowSkipOver) {
    config.emplace(RoutingGraph::AllowLaneChangeAcrossBicycleLane, true);
  }
  return RoutingGraph::build(map, *rules, defaultRoutingCosts(), config);
}

RoutingGraphUPtr buildBicycleGraph(const LaneletMap& map) {
  auto rules = traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Bicycle);
  return RoutingGraph::build(map, *rules);
}

bool containsLanelet(const ConstLanelets& lanelets, const ConstLanelet& ll) {
  return std::find(lanelets.begin(), lanelets.end(), ll) != lanelets.end();
}

const BoundSpec kDashedBikeMarking{};
const BoundSpec kSolidBikeMarking{AttributeValueString::BikeMarking, AttributeValueString::Solid, {}};
const BoundSpec kDashedLineThin{AttributeValueString::LineThin, AttributeValueString::Dashed, {}};
const BoundSpec kSolidLineThin{AttributeValueString::LineThin, AttributeValueString::Solid, {}};
const BoundSpec kVirtual{AttributeValueString::Virtual, "", {}};
const BoundSpec kSolidWithLaneChangeYes{AttributeValueString::LineThin, AttributeValueString::Solid,
                                        Optional<bool>{true}};
const BoundSpec kDashedWithLaneChangeNo{AttributeValueString::BikeMarking, AttributeValueString::Dashed,
                                        Optional<bool>{false}};

}  // namespace

TEST(BicycleLaneSkipOver, disabledByDefault) {  // NOLINT
  auto map = makeSkipOverMap(kDashedBikeMarking, kDashedBikeMarking);
  auto rules = traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Vehicle);
  auto graph = RoutingGraph::build(*map.map, *rules);  // no config passed -> feature must stay off
  EXPECT_FALSE(!!graph->left(map.v1));
  EXPECT_FALSE(!!graph->right(map.v2));
}

TEST(BicycleLaneSkipOver, explicitlyDisabledViaConfig) {  // NOLINT
  auto map = makeSkipOverMap(kDashedBikeMarking, kDashedBikeMarking);
  auto graph = buildVehicleGraph(*map.map, /*allowSkipOver=*/false);
  EXPECT_FALSE(!!graph->left(map.v1));
  EXPECT_FALSE(!!graph->right(map.v2));
}

TEST(BicycleLaneSkipOver, dashedBikeMarkingAllowsVehicleSkip) {  // NOLINT
  auto map = makeSkipOverMap(kDashedBikeMarking, kDashedBikeMarking);
  auto graph = buildVehicleGraph(*map.map);
  EXPECT_NO_THROW(graph->checkValidity());
  auto left = graph->left(map.v1);
  ASSERT_TRUE(!!left);
  EXPECT_EQ(*left, map.v2);
  auto right = graph->right(map.v2);
  ASSERT_TRUE(!!right);
  EXPECT_EQ(*right, map.v1);
  EXPECT_FALSE(graph->passableSubmap()->laneletLayer.exists(map.bike.id()));
}

TEST(BicycleLaneSkipOver, dashedLineThinAllowsVehicleSkip) {  // NOLINT
  auto map = makeSkipOverMap(kDashedLineThin, kDashedLineThin);
  auto graph = buildVehicleGraph(*map.map);
  auto left = graph->left(map.v1);
  ASSERT_TRUE(!!left);
  EXPECT_EQ(*left, map.v2);
  auto right = graph->right(map.v2);
  ASSERT_TRUE(!!right);
  EXPECT_EQ(*right, map.v1);
}

TEST(BicycleLaneSkipOver, solidOnNearBoundBlocksSkip) {  // NOLINT
  auto map = makeSkipOverMap(kSolidBikeMarking, kDashedBikeMarking);
  auto graph = buildVehicleGraph(*map.map);
  EXPECT_FALSE(!!graph->left(map.v1));
  EXPECT_FALSE(!!graph->right(map.v2));
}

TEST(BicycleLaneSkipOver, solidOnFarBoundBlocksSkip) {  // NOLINT
  auto map = makeSkipOverMap(kDashedBikeMarking, kSolidBikeMarking);
  auto graph = buildVehicleGraph(*map.map);
  EXPECT_FALSE(!!graph->left(map.v1));
  EXPECT_FALSE(!!graph->right(map.v2));
}

TEST(BicycleLaneSkipOver, solidLineThinBlocksSkip) {  // NOLINT
  auto map = makeSkipOverMap(kSolidLineThin, kDashedLineThin);
  auto graph = buildVehicleGraph(*map.map);
  EXPECT_FALSE(!!graph->left(map.v1));
  EXPECT_FALSE(!!graph->right(map.v2));
}

TEST(BicycleLaneSkipOver, laneChangeYesOnSolidAllowsSkip) {  // NOLINT
  auto map = makeSkipOverMap(kSolidWithLaneChangeYes, kSolidWithLaneChangeYes);
  auto graph = buildVehicleGraph(*map.map);
  auto left = graph->left(map.v1);
  ASSERT_TRUE(!!left);
  EXPECT_EQ(*left, map.v2);
}

TEST(BicycleLaneSkipOver, laneChangeNoOnDashedBlocksSkip) {  // NOLINT
  auto map = makeSkipOverMap(kDashedWithLaneChangeNo, kDashedWithLaneChangeNo);
  auto graph = buildVehicleGraph(*map.map);
  EXPECT_FALSE(!!graph->left(map.v1));
  EXPECT_FALSE(!!graph->right(map.v2));
}

TEST(BicycleLaneSkipOver, virtualBoundBlocksSkip) {  // NOLINT
  auto map = makeSkipOverMap(kVirtual, kDashedBikeMarking);
  auto graph = buildVehicleGraph(*map.map);
  EXPECT_FALSE(!!graph->left(map.v1));
  EXPECT_FALSE(!!graph->right(map.v2));
}

TEST(BicycleLaneSkipOver, missingFarLaneletBlocksSkip) {  // NOLINT
  auto map = makeSkipOverMap(kDashedBikeMarking, kDashedBikeMarking, /*includeV2=*/false);
  auto graph = buildVehicleGraph(*map.map);
  EXPECT_FALSE(!!graph->left(map.v1));
  EXPECT_FALSE(!!graph->right(map.v1));
}

TEST(BicycleLaneSkipOver, bicycleGraphKeepsBikeLaneAndDoesNotSkip) {  // NOLINT
  auto map = makeSkipOverMap(kDashedBikeMarking, kDashedBikeMarking);
  auto graph = buildBicycleGraph(*map.map);
  EXPECT_NO_THROW(graph->checkValidity());
  EXPECT_TRUE(graph->passableSubmap()->laneletLayer.exists(map.bike.id()));
  auto left = graph->left(map.v1);
  ASSERT_TRUE(!!left);
  EXPECT_EQ(*left, map.bike);
  EXPECT_NE(*left, map.v2);
}

TEST(BicycleLaneSkipOver, shortestPathUsesSkipOver) {  // NOLINT
  auto map = makeSkipOverMap(kDashedBikeMarking, kDashedBikeMarking);
  auto graph = buildVehicleGraph(*map.map);
  auto path = graph->shortestPath(map.v1, map.v2, 0, true);
  ASSERT_TRUE(!!path);
  ASSERT_EQ(path->size(), 2ul);
  EXPECT_EQ((*path)[0], map.v1);
  EXPECT_EQ((*path)[1], map.v2);
}

TEST(BicycleLaneSkipOver, routeAndBesidesPickUpSkipOverWithoutBikeLane) {  // NOLINT
  auto map = makeSkipOverMap(kDashedBikeMarking, kDashedBikeMarking);
  auto graph = buildVehicleGraph(*map.map);
  auto route = graph->getRoute(map.v1, map.v2, 0, true);
  ASSERT_TRUE(!!route);
  EXPECT_TRUE(route->contains(map.v1));
  EXPECT_TRUE(route->contains(map.v2));
  EXPECT_FALSE(route->contains(map.bike));
  EXPECT_FALSE(route->laneletSubmap()->laneletLayer.exists(map.bike.id()));

  const ConstLanelets besides = graph->besides(map.v1);
  EXPECT_TRUE(containsLanelet(besides, map.v1));
  EXPECT_TRUE(containsLanelet(besides, map.v2));
  EXPECT_FALSE(containsLanelet(besides, map.bike));
}
