#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/utility/Units.h>
#include "TrafficRules.h"
#include "TrafficRulesFactory.h"
#include "gtest/gtest.h"

using Attr = lanelet::AttributeName;
using AttrStr = lanelet::AttributeNamesString;
using Value = lanelet::AttributeValueString;

lanelet::RegulatoryElementPtr getSpeedLimit(const std::string& type, const lanelet::AttributeMap& attributes = {}) {
  using namespace lanelet;
  Point3d p1{10, 0, -1}, p2{11, 0, -2};
  LineString3d sign{7, {p1, p2}};
  return SpeedLimit::make(5, attributes, {{sign}, type});
}

lanelet::TrafficRulesPtr germanVehicleRules() {
  using namespace lanelet;
  return TrafficRulesFactory::instance().create(Locations::Germany, Participants::Vehicle, {});
};

lanelet::TrafficRulesPtr germanBikeRules() {
  using namespace lanelet;
  return TrafficRulesFactory::instance().create(Locations::Germany, Participants::Bicycle, {});
}

lanelet::TrafficRulesPtr germanPedestrianRules() {
  using namespace lanelet;
  return TrafficRulesFactory::instance().create(Locations::Germany, Participants::Pedestrian, {});
}

class TrafficRules : public ::testing::Test {
  /* looks like this:
   * p1----ls4----p8
   *
   *       left
   *
   * p5----ls3----p6----ls6----p10
   *
   *     lanelet        next
   *
   * p3----ls2----p4----ls5----p9-----ls9--\
   *              |            |            \
   *      right   ls7  area    ls8 nextArea  p12
   *              |            |            /
   * p1----ls1----p2----ls8----p11----ls9--/
   */
 public:
  lanelet::Point3d p1{1, 0, 0}, p2{2, 2, 0}, p3{3, 0, 2}, p4{4, 2, 2}, p5{5, 0, 4}, p6{6, 2, 4}, p7{7, 0, 6},
      p8{8, 2, 6}, p9{9, 4, 2}, p10{10, 4, 4}, p11{11, 4, 0}, p12{12, 5, 1};
  lanelet::LineString3d ls1{1, {p1, p2}}, ls2{2, {p3, p4}}, ls3{3, {p5, p6}}, ls4{4, {p7, p8}}, ls5{5, {p4, p9}},
      ls6{6, {p6, p10}}, ls7{7, {p2, p4}}, ls8{8, {p9, p11, p2}}, ls9{9, {p9, p12, p11}};
  lanelet::AttributeMap vehicleAttr{{AttrStr::Vehicle, true}, {AttrStr::Location, Value::Urban}};
  lanelet::AttributeMap pedestrianAttr{{AttrStr::Pedestrian, true}, {AttrStr::Location, Value::Urban}};
  lanelet::Lanelet lanelet{1, ls3, ls2, vehicleAttr}, left{2, ls4, ls3, vehicleAttr}, right{3, ls2, ls1, vehicleAttr},
      next{4, ls6, ls5, vehicleAttr};
  lanelet::Area area{1, {ls8, ls7, ls5}, {}, vehicleAttr}, nextArea{2, {ls9, ls8.invert()}};
};

class GermanTrafficRulesVehicle : public TrafficRules {
 public:
  lanelet::TrafficRulesPtr germanVehicle{germanVehicleRules()};
};

class GermanTrafficRulesBike : public TrafficRules {
 public:
  lanelet::TrafficRulesPtr germanBike{germanBikeRules()};
};

class GermanTrafficRulesPedestrian : public TrafficRules {
 public:
  GermanTrafficRulesPedestrian() {
    lanelet.attributes() = pedestrianAttr;
    right.attributes() = pedestrianAttr;
    next.attributes() = pedestrianAttr;
    area.attributes() = pedestrianAttr;
  }
  lanelet::TrafficRulesPtr germanPedestrian{germanPedestrianRules()};
};

namespace can_drive {
TEST_F(GermanTrafficRulesVehicle, vehicleCanPassVehicleLanelet) {  // NOLINT
  lanelet.setAttribute(Attr::Vehicle, true);
  EXPECT_TRUE(germanVehicle->canPass(lanelet));
  EXPECT_TRUE(germanVehicle->isOneWay(lanelet));
}

TEST_F(GermanTrafficRulesVehicle, vehicleCanNotPassNonVehicleLanelet) {  // NOLINT
  lanelet.setAttribute(Attr::Vehicle, false);
  EXPECT_FALSE(germanVehicle->canPass(lanelet));
}

TEST_F(GermanTrafficRulesVehicle, oneWayLanelet) {  // NOLINT
  lanelet.setAttribute(Attr::OneWay, false);
  EXPECT_FALSE(germanVehicle->isOneWay(lanelet));
}

TEST_F(GermanTrafficRulesVehicle, vehicleCanDriveIntoNextLanelet) {  // NOLINT
  EXPECT_TRUE(germanVehicle->canPass(lanelet, next));
}

TEST_F(GermanTrafficRulesVehicle, vehicleCanNotDriveBackwardsFromLanelet) {  // NOLINT
  EXPECT_FALSE(germanVehicle->canPass(next, lanelet));
}

TEST_F(GermanTrafficRulesVehicle, vehicleCanNotDriveAgainstDrivingDirection) {  // NOLINT
  EXPECT_FALSE(germanVehicle->canPass(next.invert(), lanelet.invert()));
}

TEST_F(GermanTrafficRulesVehicle, vehicleDriveBackwardsFromTwoWayLanelet) {  // NOLINT
  lanelet.setAttribute(Attr::OneWay, false);
  next.setAttribute(Attr::OneWay, false);
  EXPECT_FALSE(germanVehicle->canPass(next, lanelet));
  EXPECT_TRUE(germanVehicle->canPass(next.invert(), lanelet.invert()));
}

TEST_F(GermanTrafficRulesVehicle, canNotDriveToRandomLanelet) {  // NOLINT
  EXPECT_FALSE(germanVehicle->canPass(left, right));
}

TEST_F(GermanTrafficRulesBike, bikeCanNotPassVehicleLanelet) {  // NOLINT
  lanelet.setAttribute(Attr::Vehicle, true);
  EXPECT_FALSE(germanBike->canPass(lanelet));
  EXPECT_TRUE(germanBike->isOneWay(lanelet));
}

TEST_F(GermanTrafficRulesBike, bikeCanPassBikeLanelet) {  // NOLINT
  lanelet.setAttribute(Attr::Bicycle, true);
  EXPECT_TRUE(germanBike->canPass(lanelet));
  EXPECT_TRUE(germanBike->isOneWay(lanelet));
}

TEST_F(GermanTrafficRulesBike, bikeCanNotPassNonVehicleLanelet) {  // NOLINT
  lanelet.setAttribute(Attr::Vehicle, false);
  EXPECT_FALSE(germanBike->canPass(lanelet));
}

TEST_F(GermanTrafficRulesBike, oneWayLanelet) {  // NOLINT
  lanelet.setAttribute(Attr::OneWay, false);
  EXPECT_FALSE(germanBike->isOneWay(lanelet));
}

TEST_F(GermanTrafficRulesBike, bikeCanDriveIntoNextLanelet) {  // NOLINT
  lanelet.setAttribute(Attr::Bicycle, true);
  next.setAttribute(Attr::Bicycle, true);
  EXPECT_TRUE(germanBike->canPass(lanelet, next));
}

TEST_F(GermanTrafficRulesBike, bikeCanNotDriveBackwardsFromLanelet) {  // NOLINT
  lanelet.setAttribute(Attr::Bicycle, true);
  next.setAttribute(Attr::Bicycle, true);
  EXPECT_FALSE(germanBike->canPass(next, lanelet));
}

TEST_F(GermanTrafficRulesBike, bikeCanNotDriveAgainstDrivingDirection) {  // NOLINT
  lanelet.setAttribute(Attr::Bicycle, true);
  next.setAttribute(Attr::Bicycle, true);
  EXPECT_FALSE(germanBike->canPass(next.invert(), lanelet.invert()));
}

TEST_F(GermanTrafficRulesBike, bikeDriveBackwardsFromTwoWayLanelet) {  // NOLINT
  lanelet.setAttribute(Attr::Bicycle, true);
  next.setAttribute(Attr::Bicycle, true);
  lanelet.setAttribute(Attr::OneWay, false);
  next.setAttribute(Attr::OneWay, false);
  EXPECT_FALSE(germanBike->canPass(next, lanelet));
  EXPECT_TRUE(germanBike->canPass(next.invert(), lanelet.invert()));
}

TEST_F(GermanTrafficRulesBike, canNotDriveToRandomLanelet) {  // NOLINT
  EXPECT_FALSE(germanBike->canPass(left, right));
}

TEST_F(GermanTrafficRulesBike, canNotDriveBikeExplicitlyForbidden) {  // NOLINT
  lanelet.setAttribute(Attr::Bicycle, false);
  EXPECT_FALSE(germanBike->canPass(lanelet));
}

TEST_F(GermanTrafficRulesBike, canNotDriveBikeExplicitlyForbiddenVehiclesAllowed) {  // NOLINT
  lanelet.setAttribute(Attr::Bicycle, false);
  lanelet.setAttribute(Attr::Vehicle, true);
  EXPECT_FALSE(germanBike->canPass(lanelet));
}

TEST_F(GermanTrafficRulesBike, canNotDriveIntoBikeExplicitlyForbidden) {  // NOLINT
  lanelet.setAttribute(Attr::Vehicle, true);
  next.setAttribute(Attr::Bicycle, false);
  next.setAttribute(Attr::Vehicle, true);
  EXPECT_FALSE(germanBike->canPass(lanelet, next));
}

TEST_F(GermanTrafficRulesPedestrian, canGoToNextLanelet) {  // NOLINT
  EXPECT_TRUE(germanPedestrian->canPass(lanelet, next));
}

TEST_F(GermanTrafficRulesPedestrian, canNotWalkOnVehicleLanelet) {  // NOLINT
  EXPECT_FALSE(germanPedestrian->canPass(left));
}

TEST_F(GermanTrafficRulesPedestrian, canNotGoBackFromOneWayLanelet) {  // NOLINT
  EXPECT_FALSE(germanPedestrian->canPass(next.invert(), lanelet.invert()));
}

TEST_F(GermanTrafficRulesPedestrian, canGoBackFromTwoWayLanelet) {  // NOLINT
  lanelet.setAttribute(Attr::OneWay, false);
  next.setAttribute(Attr::OneWay, false);
  EXPECT_TRUE(germanPedestrian->canPass(next.invert(), lanelet.invert()));
}

TEST_F(GermanTrafficRulesPedestrian, canStandOnPedestrianArea) {  // NOLINT
  area.attributes() = pedestrianAttr;
  EXPECT_TRUE(germanPedestrian->canPass(area));
}

TEST_F(GermanTrafficRulesPedestrian, canNotGoIntoUnknownArea) {  // NOLINT
  area.attributes() = pedestrianAttr;
  right.attributes() = pedestrianAttr;
  EXPECT_FALSE(germanPedestrian->canPass(area, right));
}

TEST_F(GermanTrafficRulesPedestrian, canGoAcrossLowCurbstone) {  // NOLINT
  area.attributes() = pedestrianAttr;
  right.attributes() = pedestrianAttr;
  ls7.setAttribute(Attr::Type, Value::Curbstone);
  ls7.setAttribute(Attr::Subtype, Value::Low);
  right.setAttribute(Attr::OneWay, false);
  EXPECT_TRUE(germanPedestrian->canPass(area, right.invert()));
  EXPECT_FALSE(germanPedestrian->canPass(area, right));
  EXPECT_TRUE(germanPedestrian->canPass(right, area));
}

TEST_F(GermanTrafficRulesPedestrian, canGoAcrossLowCurbstonetoNextArea) {  // NOLINT
  area.attributes() = pedestrianAttr;
  nextArea.attributes() = pedestrianAttr;
  ls8.setAttribute(Attr::Type, Value::Curbstone);
  ls8.setAttribute(Attr::Subtype, Value::Low);
  EXPECT_TRUE(germanPedestrian->canPass(area, nextArea));
  EXPECT_TRUE(germanPedestrian->canPass(nextArea, area));
}

TEST_F(GermanTrafficRulesPedestrian, canGoRightAcrossLowCurbstone) {  // NOLINT
  area.attributes() = pedestrianAttr;
  next.attributes() = pedestrianAttr;
  ls5.setAttribute(Attr::Type, Value::Curbstone);
  ls5.setAttribute(Attr::Subtype, Value::Low);
  EXPECT_TRUE(germanPedestrian->canPass(area, next));
  EXPECT_TRUE(germanPedestrian->canPass(next, area));
}

TEST_F(GermanTrafficRulesPedestrian, cannotCrossUnrelated) {  // NOLINT
  area.attributes() = pedestrianAttr;
  left.attributes() = pedestrianAttr;
  ls5.setAttribute(Attr::Type, Value::Curbstone);
  ls5.setAttribute(Attr::Subtype, Value::Low);
  EXPECT_FALSE(germanPedestrian->canPass(area, left));
  EXPECT_FALSE(germanPedestrian->canPass(left, area));
}
}  // namespace can_drive

namespace speed_limits {
using namespace lanelet::units::literals;
TEST_F(GermanTrafficRulesVehicle, defaultUrbanSpeedlimitIfNoSpeedLimitSet) {  // NOLINT
  lanelet.setAttribute(Attr::Location, Value::Urban);
  auto limit = germanVehicle->speedLimit(lanelet);
  EXPECT_EQ(limit.speedLimit, 50_kmh);
  EXPECT_TRUE(limit.isMandatory);
}

TEST_F(GermanTrafficRulesVehicle, defaultNonUrbanSpeedlimitIfNoSpeedLimitSet) {  // NOLINT
  lanelet.setAttribute(Attr::Location, Value::Nonurban);
  auto limit = germanVehicle->speedLimit(lanelet);
  EXPECT_EQ(limit.speedLimit, 100_kmh);
  EXPECT_TRUE(limit.isMandatory);
}

TEST_F(GermanTrafficRulesVehicle, NoSpeedLimitIfOnHighway) {  // NOLINT
  lanelet.setAttribute(Attr::Location, Value::Nonurban);
  lanelet.setAttribute(Attr::Subtype, Value::Highway);
  auto limit = germanVehicle->speedLimit(lanelet);
  EXPECT_EQ(limit.speedLimit, 130_kmh);
  EXPECT_FALSE(limit.isMandatory);
}
TEST_F(GermanTrafficRulesVehicle, determinesSpeedLimit30) {  // NOLINT
  lanelet.addRegulatoryElement(getSpeedLimit("de274"));
  auto limit = germanVehicle->speedLimit(lanelet);
  EXPECT_EQ(limit.speedLimit, 30_kmh);
  EXPECT_TRUE(limit.isMandatory);
}

TEST_F(GermanTrafficRulesVehicle, determinesSpeedLimit50) {  // NOLINT
  lanelet.setAttribute(Attr::Location, Value::Nonurban);
  lanelet.addRegulatoryElement(getSpeedLimit("de274-50"));
  auto limit = germanVehicle->speedLimit(lanelet);
  EXPECT_EQ(limit.speedLimit, 50_kmh);
  EXPECT_TRUE(limit.isMandatory);
}

TEST_F(GermanTrafficRulesVehicle, determinesSpeedLimit120) {  // NOLINT
  lanelet.setAttribute(Attr::Location, Value::Nonurban);
  lanelet.setAttribute(Attr::Subtype, Value::Highway);
  lanelet.addRegulatoryElement(getSpeedLimit("de274-120"));
  auto limit = germanVehicle->speedLimit(lanelet);
  EXPECT_EQ(limit.speedLimit, 120_kmh);
  EXPECT_TRUE(limit.isMandatory);
}
}  // namespace speed_limits

namespace lane_change {

TEST_F(GermanTrafficRulesVehicle, canNotLaneChangeLeftWithDefaultLine) {  // NOLINT
  EXPECT_FALSE(germanVehicle->canChangeLane(lanelet, left));
}

TEST_F(GermanTrafficRulesVehicle, canNotLaneChangeToUnconnectedLanelet) {  // NOLINT
  EXPECT_FALSE(germanVehicle->canChangeLane(right, left));
}

TEST_F(GermanTrafficRulesVehicle, canLaneChangeLeftViaDashedLine) {  // NOLINT
  ls3.setAttribute(Attr::Type, Value::LineThin);
  ls3.setAttribute(Attr::Subtype, Value::Dashed);
  EXPECT_TRUE(germanVehicle->canChangeLane(lanelet, left));
}

TEST_F(GermanTrafficRulesVehicle, canLaneChangeRightViaDashedLine) {  // NOLINT
  ls3.setAttribute(Attr::Type, Value::LineThin);
  ls3.setAttribute(Attr::Subtype, Value::Dashed);
  EXPECT_TRUE(germanVehicle->canChangeLane(left, lanelet));
}

TEST_F(GermanTrafficRulesVehicle, canNotLaneChangeToInvertedLanelet) {  // NOLINT
  ls3.setAttribute(Attr::Type, Value::LineThin);
  ls3.setAttribute(Attr::Subtype, Value::Dashed);
  EXPECT_FALSE(germanVehicle->canChangeLane(lanelet, left.invert()));
}
TEST_F(GermanTrafficRulesVehicle, canNotLaneChangeLaneIfNotDrivable) {  // NOLINT
  ls3.setAttribute(Attr::Type, Value::LineThin);
  ls3.setAttribute(Attr::Subtype, Value::Dashed);
  left.setAttribute(Attr::Vehicle, false);
  EXPECT_FALSE(germanVehicle->canChangeLane(lanelet, left));
}
TEST_F(GermanTrafficRulesVehicle, canLaneChangeRightViaDashedStraightLine) {  // NOLINT
  ls3.setAttribute(Attr::Type, Value::LineThin);
  ls3.setAttribute(Attr::Subtype, Value::DashedStraight);
  EXPECT_TRUE(germanVehicle->canChangeLane(left, lanelet));
  EXPECT_FALSE(germanVehicle->canChangeLane(lanelet, left));
}
TEST_F(GermanTrafficRulesVehicle, canNotLaneChangeViaStraightLine) {  // NOLINT
  ls3.setAttribute(Attr::Type, Value::LineThin);
  ls3.setAttribute(Attr::Subtype, Value::Straight);
  EXPECT_FALSE(germanVehicle->canChangeLane(lanelet, left));
}

TEST_F(GermanTrafficRulesVehicle, canNotLaneChangeIfOverridden) {  // NOLINT
  ls3.setAttribute(Attr::Type, Value::LineThin);
  ls3.setAttribute(Attr::Subtype, Value::Dashed);
  ls3.setAttribute(Attr::LaneChange, false);
  EXPECT_FALSE(germanVehicle->canChangeLane(lanelet, left));
}

TEST_F(GermanTrafficRulesVehicle, canLaneChangeInverted) {  // NOLINT
  ls3.setAttribute(Attr::Type, Value::LineThin);
  ls3.setAttribute(Attr::Subtype, Value::StraightDashed);
  lanelet.setAttribute(Attr::OneWay, false);
  left.setAttribute(Attr::OneWay, false);
  EXPECT_TRUE(germanVehicle->canChangeLane(lanelet.invert(), left.invert()));
  EXPECT_FALSE(germanVehicle->canChangeLane(left.invert(), lanelet.invert()));
}

TEST_F(GermanTrafficRulesVehicle, canNotLaneChangeOnVirtualLine) {  // NOLINT
  ls3.setAttribute(Attr::Type, Value::Virtual);
  EXPECT_FALSE(germanVehicle->canChangeLane(lanelet, left));
  EXPECT_FALSE(germanVehicle->canChangeLane(left, lanelet));
}

TEST_F(GermanTrafficRulesVehicle, canLaneChangeBothSidesExplicitly) {  // NOLINT
  ls3.setAttribute(Attr::Type, Value::LineThin);
  ls3.setAttribute(Attr::Subtype, Value::Straight);
  ls3.setAttribute(Attr::LaneChange, true);
  EXPECT_TRUE(germanVehicle->canChangeLane(lanelet, left));
  EXPECT_TRUE(germanVehicle->canChangeLane(left, lanelet));
}

TEST_F(GermanTrafficRulesVehicle, canLaneChangeBothSidesBothExplicitly) {  // NOLINT
  ls3.setAttribute(Attr::Type, Value::Virtual);
  ls3.setAttribute(Attr::LaneChangeLeft, true);
  ls3.setAttribute(Attr::LaneChangeRight, true);
  EXPECT_TRUE(germanVehicle->canChangeLane(lanelet, left));
  EXPECT_TRUE(germanVehicle->canChangeLane(left, lanelet));
}

TEST_F(GermanTrafficRulesVehicle, canLaneChangeLeftExplicitly) {  // NOLINT
  ls3.setAttribute(Attr::Type, Value::Virtual);
  ls3.setAttribute(Attr::LaneChangeLeft, true);
  EXPECT_TRUE(germanVehicle->canChangeLane(lanelet, left));
  EXPECT_FALSE(germanVehicle->canChangeLane(left, lanelet));
}
TEST_F(GermanTrafficRulesVehicle, canLaneChangeRightExplicitly) {  // NOLINT
  ls3.setAttribute(Attr::Type, Value::Virtual);
  ls3.setAttribute(Attr::LaneChangeRight, true);
  EXPECT_FALSE(germanVehicle->canChangeLane(lanelet, left));
  EXPECT_TRUE(germanVehicle->canChangeLane(left, lanelet));
}
}  // namespace lane_change

namespace other {

TEST_F(GermanTrafficRulesVehicle, normalRegelemIsNotDynamic) {  // NOLINT
  lanelet.addRegulatoryElement(getSpeedLimit("de274-60"));
  EXPECT_FALSE(germanVehicle->hasDynamicRules(lanelet));
}

TEST_F(GermanTrafficRulesVehicle, dynamicRegelemIsDynamic) {  // NOLINT
  auto regelem = getSpeedLimit("de274", {{AttrStr::Dynamic, true}});
  lanelet.addRegulatoryElement(regelem);
  EXPECT_TRUE(germanVehicle->hasDynamicRules(lanelet));
}

TEST_F(GermanTrafficRulesVehicle, locationIsGermany) {  // NOLINT
  using namespace lanelet;
  EXPECT_EQ(germanVehicle->location(), Locations::Germany);
}

TEST_F(GermanTrafficRulesPedestrian, locationIsGermany) {  // NOLINT
  using namespace lanelet;
  EXPECT_EQ(germanPedestrian->location(), Locations::Germany);
}

TEST_F(GermanTrafficRulesVehicle, factoryReturnsVehicleInsteadOfCar) {  // NOLINT
  using namespace lanelet;
  auto trafficRules = TrafficRulesFactory::create(Locations::Germany, Participants::Car, {});
  EXPECT_EQ(germanVehicle->participant(), Participants::Vehicle);
}

TEST_F(GermanTrafficRulesVehicle, laneletsHaveSameTrafficRules) {  // NOLINT
  auto sl = getSpeedLimit("de274");
  lanelet.addRegulatoryElement(sl);
  left.addRegulatoryElement(sl);
  EXPECT_TRUE(germanVehicle->hasSameTrafficRules(lanelet, left));
}

TEST_F(GermanTrafficRulesVehicle, differingLaneletsDontHaveSameRules) {  // NOLINT
  lanelet.addRegulatoryElement(getSpeedLimit("de274"));
  EXPECT_FALSE(germanVehicle->hasSameTrafficRules(lanelet, left));
}
}  // namespace other
