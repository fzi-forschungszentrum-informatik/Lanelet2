#include <gtest/gtest.h>

#include "lanelet2_core/geometry/RegulatoryElement.h"
#include "lanelet2_core/primitives/BasicRegulatoryElements.h"
#include "lanelet2_core/utility/Utilities.h"

using namespace lanelet;

using GenericRegulatoryElementPtr = std::shared_ptr<GenericRegulatoryElement>;

class RegulatoryElementTest : public ::testing::Test {
 protected:
  void SetUp() override {
    p1 = Point3d(++id, 0., 1., 1.);
    p2 = Point3d(++id, 1., 1., 1.);
    p3 = Point3d(++id, 0., 0., 0.);
    p4 = Point3d(++id, 1., 0., 0.);
    p5 = Point3d(++id, 1, -1, 0.5);
    p6 = Point3d(++id, 1, -2, 0.5);
    ls1 = LineString3d(++id, Points3d{p1, p2});
    ls2 = LineString3d(++id, Points3d{p3, p4});
    ls3 = LineString3d(++id, Points3d{p3, p1});
    ls4 = LineString3d(++id, Points3d{p4, p2});
    ls5 = LineString3d(++id, Points3d{p5, p6});
    poly1 = Polygon3d(++id, Points3d{p1, p2, p3});

    ll1 = Lanelet(++id, ls1, ls2);
    ll2 = Lanelet(++id, ls2, ls3);
    ll3 = Lanelet(++id, ls1, ls2);
    ar = Area(++id, {ls1, ls3.invert(), ls2.invert(), ls4});

    regelemData = std::make_shared<RegulatoryElementData>(++id);
  }
  GenericRegulatoryElementPtr getGenericRegelem() {
    regelemData->parameters[RoleName::Yield].emplace_back(ll1);
    regelemData->parameters[RoleName::Cancels].emplace_back(ls1);
    regelemData->parameters[RoleName::Refers].emplace_back(ar);
    auto regelem = RegulatoryElementFactory::create("", regelemData);
    auto genericRegelem = std::dynamic_pointer_cast<GenericRegulatoryElement>(regelem);
    return genericRegelem;
  }

 public:
  Id id{0};
  Point3d p1, p2, p3, p4, p5, p6;
  LineString3d ls1, ls2, ls3, ls4, ls5, ls6;
  Polygon3d poly1;
  Lanelet ll1, ll2, ll3;
  Area ar;
  RegulatoryElementDataPtr regelemData;
};

TEST_F(RegulatoryElementTest, factoryHasGenericRegelem) {  // NOLINT
  EXPECT_TRUE(utils::contains(RegulatoryElementFactory::availableRules(), "regulatory_element"));
}

TEST_F(RegulatoryElementTest, emptyGenericRegelem) {  // NOLINT
  auto regelem = RegulatoryElementFactory::create("", regelemData);
  EXPECT_TRUE(regelem->empty());
  EXPECT_TRUE(regelem->roles().empty());
}

TEST_F(RegulatoryElementTest, fullGenericRegelem) {  // NOLINT
  auto genericRegelem = getGenericRegelem();
  ASSERT_TRUE(!!genericRegelem);
  genericRegelem->addParameter(RoleName::Yield, ll1);
  genericRegelem->addParameter(RoleName::Cancels, ls1);
  genericRegelem->addParameter(RoleName::Refers, ar);
  EXPECT_FALSE(genericRegelem->empty());
  EXPECT_TRUE(utils::contains(genericRegelem->roles(), RoleNameString::Yield));
}

TEST_F(RegulatoryElementTest, findWorks) {  // NOLINT
  auto regelem = getGenericRegelem();
  EXPECT_TRUE(!!regelem->find<ConstLanelet>(ll1.id()));
  EXPECT_FALSE(!!regelem->find<ConstPoint3d>(ll1.id()));
}

TEST_F(RegulatoryElementTest, boundingBox2dWorks) {  // NOLINT
  auto regelem = getGenericRegelem();
  auto bbox = geometry::boundingBox2d(regelem);
  EXPECT_DOUBLE_EQ(0, bbox.min().x());
  EXPECT_DOUBLE_EQ(0, bbox.min().y());
  EXPECT_DOUBLE_EQ(1, bbox.max().x());
  EXPECT_DOUBLE_EQ(1, bbox.max().y());
}

TEST_F(RegulatoryElementTest, boundingBox3dWorks) {  // NOLINT
  auto regelem = getGenericRegelem();
  auto bbox = geometry::boundingBox3d(regelem);
  EXPECT_DOUBLE_EQ(0, bbox.min().x());
  EXPECT_DOUBLE_EQ(0, bbox.min().y());
  EXPECT_DOUBLE_EQ(0, bbox.min().z());
  EXPECT_DOUBLE_EQ(1, bbox.max().x());
  EXPECT_DOUBLE_EQ(1, bbox.max().y());
  EXPECT_DOUBLE_EQ(1, bbox.max().z());
}

TEST_F(RegulatoryElementTest, hasWorksForRegelem) {  // NOLINT
  auto regelem = getGenericRegelem();
  EXPECT_TRUE(utils::has(regelem, ls1.id()));
  EXPECT_TRUE(utils::has(regelem, p1.id()));
  EXPECT_FALSE(utils::has(regelem, id + 1));
}

TEST_F(RegulatoryElementTest, addGenericRegulatoryElementToLanelet) {  // NOLINT
  auto regelem = RegulatoryElementFactory::create("", regelemData);
  ll1.addRegulatoryElement(regelem);
  EXPECT_EQ(1ul, ll1.regulatoryElementsAs<GenericRegulatoryElement>().size());
}

// ============ Traffic Light ===========================
TEST_F(RegulatoryElementTest,  // NOLINT
       FactoryCannotConstructInvalidTrafficLight) {
  // NOLINTNEXTLINE
  EXPECT_THROW(RegulatoryElementFactory::create(AttributeValueString::TrafficLight, regelemData), InvalidInputError);
}

TEST_F(RegulatoryElementTest, FactoryConstructsTrafficLight) {  // NOLINT
  auto tl = TrafficLight::make(++id, AttributeMap(), {ls5}, ls4);
  auto factoryTl = RegulatoryElementFactory::create(tl->attribute(AttributeName::Subtype).value(),
                                                    std::const_pointer_cast<RegulatoryElementData>(tl->constData()));
  EXPECT_TRUE(!!std::dynamic_pointer_cast<TrafficLight>(factoryTl));
}

TEST_F(RegulatoryElementTest, TrafficLightWorksAsExpected) {  // NOLINT
  auto tl = TrafficLight::make(++id, AttributeMap(), {ls5}, ls4);
  tl->setStopLine(ls5);
  EXPECT_EQ(ls5, tl->stopLine());
  tl->addTrafficLight(ls1);
  EXPECT_EQ(2ul, tl->trafficLights().size());
}

TEST_F(RegulatoryElementTest, TrafficLightCreationFomPolygonWorks) {  // NOLINT
  auto tl = TrafficLight::make(++id, {}, {poly1});
  auto lights = tl->trafficLights();
  ASSERT_FALSE(lights.empty());
  EXPECT_TRUE(lights.front().isPolygon());
  EXPECT_EQ(lights.front(), poly1);
}

// ============ Traffic Sign ===========================
TEST_F(RegulatoryElementTest, FactoryCannotConstructInvalidTrafficSign) {  // NOLINT
  // NOLINTNEXTLINE
  EXPECT_THROW(RegulatoryElementFactory::create(AttributeValueString::TrafficSign, regelemData), InvalidInputError);
}

TEST_F(RegulatoryElementTest, FactoryConstructsTrafficSign) {  // NOLINT
  auto ts = TrafficSign::make(++id, AttributeMap(), {{ls5}, "de205"}, {}, {ls4});
  auto factoryTs = RegulatoryElementFactory::create(ts->attribute(AttributeName::Subtype).value(),
                                                    std::const_pointer_cast<RegulatoryElementData>(ts->constData()));
  EXPECT_TRUE(!!std::dynamic_pointer_cast<TrafficSign>(factoryTs));
}

TEST_F(RegulatoryElementTest, ConstructValidTrafficSign) {  // NOLINT
  auto ts = TrafficSign::make(++id, AttributeMap(), {{ls5}, "de205"}, {{poly1}, "de206"}, {ls4});
  ts->addRefLine(ls5);
  EXPECT_EQ(2ul, ts->refLines().size());

  ts->addCancellingRefLine(ls1);
  EXPECT_TRUE(utils::contains(ts->cancelLines(), ls1));

  EXPECT_EQ(ts->type(), "de205");

  ASSERT_TRUE(!ts->cancelTypes().empty());
  EXPECT_EQ(ts->cancelTypes().front(), "de206");

  ts->addCancellingTrafficSign({{ls2}, "de207"});
  std::vector<std::string> expect{"de206", "de207"};
  EXPECT_EQ(ts->cancelTypes(), expect);

  EXPECT_TRUE(ts->removeCancellingTrafficSign(poly1));
  ASSERT_TRUE(!ts->cancelTypes().empty());
  EXPECT_EQ(ts->cancelTypes().front(), "de207");
}

// ============ Speed limit ===========================
TEST_F(RegulatoryElementTest, FactoryCannotConstructInvalidSpeedLimit) {  // NOLINT
  // NOLINTNEXTLINE
  EXPECT_THROW(RegulatoryElementFactory::create(AttributeValueString::SpeedLimit, regelemData), InvalidInputError);
}
TEST_F(RegulatoryElementTest, FactoryConstructsSpeedLimit) {  // NOLINT
  auto sl = SpeedLimit::make(++id, AttributeMap(), {{ls5}, "de205"}, {}, {ls4});
  auto factorySl = RegulatoryElementFactory::create(sl->attribute(AttributeName::Subtype).value(),
                                                    std::const_pointer_cast<RegulatoryElementData>(sl->constData()));
  EXPECT_TRUE(!!std::dynamic_pointer_cast<SpeedLimit>(factorySl));
}

// ============ Right of way ===========================
TEST_F(RegulatoryElementTest, FactoryCannotConstructInvalidRightOfWay) {  // NOLINT
  // NOLINTNEXTLINE
  EXPECT_THROW(RegulatoryElementFactory::create(AttributeValueString::RightOfWay, regelemData), InvalidInputError);
}

TEST_F(RegulatoryElementTest, FactoryConstructsRightOfWay) {  // NOLINT
  auto row = RightOfWay::make(++id, AttributeMap(), {ll1}, {ll2}, ls1);
  auto factoryRow = RegulatoryElementFactory::create(row->attribute(AttributeName::Subtype).value(),
                                                     std::const_pointer_cast<RegulatoryElementData>(row->constData()));
  EXPECT_TRUE(!!std::dynamic_pointer_cast<RightOfWay>(factoryRow));
}

TEST_F(RegulatoryElementTest, ConstructValidRightOfWay) {  // NOLINT
  auto row = RightOfWay::make(++id, AttributeMap(), {ll1}, {ll2});
  EXPECT_EQ(row->getManeuver(ll1), ManeuverType::RightOfWay);
  EXPECT_EQ(row->getManeuver(ll2), ManeuverType::Yield);
  EXPECT_EQ(row->getManeuver(ll3), ManeuverType::Unknown);
  EXPECT_FALSE(row->stopLine());

  row->addRightOfWayLanelet(ll3);
  row->setStopLine(ls1);
  EXPECT_EQ(row->getManeuver(ll3), ManeuverType::RightOfWay);
  EXPECT_EQ(ls1, *row->stopLine());
}

// =========== All Way Stop ==============================
TEST_F(RegulatoryElementTest, FactoryCannotConstructInvalidAllWayStop) {  // NOLINT
  regelemData->parameters[RoleName::Yield].emplace_back(ll1);
  regelemData->parameters[RoleName::Yield].emplace_back(ll2);
  regelemData->parameters[RoleName::RefLine].emplace_back(ls1);
  // NOLINTNEXTLINE
  EXPECT_THROW(RegulatoryElementFactory::create(AttributeValueString::AllWayStop, regelemData), InvalidInputError);
  regelemData->parameters[RoleName::RefLine].emplace_back(ls1);
  regelemData->parameters[RoleName::RightOfWay].emplace_back(ll1);
  // NOLINTNEXTLINE
  EXPECT_THROW(RegulatoryElementFactory::create(AttributeValueString::AllWayStop, regelemData), InvalidInputError);
}

TEST_F(RegulatoryElementTest, FactoryConstructsAllWayStop) {  // NOLINT
  auto row = AllWayStop::make(++id, AttributeMap(), {{ll1, ls1}});
  auto factoryAws = RegulatoryElementFactory::create(row->attribute(AttributeName::Subtype).value(),
                                                     std::const_pointer_cast<RegulatoryElementData>(row->constData()));
  EXPECT_TRUE(!!std::dynamic_pointer_cast<AllWayStop>(factoryAws));
}

TEST_F(RegulatoryElementTest, ConstructValidAllWayStop) {  // NOLINT
  auto row = AllWayStop::make(++id, AttributeMap(), {{ll1, ls1}});
  EXPECT_EQ(row->stopLines().size(), 1UL);
  EXPECT_EQ(row->lanelets().size(), 1UL);
  EXPECT_TRUE(row->trafficSigns().empty());
  EXPECT_EQ(row->getStopLine(ll1), ls1);
  EXPECT_TRUE(row->removeLanelet(ll1));
  EXPECT_TRUE(row->lanelets().empty());
  EXPECT_TRUE(row->trafficSigns().empty());
  row->addLanelet({ll2, {}});
  EXPECT_THROW(row->addLanelet({ll1, ls1}), InvalidInputError);  // NOLINT
  EXPECT_TRUE(row->trafficSigns().empty());
}
