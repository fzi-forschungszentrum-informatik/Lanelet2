#include <gtest/gtest.h>

#include <future>

#include "lanelet2_core/LaneletMap.h"
#include "lanelet_map_test_case.h"

using namespace lanelet;
using namespace std::string_literals;

using utils::getId;

class LaneletMapTest : public ::testing::Test, public test_cases::LaneletMapTestCase {};

TEST(UniqueId, registerIdParallel) {  // NOLINT
  Ids ids{2000, 3000, 2001, -5, -200};
  auto registerId = [](auto id) { return std::async(std::launch::async, [id]() { utils::registerId(id); }); };
  auto futs = utils::transform(ids, registerId);
  utils::forEach(futs, [](auto& f) { f.get(); });
  auto newId = utils::getId();
  EXPECT_FALSE(!!utils::find(ids, newId));
}

TEST_F(LaneletMapTest, AddWorks) {  // NOLINT
  EXPECT_FALSE(map->pointLayer.exists(p5.id()));
  map->add(ll2);
  EXPECT_TRUE(map->pointLayer.exists(p5.id()));
  EXPECT_TRUE(map->laneletLayer.exists(ll2.id()));
}

TEST_F(LaneletMapTest, AddAPolygon) {  // NOLINT
  poly1.setId(InvalId);
  auto map = utils::createMap({poly1});
  Points3d pointsInLayer(map->pointLayer.begin(), map->pointLayer.end());
  std::sort(pointsInLayer.begin(), pointsInLayer.end(), utils::idLess<Point3d>);
  EXPECT_EQ(pointsInLayer, Points3d({p1, p2, p3, p4}));
}

TEST_F(LaneletMapTest, AddRegelemWorks) {  // NOLINT
  auto regelemData = std::make_shared<RegulatoryElementData>(InvalId);
  regelemData->parameters[RoleName::Refers].emplace_back(p1);
  regelemData->parameters[RoleName::RefLine].emplace_back(left);
  regelemData->parameters[RoleName::Refers].emplace_back(ll1);
  regelemData->parameters[RoleName::Refers].emplace_back(ar1);
  auto regelem = lanelet::RegulatoryElementFactory::create("", regelemData);
  LaneletMap map;
  EXPECT_EQ(InvalId, regelem->id());
  map.add(regelem);
  EXPECT_NE(InvalId, regelem->id());
  EXPECT_TRUE(map.pointLayer.exists(p1.id()));
  EXPECT_TRUE(map.lineStringLayer.exists(left.id()));
  EXPECT_TRUE(map.laneletLayer.exists(ll1.id()));
  EXPECT_TRUE(map.areaLayer.exists(ar1.id()));
  EXPECT_TRUE(map.pointLayer.exists(p1.id()));
}

TEST_F(LaneletMapTest, CanAddEmptyRegelem) {  // NOLINT
  LaneletMap map;
  auto regelemData = std::make_shared<RegulatoryElementData>(InvalId);
  auto regelem = lanelet::RegulatoryElementFactory::create("", regelemData);
  map.add(regelem);
  EXPECT_NE(InvalId, regelem->id());
}

TEST_F(LaneletMapTest, CanAddExistingElement) {  // NOLINT
  EXPECT_EQ(map->areaLayer.size(), 1ul);
  map->add(ar1);
  testConstAndNonConst([](auto& map) { EXPECT_EQ(map->areaLayer.size(), 1ul); });
}

TEST_F(LaneletMapTest, AddAssignsCorrectId) {  // NOLINT
  Point3d newPoint;
  EXPECT_EQ(newPoint.id(), InvalId);
  map->add(newPoint);
  EXPECT_NE(newPoint.id(), InvalId);
  EXPECT_TRUE(newPoint.id() < p1.id() || newPoint.id() > p4.id());
}

TEST_F(LaneletMapTest, FindWorks) {  // NOLINT
  auto elem = map->laneletLayer.get(ll1.id());
  EXPECT_EQ(elem, ll1);
}

TEST_F(LaneletMapTest, FindThrowsIfNotExistent) {  // NOLINT
  testConstAndNonConst([](auto& map) {
    EXPECT_THROW(map->laneletLayer.get(InvalId), NoSuchPrimitiveError);  // NOLINT
    EXPECT_THROW(map->laneletLayer.get(-5), NoSuchPrimitiveError);       // NOLINT
  });
}

TEST_F(LaneletMapTest, emptyWorks) {  // NOLINT
  testConstAndNonConst([](auto& map) {
    EXPECT_TRUE(LaneletMap().polygonLayer.empty());
    EXPECT_FALSE(map->areaLayer.empty());
  });
}

TEST_F(LaneletMapTest, nearestWorksForPoints) {  // NOLINT
  testConstAndNonConst([this](auto& map) {
    auto pts = map->pointLayer.nearest(Point2d(p3), 1);
    ASSERT_EQ(pts.size(), 1ul);
    EXPECT_EQ(pts[0], p3);
  });
}

TEST_F(LaneletMapTest, nearestWorksForLanelets) {  // NOLINT
  map->add(ll2);
  testConstAndNonConst([this](auto& map) {
    auto llts = map->laneletLayer.nearest(Point2d(p2), 1);
    ASSERT_EQ(llts.size(), 1ul);
    EXPECT_EQ(llts[0], ll1);
  });
}

TEST_F(LaneletMapTest, findWorksForPoints) {  // NOLINT
  map->add(ll2);
  testConstAndNonConst([this](auto& map) {
    auto pts = map->pointLayer.search(BoundingBox2d(BasicPoint2d(0, 0.2), BasicPoint2d(2, 1)));
    auto in = [pts](const Point3d& p) { return std::find(pts.begin(), pts.end(), p) != pts.end(); };
    ASSERT_EQ(pts.size(), 5ul);
    EXPECT_TRUE(in(p1));
    EXPECT_TRUE(in(p2));
    EXPECT_TRUE(in(p5));
    EXPECT_TRUE(in(p6));
    EXPECT_TRUE(in(p7));
  });
}

TEST_F(LaneletMapTest, findWorksForLanelets) {  // NOLINT
  map->add(ll2);
  testConstAndNonConst([this](auto& map) {
    auto llts = map->laneletLayer.search(BoundingBox2d(BasicPoint2d(0, 0.7), BasicPoint2d(2, 1)));
    ASSERT_EQ(llts.size(), 1ul);
    EXPECT_EQ(llts[0], ll1);
  });
}

TEST_F(LaneletMapTest, findWithLargeBoxWorksForLanelets) {  // NOLINT
  testConstAndNonConst([this](auto& map) {
    auto llts = map->laneletLayer.search(BoundingBox2d(BasicPoint2d(-5, -5), BasicPoint2d(5, 5)));
    ASSERT_EQ(llts.size(), 1ul);
    EXPECT_EQ(llts[0], ll1);
  });
}

TEST_F(LaneletMapTest, findNearestWorksForLanelets) {  // NOLINT
  this->map = utils::createMap({ll1, ll2});
  testConstAndNonConst([this](auto& map) {
    auto llts = geometry::findNearest(map->laneletLayer, BasicPoint2d(0, -10), 10);
    ASSERT_EQ(2ul, llts.size());
    EXPECT_DOUBLE_EQ(9, llts.front().first);
    EXPECT_EQ(ll2, llts.front().second);
  });
}

TEST_F(LaneletMapTest, findNearestWorksForRegElems) {  // NOLINT
  testConstAndNonConst([this](auto& map) {
    auto regElem = geometry::findNearest(map->regulatoryElementLayer, BasicPoint2d(0, -1), 10);
    ASSERT_EQ(1ul, regElem.size());
    EXPECT_DOUBLE_EQ(1, regElem.front().first);
    EXPECT_EQ(regelem1, regElem.front().second);
  });
}

TEST_F(LaneletMapTest, findNearestWorksForComplexRegElems) {  // NOLINT
  RuleParameterMap rules{
      {"test"s, {ll1}}, {"point"s, {p1, p9}}, {"areas"s, {ar1}}, {"linestr"s, {outside}}, {"poly"s, {poly1}}};
  auto regelem = std::make_shared<GenericRegulatoryElement>(getId(), rules);
  ll1.addRegulatoryElement(regelem);
  ar1.addRegulatoryElement(regelem);
  this->map = utils::createMap({ar1});
  this->map->add(ll1);
  testConstAndNonConst([this, regelem](auto& map) {
    auto regElem = geometry::findNearest(map->regulatoryElementLayer, BasicPoint2d(0, -1), 10);
    ASSERT_EQ(1ul, regElem.size());
    EXPECT_DOUBLE_EQ(0, regElem.front().first);
    EXPECT_EQ(regelem, regElem.front().second);
  });
}

TEST_F(LaneletMapTest, findNearestWorksOnEmptyMap) {  // NOLINT
  this->map = utils::createMap(Points3d());
  testConstAndNonConst([](auto& map) {
    auto pts = geometry::findNearest(map->pointLayer, BasicPoint2d(0, -10), 10);
    ASSERT_EQ(0ul, pts.size());
  });
}

TEST_F(LaneletMapTest, findUsagesInLanelet) {  // NOLINT
  testConstAndNonConst([this](auto& map) {
    auto llts = utils::findUsagesInLanelets(*map, p4);
    ASSERT_EQ(llts.size(), 1ul);
    EXPECT_EQ(llts[0], ll1);
  });
}

TEST_F(LaneletMapTest, findUsagesInAreas) {  // NOLINT
  testConstAndNonConst([this](auto& map) {
    auto ars = utils::findUsagesInAreas(*map, p4);
    ASSERT_EQ(ars.size(), 1ul);
    EXPECT_EQ(ars[0], ar1);
  });
}

TEST_F(LaneletMapTest, sizeAndEmpty) {  // NOLINT
  auto pointMap = utils::createMap(Points3d{map->pointLayer.begin(), map->pointLayer.end()});
  EXPECT_FALSE(pointMap->empty());
  EXPECT_EQ(pointMap->size(), pointMap->pointLayer.size());
  EXPECT_TRUE(LaneletMap().empty());
}

TEST_F(LaneletMapTest, findRegelemUsagesInLanelet) {  // NOLINT
  ll2.addRegulatoryElement(regelem1);
  map->add(ll2);
  testConstAndNonConst([this](auto& map) {  // NOLINT
    auto usages = map->laneletLayer.findUsages(regelem1);
    ASSERT_EQ(1ul, usages.size());
    EXPECT_EQ(ll2, usages[0]);
  });
}

TEST_F(LaneletMapTest, findRegelemUsagesInArea) {  // NOLINT
  ar1.addRegulatoryElement(regelem1);
  this->map = utils::createMap({ar1});
  testConstAndNonConst([this](auto& map) {
    auto usages = map->areaLayer.findUsages(regelem1);
    ASSERT_EQ(1ul, usages.size());
    EXPECT_EQ(ar1, usages[0]);
  });
}

TEST_F(LaneletMapTest, findUsagesInPolygon) {  // NOLINT
  testConstAndNonConst([this](auto& map) {
    auto polys = map->polygonLayer.findUsages(p1);
    ASSERT_EQ(polys.size(), 1ul);
    EXPECT_EQ(poly1, polys[0]);

    polys = map->polygonLayer.findUsages(p9);
    EXPECT_TRUE(polys.empty());
  });
}

TEST_F(LaneletMapTest, createConstMap) {  // NOLINT
  auto map = utils::createConstMap(ConstLanelets{ll1, ll2}, ConstAreas{ar1});
  EXPECT_TRUE(map->laneletLayer.exists(ll1.id()));
  EXPECT_TRUE(map->areaLayer.exists(ar1.id()));
}

TEST_F(LaneletMapTest, createSubmap) {  // NOLINT
  auto submap = utils::createSubmap(Lanelets{ll1, ll2});
  EXPECT_EQ(submap->size(), 2ul);
  auto llMap = submap->laneletMap();
  EXPECT_LT(0ul, llMap->lineStringLayer.size());
  EXPECT_LT(0ul, llMap->pointLayer.size());
}
