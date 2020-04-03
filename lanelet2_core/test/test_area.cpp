#include <gtest/gtest.h>
#include "geometry/Area.h"
#include "geometry/Lanelet.h"
#include "primitives/Area.h"
#include "primitives/Lanelet.h"
#include "primitives/RegulatoryElement.h"

using namespace lanelet;

class TestArea : public testing::Test {
 public:
  TestArea() {
    for (auto i = 0u; i < 4; ++i) {
      pointsLeft.push_back(Point3d(++id, i, 1));
      pointsRight.push_back(Point3d(++id, i, 0));
    }
    firstCircle = LineString3d(++id, {pointsLeft[0], pointsLeft[1], pointsRight[1], pointsRight[0]});
    firstCircle1 = LineString3d(++id, {pointsLeft[0], pointsLeft[1]});
    firstCircle2 = LineString3d(++id, {pointsLeft[1], pointsRight[1]});
    firstCircle3 = LineString3d(++id, {pointsRight[1], pointsRight[0]});
    secondCircle1 = LineString3d(++id, {pointsLeft[1], pointsLeft[2]});
    secondCircle2 = LineString3d(++id, {pointsLeft[2], pointsRight[2]});
    secondCircle3 = LineString3d(++id, {pointsRight[2], pointsRight[1]});
    auto holeId = ++id;
    hole = LineString3d(holeId, {Point3d(++id, 1.25, 0.25), Point3d(++id, 1.75, 0.25), Point3d(++id, 1.75, 0.75),
                                 Point3d(++id, 1.25, 0.75)});
    thirdLine1 = LineString3d(++id, {pointsLeft[2], pointsLeft[3]});
    thirdLine2 = LineString3d(++id, {pointsRight[2], pointsRight[3]});
    auto lineId = ++id;
    rightLine = LineString3d(lineId, {Point3d(++id, 1, -1), Point3d(++id, 2, -1)});
    area1 = Area(++id, {firstCircle});
    area2 = Area(++id, {secondCircle1, secondCircle2, secondCircle3}, {{hole}});
    area3 = Area(++id, {firstCircle1, firstCircle2, firstCircle3});
    area4 = Area(++id, {secondCircle1, secondCircle2, secondCircle3, firstCircle2.invert()});
    laneletFollowing = Lanelet(++id, thirdLine1, thirdLine2);
    laneletRight = Lanelet(++id, secondCircle2.invert(), rightLine);
    regelem = std::make_shared<GenericRegulatoryElement>(++id);
  }

  Id id{0};
  Points3d pointsLeft, pointsRight;
  LineString3d firstCircle, firstCircle1, firstCircle2, firstCircle3, secondCircle1, secondCircle2, secondCircle3, hole,
      thirdLine1, thirdLine2, rightLine;
  Area area1, area2, area3, area4;
  Lanelet laneletFollowing, laneletRight;
  RegulatoryElementPtr regelem;
};

TEST_F(TestArea, attributes) {  // NOLINT
  area1.attributes()["test"] = true;
  EXPECT_TRUE(area1.attributeOr("test", false));
  EXPECT_TRUE(area1.hasAttribute("test"));
}

TEST_F(TestArea, id) {  // NOLINT
  area1.setId(123);
  EXPECT_EQ(area1.id(), 123);
}

TEST_F(TestArea, regelem) {  // NOLINT
  area1.addRegulatoryElement(regelem);
  EXPECT_EQ(1ul, area1.regulatoryElementsAs<GenericRegulatoryElement>().size());
  area1.removeRegulatoryElement(regelem);
  EXPECT_TRUE(area1.regulatoryElementsAs<GenericRegulatoryElement>().empty());
}

TEST_F(TestArea, inside) {  // NOLINT
  EXPECT_TRUE(geometry::inside(area1, BasicPoint2d(0.5, 0.5)));
  EXPECT_FALSE(geometry::inside(area2, BasicPoint2d(0.5, 0.5)));
  EXPECT_FALSE(geometry::inside(area2, BasicPoint2d(1.5, 0.5)));
}

TEST_F(TestArea, distance2d) {  // NOLINT
  auto d = geometry::distance2d(area2, utils::to2D(pointsLeft[3].basicPoint()));
  EXPECT_DOUBLE_EQ(1, d);
}

TEST_F(TestArea, distance2dWithHole) {  // NOLINT
  auto d = geometry::distance2d(area2, BasicPoint2d(1.5, 0.5));
  EXPECT_DOUBLE_EQ(0.25, d);
}

TEST_F(TestArea, distance3dWithHole) {  // NOLINT
  auto d = geometry::distance3d(area2, BasicPoint3d(1.5, 0.5, 1));
  EXPECT_DOUBLE_EQ(std::sqrt(0.25 * 0.25 + 1), d);
}

TEST_F(TestArea, area) {  // NOLINT
  auto area = geometry::area(area1.basicPolygonWithHoles2d());
  EXPECT_DOUBLE_EQ(1, area);
}

TEST_F(TestArea, areaWithHole) {  // NOLINT
  auto area = geometry::area(area2.basicPolygonWithHoles2d());
  EXPECT_DOUBLE_EQ(0.75, area);
}

TEST_F(TestArea, boundingBox) {  // NOLINT
  auto box = geometry::boundingBox2d(area2);
  EXPECT_EQ(box.min().x(), 1);
  EXPECT_EQ(box.max().x(), 2);
}

TEST_F(TestArea, adjacent) {  // NOLINT
  EXPECT_TRUE(geometry::adjacent(area2, area1));
  EXPECT_TRUE(geometry::adjacent(area1, area2));
  area1.setOuterBound(area2.outerBound());
  EXPECT_FALSE(geometry::adjacent(area1, area2));
}

TEST_F(TestArea, overlaps) {  // NOLINT
  EXPECT_FALSE(geometry::overlaps2d(area2, area1));
  EXPECT_FALSE(geometry::overlaps3d(area2, area1, 1));
  EXPECT_FALSE(geometry::overlaps2d(area2, laneletRight));
  EXPECT_FALSE(geometry::overlaps3d(area2, laneletFollowing, 1));
}

TEST_F(TestArea, follows) {  // NOLINT
  EXPECT_TRUE(geometry::follows(area2, laneletFollowing));
  EXPECT_TRUE(geometry::follows(laneletFollowing.invert(), area2));
  EXPECT_FALSE(geometry::follows(laneletFollowing, area2));
  EXPECT_FALSE(geometry::follows(laneletFollowing, area1));
}

TEST_F(TestArea, leftOf) {  // NOLINT
  EXPECT_TRUE(geometry::leftOf(laneletRight, area2));
  EXPECT_FALSE(geometry::leftOf(laneletRight.invert(), area2));
  EXPECT_FALSE(geometry::leftOf(laneletRight, area1));
}

TEST_F(TestArea, rightOf) {  // NOLINT
  EXPECT_FALSE(geometry::rightOf(laneletRight, area2));
  EXPECT_TRUE(geometry::rightOf(laneletRight.invert(), area2));
  EXPECT_FALSE(geometry::rightOf(laneletRight, area1));
}

TEST_F(TestArea, determineCommonLinePreceding) {
  auto res = geometry::determineCommonLinePreceding(laneletFollowing.invert(), area2);
  ASSERT_TRUE(!!res);
  EXPECT_EQ(res->id(), 14);
  EXPECT_FALSE(!!geometry::determineCommonLinePreceding(laneletFollowing, area2));
  EXPECT_FALSE(!!geometry::determineCommonLinePreceding(laneletFollowing.invert(), area1));
}

TEST_F(TestArea, determineCommonLineFollowing) {
  auto res = geometry::determineCommonLineFollowing(area2, laneletFollowing);
  ASSERT_TRUE(!!res);
  EXPECT_EQ(res->id(), 14);
  EXPECT_FALSE(!!geometry::determineCommonLineFollowing(area2, laneletFollowing.invert()));
  EXPECT_FALSE(!!geometry::determineCommonLineFollowing(area1, laneletFollowing));
}

TEST_F(TestArea, determineCommonLine) {
  auto res = geometry::determineCommonLine(area2, laneletFollowing);
  ASSERT_TRUE(!!res);
  EXPECT_EQ(res->id(), 14);
  res = geometry::determineCommonLine(area2, laneletFollowing.invert());
  ASSERT_TRUE(!!res);
  EXPECT_EQ(res->id(), 14);
  EXPECT_FALSE(!!geometry::determineCommonLine(area1, laneletFollowing.invert()));
  EXPECT_FALSE(!!geometry::determineCommonLine(area1, laneletFollowing));
}

TEST_F(TestArea, determineCommonLineArea) {
  auto res = geometry::determineCommonLine(area3, area4);
  ASSERT_TRUE(!!res);
  EXPECT_EQ(res->id(), 11);
  EXPECT_FALSE(res->inverted());
  res = geometry::determineCommonLine(area4, area3);
  ASSERT_TRUE(!!res);
  EXPECT_EQ(res->id(), 11);
  EXPECT_TRUE(res->inverted());
}