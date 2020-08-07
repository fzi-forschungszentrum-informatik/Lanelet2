#include <gtest/gtest.h>

#include "lanelet2_core/geometry/Area.h"
#include "lanelet2_core/geometry/Lanelet.h"
#include "lanelet2_core/primitives/Area.h"
#include "lanelet2_core/primitives/Lanelet.h"
#include "lanelet2_core/primitives/RegulatoryElement.h"

using namespace lanelet;

/*   1  f1   3   s1  5  tl1  7
 *   X------>X------>X------>X
 *           |  llU  |
 *           |f2  h  |s2 llF
 *     a1/3  V a2/4  V
 *   X<------X<------X------>X
 *   2  f3   4  s3   6  tl2  8
 *
 *             llB/R
 *           X-------X
 *           rightLine
 * y ^
 *   |
 *   O--->
 *       x
 */
class TestArea : public testing::Test {
 public:
  TestArea() {
    for (auto i = 0u; i < 4; ++i) {
      pointsLeft.push_back(Point3d(++id, i, 1));   // 1, 3, 5, 7
      pointsRight.push_back(Point3d(++id, i, 0));  // 2, 4, 6, 8
    }
    firstCircle = LineString3d(++id, {pointsLeft[0], pointsLeft[1], pointsRight[1], pointsRight[0]});  // 9
    firstCircle1 = LineString3d(++id, {pointsLeft[0], pointsLeft[1]});                                 // 10
    firstCircle2 = LineString3d(++id, {pointsLeft[1], pointsRight[1]});                                // 11
    firstCircle3 = LineString3d(++id, {pointsRight[1], pointsRight[0]});                               // 12
    secondCircle1 = LineString3d(++id, {pointsLeft[1], pointsLeft[2]});                                // 13
    secondCircle2 = LineString3d(++id, {pointsLeft[2], pointsRight[2]});                               // 14
    secondCircle3 = LineString3d(++id, {pointsRight[2], pointsRight[1]});                              // 15
    auto holeId = ++id;                                                                                // 16
    hole = LineString3d(holeId, {Point3d(++id, 1.25, 0.25), Point3d(++id, 1.75, 0.25), Point3d(++id, 1.75, 0.75),
                                 Point3d(++id, 1.25, 0.75)});                                  // 17, 18, 19, 20
    thirdLine1 = LineString3d(++id, {pointsLeft[2], pointsLeft[3]});                           // 21
    thirdLine2 = LineString3d(++id, {pointsRight[2], pointsRight[3]});                         // 22
    auto lineId = ++id;                                                                        // 23
    rightLine = LineString3d(lineId, {Point3d(++id, 1, -1), Point3d(++id, 2, -1)});            // 24, 25
    area1 = Area(++id, {firstCircle});                                                         // 26
    area2 = Area(++id, {secondCircle1, secondCircle2, secondCircle3}, {{hole}});               // 27
    area3 = Area(++id, {firstCircle1, firstCircle2, firstCircle3});                            // 28
    area4 = Area(++id, {secondCircle1, secondCircle2, secondCircle3, firstCircle2.invert()});  // 29
    laneletFollowing = Lanelet(++id, thirdLine1, thirdLine2);                                  // 30
    laneletRight = Lanelet(++id, secondCircle2.invert(), rightLine);                           // 31
    regelem = std::make_shared<GenericRegulatoryElement>(++id);                                // 32
    laneletBelow = Lanelet(++id, secondCircle3.invert(), rightLine);                           // 33
    laneletUpwards = Lanelet(++id, firstCircle2.invert(), secondCircle2.invert());             // 34
  }

  Id id{0};
  Points3d pointsLeft, pointsRight;
  LineString3d firstCircle, firstCircle1, firstCircle2, firstCircle3, secondCircle1, secondCircle2, secondCircle3, hole,
      thirdLine1, thirdLine2, rightLine;
  Area area1, area2, area3, area4;
  Lanelet laneletFollowing, laneletRight, laneletBelow, laneletUpwards;
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

TEST_F(TestArea, determineCommonLineFollowingOrPreceding) {
  auto res = geometry::determineCommonLineFollowingOrPreceding(area2, laneletFollowing);
  ASSERT_TRUE(!!res);
  EXPECT_EQ(res->id(), 14);
  res = geometry::determineCommonLineFollowingOrPreceding(area2, laneletFollowing.invert());
  ASSERT_TRUE(!!res);
  EXPECT_EQ(res->id(), 14);
  EXPECT_FALSE(!!geometry::determineCommonLineFollowingOrPreceding(area1, laneletFollowing.invert()));
  EXPECT_FALSE(!!geometry::determineCommonLineFollowingOrPreceding(area1, laneletFollowing));
  EXPECT_FALSE(!!geometry::determineCommonLineFollowingOrPreceding(area2, laneletRight));
  EXPECT_FALSE(!!geometry::determineCommonLineFollowingOrPreceding(area1, laneletBelow));
  EXPECT_FALSE(!!geometry::determineCommonLineFollowingOrPreceding(area2, laneletBelow));
}

TEST_F(TestArea, determineCommonLineLeft) {
  auto res = geometry::determineCommonLineLeft(laneletUpwards, area3);
  ASSERT_TRUE(!!res);
  EXPECT_EQ(res->id(), 11);
  EXPECT_TRUE(res->inverted());
  EXPECT_FALSE(!!geometry::determineCommonLineLeft(laneletUpwards, area2));
  EXPECT_FALSE(!!geometry::determineCommonLineLeft(laneletUpwards, area4));
}

TEST_F(TestArea, determineCommonLineRight) {
  auto res = geometry::determineCommonLineRight(laneletUpwards.invert(), area3);
  ASSERT_TRUE(!!res);
  EXPECT_EQ(res->id(), 11);
  EXPECT_FALSE(res->inverted());
  EXPECT_FALSE(!!geometry::determineCommonLineRight(laneletUpwards.invert(), area2));
  EXPECT_FALSE(!!geometry::determineCommonLineRight(laneletUpwards.invert(), area4));
}

TEST_F(TestArea, determineCommonLineSidewaysLanelet) {
  auto res = geometry::determineCommonLineSideways(laneletUpwards, area3);
  ASSERT_TRUE(!!res);
  EXPECT_EQ(res->id(), 11);
  EXPECT_TRUE(res->inverted());
  res = geometry::determineCommonLineSideways(laneletUpwards.invert(), area3);
  ASSERT_TRUE(!!res);
  EXPECT_EQ(res->id(), 11);
  EXPECT_FALSE(res->inverted());
  EXPECT_FALSE(!!geometry::determineCommonLineSideways(laneletUpwards, area2));
  EXPECT_FALSE(!!geometry::determineCommonLineSideways(laneletUpwards, area4));
}

TEST_F(TestArea, determineCommonLineSidewaysArea) {
  auto res = geometry::determineCommonLineSideways(area3, laneletUpwards);
  ASSERT_TRUE(!!res);
  EXPECT_EQ(res->id(), 11);
  EXPECT_FALSE(res->inverted());
  res = geometry::determineCommonLineSideways(area3, laneletUpwards.invert());
  ASSERT_TRUE(!!res);
  EXPECT_EQ(res->id(), 11);
  EXPECT_FALSE(res->inverted());
  EXPECT_FALSE(!!geometry::determineCommonLineSideways(area2, laneletUpwards));
  EXPECT_FALSE(!!geometry::determineCommonLineSideways(area4, laneletUpwards));
}

TEST_F(TestArea, determineCommonLineInArea) {
  auto res = geometry::determineCommonLine(area2, laneletBelow);
  ASSERT_TRUE(!!res);
  EXPECT_EQ(res->id(), 15);
  EXPECT_FALSE(res->inverted());
  res = geometry::determineCommonLine(area2, laneletFollowing);
  ASSERT_TRUE(!!res);
  EXPECT_EQ(res->id(), 14);
  EXPECT_FALSE(res->inverted());
}

TEST_F(TestArea, determineCommonLineInAreaInverted) {
  auto res = geometry::determineCommonLine(area2, laneletBelow.invert());
  ASSERT_TRUE(!!res);
  EXPECT_EQ(res->id(), 15);
  EXPECT_FALSE(res->inverted());
  res = geometry::determineCommonLine(area2, laneletFollowing.invert());
  ASSERT_TRUE(!!res);
  EXPECT_EQ(res->id(), 14);
  EXPECT_FALSE(res->inverted());
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
