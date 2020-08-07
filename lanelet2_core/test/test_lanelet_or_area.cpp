#include <gtest/gtest.h>

#include "lanelet2_core/primitives/LaneletOrArea.h"

using namespace lanelet;

Point3d makePoint(double x, double y) {
  static Id id{1};
  return Point3d(id++, x, y);
}

class TestLaneletOrArea : public ::testing::Test {
 public:
  ConstLaneletOrArea getLanelet() const { return lanelet; }
  ConstLaneletOrArea getArea() const { return area; }
  Point3d p1{makePoint(0, 0)}, p2{makePoint(1, 0)}, p3{makePoint(0, 1)}, p4{makePoint(1, 1)};
  LineString3d ls1{LineString3d(10, {p1, p2})}, ls2{LineString3d(11, {p3, p4})}, ls3{LineString3d(12, {p1, p3})},
      ls4{LineString3d(13, {p4, p2})};
  Area area{Area(14, {ls2, ls4, ls1.invert(), ls3})};
  Lanelet lanelet{Lanelet(15, ls2, ls1)};
};

TEST_F(TestLaneletOrArea, getLanelet) {  // NOLINT
  auto ll = getLanelet();
  lanelet.setAttribute("test", true);
  EXPECT_TRUE(ll.isLanelet());
  EXPECT_FALSE(ll.isArea());
  EXPECT_EQ(ll.id(), lanelet.id());
  EXPECT_EQ(ll.attributes(), lanelet.attributes());
  EXPECT_EQ(ll, lanelet);
  EXPECT_NE(ll, area);
}

TEST_F(TestLaneletOrArea, getArea) {  // NOLINT
  auto ar = getArea();
  area.setAttribute("test", true);
  EXPECT_FALSE(ar.isLanelet());
  EXPECT_TRUE(ar.isArea());
  EXPECT_EQ(ar.id(), area.id());
  EXPECT_EQ(ar.attributes(), area.attributes());
  EXPECT_EQ(ar, area);
  EXPECT_NE(ar, lanelet);
}

TEST_F(TestLaneletOrArea, boundingPolygon) {  // NOLINT
  auto poly = getArea().boundingPolygon();
  ASSERT_EQ(poly.size(), area.outerBoundPolygon().size());
  for (size_t i = 0; i < poly.size(); ++i) {
    EXPECT_EQ(poly[i], area.outerBoundPolygon()[i]);
  }
  poly = getLanelet().boundingPolygon();
  ASSERT_EQ(poly.size(), lanelet.polygon3d().size());
  for (size_t i = 0; i < poly.size(); ++i) {
    EXPECT_EQ(poly[i], lanelet.polygon3d()[i]);
  }
}
