#include <gtest/gtest.h>

#include "LaneletPath.h"
#include "lanelet2_core/geometry/Polygon.h"

using namespace lanelet;
using namespace lanelet::routing;

class LaneletOrAreaTest : public ::testing::Test {
 private:
  void SetUp() override {
    Id id(1000);
    p1 = Point3d(++id, 0, 0, 0);
    p2 = Point3d(++id, 1, 0, 0);
    p3 = Point3d(++id, 2, 0, 0);
    p4 = Point3d(++id, 3, 0, 0);
    p5 = Point3d(++id, 4, 0, 0);
    p6 = Point3d(++id, 0, 1, 0);
    p7 = Point3d(++id, 1, 1, 0);
    p8 = Point3d(++id, 2, 1, 0);
    p9 = Point3d(++id, 3, 1, 0);
    p10 = Point3d(++id, 4, 1, 0);

    ls1 = LineString3d(++id, {p1, p2});
    ls2 = LineString3d(++id, {p2, p3});
    ls3 = LineString3d(++id, {p3, p4});
    ls4 = LineString3d(++id, {p4, p5});
    ls5 = LineString3d(++id, {p10, p9});
    ls6 = LineString3d(++id, {p9, p8});
    ls7 = LineString3d(++id, {p8, p7});
    ls8 = LineString3d(++id, {p7, p6});
    ls9 = LineString3d(++id, {p1, p6});
    ls10 = LineString3d(++id, {p2, p7});
    ls11 = LineString3d(++id, {p3, p8});

    l1 = Lanelet(++id, ls6.invert(), ls3);
    l2 = Lanelet(++id, ls5.invert(), ls4);

    a1 = Area(++id, {ls9, ls8.invert(), ls10.invert(), ls1.invert()});
    a2 = Area(++id, {ls10, ls7.invert(), ls11.invert(), ls2.invert()});

    areaPath = LaneletOrAreaPath({ConstLaneletOrArea(a1), ConstLaneletOrArea(a2)});
    laneletPath = LaneletOrAreaPath({ConstLaneletOrArea(l1), ConstLaneletOrArea(l2)});
    bothPath = LaneletOrAreaPath({ConstLaneletOrArea(a2), ConstLaneletOrArea(l1)});
    invAreaPath = LaneletOrAreaPath({ConstLaneletOrArea(l1.invert()), ConstLaneletOrArea(a2)});
    invLLPath = LaneletOrAreaPath({ConstLaneletOrArea(l2.invert()), ConstLaneletOrArea(l1.invert())});
    longPath = LaneletOrAreaPath(
        {ConstLaneletOrArea(a1), ConstLaneletOrArea(a2), ConstLaneletOrArea(l1), ConstLaneletOrArea(l2)});
    longInvPath = LaneletOrAreaPath({ConstLaneletOrArea(l2.invert()), ConstLaneletOrArea(l1.invert()),
                                     ConstLaneletOrArea(a2), ConstLaneletOrArea(a1)});
  }

 public:
  Point3d p1, p2, p3, p4, p5, p6, p7, p8, p9, p10;
  LineString3d ls1, ls2, ls3, ls4, ls5, ls6, ls7, ls8, ls9, ls10, ls11;
  Lanelet l1, l2;
  Area a1, a2;
  LaneletOrAreaPath areaPath, laneletPath, bothPath, invAreaPath, invLLPath, longPath, longInvPath;
};
namespace {
void checkIdentical(const BasicPolygon3d& lhs, const BasicPolygon3d& rhs) {
  auto to2D = [](const BasicPolygon3d& p3d) {
    BasicPolygon2d p2d(p3d.size());
    for (size_t i = 0; i < p3d.size(); ++i) {
      p2d[i] = p3d.at(i).head<2>();
    }
    return p2d;
  };
  BasicPolygon2d lhs2d = to2D(lhs);
  BasicPolygon2d rhs2d = to2D(rhs);
  EXPECT_TRUE(boost::geometry::equals(lhs2d, rhs2d));
}
}  // namespace

TEST_F(LaneletOrAreaTest, enclosingPolygonAreas) {  // NOLINT
  BasicPolygons2d intersect;
  BasicPolygon3d joined = getEnclosingPolygon3d(areaPath);
  BasicPolygon3d expected{p1, p6, p8, p3};
  checkIdentical(joined, expected);
  EXPECT_EQ(joined.size(), 6ul);
}

TEST_F(LaneletOrAreaTest, enclosingPolygonLanelets) {  // NOLINT
  BasicPolygons2d intersect;
  BasicPolygon3d joined = getEnclosingPolygon3d(laneletPath);
  BasicPolygon3d expected{p3, p8, p10, p5};
  checkIdentical(joined, expected);
  EXPECT_EQ(joined.size(), 6ul);
}

TEST_F(LaneletOrAreaTest, enclosingPolygonLaneletsInverted) {  // NOLINT
  BasicPolygons2d intersect;
  BasicPolygon3d joined = getEnclosingPolygon3d(invLLPath);
  BasicPolygon3d expected{p3, p8, p10, p5};
  checkIdentical(joined, expected);
  EXPECT_EQ(joined.size(), 6ul);
}

TEST_F(LaneletOrAreaTest, enclosingPolygonMixed) {  // NOLINT
  BasicPolygons2d intersect;
  BasicPolygon3d joined = getEnclosingPolygon3d(bothPath);
  BasicPolygon3d expected{p2, p7, p9, p4};
  checkIdentical(joined, expected);
  EXPECT_EQ(joined.size(), 6ul);
}

TEST_F(LaneletOrAreaTest, enclosingPolygonAreaInverted) {  // NOLINT
  BasicPolygons2d intersect;
  BasicPolygon3d joined = getEnclosingPolygon3d(invAreaPath);
  BasicPolygon3d expected{p2, p7, p9, p4};
  checkIdentical(joined, expected);
  EXPECT_EQ(joined.size(), 6ul);
}

TEST_F(LaneletOrAreaTest, enclosingPolygonLong) {  // NOLINT
  BasicPolygons2d intersect;
  BasicPolygon3d joined = getEnclosingPolygon3d(longPath);
  BasicPolygon3d expected{p1, p6, p10, p5};
  checkIdentical(joined, expected);
  EXPECT_EQ(joined.size(), 10ul);
}

TEST_F(LaneletOrAreaTest, enclosingPolygonLongInverted) {  // NOLINT
  BasicPolygons2d intersect;
  BasicPolygon3d joined = getEnclosingPolygon3d(longInvPath);
  BasicPolygon3d expected{p1, p6, p10, p5};
  checkIdentical(joined, expected);
  EXPECT_EQ(joined.size(), 10ul);
}