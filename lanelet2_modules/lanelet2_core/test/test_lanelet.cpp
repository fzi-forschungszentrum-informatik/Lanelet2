#include <gtest/gtest.h>
#include <iostream>
#include "geometry/Area.h"
#include "geometry/Lanelet.h"
#include "geometry/LineString.h"
#include "geometry/Polygon.h"
#include "primitives/Lanelet.h"

using namespace std::literals;
using namespace lanelet;

Lanelet bufferLanelet(Lanelet llt, double z) {
  auto bufferPoints = [&llt, z](const auto& elem) {
    return Point3d(llt.id() + elem.id(), elem.x(), elem.y(), elem.z() + z);
  };
  LineString3d left(llt.id() + llt.leftBound().id(), utils::transform(llt.leftBound(), bufferPoints));
  LineString3d right(llt.id() + llt.rightBound().id(), utils::transform(llt.rightBound(), bufferPoints));
  return Lanelet(llt.id(), left, right);
};

void testCenterline(const ConstLineString3d& centerline, const ConstLineString3d& leftBound,
                    const ConstLineString3d& rightBound) {
  EXPECT_GE(centerline.size(), 2ul);
  EXPECT_FALSE(geometry::intersects(ConstHybridLineString2d(centerline), ConstHybridLineString2d(leftBound)));
  EXPECT_FALSE(geometry::intersects(ConstHybridLineString2d(centerline), ConstHybridLineString2d(rightBound)));
}

class LaneletTest : public ::testing::Test {
 protected:
  void SetUp() override {
    id = 0;
    p1 = Point3d(++id, 0., 1., 1.);
    p2 = Point3d(++id, 1., 1., 1.);
    p3 = Point3d(++id, 0., 0., 0.);
    p4 = Point3d(++id, 1., 0., 0.);
    p5 = Point3d(++id, 0., 0.5, 0.5);
    p6 = Point3d(++id, 0.5, 0.5, 0.5);
    p7 = Point3d(++id, 1., 0.5, 0.);
    p8 = Point3d(++id, 0., -1., 0.);
    p9 = Point3d(++id, 1., -1., 0.);
    left = LineString3d(++id, Points3d{p1, p2});
    right = LineString3d(++id, Points3d{p3, p4});
    other = LineString3d(++id, Points3d{p5, p6, p7});
    outside = LineString3d(++id, Points3d{p8, p9});

    ritterLanelet = Lanelet(++id, left, right);
    constRitterLanelet = ritterLanelet;
  }

 public:
  Id id{1};
  Point3d p1, p2, p3, p4, p5, p6, p7, p8, p9;
  LineString3d left, right, other, outside;
  Lanelet ritterLanelet;  //!< quadratisch, praktisch, gut... [1x1]
  ConstLanelet constRitterLanelet;
};

TEST_F(LaneletTest, id) {  // NOLINT
  ritterLanelet.setId(100);
  EXPECT_EQ(100, ritterLanelet.id());
  EXPECT_EQ(100, constRitterLanelet.id());
}

TEST_F(LaneletTest, nullptrConstruct) {                                      // NOLINT
  EXPECT_THROW(Lanelet(std::shared_ptr<LaneletData>()), NullptrError);       // NOLINT
  EXPECT_THROW(ConstLanelet(std::shared_ptr<LaneletData>()), NullptrError);  // NOLINT
}

TEST_F(LaneletTest, bounds) {  // NOLINT
  EXPECT_EQ(left, ritterLanelet.leftBound());
  EXPECT_EQ(right, ritterLanelet.rightBound());
  EXPECT_EQ(left, constRitterLanelet.leftBound());
  EXPECT_EQ(right, constRitterLanelet.rightBound());
  ritterLanelet.setLeftBound(other);
  EXPECT_EQ(other, ritterLanelet.leftBound());
  EXPECT_EQ(other, constRitterLanelet.leftBound());
}

TEST_F(LaneletTest, attributes) {  // NOLINT
  ritterLanelet.setAttribute("test", "value");
  ritterLanelet.setAttribute(AttributeName::Subtype, AttributeValueString::Road);
  EXPECT_TRUE(ritterLanelet.hasAttribute("test"));
  EXPECT_EQ("value"s, ritterLanelet.attribute("test").value());
  EXPECT_EQ("value"s, constRitterLanelet.attribute("test").value());
  EXPECT_EQ(ritterLanelet.attribute(AttributeName::Subtype), AttributeValueString::Road);
  EXPECT_EQ(constRitterLanelet.attribute(AttributeName::Subtype), AttributeValueString::Road);
  EXPECT_THROW(ritterLanelet.attribute("doesnotexist"), NoSuchAttributeError);  // NOLINT
}

TEST_F(LaneletTest, invert) {  // NOLINT
  auto invertedLanelet = ritterLanelet.invert();
  EXPECT_TRUE(invertedLanelet.inverted());
  EXPECT_NE(ritterLanelet, invertedLanelet);
  EXPECT_EQ(ritterLanelet, invertedLanelet.invert());
  EXPECT_EQ(right.id(), invertedLanelet.leftBound().id());
  EXPECT_EQ(left.id(), invertedLanelet.rightBound().id());

  auto constInvertedLanelet = constRitterLanelet.invert();
  EXPECT_EQ(right.id(), constInvertedLanelet.leftBound().id());
  EXPECT_EQ(left.id(), constInvertedLanelet.rightBound().id());

  auto constInvertedInvertedLanelet = constInvertedLanelet.invert();
  EXPECT_EQ(constRitterLanelet, constInvertedInvertedLanelet);
  EXPECT_EQ(right, constInvertedInvertedLanelet.rightBound());
  EXPECT_EQ(left, constInvertedInvertedLanelet.leftBound());
}

TEST_F(LaneletTest, modifyInvert) {  // NOLINT
  auto invertedLanelet = ritterLanelet.invert();
  invertedLanelet.setLeftBound(right);
  invertedLanelet.setRightBound(left);

  EXPECT_EQ(invertedLanelet.leftBound().id(), right.id());
  EXPECT_EQ(invertedLanelet.leftBound().front(), right.front());
  EXPECT_EQ(ritterLanelet.leftBound().front(), left.back());
  EXPECT_EQ(ritterLanelet.rightBound().front(), right.back());
}

TEST_F(LaneletTest, centerline) {  // NOLINT
  auto centerline = ritterLanelet.centerline();
  testCenterline(centerline, ritterLanelet.leftBound(), ritterLanelet.rightBound());
  EXPECT_DOUBLE_EQ(double(geometry::length(centerline)), 1);

  auto invCenterline = ritterLanelet.invert().centerline();
  EXPECT_EQ(invCenterline.front().x(), centerline.back().x());
  EXPECT_EQ(invCenterline.front().y(), centerline.back().y());
}

TEST_F(LaneletTest, setCenterline) {  // NOLINT
  ritterLanelet.setCenterline(other);
  EXPECT_EQ(other.id(), ritterLanelet.centerline().id());
  ritterLanelet.setRightBound(outside);
  EXPECT_TRUE(ritterLanelet.hasCustomCenterline());
  EXPECT_EQ(other.id(), ritterLanelet.centerline().id());
}

TEST_F(LaneletTest, area) {  // NOLINT
  EXPECT_DOUBLE_EQ(1, geometry::area(ritterLanelet.polygon2d()));
  EXPECT_DOUBLE_EQ(1, geometry::area(constRitterLanelet.polygon2d()));
}

TEST_F(LaneletTest, inside) {  // NOLINT
  using traits::to2D;
  EXPECT_TRUE(geometry::inside(ritterLanelet, to2D(p5)));   // point on edge
  EXPECT_TRUE(geometry::inside(ritterLanelet, to2D(p6)));   // point in middle
  EXPECT_FALSE(geometry::inside(ritterLanelet, to2D(p8)));  // point outside

  EXPECT_TRUE(geometry::inside(ritterLanelet, BasicPoint2d(0.5, 0.5)));
  EXPECT_FALSE(geometry::inside(ritterLanelet, BasicPoint2d(-1, -1)));
}

TEST_F(LaneletTest, boundingbox) {  // NOLINT
  auto box = geometry::boundingBox2d(ritterLanelet);
  EXPECT_DOUBLE_EQ(0, box.corner(BoundingBox2d::BottomLeft).x());
  EXPECT_DOUBLE_EQ(0, box.corner(BoundingBox2d::BottomLeft).y());
  EXPECT_DOUBLE_EQ(1, box.corner(BoundingBox2d::TopRight).x());
  EXPECT_DOUBLE_EQ(1, box.corner(BoundingBox2d::TopRight).y());
}

TEST_F(LaneletTest, intersects) {  // NOLINT
  EXPECT_TRUE(geometry::intersects2d(ritterLanelet, constRitterLanelet));
  auto lanelet1 = Lanelet(++id, left, other);
  auto lanelet2 = Lanelet(++id, right, outside);
  EXPECT_FALSE(geometry::intersects2d(lanelet1, lanelet2));
  EXPECT_TRUE(geometry::intersects2d(constRitterLanelet, lanelet2));
  EXPECT_TRUE(geometry::intersects2d(constRitterLanelet, lanelet1));
}

TEST_F(LaneletTest, overlaps) {  // NOLINT
  EXPECT_TRUE(geometry::overlaps2d(ritterLanelet, constRitterLanelet));
  auto lanelet1 = Lanelet(++id, left, other);
  auto lanelet2 = Lanelet(++id, right, outside);
  EXPECT_FALSE(geometry::overlaps2d(lanelet1, lanelet2));
  EXPECT_FALSE(geometry::overlaps2d(constRitterLanelet, lanelet2));
  EXPECT_TRUE(geometry::overlaps2d(constRitterLanelet, lanelet1));
}

TEST_F(LaneletTest, length) {  // NOLINT
  EXPECT_FLOAT_EQ(geometry::length2d(ritterLanelet), 1);
}

TEST_F(LaneletTest, approxLength) {  // NOLINT
  auto l = geometry::approximatedLength2d(ritterLanelet);
  EXPECT_LT(0.5, l);
  EXPECT_GE(1, l);
}

TEST_F(LaneletTest, intersects3d) {  // NOLINT
  using geometry::intersects3d;
  auto lanelet1 = Lanelet(++id, left, other);
  auto lanelet2 = Lanelet(++id, right, outside);
  EXPECT_TRUE(intersects3d(ritterLanelet, constRitterLanelet));
  EXPECT_TRUE(intersects3d(ritterLanelet, bufferLanelet(ritterLanelet, 0), 1.));
  EXPECT_FALSE(intersects3d(ritterLanelet, bufferLanelet(ritterLanelet, 2), 1.));
  EXPECT_FALSE(intersects3d(ritterLanelet, bufferLanelet(ritterLanelet, -2), 1.));
  EXPECT_TRUE(intersects3d(bufferLanelet(ritterLanelet, -100), bufferLanelet(ritterLanelet, -101), 3.));
  EXPECT_FALSE(intersects3d(lanelet1, lanelet2));
  EXPECT_TRUE(intersects3d(ritterLanelet, lanelet2, 3));
}

TEST_F(LaneletTest, overlaps3d) {  // NOLINT
  using geometry::overlaps3d;
  auto lanelet1 = Lanelet(++id, left, other);
  auto lanelet2 = Lanelet(++id, right, outside);
  EXPECT_TRUE(overlaps3d(ritterLanelet, constRitterLanelet));
  EXPECT_TRUE(overlaps3d(ritterLanelet, bufferLanelet(ritterLanelet, 0), 1.));
  EXPECT_FALSE(overlaps3d(this->ritterLanelet, bufferLanelet(ritterLanelet, 2), 1.));
  EXPECT_FALSE(overlaps3d(lanelet1, lanelet2));
  EXPECT_FALSE(overlaps3d(ritterLanelet, lanelet2, 3));
}

TEST_F(LaneletTest, distance) {  // NOLINT
  using traits::to2D;
  EXPECT_DOUBLE_EQ(0., geometry::distance2d(constRitterLanelet, to2D(p1)));
  EXPECT_DOUBLE_EQ(0., geometry::distance3d(constRitterLanelet, p5));
  EXPECT_DOUBLE_EQ(0., geometry::distance2d(constRitterLanelet, to2D(p6)));
  EXPECT_DOUBLE_EQ(1., geometry::distance2d(constRitterLanelet, to2D(p8)));
  EXPECT_DOUBLE_EQ(0., geometry::distance2d(constRitterLanelet, Point2d(InvalId, 0, 0)));
  EXPECT_DOUBLE_EQ(1., geometry::distance2d(constRitterLanelet, Point2d(InvalId, -1, 0)));
  EXPECT_DOUBLE_EQ(std::sqrt(.5), geometry::distanceToCenterline3d(constRitterLanelet, p1));
  EXPECT_DOUBLE_EQ(0.5, geometry::distanceToCenterline2d(constRitterLanelet, Point2d(InvalId, 0, 0)));
}

TEST_F(LaneletTest, comparison) {  // NOLINT
  EXPECT_EQ(constRitterLanelet, ritterLanelet);

  auto invLanelet = ritterLanelet.invert();
  EXPECT_NE(ritterLanelet, invLanelet);
  EXPECT_NE(ritterLanelet, Lanelet());
  EXPECT_NE(Lanelet(), Lanelet());
}

TEST_F(LaneletTest, weakLanelet) {  // NOLINT
  WeakLanelet wll = ritterLanelet;
  EXPECT_EQ(wll, ritterLanelet);
  EXPECT_FALSE(wll.expired());
  EXPECT_EQ(wll.lock(), ritterLanelet);
}

TEST(LaneletBasic, emptyLanelet) {  // NOLINT
  Lanelet empty;
  EXPECT_EQ(empty.polygon2d().size(), 0ul);
  EXPECT_EQ(empty.centerline().size(), 0ul);
}

Lanelet buildComplexTestCase() {
  /*
   * Shape looks roughly like this:
   *  |  |\    /| |
   *  |  | \  / | |
   *  |  |  \/  | |
   *  |___________|
   *
   */
  Point3d p11, p12, p13, p14, p15, p21, p22, p23, p24;
  LineString3d left, right;
  Lanelet lanelet;
  Id id{1};
  p11 = Point3d(++id, 1., 5.);
  p12 = Point3d(++id, 2., 8.);
  p13 = Point3d(++id, 3., 2.);
  p14 = Point3d(++id, 4., 10.);
  p15 = Point3d(++id, 5., 4.);
  p21 = Point3d(++id, 0., 10.);
  p22 = Point3d(++id, 0., 0.);
  p23 = Point3d(++id, 6., 0.);
  p24 = Point3d(++id, 6., 10.);
  left = LineString3d(++id, Points3d{p11, p12, p13, p14, p15});
  right = LineString3d(++id, Points3d{p21, p22, p23, p24});
  lanelet = Lanelet(++id, left, right);
  return lanelet;
}

Lanelet buildLinearTestCase(size_t numPoints) {
  Id id{1};
  LineString3d left(++id), right(++id);
  for (auto i = 0u; i < numPoints; i++) {
    left.push_back(Point3d(++id, i, 1, 0));
    right.push_back(Point3d(++id, i, 0, 0));
  }
  return Lanelet(++id, left, right);
}

TEST(ComplexLaneletTest, complexCenterline) {  // NOLINT
  auto lanelet = buildComplexTestCase();
  auto centerline = lanelet.centerline();
  testCenterline(centerline, lanelet.leftBound(), lanelet.rightBound());
}

TEST(ComplexLaneletTest, linearCenterline) {  // NOLINT
  auto lanelet = buildLinearTestCase(20);
  auto centerline = lanelet.centerline();
  testCenterline(centerline, lanelet.leftBound(), lanelet.rightBound());
}
