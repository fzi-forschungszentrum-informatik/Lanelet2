#include <gtest/gtest.h>
#include <boost/geometry/algorithms/perimeter.hpp>
#include "geometry/Polygon.h"
#include "primitives/CompoundPolygon.h"
#include "primitives/Polygon.h"

using namespace lanelet;
template <typename T>
T toPolygon(const Polygon3d& p) {
  return T(p);
}
template <>
Polygon2d toPolygon<Polygon2d>(const Polygon3d& p) {
  return utils::to2D(p);
}
template <>
ConstPolygon2d toPolygon<ConstPolygon2d>(const Polygon3d& p) {
  return utils::to2D(p);
}
template <>
BasicPolygon2d toPolygon<BasicPolygon2d>(const Polygon3d& p) {
  return utils::to2D(p).basicPolygon();
}
template <>
BasicPolygon3d toPolygon<BasicPolygon3d>(const Polygon3d& p) {
  return p.basicPolygon();
}

class PolygonPoints : public ::testing::Test {
 public:
  PolygonPoints() {
    Id id{1};
    p11 = Point3d(++id, 0., 0., 0.);
    p12 = Point3d(++id, 1., 2., 1.);
    p13 = Point3d(++id, 1., 0., 0.);
    p21 = Point3d(++id, 2., 0., 0.);
    p22 = Point3d(++id, 3., 1., 0.);
    p23 = Point3d(++id, 3., 0., 0.);
    p31 = Point3d(++id, 1., 0., 2.);
    p32 = Point3d(++id, 1., 1., 2.);
    p33 = Point3d(++id, 2., 1., 2.);
    p34 = Point3d(++id, 2., 0., 2.);
  }

  Point3d p11, p12, p13;
  Point3d p21, p22, p23;
  Point3d p31, p32, p33, p34;
};

template <typename T>
class PolygonTypeTest : public PolygonPoints {
 public:
  using PolygonT = T;
  PolygonTypeTest() {
    Id id{10};
    AttributeMap poly1Attrs{{AttributeNamesString::Type, AttributeValueString::Curbstone}};
    poly1 = toPolygon<T>(Polygon3d{++id, {p11, p12, p13}, poly1Attrs});
    poly2 = toPolygon<T>(Polygon3d{++id, {p21, p22, p23}});
    poly3 = toPolygon<T>(Polygon3d{++id, {p31, p32, p33, p34}});
  }

  PolygonT poly1, poly2, poly3;
};

template <typename PolygonT>
PolygonT composePolygon(std::initializer_list<ConstLineString3d> list) {
  return PolygonT(list);
}

template <typename T>
class CompoundPolygonTypeTest : public PolygonPoints {
 protected:
  using PolygonT = T;
  CompoundPolygonTypeTest() {
    Id id{10};
    ConstLineString3d tempLs11{++id, {p11, p12, p13}};
    ConstLineString3d tempLs21{++id, {p21, p22}};
    ConstLineString3d tempLs22{++id, {p22, p23}};
    ConstLineString3d tempLs31{++id, {p31, p32}};
    ConstLineString3d tempLs32{++id, {p33, p34}};
    poly1 = PolygonT({tempLs11});
    poly2 = PolygonT({tempLs21, tempLs22});
    poly3 = PolygonT({tempLs31, tempLs32});
  }

 public:
  PolygonT poly1, poly2, poly3;
};

template <typename T>
auto getZ(const T& p) -> std::enable_if_t<!traits::is2D<T>(), double> {
  return p.z();
}
template <typename T>
auto getZ(const T & /*p*/) -> std::enable_if_t<traits::is2D<T>(), double> {
  return 0.;
}

template <>
class PolygonTypeTest<CompoundPolygon2d> : public CompoundPolygonTypeTest<CompoundPolygon2d> {};

template <>
class PolygonTypeTest<CompoundPolygon3d> : public CompoundPolygonTypeTest<CompoundPolygon3d> {};

template <>
class PolygonTypeTest<CompoundHybridPolygon2d> : public CompoundPolygonTypeTest<CompoundHybridPolygon2d> {};

template <>
class PolygonTypeTest<CompoundHybridPolygon3d> : public CompoundPolygonTypeTest<CompoundHybridPolygon3d> {};

template <typename T>
class AllPolygonsTest : public PolygonTypeTest<T> {};

template <typename T>
class NormalPolygonsTest : public PolygonTypeTest<T> {};

template <typename T>
class MutablePolygonsTest : public PolygonTypeTest<T> {};

template <typename T>
class PrimitivePolygonsTest : public PolygonTypeTest<T> {};

template <typename T>
class NonHybridPolygonsTest : public PolygonTypeTest<T> {};

template <typename T>
class HybridPolygonsTest : public PolygonTypeTest<T> {};

template <typename T>
class HybridPolygonsTwoDTest : public PolygonTypeTest<T> {};

template <typename T>
class ThreeDPolygonsTest : public PolygonTypeTest<T> {};

template <typename T>
class TwoDPolygonsTest : public PolygonTypeTest<T> {};

template <typename T>
class ThreeDAndBasicPolygonsTest : public PolygonTypeTest<T> {};

template <typename T>
class TwoDAndBasicPolygonsTest : public PolygonTypeTest<T> {};

using AllPolygons =
    testing::Types<Polygon2d, Polygon3d, ConstPolygon2d, ConstPolygon3d, ConstHybridPolygon2d, ConstHybridPolygon3d,
                   CompoundPolygon2d, CompoundPolygon3d, CompoundHybridPolygon2d, CompoundHybridPolygon3d>;
using NormalPolygons = testing::Types<Polygon2d, Polygon3d, ConstPolygon2d, ConstPolygon3d>;
using ThreeDPolygons =
    testing::Types<Polygon3d, ConstPolygon3d, ConstHybridPolygon3d, CompoundPolygon3d, CompoundHybridPolygon3d>;
using TwoDPolygons =
    testing::Types<Polygon2d, ConstPolygon2d, ConstHybridPolygon2d, CompoundPolygon2d, CompoundHybridPolygon2d>;
using ThreeDAndBasicPolygons = testing::Types<BasicPolygon3d, Polygon3d, ConstPolygon3d, ConstHybridPolygon3d,
                                              CompoundPolygon3d, CompoundHybridPolygon3d>;
using TwoDAndBasicPolygons = testing::Types<BasicPolygon2d, Polygon2d, ConstPolygon2d, ConstHybridPolygon2d,
                                            CompoundPolygon2d, CompoundHybridPolygon2d>;
using MutablePolygons = testing::Types<Polygon2d, Polygon3d>;
using PrimitivePolygons =
    testing::Types<Polygon2d, Polygon3d, ConstPolygon2d, ConstPolygon3d, ConstHybridPolygon2d, ConstHybridPolygon3d>;
using NonHybridPolygons =
    testing::Types<Polygon2d, Polygon3d, ConstPolygon2d, ConstPolygon3d, CompoundPolygon2d, CompoundPolygon3d>;
using HybridPolygons =
    testing::Types<ConstHybridPolygon2d, ConstHybridPolygon3d, CompoundHybridPolygon2d, CompoundHybridPolygon3d>;
using HybridPolygonsTwoD = testing::Types<ConstHybridPolygon2d, CompoundHybridPolygon2d>;

TYPED_TEST_CASE(AllPolygonsTest, AllPolygons);
TYPED_TEST_CASE(TwoDPolygonsTest, TwoDPolygons);
TYPED_TEST_CASE(ThreeDPolygonsTest, ThreeDPolygons);
TYPED_TEST_CASE(TwoDAndBasicPolygonsTest, TwoDAndBasicPolygons);
TYPED_TEST_CASE(ThreeDAndBasicPolygonsTest, ThreeDAndBasicPolygons);
TYPED_TEST_CASE(NormalPolygonsTest, NormalPolygons);
TYPED_TEST_CASE(MutablePolygonsTest, MutablePolygons);
TYPED_TEST_CASE(PrimitivePolygonsTest, PrimitivePolygons);
TYPED_TEST_CASE(NonHybridPolygonsTest, NonHybridPolygons);
TYPED_TEST_CASE(HybridPolygonsTest, HybridPolygons);
TYPED_TEST_CASE(HybridPolygonsTwoDTest, HybridPolygonsTwoD);

TYPED_TEST(MutablePolygonsTest, id) {  // NOLINT
  this->poly1.setId(100);
  EXPECT_EQ(100, this->poly1.id());
}

TYPED_TEST(MutablePolygonsTest, readAttributes) {  // NOLINT
  EXPECT_TRUE(this->poly1.hasAttribute(AttributeName::Type));
  EXPECT_TRUE(this->poly1.hasAttribute(AttributeNamesString::Type));
  EXPECT_EQ(this->poly1.attribute(AttributeName::Type), AttributeValueString::Curbstone);
  EXPECT_EQ(this->poly1.attribute(AttributeNamesString::Type), AttributeValueString::Curbstone);
}

TYPED_TEST(TwoDAndBasicPolygonsTest, bounds2d) {  // NOLINT
  BoundingBox2d bbox = geometry::boundingBox2d(this->poly2);
  EXPECT_EQ(bbox.min().x(), 2);
  EXPECT_EQ(bbox.min().y(), 0);
  EXPECT_EQ(bbox.max().x(), 3);
  EXPECT_EQ(bbox.max().y(), 1);
}

TYPED_TEST(ThreeDAndBasicPolygonsTest, bounds3d) {  // NOLINT
  BoundingBox3d bbox = geometry::boundingBox3d(this->poly1);
  EXPECT_EQ(bbox.min().x(), 0);
  EXPECT_EQ(bbox.min().y(), 0);
  EXPECT_EQ(bbox.min().z(), 0);
  EXPECT_EQ(bbox.max().x(), 1);
  EXPECT_EQ(bbox.max().y(), 2);
  EXPECT_EQ(bbox.max().z(), 1);
}

TYPED_TEST(HybridPolygonsTwoDTest, distance2d) {  // NOLINT
  EXPECT_DOUBLE_EQ(geometry::distance(this->poly1, this->poly3), 0.);
  EXPECT_DOUBLE_EQ(geometry::distance(this->poly1, this->poly2), 1.);
}

TYPED_TEST(HybridPolygonsTwoDTest, distancePoint2d) {  // NOLINT
  EXPECT_DOUBLE_EQ(geometry::distance(this->poly1, BasicPoint2d(2, 1)), 1.);
}

TYPED_TEST(HybridPolygonsTwoDTest, area) {  // NOLINT
  auto a = geometry::area(this->poly1);
  EXPECT_DOUBLE_EQ(a, 1.);
}

TYPED_TEST(HybridPolygonsTwoDTest, centroid) {  // NOLINT
  BasicPoint2d p{0, 0};
  boost::geometry::centroid(this->poly3, p);
  EXPECT_DOUBLE_EQ(p.x(), 1.5);
  EXPECT_DOUBLE_EQ(p.y(), 0.5);
}

TYPED_TEST(AllPolygonsTest, perimeter) {  // NOLINT
  auto p = boost::geometry::perimeter(this->poly3);
  EXPECT_DOUBLE_EQ(p, 4);
}

TYPED_TEST(TwoDPolygonsTest, toBasicPolygon) {  // NOLINT
  auto pBasic = this->poly3.basicPolygon();
  EXPECT_EQ(4, boost::geometry::perimeter(pBasic));
  EXPECT_EQ(4ul, pBasic.size());
}
