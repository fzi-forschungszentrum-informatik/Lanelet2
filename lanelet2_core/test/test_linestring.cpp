#include <gtest/gtest.h>

#include "lanelet2_core/geometry/LineString.h"
#include "lanelet2_core/primitives/LineString.h"
using namespace lanelet;

class LineStringPoints : public ::testing::Test {
 protected:
  void SetUp() override {
    Id id{1};
    p11 = Point3d(++id, 0., 0., 0.);
    p12 = Point3d(++id, 0., 1., 0.);
    p13 = Point3d(++id, 0., 2., 1.);
    p21 = Point3d(++id, 1., 0., 0.);
    p22 = Point3d(++id, 2., 0., 0.);
    p23 = Point3d(++id, -1., 2., 0.);
    p31 = Point3d(++id, -1., 0., 2.);
    p32 = Point3d(++id, 0., 1., 2.);
    p33 = Point3d(++id, 1., 2., 2.);
    p43 = Point3d(++id, 1, 1. + sqrt(3) / 3., 0.);
  }

 public:
  Point3d p11, p12, p13;
  Point3d p21, p22, p23;
  Point3d p31, p32, p33;
  Point3d p43;
};

template <typename T>
class Point2dTypeTest : public ::testing::Test {
 protected:
  using Point2dT = T;
  void SetUp() override {
    Id id{1};
    p1 = Point2d(++id, 1., 0., 0.);
    p2 = Point2d(++id, 0., 1., 0.);
    p3 = Point2d(++id, -1., 0., 0.);
    p4 = Point2d(++id, 0., 1., 0.);
  }

 public:
  Point2dT p1, p2, p3, p4;
};

template <typename T>
class LineStringTypeTest : public LineStringPoints {
 protected:
  using LineStringT = T;
  void SetUp() override {
    LineStringPoints::SetUp();
    Id id{10};
    ls1 =
        LineStringT(++id, {p11, p12, p13}, AttributeMap{{AttributeNamesString::Type, AttributeValueString::Curbstone}});
    ls2 = LineStringT(++id, {p21, p22, p23});
    ls3 = LineStringT(++id, {p31, p32, p33});
    ls4 = LineStringT(++id, {p11, p12, p43});
  }

 public:
  LineStringT ls1, ls2, ls3, ls4;
};

template <typename T>
class CompoundLineStringTypeTest : public LineStringPoints {
 protected:
  using LineStringT = T;
  void SetUp() override {
    LineStringPoints::SetUp();
    Id id{10};
    LineString3d tempLs1 = LineString3d{++id, {p11, p12}};
    LineString3d tempLs2 = LineString3d{++id, {p12, p13}};
    LineString3d tempLs3 = LineString3d{++id, {p21, p22}};
    LineString3d tempLs4 = LineString3d{++id, {p22, p23}};
    LineString3d tempLs5 = LineString3d{++id, {p12, p43}};
    ls1 = LineStringT({tempLs1, tempLs2});
    ls2 = LineStringT({tempLs3, tempLs4});
    ls3 = LineStringT({LineString3d(++id, {p31, p32, p33})});
    ls4 = LineStringT({tempLs1, tempLs5});
  }

 public:
  LineStringT ls1, ls2, ls3, ls4;
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
class LineStringTypeTest<CompoundLineString2d> : public CompoundLineStringTypeTest<CompoundLineString2d> {};

template <>
class LineStringTypeTest<CompoundLineString3d> : public CompoundLineStringTypeTest<CompoundLineString3d> {};

template <>
class LineStringTypeTest<CompoundHybridLineString2d> : public CompoundLineStringTypeTest<CompoundHybridLineString2d> {};

template <>
class LineStringTypeTest<CompoundHybridLineString3d> : public CompoundLineStringTypeTest<CompoundHybridLineString3d> {};

template <typename T>
class AllLineStringsTest : public LineStringTypeTest<T> {};

template <typename T>
class NormalLineStringsTest : public LineStringTypeTest<T> {};

template <typename T>
class MutableLineStringsTest : public LineStringTypeTest<T> {};

template <typename T>
class PrimitiveLineStringsTest : public LineStringTypeTest<T> {};

template <typename T>
class NonHybridLineStringsTest : public LineStringTypeTest<T> {};

template <typename T>
class HybridLineStringsTest : public LineStringTypeTest<T> {};

template <typename T>
class ThreeDLineStringsTest : public LineStringTypeTest<T> {};

template <typename T>
class TwoDLineStringsTest : public LineStringTypeTest<T> {};

template <typename T>
class TwoDPointsTest : public Point2dTypeTest<T> {};

template <typename T>
class BasicLineStringsTest : public LineStringTypeTest<T> {};
using TwoDPoints = testing::Types<BasicPoint2d, Point2d, ConstPoint2d>;
using AllLineStrings = testing::Types<LineString2d, LineString3d, ConstLineString2d, ConstLineString3d,
                                      ConstHybridLineString2d, ConstHybridLineString3d, CompoundLineString2d,
                                      CompoundLineString3d, CompoundHybridLineString2d, CompoundHybridLineString3d>;
using NormalLineStrings = testing::Types<LineString2d, LineString3d, ConstLineString2d, ConstLineString3d>;
using ThreeDLineStrings = testing::Types<LineString3d, ConstLineString3d, ConstHybridLineString3d, CompoundLineString3d,
                                         CompoundHybridLineString3d>;
using TwoDLineStrings = testing::Types<LineString2d, ConstLineString2d, ConstHybridLineString2d, CompoundLineString2d,
                                       CompoundHybridLineString2d>;
using MutableLineStrings = testing::Types<LineString2d, LineString3d>;
using PrimitiveLineStrings = testing::Types<LineString2d, LineString3d, ConstLineString2d, ConstLineString3d,
                                            ConstHybridLineString2d, ConstHybridLineString3d>;
using NonHybridLineStrings = testing::Types<LineString2d, LineString3d, ConstLineString2d, ConstLineString3d,
                                            CompoundLineString2d, CompoundLineString3d>;
using HybridLineStrings = testing::Types<ConstHybridLineString2d, ConstHybridLineString3d, CompoundHybridLineString2d,
                                         CompoundHybridLineString3d>;

using BasicLineStrings = testing::Types<BasicLineString2d, BasicLineString3d>;

TYPED_TEST_CASE(TwoDPointsTest, TwoDPoints);
TYPED_TEST_CASE(AllLineStringsTest, AllLineStrings);
TYPED_TEST_CASE(TwoDLineStringsTest, TwoDLineStrings);
TYPED_TEST_CASE(ThreeDLineStringsTest, ThreeDLineStrings);
TYPED_TEST_CASE(NormalLineStringsTest, NormalLineStrings);
TYPED_TEST_CASE(MutableLineStringsTest, MutableLineStrings);
TYPED_TEST_CASE(PrimitiveLineStringsTest, PrimitiveLineStrings);
TYPED_TEST_CASE(NonHybridLineStringsTest, NonHybridLineStrings);
TYPED_TEST_CASE(HybridLineStringsTest, HybridLineStrings);
TYPED_TEST_CASE(BasicLineStringsTest, BasicLineStrings);

TYPED_TEST(MutableLineStringsTest, id) {  // NOLINT
  this->ls1.setId(100);
  EXPECT_EQ(100, this->ls1.id());
}

TYPED_TEST(MutableLineStringsTest, readAttributes) {  // NOLINT
  EXPECT_TRUE(this->ls1.hasAttribute(AttributeName::Type));
  EXPECT_TRUE(this->ls1.hasAttribute(AttributeNamesString::Type));
  EXPECT_EQ(this->ls1.attribute(AttributeName::Type), AttributeValueString::Curbstone);
  EXPECT_EQ(this->ls1.attribute(AttributeNamesString::Type), AttributeValueString::Curbstone);
}

TYPED_TEST(PrimitiveLineStringsTest, constConversion) {  // NOLINT
  auto constLs = traits::toConst(this->ls1);
  EXPECT_EQ(constLs, this->ls1);
}

TYPED_TEST(AllLineStringsTest, iteration) {  // NOLINT
  auto xs = utils::transform(this->ls1, [](const auto& elem) { return elem.x(); });
  EXPECT_EQ(3ul, xs.size());
  for (auto x : xs) {
    EXPECT_EQ(0, x);
  }
}

TYPED_TEST(AllLineStringsTest, invert) {  // NOLINT
  auto invertLs = this->ls1.invert();
  auto ys = utils::transform(invertLs, [](const auto& elem) { return elem.y(); });
  ASSERT_EQ(3ul, ys.size());
  EXPECT_EQ(0, ys[2]);
  EXPECT_EQ(2, ys[0]);
}

TYPED_TEST(MutableLineStringsTest, invertAndPushBack) {  // NOLINT
  using PointT = typename TypeParam::PointType;
  auto invertLs = this->ls1.invert();
  EXPECT_NE(invertLs, this->ls1);
  EXPECT_EQ(invertLs.invert(), this->ls1);
  this->ls1[0] = PointT(this->p21);
  EXPECT_EQ(PointT(this->p21), invertLs[2]);

  invertLs.push_back(this->p22);
  EXPECT_EQ(this->p22.id(), this->ls1[0].id());
}

TYPED_TEST(MutableLineStringsTest, invertAndInsertOne) {  // NOLINT
  using PointT = typename TypeParam::PointType;
  auto invertLs = this->ls1.invert();
  auto it = invertLs.insert(invertLs.begin() + 1, PointT(this->p21));
  EXPECT_EQ(PointT(this->p21), *it);
  EXPECT_EQ(PointT(this->p21), this->ls1[2]);
}

TYPED_TEST(MutableLineStringsTest, invertAndErase) {  // NOLINT
  auto invertLs = this->ls1.invert();
  auto it = invertLs.erase(invertLs.begin() + 1);
  EXPECT_EQ(2ul, this->ls1.size());
  EXPECT_EQ(this->ls1.front(), *it);
}

TYPED_TEST(MutableLineStringsTest, invertAndInsertMultiple) {  // NOLINT
  auto invertLs = this->ls1.invert();
  auto pts = std::vector<typename TypeParam::PointType>(this->ls2.begin(), this->ls2.end());
  auto it = invertLs.insert(invertLs.end(), pts.begin(), pts.end());
  EXPECT_EQ(it, invertLs.end() - 1);
  EXPECT_LT(it, invertLs.end());
  EXPECT_GT(it, invertLs.begin());
  EXPECT_EQ(this->ls2.back(), invertLs.back());
}

TYPED_TEST(MutableLineStringsTest, invertAndResize) {  // NOLINT
  auto invertLs = this->ls1.invert();
  invertLs.resize(2);
  ASSERT_EQ(2ul, invertLs.size());
  EXPECT_EQ(invertLs.front(), this->p13);
  EXPECT_EQ(invertLs.back(), this->p12);
}

TYPED_TEST(TwoDLineStringsTest, bounds2d) {  // NOLINT
  BoundingBox2d bbox = geometry::boundingBox2d(this->ls2);
  EXPECT_EQ(bbox.min().x(), -1);
  EXPECT_EQ(bbox.min().y(), 0);
  EXPECT_EQ(bbox.max().x(), 2);
  EXPECT_EQ(bbox.max().y(), 2);
}

TYPED_TEST(ThreeDLineStringsTest, bounds3d) {  // NOLINT
  BoundingBox3d bbox = geometry::boundingBox3d(this->ls1);
  EXPECT_EQ(bbox.min().x(), 0);
  EXPECT_EQ(bbox.min().y(), 0);
  EXPECT_EQ(bbox.min().z(), 0);
  EXPECT_EQ(bbox.max().x(), 0);
  EXPECT_EQ(bbox.max().y(), 2);
  EXPECT_EQ(bbox.max().z(), 1);
}

TYPED_TEST(ThreeDLineStringsTest, distance3d) {  // NOLINT
  EXPECT_NEAR(geometry::distance3d(this->ls1, this->ls2), 0.25, 0.05);
  EXPECT_DOUBLE_EQ(geometry::distance3d(this->ls2, this->ls3), 2.);
}

TYPED_TEST(AllLineStringsTest, length) {  // NOLINT
  auto l = geometry::length(this->ls1);
  auto lSeg1 = geometry::rangedLength(std::begin(this->ls1), std::prev(std::end(this->ls1)));
  if (traits::is2D<TypeParam>()) {
    EXPECT_DOUBLE_EQ(2, l);
  } else {
    EXPECT_DOUBLE_EQ(1 + std::sqrt(2), l);
  }
  EXPECT_DOUBLE_EQ(1, lSeg1);
}

TYPED_TEST(MutableLineStringsTest, boostConvert) {  // NOLINT
  auto equal = [](auto& ls1, auto& ls2) {
    return std::equal(ls1.begin(), ls1.end(), ls2.begin(), ls2.end(),
                      [](auto& p1, auto& p2) { return p1.basicPoint() == p2.basicPoint(); });
  };
  typename TypeParam::BasicLineString basicLineString;
  TypeParam convertedLineString;
  auto testLs = this->ls1.invert();
  boost::geometry::convert(testLs, basicLineString);
  boost::geometry::convert(basicLineString, convertedLineString);
  EXPECT_PRED2(equal, testLs, convertedLineString);
}

TYPED_TEST(MutableLineStringsTest, boostAppend) {  // NOLINT
  typename TypeParam::BasicPointType p = this->ls2.front();
  auto testLs = this->ls1.invert();
  boost::geometry::append(testLs, p);
  EXPECT_EQ(testLs.back().basicPoint(), p);
}

TYPED_TEST(AllLineStringsTest, lengthRatios) {  // NOLINT
  auto lRatios = geometry::lengthRatios(this->ls1);
  ASSERT_EQ(2ul, lRatios.size());
  if (traits::is2D<TypeParam>()) {
    EXPECT_DOUBLE_EQ(0.5, lRatios[0]);
  } else {
    EXPECT_DOUBLE_EQ(1 / (1 + std::sqrt(2)), lRatios[0]);
  }
}

TYPED_TEST(AllLineStringsTest, accumulatedLengthRatios) {  // NOLINT
  auto lRatios = geometry::accumulatedLengthRatios(this->ls2);
  ASSERT_EQ(2ul, lRatios.size());
  EXPECT_DOUBLE_EQ(1, lRatios[1]);
}

TYPED_TEST(AllLineStringsTest, interpolatedPoint) {  // NOLINT
  const auto twoD = traits::is2D<TypeParam>();
  const auto pos = twoD ? 1.5 : 1 + std::sqrt(0.5);
  const auto l = geometry::length(this->ls1);
  auto p = geometry::interpolatedPointAtDistance(this->ls1, pos);
  auto pInv = geometry::interpolatedPointAtDistance(this->ls1, pos - l);
  EXPECT_DOUBLE_EQ(p.x(), 0);
  EXPECT_DOUBLE_EQ(pInv.x(), 0);
  EXPECT_DOUBLE_EQ(p.y(), 1.5);
  EXPECT_DOUBLE_EQ(pInv.y(), 1.5);
  if (!twoD) {
    EXPECT_DOUBLE_EQ(getZ(p), 0.5);
    EXPECT_DOUBLE_EQ(getZ(pInv), 0.5);
  }
}

TYPED_TEST(AllLineStringsTest, nearestPoint) {  // NOLINT
  const auto twoD = traits::is2D<TypeParam>();
  auto pos = twoD ? 1.4 : 1 + std::sqrt(0.4);
  const auto l = geometry::length(this->ls1);
  auto p = geometry::nearestPointAtDistance(this->ls1, pos);
  auto pInv = geometry::nearestPointAtDistance(this->ls1, pos - l);
  EXPECT_EQ(p, this->ls1[1]);
  EXPECT_EQ(pInv, this->ls1[1]);
}

TYPED_TEST(AllLineStringsTest, segments) {  // NOLINT
  auto segment = this->ls1.segment(0);
  EXPECT_EQ(2ul, this->ls1.numSegments());
  EXPECT_EQ(segment.first, this->ls1[0]);
  EXPECT_EQ(segment.second, this->ls1[1]);
}

TYPED_TEST(AllLineStringsTest, segmentsInverse) {  // NOLINT
  auto lsInv = this->ls1.invert();
  auto segment = lsInv.segment(0);
  EXPECT_EQ(segment.first, lsInv[0]);
  EXPECT_EQ(segment.second, lsInv[1]);
}

TYPED_TEST(TwoDPointsTest, checkCurvature) {
  EXPECT_DOUBLE_EQ(1., geometry::curvature2d(this->p1, this->p2, this->p3));
  EXPECT_DOUBLE_EQ(std::numeric_limits<double>::infinity(), geometry::curvature2d(this->p1, this->p2, this->p4));
}

TYPED_TEST(TwoDLineStringsTest, signedDistance) {  // NOLINT
  auto isLeft = [this](const BasicPoint2d& p) { return geometry::signedDistance(this->ls2, p) > 0; };
  auto d = geometry::signedDistance(this->ls2, BasicPoint2d(2, -1));
  EXPECT_DOUBLE_EQ(-1., d);
  auto d2 = geometry::signedDistance(this->ls2, BasicPoint2d(3, 0.5));
  EXPECT_DOUBLE_EQ(-std::sqrt(1.25), d2);

  EXPECT_FALSE(isLeft(BasicPoint2d(0, -1)));
  EXPECT_FALSE(isLeft(BasicPoint2d(0, 2)));
  EXPECT_FALSE(isLeft(BasicPoint2d(3, 0)));
  EXPECT_TRUE(isLeft(BasicPoint2d(0, 1)));
  EXPECT_TRUE(isLeft(BasicPoint2d(-5, 1)));
  EXPECT_FALSE(isLeft(BasicPoint2d(-2, 5)));
}

TYPED_TEST(ThreeDLineStringsTest, signedDistance) {  // NOLINT
  auto isLeft = [this](const BasicPoint3d& p) { return geometry::signedDistance(this->ls1, p) > 0; };
  auto d = geometry::signedDistance(this->ls1, BasicPoint3d(2, 0, 0));
  EXPECT_DOUBLE_EQ(-2., d);
  auto d2 = geometry::signedDistance(this->ls1, BasicPoint3d(-1, 1, 0));
  EXPECT_DOUBLE_EQ(1, d2);

  EXPECT_TRUE(isLeft(BasicPoint3d(-1, -1, 10)));
  EXPECT_FALSE(isLeft(BasicPoint3d(5, 5, -10)));
}

TYPED_TEST(ThreeDLineStringsTest, intersects3d) {  // NOLINT
  EXPECT_FALSE(geometry::intersects3d(this->ls1, this->ls3, 1.));
  EXPECT_TRUE(geometry::intersects3d(this->ls3, this->ls1, 3.));
  EXPECT_TRUE(geometry::intersects3d(this->ls2, this->ls1, 2.));
}

TYPED_TEST(TwoDLineStringsTest, arcCoordinates) {  // NOLINT
  ArcCoordinates arcPoint = geometry::toArcCoordinates(this->ls1, BasicPoint2d(1, -1));
  EXPECT_EQ(-std::sqrt(2), arcPoint.distance);
  EXPECT_EQ(0, arcPoint.length);
  ArcCoordinates arcPoint2 = geometry::toArcCoordinates(this->ls1, BasicPoint2d(-1, 1.5));
  EXPECT_EQ(1, arcPoint2.distance);
  EXPECT_EQ(1.5, arcPoint2.length);
}

TYPED_TEST(TwoDLineStringsTest, projectedPoint) {  // NOLINT
  auto p = BasicPoint2d(0, 1.5);
  auto projectedPoint = geometry::project(this->ls1, p);
  EXPECT_DOUBLE_EQ(0, projectedPoint.x());
  EXPECT_DOUBLE_EQ(1.5, projectedPoint.y());
}

TYPED_TEST(PrimitiveLineStringsTest, align) {  // NOLINT
  TypeParam ls3(20, {Point3d(21, 2., 0., 0.), Point3d(22, 1.5, 0.5, 0.)});
  auto aligned = geometry::align(this->ls1, ls3);
  EXPECT_EQ(aligned.first, this->ls1);
  EXPECT_EQ(aligned.second, ls3);

  aligned = geometry::align(ls3, this->ls1);
  EXPECT_EQ(aligned.first.invert(), ls3);
  EXPECT_EQ(aligned.second.invert(), this->ls1);
}

TYPED_TEST(HybridLineStringsTest, segmentLength) {  // NOLINT
  auto segment = this->ls1.segment(0);
  EXPECT_DOUBLE_EQ(geometry::length(segment), 1.);
}

/*
 *    O
 *    |
 *    |
 *    |
 * O--X
 *    |
 *    |
 *    |
 *    O
 */
TYPED_TEST(TwoDLineStringsTest, fromArcCoords) {
  Id id = 0;
  Point3d p1 = Point3d(++id, 0., 0.);
  Point3d p2 = Point3d(++id, 0., 10.);
  LineString3d ls(++id, Points3d{p1, p2});
  auto ap = geometry::fromArcCoordinates(utils::to2D(ls), ArcCoordinates({5, 2}));
  EXPECT_TRUE(boost::geometry::equals(ap, BasicPoint2d{-2, 5}));
}

TYPED_TEST(TwoDLineStringsTest, offset) {
  Id id = 0;
  Point3d p1 = Point3d(++id, 1., 0.);
  Point3d p2 = Point3d(++id, 1., 1.);
  Point3d p3 = Point3d(++id, 4., 1.);
  Point3d p4 = Point3d(++id, 4., 3.);
  LineString3d ls(++id, Points3d{p1, p2, p3, p4});
  auto ap = geometry::offset(utils::to2D(ls), 1.);
  BasicLineString2d comp({BasicPoint2d(0, 0), BasicPoint2d(0, 2), BasicPoint2d(3, 2), BasicPoint2d(3, 3)});
  // required due to numeric approximation errors
  for (size_t i = 0; i < ls.size(); ++i) {
    EXPECT_NEAR(ap[i].x(), comp[i].x(), 1e-9);
    EXPECT_NEAR(ap[i].y(), comp[i].y(), 1e-9);
  }

  Point3d p11 = Point3d(++id, 1, 0);
  Point3d p12 = Point3d(++id, 1, 1);
  Point3d p13 = Point3d(++id, 0, 1);
  Point3d p13a = Point3d(++id, 2, 1);
  Point3d p14 = Point3d(++id, 0.5, 0);
  Point3d p15 = Point3d(++id, 0.5, 1);
  Point3d p16 = Point3d(++id, 1, 0.5);
  Point3d p17 = Point3d(++id, 0, 0.5);
  Point3d p18 = Point3d(++id, 0.5, 1.5);
  Point3d p19 = Point3d(++id, 2, 1.5);

  BasicLineString2d l1{utils::toBasicPoint(utils::to2D(p11)), utils::toBasicPoint(utils::to2D(p12)),
                       utils::toBasicPoint(utils::to2D(p13))};
  BasicLineString2d l2{utils::toBasicPoint(utils::to2D(p11)), utils::toBasicPoint(utils::to2D(p12)),
                       utils::toBasicPoint(utils::to2D(p13a))};
  auto lss = geometry::offset(l1, 0.5);
  EXPECT_TRUE(boost::geometry::equals(
      lss, BasicLineString2d{utils::toBasicPoint(utils::to2D(p14)), BasicPoint2d{0.5, 0.5}, BasicPoint2d{0, 0.5}}));
  auto lss2 = geometry::offset(l2, 0.5);
  EXPECT_TRUE(boost::geometry::equals(
      lss2, BasicLineString2d{utils::toBasicPoint(utils::to2D(p14)), utils::toBasicPoint(utils::to2D(p18)),
                              utils::toBasicPoint(utils::to2D(p19))}));
}

TYPED_TEST(MutableLineStringsTest, closestSegment) {
  traits::PointType<typename TestFixture::LineStringT> p3d(100000000000, 0, 1.5, 1.5);
  traits::PointType<typename TestFixture::LineStringT> ref3d(this->p12);
  auto bp = utils::toBasicPoint(p3d);
  auto refp = utils::toBasicPoint(ref3d);
  auto cs = geometry::closestSegment(this->ls1, bp);
  EXPECT_TRUE(boost::geometry::equals(cs.first, refp));

  auto b2dcs = geometry::closestSegment(utils::to2D(this->ls1).basicLineString(), utils::to2D(bp));
  EXPECT_TRUE(boost::geometry::equals(b2dcs.first, utils::to2D(refp)));
  auto b3dcs = geometry::closestSegment(this->ls1.basicLineString(), bp);
  EXPECT_TRUE(boost::geometry::equals(b3dcs.first, refp));
}

TYPED_TEST(TwoDLineStringsTest, shiftLateral) {
  EXPECT_EQ(this->ls4.size(), 3ul);

  auto shifted = geometry::internal::shiftLateral(this->ls4, 1, 1., geometry::internal::makeVincinity(this->ls4, 1));
  EXPECT_TRUE(boost::geometry::equals(shifted, BasicPoint2d(-1, 1. + sqrt(3) / 3)));
}

/*   O   O
 *  /|   |\
 * O-X---X-O
 *   |   |
 *   O   O
 */
TEST(TwoDLineStringsTest, removeSelfIntersections) {
  auto p1 = BasicPoint2d(2, 0);
  auto p2 = BasicPoint2d(2, 2);
  auto p3 = BasicPoint2d(3, 1);
  auto p4 = BasicPoint2d(0, 1);
  auto p5 = BasicPoint2d(1, 2);
  auto p6 = BasicPoint2d(1, 0);
  auto p7 = BasicPoint2d(2, 1);
  auto p8 = BasicPoint2d(1, 1);
  BasicLineString2d l1{p1, p2, p3, p4, p5, p6};
  BasicLineString2d l2{p1, p7, p8, p6};
  auto ret = geometry::internal::removeSelfIntersections(l1);
  EXPECT_TRUE(boost::geometry::equals(ret, l2));
}

TEST(TwoDLineStringsTest, extractConvex) {
  auto p1 = BasicPoint2d(1, 0);
  auto p2 = BasicPoint2d(1, 1);
  auto p3 = BasicPoint2d(0, 1);
  auto p3a = BasicPoint2d(2, 1);
  auto p3b = BasicPoint2d(0, 0);
  auto p4 = BasicPoint2d(0.5, 0);
  auto p5 = BasicPoint2d(0.5, 1);
  auto p6 = BasicPoint2d(1, 0.5);
  auto p7 = BasicPoint2d(0, 0.5);
  auto p8 = BasicPoint2d(0.5, 1.5);
  auto p9 = BasicPoint2d(2, 1.5);

  BasicLineString2d l1{p1, p2, p3};
  BasicLineString2d l2{p1, p2, p3a};
  BasicLineString2d l3{p1, p2, p3b};
  auto lss = geometry::internal::extractConvex(l1, 0.5);
  ASSERT_EQ(lss.size(), 2ul);
  EXPECT_TRUE(boost::geometry::equals(lss.front(), BasicLineString2d{p4, p5}));
  EXPECT_TRUE(boost::geometry::equals(lss.back(), BasicLineString2d{p6, p7}));
  auto lss2 = geometry::internal::extractConvex(l2, 0.5);
  ASSERT_EQ(lss2.size(), 1ul);
  EXPECT_TRUE(boost::geometry::equals(lss2.front(), BasicLineString2d{p4, p8, p9}));
  auto lss3 = geometry::internal::extractConvex(l3, -0.5);
  ASSERT_EQ(lss3.size(), 1ul);
  ASSERT_EQ(lss3.front().size(), 4ul);
  EXPECT_TRUE(boost::geometry::equals(lss2.front(), BasicLineString2d{p4, p8, p9}));
}

/*
 * O----O
 * |    |
 * |    O
 * |
 * |
 * O------O
 *
 */

TEST(TwoDLineStringsTest, checkInversion) {
  auto p1 = BasicPoint2d(3, 3);
  auto p2 = BasicPoint2d(3, 5);
  auto p3 = BasicPoint2d(0, 5);
  auto p4 = BasicPoint2d(0, 0);
  auto p5 = BasicPoint2d(4, 0);

  BasicLineString2d l1{p1, p2, p3, p4, p5};
  EXPECT_THROW(geometry::offset(l1, 2), GeometryError);  // NOLINT
  EXPECT_NO_THROW(geometry::offset(l1, 1));              // NOLINT
}
