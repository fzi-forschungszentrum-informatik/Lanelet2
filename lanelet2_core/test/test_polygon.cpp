#include <gtest/gtest.h>

#include <boost/geometry/algorithms/perimeter.hpp>
#include <random>

#include "lanelet2_core/geometry/Polygon.h"
#include "lanelet2_core/primitives/CompoundPolygon.h"
#include "lanelet2_core/primitives/Polygon.h"

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
    /*
     *      p42    X
     *           /   \
     *         /   X   \
     *       / -/ p44 \- \
     * p41 X/            \X p43
     */
    p41 = Point3d(++id, 1.5, 1.5, 2.);
    p42 = Point3d(++id, 1.5, 0.5, 2.);
  }

  Point3d p11, p12, p13;
  Point3d p21, p22, p23;
  Point3d p31, p32, p33, p34;
  Point3d p41, p42;
};

template <typename T>
class PolygonTypeTest : public PolygonPoints {
 public:
  using PolygonT = T;
  PolygonTypeTest() {
    Id id{20};
    AttributeMap poly1Attrs{{AttributeNamesString::Type, AttributeValueString::Curbstone}};
    poly1 = toPolygon<T>(Polygon3d{++id, {p11, p12, p13}, poly1Attrs});
    poly2 = toPolygon<T>(Polygon3d{++id, {p21, p22, p23}});
    poly3 = toPolygon<T>(Polygon3d{++id, {p31, p32, p33, p34}});
    Polygon3d temp{++id, {p31, p41, p34, p42}};
    simpleStar = toPolygon<T>(temp);
    subdivide(temp, 2, ++id);
    fancyStar = toPolygon<T>(temp);
  }

  PolygonT poly1, poly2, poly3, fancyStar, simpleStar;  // NOLINT

 private:
  void randomSubdivide(Polygon3d& poly, const size_t after, const int nNew, Id id) const {
    BasicPoint3d delta =
        (utils::toBasicPoint(poly[(after + 1) % poly.size()]) - utils::toBasicPoint(poly[after])) / (2 * nNew + 1);
    std::vector<Point3d> points;
    points.reserve(size_t(nNew));
    for (int i = 0; i < nNew; ++i) {
      points.emplace_back(Point3d(++id, utils::toBasicPoint(poly[after]) + (2 * double(i) + randomNum_() + 1) * delta));
    }
    Polygon3d newLs(++id, points);
    poly.insert(poly.begin() + long((after + 1) % poly.size()), newLs.begin(), newLs.end());
  }
  void subdivide(Polygon3d& poly, const int n, Id id) {
    auto initSize = poly.size();
    for (size_t i = 0; i < initSize; ++i) {
      randomSubdivide(poly, (n + 1) * i, n, id);
    }
  }
  std::function<double(void)> randomNum_ = [distr = std::uniform_real_distribution<double>(-0.5, 0.5),
                                            gen = std::mt19937(133769420)]() mutable { return distr(gen); };
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
    ConstLineString3d tempStar1{++id, {p31, p41, p34}};
    ConstLineString3d tempStar2{++id, {p34, p42}};

    poly1 = PolygonT({tempLs11});
    poly2 = PolygonT({tempLs21, tempLs22});
    poly3 = PolygonT({tempLs31, tempLs32});
    simpleStar = PolygonT({tempStar1, tempStar2});
    fancyStar = simpleStar;
  }

 public:
  PolygonT poly1, poly2, poly3, fancyStar, simpleStar;
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

inline auto crossProd(const Eigen::Matrix<double, 2, 1>& p1, const Eigen::Matrix<double, 2, 1>& p2) {
  return BasicPoint3d(p1.x(), p1.y(), 0.).cross(BasicPoint3d(p2.x(), p2.y(), 0.)).eval();
}

bool isConnectionConvex(const BasicPoint2d& seg1, const BasicPoint2d& seg2,
                        const double eps = 4 * std::numeric_limits<double>::epsilon()) {
  return crossProd(seg1.normalized(), seg2.normalized()).z() <= eps;
}

bool isConvex(const BasicPolygon2d& poly) {
  for (size_t i = 2; i < poly.size(); ++i) {
    if (!isConnectionConvex(poly.at(i - 1) - poly.at(i - 2), poly.at(i) - poly.at(i - 1))) {
      return false;
    }
  }
  return (isConnectionConvex(poly.back() - poly.at(poly.size() - 2), poly.front() - poly.back()) &&
          isConnectionConvex(poly.front() - poly.back(), poly.at(1) - poly.front()));
}

void checkPartitionConsistency(const BasicPolygon2d& poly, const BasicPolygons2d& parts) {
  double areaSum{0.};
  for (auto i = 0ul; i < parts.size(); ++i) {
    areaSum += boost::geometry::area(parts.at(i));
    for (auto j = i + 1; j < parts.size(); ++j) {
      BasicPolygon2d intersection;
      boost::geometry::intersection(parts.at(i), parts.at(j), intersection);
      EXPECT_DOUBLE_EQ(boost::geometry::area(intersection), 0.);
    }
    for (const auto& p : parts.at(i)) {
      Eigen::Matrix<double, 2, 1> pCopy(p.x(), p.y());
      EXPECT_DOUBLE_EQ(boost::geometry::distance(pCopy, poly), 0.);
    }
  }
  EXPECT_DOUBLE_EQ(areaSum, boost::geometry::area(poly));
}

TYPED_TEST(TwoDPolygonsTest, convexPartition) {
  auto t1 = lanelet::geometry::convexPartition(this->poly3);
  EXPECT_EQ(t1.size(), 1ul);
  EXPECT_TRUE(isConvex(t1.front()));
  checkPartitionConsistency(BasicPolygon2d(this->poly3.basicBegin(), this->poly3.basicEnd()), t1);
  auto t2 = lanelet::geometry::convexPartition(this->simpleStar);
  EXPECT_EQ(t2.size(), 2ul);
  for (const auto& poly : t2) {
    EXPECT_TRUE(isConvex(poly));
  }
  checkPartitionConsistency(BasicPolygon2d(this->simpleStar.basicBegin(), this->simpleStar.basicEnd()), t2);
  auto t3 = lanelet::geometry::convexPartition(this->fancyStar);
  EXPECT_EQ(t3.size(), 2ul);
  for (const auto& poly : t3) {
    EXPECT_TRUE(isConvex(poly));
  }
  checkPartitionConsistency(BasicPolygon2d(this->fancyStar.basicBegin(), this->fancyStar.basicEnd()), t3);
}

TYPED_TEST(TwoDAndBasicPolygonsTest, triangulate) {
  auto isIn = [](const geometry::IndexedTriangle& p, const size_t idx) {
    return std::find(p.begin(), p.end(), idx) != p.end();
  };
  auto res = lanelet::geometry::triangulate(this->poly3);
  ASSERT_EQ(res.size(), 2ul);
  EXPECT_EQ(res.front().size(), 3ul);
  EXPECT_EQ(res.back().size(), 3ul);
  const auto& t1 = res.front();
  const auto& t2 = res.back();
  EXPECT_TRUE((isIn(t1, 1) && isIn(t1, 3) && isIn(t2, 1) && isIn(t2, 3)) ||
              (isIn(t1, 0) && isIn(t1, 2) && isIn(t2, 0) && isIn(t2, 2)));
}
