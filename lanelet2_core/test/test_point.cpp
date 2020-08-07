#include <gtest/gtest.h>

#include <sstream>

#include "lanelet2_core/geometry/Point.h"
#include "lanelet2_core/primitives/Point.h"

using namespace lanelet;

constexpr bool isTwod(const BasicPoint2d& /*unused*/) { return true; }
constexpr bool isTwod(const BasicPoint3d& /*unused*/) { return false; }

TEST(testPoint, constructDestroy) {  // NOLINT
  Point3d p;

  p = Point3d(-1, 1, 2, 3, {{"key", "value"}});
  EXPECT_EQ(-1, p.id());
  EXPECT_EQ(1, p.x());
  EXPECT_EQ(2, p.y());
  EXPECT_EQ(3, p.z());
  EXPECT_EQ("value", p.attribute("key").value());

  auto p2 = p;
  EXPECT_EQ(-1, p2.id());
  p2.x() = 2;
  EXPECT_EQ(2, p.x());

  EXPECT_THROW(Point3d(std::shared_ptr<PointData>(nullptr)), NullptrError);  // NOLINT
}

TEST(testPoint, convert) {  // NOLINT
  Point3d p(-1, 1, 2, 3, {{"key", "value"}});
  Point2d p2d = p;
  ConstPoint2d cp2d = p;
  ConstPoint3d cp3d(cp2d);

  p.y() = 3;
  EXPECT_EQ(3, p.y());
  EXPECT_EQ(3, p2d.y());
  EXPECT_EQ(3, cp2d.y());
  EXPECT_EQ(3, cp3d.y());

  BasicPoint2d bp2 = p2d;
  BasicPoint3d bp3 = p;
  EXPECT_EQ(3, bp2.y());
  EXPECT_EQ(3, bp3.y());

  ConstPoint3d newCp3d(cp2d);
  EXPECT_EQ(3, newCp3d.z());

  EXPECT_TRUE(isTwod(bp2));
  EXPECT_TRUE(!isTwod(newCp3d.basicPoint()));
  EXPECT_FALSE(isTwod(p.basicPoint()));
}

TEST(testPoint2d, twoDthreeD) {  // NOLINT
  Point3d p(-1, 1, 2, 3);
  Point2d p2d = p;
  ConstPoint2d cp2d = p2d;
  EXPECT_EQ(1, cp2d.basicPoint2d().x());
  EXPECT_EQ(2, cp2d.basicPoint2d().y());
  p2d.basicPoint().x() = 3;
  EXPECT_EQ(3, cp2d.basicPoint2d().x());
}

TEST(testPoint3d, assign) {  // NOLINT
  Point3d p(-1, 1, 2, 3);
  p.basicPoint() = BasicPoint3d(4, 4, 4);
  EXPECT_EQ(4, p.x());
}

TEST(testPoint2d, assign) {  // NOLINT
  Point2d p(-1, 1, 2, 3);
  p.basicPoint() = BasicPoint2d(4, 4);
  EXPECT_EQ(4, p.x());
}

TEST(testPoint, distance) {  // NOLINT
  using geometry::distance;
  using traits::to2D;
  Point3d p1(-1, 1, 2, 3, {{"key", "value"}});
  Point3d p2(-1, 0, 2, 2, {{"key", "value"}});

  EXPECT_DOUBLE_EQ(std::sqrt(2), distance(p1, p2));
  EXPECT_DOUBLE_EQ(1., distance(to2D(p1), to2D(p2)));
}

TEST(testPoint2d, stream) {  // NOLINT
  Point2d p1(-1, 1, 2, 3, {{"key", "value"}});
  std::string expect2d = "[id: -1 x: 1 y: 2]";
  std::stringstream ss;

  ss << ConstPoint2d(p1);
  EXPECT_EQ(expect2d, ss.str());

  ss.str("");
  ss << ConstPoint2d(p1);
  EXPECT_EQ(expect2d, ss.str());
}

TEST(testPoint3d, stream) {  // NOLINT
  Point3d p1(-1, 1, 2, 3, {{"key", "value"}});
  std::string expect3d = "[id: -1 x: 1 y: 2 z: 3]";
  std::stringstream ss;
  ss << p1;
  EXPECT_EQ(expect3d, ss.str());

  ss.str("");
  ss << ConstPoint3d(p1);
  EXPECT_EQ(expect3d, ss.str());
}
