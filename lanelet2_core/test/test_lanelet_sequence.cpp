#include <gtest/gtest.h>

#include <iostream>

#include "lanelet2_core/geometry/LineString.h"
#include "lanelet2_core/geometry/RegulatoryElement.h"
#include "lanelet2_core/primitives/BasicRegulatoryElements.h"
#include "lanelet2_core/primitives/LaneletSequence.h"
#include "lanelet2_core/utility/Utilities.h"

using namespace std::literals;
using namespace lanelet;

class LaneletSequenceTest : public ::testing::Test {
 protected:
  void SetUp() override {
    id = 0;
    p1 = Point3d(++id, 0., 1., 1.);
    p2 = Point3d(++id, 1., 1., 1.);
    p3 = Point3d(++id, 2., 1., 1.);
    p4 = Point3d(++id, 0., 0., 0.);
    p5 = Point3d(++id, 0.5, 0., 0.);
    p6 = Point3d(++id, 1, 0., 0.);
    p7 = Point3d(++id, 2., 0., 0.);
    left1 = LineString3d(++id, Points3d{p1, p2});
    right1 = LineString3d(++id, Points3d{p4, p5, p6});
    left2 = LineString3d(++id, Points3d{p2, p3});
    right2 = LineString3d(++id, Points3d{p6, p7});

    ll1 = Lanelet(++id, left1, right1);
    ll2 = Lanelet(++id, left2, right2);
    cll = LaneletSequence({ll1, ll2});
  }

 public:
  Id id{1};
  Point3d p1, p2, p3, p4, p5, p6, p7;
  Points3d pointsInOrder;
  LineString3d left1, right1, left2, right2;
  Lanelet ll1, ll2;
  LaneletSequence cll;
};

TEST_F(LaneletSequenceTest, LeftBoundHasNoDuplicates) {  // NOLINT
  EXPECT_EQ(cll.leftBound(), ConstPoints3d({p1, p2, p3}));
}

TEST_F(LaneletSequenceTest, RightBoundHasNoDuplicates) {  // NOLINT
  auto rb = cll.rightBound();
  EXPECT_EQ(rb, ConstPoints3d({p4, p5, p6, p7}));
}

TEST_F(LaneletSequenceTest, PolygonHasNoDuplicates) {  // NOLINT
  EXPECT_EQ(cll.polygon3d().basicPolygon(), BasicPolygon3d({p1, p2, p3, p7, p6, p5, p4}));
}

TEST_F(LaneletSequenceTest, AppendsCenterline) {  // NOLINT
  auto cl = cll.centerline();
  BasicLineString3d clExpect{BasicPoint3d(0, 0.5, 0.5), BasicPoint3d(0.5, 0.5, 0.5), BasicPoint3d(2, 0.5, 0.5)};
  for (const auto& p : clExpect) {
    EXPECT_FLOAT_EQ(geometry::distance(p, cl), 0.f);
  }
}

TEST_F(LaneletSequenceTest, InvertIteratesInReverseOrder) {  // NOLINT
  EXPECT_EQ(cll.invert().rightBound(), ConstPoints3d({p3, p2, p1}));
}

TEST_F(LaneletSequenceTest, LaneletsAreOk) {  // NOLINT
  EXPECT_EQ(cll.lanelets(), ConstLanelets({ll1, ll2}));
}

TEST_F(LaneletSequenceTest, ConstructFromLaneletSequences) {  // NOLINT
  LaneletSequence cl1{{ll1}};
  LaneletSequence cl2{{ll2}};
  LaneletSequence ccl{{cl1, cl2}};
  EXPECT_EQ(ccl, cll);
}

TEST_F(LaneletSequenceTest, RegulatoryElementExtraction) {  // NOLINT
  auto tl = TrafficLight::make(++id, AttributeMap(), LineStringsOrPolygons3d({left1, right1}), left1);
  auto ts = TrafficSign::make(++id, AttributeMap(), {{left2}, "de205"}, {}, {right2});
  ll1.addRegulatoryElement(tl);
  ll2.addRegulatoryElement(ts);
  EXPECT_EQ(1ul, cll.regulatoryElementsAs<TrafficLight>().size());
  EXPECT_EQ(1ul, cll.regulatoryElementsAs<TrafficSign>().size());
  EXPECT_TRUE(cll.regulatoryElementsAs<RightOfWay>().empty());
}
