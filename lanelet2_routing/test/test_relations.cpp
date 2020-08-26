#include <gtest/gtest.h>

#include "lanelet2_routing/RoutingGraph.h"
#include "test_routing_map.h"

using namespace lanelet;
using namespace lanelet::routing::tests;

TEST_F(GermanVehicleGraph, LaneChangesLeft) {  // NOLINT
  // Multiple1
  Optional<ConstLanelet> left = graph->left(lanelets.at(2001));
  EXPECT_TRUE((!!left));
  EXPECT_TRUE(*left == lanelets.at(2003));

  left = graph->left(lanelets.at(2002));
  EXPECT_FALSE((!!left));
}

TEST_F(GermanVehicleGraph, LaneChangesLeftMerging) {  // NOLINT
  Optional<ConstLanelet> left = graph->left(lanelets.at(2005));
  EXPECT_FALSE((!!left));

  left = graph->left(lanelets.at(2006));
  EXPECT_FALSE((!!left));
}

TEST_F(GermanVehicleGraph, LaneChangesLeftDiverging) {  // NOLINT
  Optional<ConstLanelet> left = graph->left(lanelets.at(2008));
  EXPECT_FALSE((!!left));

  left = graph->left(lanelets.at(2009));
  EXPECT_FALSE((!!left));
}

TEST_F(GermanVehicleGraph, LaneChangesLeftMaxHose) {  // NOLINT
  Optional<ConstLanelet> left = graph->left(lanelets.at(2016));
  EXPECT_FALSE((!!left));

  left = graph->left(lanelets.at(2020));
  EXPECT_FALSE((!!left));
}

TEST_F(GermanVehicleGraph, LaneChangesLeftInvalid) {  // NOLINT
  ConstLanelet invalidLanelet;
  Optional<ConstLanelet> left = graph->left(invalidLanelet);
  EXPECT_FALSE((!!left));
}

TEST_F(GermanVehicleGraph, LaneChangesRight) {  // NOLINT
  // Multiple1
  Optional<ConstLanelet> right = graph->right(lanelets.at(2003));
  EXPECT_TRUE((!!right));
  EXPECT_TRUE(*right == lanelets.at(2001));

  right = graph->right(lanelets.at(2004));
  EXPECT_FALSE((!!right));

  right = graph->right(lanelets.at(2001));
  EXPECT_FALSE((!!right));

  right = graph->right(lanelets.at(2002));
  EXPECT_FALSE((!!right));
}

TEST_F(GermanVehicleGraph, LaneChangesRightMerging) {  // NOLINT
  Optional<ConstLanelet> right = graph->right(lanelets.at(2006));
  EXPECT_FALSE((!!right));

  right = graph->right(lanelets.at(2005));
  EXPECT_FALSE((!!right));
}

TEST_F(GermanVehicleGraph, LaneChangesRightDiverging) {  // NOLINT
  Optional<ConstLanelet> right = graph->right(lanelets.at(2008));
  EXPECT_FALSE((!!right));

  right = graph->right(lanelets.at(2009));
  EXPECT_FALSE((!!right));
}

TEST_F(GermanVehicleGraph, LaneChangesRightMaxHose) {  // NOLINT
  Optional<ConstLanelet> right = graph->right(lanelets.at(2016));
  EXPECT_FALSE((!!right));

  right = graph->right(lanelets.at(2020));
  EXPECT_FALSE((!!right));
}

TEST_F(GermanVehicleGraph, LaneChangesRightInvalid) {  // NOLINT
  // Invalid
  ConstLanelet invalidLanelet;
  Optional<ConstLanelet> right = graph->right(invalidLanelet);
  EXPECT_FALSE((!!right));
}

TEST_F(GermanVehicleGraph, AllLaneChangesLeft) {  // NOLINT
  // Multiple2
  ConstLanelets left;
  left = graph->lefts(lanelets.at(2012));
  EXPECT_EQ(left.size(), 1ul);
  EXPECT_TRUE(std::find(left.begin(), left.end(), lanelets.at(2011)) != left.end());

  left = graph->lefts(lanelets.at(2011));
  EXPECT_TRUE(left.empty());

  left = graph->lefts(lanelets.at(2014));
  EXPECT_EQ(left.size(), 1ul);
  EXPECT_TRUE(std::find(left.begin(), left.end(), lanelets.at(2013)) != left.end());

  left = graph->lefts(lanelets.at(2015));
  EXPECT_EQ(left.size(), 2ul);
  EXPECT_TRUE(std::find(left.begin(), left.end(), lanelets.at(2014)) != left.end());
  EXPECT_TRUE(std::find(left.begin(), left.end(), lanelets.at(2013)) != left.end());
}

TEST_F(GermanVehicleGraph, AllLaneChangesRight) {  // NOLINT
  // Multiple2
  ConstLanelets right;
  right = graph->rights(lanelets.at(2011));
  EXPECT_EQ(right.size(), 1ul);
  EXPECT_TRUE(std::find(right.begin(), right.end(), lanelets.at(2012)) != right.end());

  right = graph->rights(lanelets.at(2012));
  EXPECT_TRUE(right.empty());

  right = graph->rights(lanelets.at(2013));
  EXPECT_EQ(right.size(), 2ul);
  EXPECT_TRUE(std::find(right.begin(), right.end(), lanelets.at(2014)) != right.end());
  EXPECT_TRUE(std::find(right.begin(), right.end(), lanelets.at(2015)) != right.end());

  right = graph->rights(lanelets.at(2014));
  EXPECT_EQ(right.size(), 1ul);
  EXPECT_TRUE(std::find(right.begin(), right.end(), lanelets.at(2015)) != right.end());
}

TEST_F(GermanVehicleGraph, FollowingWithoutLaneChange) {  // NOLINT
  // Multiple1
  ConstLanelets following = graph->following(lanelets.at(2001), false);
  EXPECT_EQ(following.size(), 1ul);
  EXPECT_TRUE(std::find(following.begin(), following.end(), lanelets.at(2002)) != following.end());

  following = graph->following(lanelets.at(2003), false);
  EXPECT_EQ(following.size(), 1ul);
  EXPECT_TRUE(std::find(following.begin(), following.end(), lanelets.at(2004)) != following.end());

  following = graph->following(lanelets.at(2002), false);
  EXPECT_EQ(following.size(), 1ul);
  EXPECT_TRUE(std::find(following.begin(), following.end(), lanelets.at(2006)) != following.end());

  following = graph->following(lanelets.at(2004), false);
  EXPECT_EQ(following.size(), 1ul);
  EXPECT_TRUE(std::find(following.begin(), following.end(), lanelets.at(2005)) != following.end());
}

TEST_F(GermanVehicleGraph, FollowingWithoutLaneChangeMerging) {  // NOLINT
  ConstLanelets following = graph->following(lanelets.at(2005), false);
  EXPECT_EQ(following.size(), 1ul);
  EXPECT_TRUE(std::find(following.begin(), following.end(), lanelets.at(2007)) != following.end());
}

TEST_F(GermanVehicleGraph, FollowingWithoutLaneChangeDiverging) {  // NOLINT
  // Single Lane -> Diverging
  ConstLanelets following = graph->following(lanelets.at(2007), false);
  EXPECT_EQ(following.size(), 2ul);
  EXPECT_TRUE(std::find(following.begin(), following.end(), lanelets.at(2008)) != following.end());
  EXPECT_TRUE(std::find(following.begin(), following.end(), lanelets.at(2009)) != following.end());
}

TEST_F(GermanVehicleGraph, FollowingWithoutLaneChangeMaxHose) {  // NOLINT
  // Max Hose Problem
  ConstLanelets following = graph->following(lanelets.at(2018), false);
  EXPECT_EQ(following.size(), 1ul);
  EXPECT_TRUE(std::find(following.begin(), following.end(), lanelets.at(2016)) != following.end());

  following = graph->following(lanelets.at(2019), false);
  EXPECT_EQ(following.size(), 1ul);
  EXPECT_TRUE(std::find(following.begin(), following.end(), lanelets.at(2020)) != following.end());

  following = graph->following(lanelets.at(2020), false);
  EXPECT_EQ(following.size(), 1ul);
  EXPECT_TRUE(std::find(following.begin(), following.end(), lanelets.at(2022)) != following.end());

  following = graph->following(lanelets.at(2022), false);
  EXPECT_EQ(following.size(), 1ul);
  EXPECT_TRUE(std::find(following.begin(), following.end(), lanelets.at(2024)) != following.end());

  following = graph->following(lanelets.at(2021), false);
  EXPECT_EQ(following.size(), 1ul);
  EXPECT_TRUE(std::find(following.begin(), following.end(), lanelets.at(2020).invert()) != following.end());

  following = graph->following(lanelets.at(2020).invert(), false);
  EXPECT_EQ(following.size(), 1ul);
  EXPECT_TRUE(std::find(following.begin(), following.end(), lanelets.at(2018)) != following.end());
}

TEST_F(GermanVehicleGraph, FollowingWithoutLaneChangeInvalid) {  // NOLINT
  // Invalid
  ConstLanelet invalidLanelet;
  ConstLanelets following = graph->following(invalidLanelet, false);
  EXPECT_TRUE(following.empty());
}

TEST_F(GermanVehicleGraph, FollowingWithLaneChange) {  // NOLINT
  // Multiple1
  ConstLanelets following = graph->following(lanelets.at(2001), true);
  EXPECT_EQ(following.size(), 2ul);
  EXPECT_TRUE(std::find(following.begin(), following.end(), lanelets.at(2002)) != following.end());
  EXPECT_TRUE(std::find(following.begin(), following.end(), lanelets.at(2003)) != following.end());

  following = graph->following(lanelets.at(2003), true);
  EXPECT_EQ(following.size(), 2ul);
  EXPECT_TRUE(std::find(following.begin(), following.end(), lanelets.at(2001)) != following.end());
  EXPECT_TRUE(std::find(following.begin(), following.end(), lanelets.at(2004)) != following.end());

  following = graph->following(lanelets.at(2002), true);
  EXPECT_EQ(following.size(), 1ul);
  EXPECT_TRUE(std::find(following.begin(), following.end(), lanelets.at(2006)) != following.end());

  following = graph->following(lanelets.at(2004), true);
  EXPECT_EQ(following.size(), 1ul);
  EXPECT_TRUE(std::find(following.begin(), following.end(), lanelets.at(2005)) != following.end());
}

TEST_F(GermanVehicleGraph, FollowingWithLaneChangeMerging) {  // NOLINT
  ConstLanelets following = graph->following(lanelets.at(2005), true);
  EXPECT_EQ(following.size(), 1ul);
  EXPECT_TRUE(std::find(following.begin(), following.end(), lanelets.at(2007)) != following.end());

  following = graph->following(lanelets.at(2006), true);
  EXPECT_EQ(following.size(), 1ul);
  EXPECT_TRUE(std::find(following.begin(), following.end(), lanelets.at(2007)) != following.end());
}

TEST_F(GermanVehicleGraph, FollowingWithLaneChangeDiverging) {  // NOLINT
  // Single Lane -> Diverging
  ConstLanelets following = graph->following(lanelets.at(2007), true);
  EXPECT_EQ(following.size(), 2ul);
  EXPECT_TRUE(std::find(following.begin(), following.end(), lanelets.at(2008)) != following.end());
  EXPECT_TRUE(std::find(following.begin(), following.end(), lanelets.at(2009)) != following.end());
}

TEST_F(GermanVehicleGraph, FollowingWithLaneChangeMaxHose) {  // NOLINT
  // Max Hose Problem
  ConstLanelets following = graph->following(lanelets.at(2018), true);
  EXPECT_EQ(following.size(), 1ul);
  EXPECT_TRUE(std::find(following.begin(), following.end(), lanelets.at(2016)) != following.end());

  following = graph->following(lanelets.at(2019), true);
  EXPECT_EQ(following.size(), 1ul);
  EXPECT_TRUE(std::find(following.begin(), following.end(), lanelets.at(2020)) != following.end());

  following = graph->following(lanelets.at(2020), true);
  EXPECT_EQ(following.size(), 1ul);
  EXPECT_TRUE(std::find(following.begin(), following.end(), lanelets.at(2022)) != following.end());

  following = graph->following(lanelets.at(2022), true);
  EXPECT_EQ(following.size(), 1ul);
  EXPECT_TRUE(std::find(following.begin(), following.end(), lanelets.at(2024)) != following.end());

  following = graph->following(lanelets.at(2021), true);
  EXPECT_EQ(following.size(), 1ul);
  EXPECT_TRUE(std::find(following.begin(), following.end(), lanelets.at(2020).invert()) != following.end());

  following = graph->following(lanelets.at(2020).invert(), true);
  EXPECT_EQ(following.size(), 1ul);
  EXPECT_TRUE(std::find(following.begin(), following.end(), lanelets.at(2018)) != following.end());
}

TEST_F(GermanVehicleGraph, PreviousWithoutLaneChange) {  // NOLINT
  // Multiple1
  ConstLanelets previous;
  previous = graph->previous(lanelets.at(2001), false);
  EXPECT_EQ(previous.size(), 0ul);

  previous = graph->previous(lanelets.at(2003), false);
  EXPECT_EQ(previous.size(), 0ul);

  previous = graph->previous(lanelets.at(2002), false);
  EXPECT_EQ(previous.size(), 1ul);
  EXPECT_TRUE(std::find(previous.begin(), previous.end(), lanelets.at(2001)) != previous.end());

  previous = graph->previous(lanelets.at(2004), false);
  EXPECT_EQ(previous.size(), 1ul);
  EXPECT_TRUE(std::find(previous.begin(), previous.end(), lanelets.at(2003)) != previous.end());
}

TEST_F(GermanVehicleGraph, PreviousWithoutLaneChangeMerging) {  // NOLINT
  ConstLanelets previous = graph->previous(lanelets.at(2005), false);
  EXPECT_EQ(previous.size(), 1ul);
  EXPECT_TRUE(std::find(previous.begin(), previous.end(), lanelets.at(2004)) != previous.end());

  previous = graph->previous(lanelets.at(2006), false);
  EXPECT_EQ(previous.size(), 1ul);
  EXPECT_TRUE(std::find(previous.begin(), previous.end(), lanelets.at(2002)) != previous.end());
}

TEST_F(GermanVehicleGraph, PreviousWithoutLaneChangeSingleLane) {  // NOLINT
  ConstLanelets previous = graph->previous(lanelets.at(2007), false);
  EXPECT_EQ(previous.size(), 2ul);
  EXPECT_TRUE(std::find(previous.begin(), previous.end(), lanelets.at(2005)) != previous.end());
  EXPECT_TRUE(std::find(previous.begin(), previous.end(), lanelets.at(2006)) != previous.end());
}

TEST_F(GermanVehicleGraph, PreviousWithoutLaneChangeDiverging) {  // NOLINT
  ConstLanelets previous = graph->previous(lanelets.at(2008), false);
  EXPECT_EQ(previous.size(), 1ul);
  EXPECT_TRUE(std::find(previous.begin(), previous.end(), lanelets.at(2007)) != previous.end());

  previous = graph->previous(lanelets.at(2009), false);
  EXPECT_EQ(previous.size(), 1ul);
  EXPECT_TRUE(std::find(previous.begin(), previous.end(), lanelets.at(2007)) != previous.end());
}

TEST_F(GermanVehicleGraph, PreviousWithoutLaneChangeMaxHose) {  // NOLINT
  // Max Hose Problem
  ConstLanelets previous = graph->previous(lanelets.at(2019), false);
  EXPECT_EQ(previous.size(), 1ul);
  EXPECT_TRUE(std::find(previous.begin(), previous.end(), lanelets.at(2017)) != previous.end());

  previous = graph->previous(lanelets.at(2020), false);
  EXPECT_EQ(previous.size(), 1ul);
  EXPECT_TRUE(std::find(previous.begin(), previous.end(), lanelets.at(2019)) != previous.end());

  previous = graph->previous(lanelets.at(2022), false);
  EXPECT_EQ(previous.size(), 1ul);
  EXPECT_TRUE(std::find(previous.begin(), previous.end(), lanelets.at(2020)) != previous.end());

  previous = graph->previous(lanelets.at(2024), false);
  EXPECT_EQ(previous.size(), 1ul);
  EXPECT_TRUE(std::find(previous.begin(), previous.end(), lanelets.at(2022)) != previous.end());

  previous = graph->previous(lanelets.at(2021), false);
  EXPECT_EQ(previous.size(), 1ul);
  EXPECT_TRUE(std::find(previous.begin(), previous.end(), lanelets.at(2023)) != previous.end());

  previous = graph->previous(lanelets.at(2020).invert(), false);
  EXPECT_EQ(previous.size(), 1ul);
  EXPECT_TRUE(std::find(previous.begin(), previous.end(), lanelets.at(2021)) != previous.end());

  previous = graph->previous(lanelets.at(2018), false);
  EXPECT_EQ(previous.size(), 1ul);
  EXPECT_TRUE(std::find(previous.begin(), previous.end(), lanelets.at(2020).invert()) != previous.end());

  previous = graph->previous(lanelets.at(2016), false);
  EXPECT_EQ(previous.size(), 1ul);
  EXPECT_TRUE(std::find(previous.begin(), previous.end(), lanelets.at(2018)) != previous.end());
}

TEST_F(GermanVehicleGraph, PreviousWithoutLaneChangeInvalid) {  // NOLINT
  ConstLanelet invalidLanelet;
  ConstLanelets previous = graph->previous(invalidLanelet, false);
  EXPECT_TRUE(previous.empty());
}

TEST_F(GermanVehicleGraph, PreviousWithLaneChange) {  // NOLINT
  // Multiple1
  ConstLanelets previous = graph->previous(lanelets.at(2001), true);
  EXPECT_EQ(previous.size(), 1ul);
  EXPECT_TRUE(std::find(previous.begin(), previous.end(), lanelets.at(2003)) != previous.end());

  previous = graph->previous(lanelets.at(2003), true);
  EXPECT_EQ(previous.size(), 1ul);
  EXPECT_TRUE(std::find(previous.begin(), previous.end(), lanelets.at(2001)) != previous.end());

  previous = graph->previous(lanelets.at(2002), true);
  EXPECT_EQ(previous.size(), 1ul);
  EXPECT_TRUE(std::find(previous.begin(), previous.end(), lanelets.at(2001)) != previous.end());

  previous = graph->previous(lanelets.at(2004), true);
  EXPECT_EQ(previous.size(), 1ul);
  EXPECT_TRUE(std::find(previous.begin(), previous.end(), lanelets.at(2003)) != previous.end());
}

TEST_F(GermanVehicleGraph, PreviousWithLaneChangeMerging) {  // NOLINT
  ConstLanelets previous = graph->previous(lanelets.at(2005), true);
  EXPECT_EQ(previous.size(), 1ul);
  EXPECT_TRUE(std::find(previous.begin(), previous.end(), lanelets.at(2004)) != previous.end());

  previous = graph->previous(lanelets.at(2006), true);
  EXPECT_EQ(previous.size(), 1ul);
  EXPECT_TRUE(std::find(previous.begin(), previous.end(), lanelets.at(2002)) != previous.end());
}

TEST_F(GermanVehicleGraph, PreviousWithLaneChangeSingleLane) {  // NOLINT
  ConstLanelets previous = graph->previous(lanelets.at(2007), true);
  EXPECT_EQ(previous.size(), 2ul);
  EXPECT_TRUE(std::find(previous.begin(), previous.end(), lanelets.at(2005)) != previous.end());
  EXPECT_TRUE(std::find(previous.begin(), previous.end(), lanelets.at(2006)) != previous.end());
}

TEST_F(GermanVehicleGraph, PreviousWithLaneChangeDiverging) {  // NOLINT
  ConstLanelets previous = graph->previous(lanelets.at(2008), true);
  EXPECT_EQ(previous.size(), 1ul);
  EXPECT_TRUE(std::find(previous.begin(), previous.end(), lanelets.at(2007)) != previous.end());

  previous = graph->previous(lanelets.at(2009), true);
  EXPECT_EQ(previous.size(), 1ul);
  EXPECT_TRUE(std::find(previous.begin(), previous.end(), lanelets.at(2007)) != previous.end());
}

TEST_F(GermanVehicleGraph, PreviousWithLaneChangeMaxHose) {  // NOLINT
  ConstLanelets previous = graph->previous(lanelets.at(2019), true);
  EXPECT_EQ(previous.size(), 1ul);
  EXPECT_TRUE(std::find(previous.begin(), previous.end(), lanelets.at(2017)) != previous.end());

  previous = graph->previous(lanelets.at(2020), true);
  EXPECT_EQ(previous.size(), 1ul);
  EXPECT_TRUE(std::find(previous.begin(), previous.end(), lanelets.at(2019)) != previous.end());

  previous = graph->previous(lanelets.at(2022), true);
  EXPECT_EQ(previous.size(), 1ul);
  EXPECT_TRUE(std::find(previous.begin(), previous.end(), lanelets.at(2020)) != previous.end());

  previous = graph->previous(lanelets.at(2024), true);
  EXPECT_EQ(previous.size(), 1ul);
  EXPECT_TRUE(std::find(previous.begin(), previous.end(), lanelets.at(2022)) != previous.end());

  previous = graph->previous(lanelets.at(2021), true);
  EXPECT_EQ(previous.size(), 1ul);
  EXPECT_TRUE(std::find(previous.begin(), previous.end(), lanelets.at(2023)) != previous.end());

  previous = graph->previous(lanelets.at(2020).invert(), true);
  EXPECT_EQ(previous.size(), 1ul);
  EXPECT_TRUE(std::find(previous.begin(), previous.end(), lanelets.at(2021)) != previous.end());

  previous = graph->previous(lanelets.at(2018), true);
  EXPECT_EQ(previous.size(), 1ul);
  EXPECT_TRUE(std::find(previous.begin(), previous.end(), lanelets.at(2020).invert()) != previous.end());

  previous = graph->previous(lanelets.at(2016), true);
  EXPECT_EQ(previous.size(), 1ul);
  EXPECT_TRUE(std::find(previous.begin(), previous.end(), lanelets.at(2018)) != previous.end());
}

TEST_F(GermanVehicleGraph, PreviousWithLaneChangeInvalid) {  // NOLINT
  ConstLanelet invalidLanelet;
  ConstLanelets previous = graph->previous(invalidLanelet, true);
  EXPECT_TRUE(previous.empty());
}

TEST_F(GermanVehicleGraph, GetBesides) {  // NOLINT
  // Diverging2
  ConstLanelets laneletSet;
  laneletSet = graph->besides(lanelets.at(2010));
  ASSERT_EQ(1ul, laneletSet.size());
  EXPECT_EQ(laneletSet[0], lanelets.at(2010));

  laneletSet = graph->besides(lanelets.at(2011));
  ASSERT_EQ(laneletSet.size(), 2ul);
  EXPECT_EQ(laneletSet[0], lanelets.at(2011));
  EXPECT_EQ(laneletSet[1], lanelets.at(2012));

  // Multiple2
  laneletSet = graph->besides(lanelets.at(2015));
  ASSERT_EQ(laneletSet.size(), 3ul);
  EXPECT_EQ(laneletSet[0], lanelets.at(2013));
  EXPECT_EQ(laneletSet[1], lanelets.at(2014));
  EXPECT_EQ(laneletSet[2], lanelets.at(2015));

  laneletSet = graph->besides(lanelets.at(2014));
  ASSERT_EQ(laneletSet.size(), 3ul);
  EXPECT_EQ(laneletSet[0], lanelets.at(2013));
  EXPECT_EQ(laneletSet[1], lanelets.at(2014));
  EXPECT_EQ(laneletSet[2], lanelets.at(2015));

  laneletSet = graph->besides(lanelets.at(2013));
  ASSERT_EQ(laneletSet.size(), 3ul);
  EXPECT_EQ(laneletSet[0], lanelets.at(2013));
  EXPECT_EQ(laneletSet[1], lanelets.at(2014));
  EXPECT_EQ(laneletSet[2], lanelets.at(2015));
}

TEST_F(GermanVehicleGraph, conflicting) {  // NOLINT
  // Normal Lanelets
  auto result = graph->conflicting(lanelets.at(2007));
  EXPECT_EQ(result.size(), 0ul);

  result = graph->conflicting(lanelets.at(2012));
  EXPECT_EQ(result.size(), 0ul);

  result = graph->conflicting(lanelets.at(2001));
  EXPECT_EQ(result.size(), 0ul);

  result = graph->conflicting(lanelets.at(2005));
  EXPECT_EQ(result.size(), 1ul);
}

TEST_F(GermanVehicleGraph, conflicting3d) {  // NOLINT
  // Bridge lanelet
  auto result = graph->conflicting(lanelets.at(2005));
  ASSERT_EQ(result.size(), 1ul);
  EXPECT_TRUE(result.front() == lanelets.at(2006));

  result = graph->conflicting(lanelets.at(2006));
  ASSERT_EQ(result.size(), 1ul);
  EXPECT_TRUE(result.front() == lanelets.at(2005));

  result = graph->conflicting(lanelets.at(2032));
  EXPECT_EQ(result.size(), 0ul);
}

TEST_F(GermanVehicleGraph, conflictingBothWays) {  // NOLINT
  // Both ways Lanelet
  auto result = graph->conflicting(lanelets.at(2020));
  ASSERT_EQ(result.size(), 1ul);
  EXPECT_NE(result.front().lanelet()->inverted(), lanelets.at(2020).inverted());

  result = graph->conflicting(lanelets.at(2020).invert());
  ASSERT_EQ(result.size(), 1ul);
  EXPECT_EQ(result.front().lanelet()->inverted(), lanelets.at(2020).inverted());
}

TEST_F(GermanVehicleGraph, conflictingMerging) {  // NOLINT
  auto result = graph->conflicting(lanelets.at(2005));
  ASSERT_EQ(result.size(), 1ul);
  EXPECT_EQ(result.front(), lanelets.at(2006));
}

TEST_F(GermanVehicleGraph, conflictingDiverging) {  // NOLINT
  auto result = graph->conflicting(lanelets.at(2008));
  ASSERT_EQ(result.size(), 1ul);
  EXPECT_EQ(result.front(), lanelets.at(2009));
}

TEST_F(RoutingGraphTest, routingCostOnLaneChange) {  // NOLINT
  // tests that all lane changes are not possible when the space for a lane change is too small according to the routing
  // cost object
  auto graph = setUpGermanVehicleGraph(*testData.laneletMap, 2, 2, 3);
  auto llt = [&](auto id) -> ConstLanelet { return lanelets.at(id); };
  EXPECT_EQ(graph->left(lanelets.at(2012)), llt(2011));
  EXPECT_EQ(graph->adjacentLeft(lanelets.at(2015)), llt(2014));
  EXPECT_EQ(graph->right(lanelets.at(2011)), llt(2012));
  EXPECT_EQ(graph->adjacentRight(lanelets.at(2014)), llt(2015));
}
