#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>

#include "lanelet2_map_learning/Forward.h"
#include "lanelet2_map_learning/MapGraph.h"
#include "lanelet2_map_learning/MapGraphContainer.h"
#include "test_map.h"

using namespace lanelet;
using namespace lanelet::map_learning;
using namespace lanelet::map_learning::tests;

class MapGraphContainerTest : public MapGraphTest {
 public:
  MapGraphContainerTest() {
    std::vector<MapGraphConstPtr> graphs{testData.vehicleGraph, testData.pedestrianGraph};
    container = std::make_unique<MapGraphContainer>(graphs);
  }

  MapGraphContainerUPtr container;
};

TEST_F(MapGraphContainerTest, ConflictingInGraph) {  // NOLINT
  ConstLanelet pedestrianLanelet{*laneletMap->laneletLayer.find(2031)};
  ConstLanelets conflictingVehicle{container->conflictingInGraph(pedestrianLanelet, 0)};
  ASSERT_EQ(conflictingVehicle.size(), 1ul);
  EXPECT_EQ(conflictingVehicle[0], *laneletMap->laneletLayer.find(2020));

  ConstLanelets conflictingPedestrian{container->conflictingInGraph(pedestrianLanelet, 1)};
  EXPECT_EQ(conflictingPedestrian.size(), 0ul);
}

TEST_F(MapGraphContainerTest, ConflictingInGraphs) {  // NOLINT
  ConstLanelet pedestrianLanelet{*laneletMap->laneletLayer.find(2031)};
  MapGraphContainer::ConflictingInGraphs result{container->conflictingInGraphs(pedestrianLanelet)};
  ASSERT_EQ(result.size(), 2ul);

  ConstLanelets conflictingVehicle{result[0].second};
  ASSERT_EQ(conflictingVehicle.size(), 1ul);
  EXPECT_EQ(conflictingVehicle[0], *laneletMap->laneletLayer.find(2020));

  ConstLanelets conflictingPedestrian{result[1].second};
  EXPECT_EQ(conflictingPedestrian.size(), 0ul);
}

TEST_F(MapGraphContainerTest, ConflictingInGraph3dFits) {  // NOLINT
  ConstLanelet bridgeLanelet{*laneletMap->laneletLayer.find(2032)};
  ConstLanelets conflictingVehicle{container->conflictingInGraph(bridgeLanelet, 0, 2.)};
  EXPECT_EQ(conflictingVehicle.size(), 0ul);

  conflictingVehicle = container->conflictingInGraph(lanelets.find(2005)->second, 0, 2.);
  ASSERT_EQ(conflictingVehicle.size(), 1ul);
  EXPECT_TRUE(conflictingVehicle.front() == lanelets.find(2006)->second);

  conflictingVehicle = container->conflictingInGraph(lanelets.find(2006)->second, 0, 2.);
  ASSERT_EQ(conflictingVehicle.size(), 1ul);
  EXPECT_TRUE(conflictingVehicle.front() == lanelets.find(2005)->second);

  ConstLanelets conflictingPedestrian{container->conflictingInGraph(bridgeLanelet, 1, 2.)};
  EXPECT_EQ(conflictingPedestrian.size(), 0ul);
}

TEST_F(MapGraphContainerTest, ConflictingInGraph3dDoesntFit) {  // NOLINT
  ConstLanelet bridgeLanelet{*laneletMap->laneletLayer.find(2032)};
  ConstLanelets conflictingVehicle{container->conflictingInGraph(bridgeLanelet, 0, 4.)};
  EXPECT_EQ(conflictingVehicle.size(), 2ul);

  conflictingVehicle = container->conflictingInGraph(lanelets.find(2005)->second, 0, 4.);
  EXPECT_EQ(conflictingVehicle.size(), 2ul);

  conflictingVehicle = container->conflictingInGraph(lanelets.find(2006)->second, 0, 4.);
  EXPECT_EQ(conflictingVehicle.size(), 2ul);

  ConstLanelets conflictingPedestrian{container->conflictingInGraph(bridgeLanelet, 1, 4.)};
  EXPECT_EQ(conflictingPedestrian.size(), 0ul);
}
