#include <gtest/gtest.h>
#include <future>
#include "geometry/LaneletMap.h"
#include "lanelet_map_test_case.h"

using namespace lanelet;

class LaneletMapGeometryTest : public ::testing::Test, public test_cases::LaneletMapTestCase {};

TEST_F(LaneletMapGeometryTest, findWithin2dPoint) {  // NOLINT
  map->add(ll2);
  EXPECT_EQ(2ul, map->laneletLayer.size());
  testConstAndNonConst([this](auto& map) {
    auto llts = geometry::findWithin2d(map->laneletLayer, Point2d(InvalId, 0.5, -1.5), 0.7);
    ASSERT_EQ(1ul, llts.size());
    EXPECT_EQ(ll2, llts.at(0).second);
  });
}

TEST_F(LaneletMapGeometryTest, findWithin2dLinestring) {  // NOLINT
  map->add(other);
  testConstAndNonConst([this](auto& map) {
    auto ls = geometry::findWithin2d(map->lineStringLayer, utils::to2D(outside), 1.7);
    ASSERT_EQ(4ul, ls.size());
    EXPECT_DOUBLE_EQ(1., ls[0].first);
    EXPECT_DOUBLE_EQ(1.5, ls.back().first);
  });
}

TEST_F(LaneletMapGeometryTest, findWithin2dLanelet) {  // NOLINT
  map->add(ll2);
  EXPECT_EQ(2ul, map->laneletLayer.size());
  testConstAndNonConst([this](auto& map) {
    auto llts = geometry::findWithin2d(map->pointLayer, this->ll2);
    ASSERT_LE(1ul, llts.size());
    utils::contains(utils::transform(llts, [](auto& t) { return t.second; }), this->p6);
  });
}

TEST_F(LaneletMapGeometryTest, findWithin3dPoint) {  // NOLINT
  map->add(ll2);
  EXPECT_EQ(2ul, map->laneletLayer.size());
  testConstAndNonConst([](auto& map) {
    auto llts = geometry::findWithin3d(map->laneletLayer, Point2d(InvalId, 0.5, -1.5));
    EXPECT_EQ(0ul, llts.size());
  });
}

TEST_F(LaneletMapGeometryTest, findWithin3dLinestring) {  // NOLINT
  map->add(other);
  testConstAndNonConst([this](auto& map) {
    auto ls = geometry::findWithin3d(map->lineStringLayer, outside, 1.7);
    ASSERT_EQ(4ul, ls.size());
    EXPECT_DOUBLE_EQ(1., ls[0].first);
    EXPECT_DOUBLE_EQ(1.5, ls.back().first);
  });
}

TEST_F(LaneletMapGeometryTest, findWithin3dLanelet) {  // NOLINT
  map->add(ll2);
  EXPECT_EQ(2ul, map->laneletLayer.size());
  testConstAndNonConst([this](auto& map) {
    auto llts = geometry::findWithin3d(map->pointLayer, this->ll2);
    ASSERT_LE(1ul, llts.size());
    utils::contains(utils::transform(llts, [](auto& t) { return t.second; }), this->p6);
  });
}
