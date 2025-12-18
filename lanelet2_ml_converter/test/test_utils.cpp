#include <gtest/gtest.h>

#include "lanelet2_ml_converter/Utils.h"
#include "test_map.h"

using namespace lanelet;
using namespace lanelet::ml_converter;
using namespace lanelet::ml_converter::tests;

TEST_F(MLConverterTest, GetRotatedRect) {  // NOLINT
  EXPECT_DOUBLE_EQ(bbox.bounds_const()[0].x(), 15);
  EXPECT_DOUBLE_EQ(bbox.bounds_const()[0].y(), -10);
  EXPECT_DOUBLE_EQ(bbox.bounds_const()[1].x(), -5);
  EXPECT_DOUBLE_EQ(bbox.bounds_const()[1].y(), -10);
  EXPECT_DOUBLE_EQ(bbox.bounds_const()[2].x(), -5);
  EXPECT_DOUBLE_EQ(bbox.bounds_const()[2].y(), 20);
  EXPECT_DOUBLE_EQ(bbox.bounds_const()[3].x(), 15);
  EXPECT_DOUBLE_EQ(bbox.bounds_const()[3].y(), 20);
}

TEST_F(MLConverterTest, ExtractSubmap) {  // NOLINT
  LaneletSubmapConstPtr submap =
      extractSubmap(laneletMap, centerBbox.head(2), extentLongitudinalBbox, extentLateralBbox);
  EXPECT_TRUE(submap->laneletLayer.exists(2000));
  EXPECT_TRUE(submap->laneletLayer.exists(2001));
  EXPECT_TRUE(submap->laneletLayer.exists(2002));
  EXPECT_TRUE(submap->laneletLayer.exists(2003));
  EXPECT_TRUE(submap->laneletLayer.exists(2004));
  EXPECT_TRUE(submap->laneletLayer.exists(2005));
  EXPECT_TRUE(submap->laneletLayer.exists(2009));
  EXPECT_TRUE(submap->laneletLayer.exists(2010));
}

TEST(UtilsTest, ResampleLineString) {  // NOLINT
  BasicLineString3d polyline{BasicPoint3d{0, 0, 0}, BasicPoint3d{3, 0, 0}, BasicPoint3d{6, 0, 0},
                             BasicPoint3d{15, 0, 0}};
  BasicLineString3d polylineResampled = resampleLineString(polyline, 11);

  EXPECT_EQ(polylineResampled.size(), 11);
  EXPECT_NEAR(polylineResampled[3].x(), 4.5, 10e-5);
  EXPECT_NEAR(polylineResampled[6].x(), 9, 10e-5);
  EXPECT_NEAR(polylineResampled[10].x(), 15, 10e-5);
}

TEST_F(MLConverterTest, CutLineString) {  // NOLINT
  BasicLineString3d polyline{BasicPoint3d{0, 0, 0}, BasicPoint3d{30, 0, 0}, BasicPoint3d{40, 0, 0},
                             BasicPoint3d{50, 0, 0}};
  std::vector<BasicLineString3d> polylineCut = cutLineString(bbox, polyline);

  EXPECT_EQ(polylineCut.size(), 1);
  EXPECT_EQ(polylineCut[0].size(), 2);
  EXPECT_NEAR(polylineCut[0][1].x(), 15, 10e-5);
}