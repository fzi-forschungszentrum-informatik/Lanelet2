#include <gtest/gtest.h>

#include "lanelet2_map_learning/MapFeatures.h"
#include "test_map.h"

using namespace lanelet;
using namespace lanelet::map_learning;
using namespace lanelet::map_learning::tests;

TEST_F(MapLearningTest, LaneLineStringFeature) {  // NOLINT
  BasicLineString3d polyline{BasicPoint3d{0, 0, 0}, BasicPoint3d{5, 0, 0}, BasicPoint3d{10, 0, 0},
                             BasicPoint3d{20, 0, 0}};
  LaneLineStringFeature feat(polyline, Id(123), LineStringType::Solid, Id(1234));

  feat.process(bbox, ParametrizationType::LineString, 4);
  EXPECT_EQ(feat.cutAndResampledFeature().size(), 4);
  EXPECT_NEAR(feat.cutAndResampledFeature()[0].x(), 0, 10e-5);
  EXPECT_NEAR(feat.cutAndResampledFeature()[1].x(), 5, 10e-5);
  EXPECT_NEAR(feat.cutAndResampledFeature()[3].x(), 15, 10e-5);

  Eigen::VectorXd vec = feat.computeFeatureVector(false, true);
  EXPECT_EQ(vec.size(), 9);
  EXPECT_NEAR(vec[0], 0, 10e-5);
  EXPECT_NEAR(vec[2], 5, 10e-5);
  EXPECT_NEAR(vec[6], 15, 10e-5);
  EXPECT_EQ(vec[8], 2);

  Eigen::MatrixXd pointMat = feat.pointMatrix(false);
  EXPECT_EQ(pointMat.rows(), 4);
  EXPECT_EQ(pointMat.cols(), 3);
  EXPECT_NEAR(pointMat(0, 0), 0, 10e-5);
  EXPECT_NEAR(pointMat(2, 0), 10, 10e-5);
  EXPECT_NEAR(pointMat(3, 0), 15, 10e-5);
}

TEST_F(MapLearningTest, LaneletFeature) {  // NOLINT
  BasicLineString3d leftBd{BasicPoint3d{0, 0, 0}, BasicPoint3d{5, 0, 0}, BasicPoint3d{10, 0, 0},
                           BasicPoint3d{20, 0, 0}};
  LaneLineStringFeaturePtr leftBdFeat =
      std::make_shared<LaneLineStringFeature>(leftBd, Id(123), LineStringType::Solid, Id(1234));
  BasicLineString3d rightBd{BasicPoint3d{0, 4, 0}, BasicPoint3d{5, 4, 0}, BasicPoint3d{10, 4, 0},
                            BasicPoint3d{20, 4, 0}};
  LaneLineStringFeaturePtr rightBdFeat =
      std::make_shared<LaneLineStringFeature>(rightBd, Id(124), LineStringType::Dashed, Id(1234));
  BasicLineString3d centerline{BasicPoint3d{0, 2, 0}, BasicPoint3d{5, 2, 0}, BasicPoint3d{10, 2, 0},
                               BasicPoint3d{20, 2, 0}};
  LaneLineStringFeaturePtr centerlineFeat =
      std::make_shared<LaneLineStringFeature>(centerline, Id(125), LineStringType::Centerline, Id(1234));

  LaneletFeature llFeat(leftBdFeat, rightBdFeat, centerlineFeat, Id(1234));
  llFeat.setReprType(LaneletRepresentationType::Boundaries);

  llFeat.process(bbox, ParametrizationType::LineString, 4);
  EXPECT_EQ(llFeat.centerline()->cutAndResampledFeature().size(), 4);
  EXPECT_NEAR(llFeat.centerline()->cutAndResampledFeature()[0].x(), 0, 10e-5);
  EXPECT_NEAR(llFeat.centerline()->cutAndResampledFeature()[1].x(), 5, 10e-5);
  EXPECT_NEAR(llFeat.centerline()->cutAndResampledFeature()[3].x(), 15, 10e-5);

  Eigen::VectorXd vec = llFeat.computeFeatureVector(false, true);

  EXPECT_EQ(vec.size(), 18);
  EXPECT_NEAR(vec[8], 0, 10e-5);
  EXPECT_NEAR(vec[10], 5, 10e-5);
  EXPECT_NEAR(vec[14], 15, 10e-5);
  EXPECT_EQ(vec[17], 1);
}

TEST_F(MapLearningTest, CompoundLaneLineStringFeature) {  // NOLINT
  BasicLineString3d p1{BasicPoint3d{-10, 0, 0}, BasicPoint3d{-5, 0, 0}};
  LaneLineStringFeaturePtr feat1 =
      std::make_shared<LaneLineStringFeature>(p1, Id(123), LineStringType::Solid, Id(1234));
  BasicLineString3d p2{BasicPoint3d{-5, 0, 0}, BasicPoint3d{0, 0, 0}};
  LaneLineStringFeaturePtr feat2 =
      std::make_shared<LaneLineStringFeature>(p2, Id(123), LineStringType::Solid, Id(1234));
  BasicLineString3d p3{BasicPoint3d{0, 0, 0}, BasicPoint3d{5, 0, 0}};
  LaneLineStringFeaturePtr feat3 =
      std::make_shared<LaneLineStringFeature>(p3, Id(123), LineStringType::Solid, Id(1234));
  BasicLineString3d p4{BasicPoint3d{5, 0, 0}, BasicPoint3d{10, 0, 0}};
  LaneLineStringFeaturePtr feat4 =
      std::make_shared<LaneLineStringFeature>(p4, Id(124), LineStringType::Solid, Id(1234));
  BasicLineString3d p5{BasicPoint3d{10, 0, 0}, BasicPoint3d{20, 0, 0}};
  LaneLineStringFeaturePtr feat5 =
      std::make_shared<LaneLineStringFeature>(p5, Id(125), LineStringType::Solid, Id(1234));

  CompoundLaneLineStringFeature cpdFeat(LaneLineStringFeatureList{feat1, feat2, feat3, feat4, feat5},
                                        LineStringType::Solid);

  cpdFeat.process(bbox, ParametrizationType::LineString, 5);
  EXPECT_EQ(cpdFeat.cutAndResampledFeature().size(), 5);
  EXPECT_NEAR(cpdFeat.cutAndResampledFeature()[0].x(), -5, 10e-5);
  EXPECT_NEAR(cpdFeat.cutAndResampledFeature()[1].x(), 0, 10e-5);
  EXPECT_NEAR(cpdFeat.cutAndResampledFeature()[3].x(), 10, 10e-5);

  EXPECT_NEAR(cpdFeat.pathLengthsProcessed().front(), 0, 10e-5);
  EXPECT_EQ(cpdFeat.processedFeaturesValid().front(), false);
  double cpdFeatLength =
      boost::geometry::length(cpdFeat.cutFeature(), boost::geometry::strategy::distance::pythagoras<double>());
  EXPECT_NEAR(cpdFeatLength, cpdFeat.pathLengthsProcessed().back(), 10e-5);

  Eigen::VectorXd vec = cpdFeat.computeFeatureVector(false, true);
  EXPECT_EQ(vec.size(), 11);
  EXPECT_NEAR(vec[0], -5, 10e-5);
  EXPECT_NEAR(vec[2], 0, 10e-5);
  EXPECT_NEAR(vec[6], 10, 10e-5);
  EXPECT_EQ(vec[10], 2);

  Eigen::MatrixXd pointMat = cpdFeat.pointMatrix(false);
  EXPECT_EQ(pointMat.rows(), 5);
  EXPECT_EQ(pointMat.cols(), 3);
  EXPECT_NEAR(pointMat(0, 0), -5, 10e-5);
  EXPECT_NEAR(pointMat(2, 0), 5, 10e-5);
  EXPECT_NEAR(pointMat(3, 0), 10, 10e-5);
}