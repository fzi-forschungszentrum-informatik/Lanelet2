#include <gtest/gtest.h>

#include "lanelet2_ml_converter/MapInstances.h"
#include "test_map.h"

using namespace lanelet;
using namespace lanelet::ml_converter;
using namespace lanelet::ml_converter::tests;

TEST_F(MLConverterTest, LaneLineStringInstance) {  // NOLINT
  BasicLineString3d polyline{BasicPoint3d{0, 0, 0}, BasicPoint3d{5, 0, 0}, BasicPoint3d{10, 0, 0},
                             BasicPoint3d{20, 0, 0}};
  LaneLineStringInstance feat(polyline, Id(123), LineStringType::Solid, Ids(1234), false);

  feat.process(bbox, ParametrizationType::LineString, 4);
  EXPECT_EQ(feat.cutAndResampledInstance().size(), 1);
  EXPECT_EQ(feat.cutAndResampledInstance()[0].size(), 4);
  EXPECT_NEAR(feat.cutAndResampledInstance()[0][0].x(), 0, 10e-5);
  EXPECT_NEAR(feat.cutAndResampledInstance()[0][1].x(), 5, 10e-5);
  EXPECT_NEAR(feat.cutAndResampledInstance()[0][3].x(), 15, 10e-5);

  std::vector<VectorXd> vec = feat.computeInstanceVectors(false, true);
  EXPECT_EQ(vec.size(), 1);
  EXPECT_EQ(vec[0].size(), 9);
  EXPECT_NEAR(vec[0][0], -5, 10e-5);
  EXPECT_NEAR(vec[0][1], 5, 10e-5);
  EXPECT_NEAR(vec[0][6], -5, 10e-5);
  EXPECT_NEAR(vec[0][7], -10, 10e-5);
  EXPECT_EQ(vec[0][8], 2);

  std::vector<MatrixXd> pointMat = feat.pointMatrices(false);
  EXPECT_EQ(pointMat[0].rows(), 4);
  EXPECT_EQ(pointMat[0].cols(), 2);
  EXPECT_NEAR(pointMat[0](0, 1), 5, 10e-5);
  EXPECT_NEAR(pointMat[0](2, 1), -5, 10e-5);
  EXPECT_NEAR(pointMat[0](3, 1), -10, 10e-5);
}

TEST_F(MLConverterTest, LaneletInstance) {  // NOLINT
  BasicLineString3d leftBd{BasicPoint3d{0, 0, 0}, BasicPoint3d{5, 0, 0}, BasicPoint3d{10, 0, 0},
                           BasicPoint3d{20, 0, 0}};
  LaneLineStringInstancePtr leftBdFeat =
      std::make_shared<LaneLineStringInstance>(leftBd, Id(123), LineStringType::Solid, Ids(1234), false);
  BasicLineString3d rightBd{BasicPoint3d{0, 4, 0}, BasicPoint3d{5, 4, 0}, BasicPoint3d{10, 4, 0},
                            BasicPoint3d{20, 4, 0}};
  LaneLineStringInstancePtr rightBdFeat =
      std::make_shared<LaneLineStringInstance>(rightBd, Id(124), LineStringType::Dashed, Ids(1234), false);
  BasicLineString3d centerline{BasicPoint3d{0, 2, 0}, BasicPoint3d{5, 2, 0}, BasicPoint3d{10, 2, 0},
                               BasicPoint3d{20, 2, 0}};
  LaneLineStringInstancePtr centerlineFeat =
      std::make_shared<LaneLineStringInstance>(centerline, Id(125), LineStringType::Centerline, Ids(1234), false);

  LaneletInstance llFeat(leftBdFeat, rightBdFeat, centerlineFeat, Id(1234));
  llFeat.setReprType(LaneletRepresentationType::Boundaries);

  llFeat.process(bbox, ParametrizationType::LineString, 4);
  EXPECT_EQ(llFeat.centerline()->cutAndResampledInstance()[0].size(), 4);
  EXPECT_NEAR(llFeat.centerline()->cutAndResampledInstance()[0][0].x(), 0, 10e-5);
  EXPECT_NEAR(llFeat.centerline()->cutAndResampledInstance()[0][1].x(), 5, 10e-5);
  EXPECT_NEAR(llFeat.centerline()->cutAndResampledInstance()[0][3].x(), 15, 10e-5);

  std::vector<VectorXd> vec = llFeat.computeInstanceVectors(false, true);
  EXPECT_EQ(vec.size(), 1);
  EXPECT_EQ(vec[0].size(), 18);
  EXPECT_NEAR(vec[0][9], 5, 10e-5);
  EXPECT_NEAR(vec[0][10], -1, 10e-5);
  EXPECT_NEAR(vec[0][15], -10, 10e-5);
  EXPECT_EQ(vec[0][17], 1);
}

TEST_F(MLConverterTest, CompoundLaneLineStringInstance) {  // NOLINT
  BasicLineString3d p1{BasicPoint3d{-10, 0, 0}, BasicPoint3d{-5, 0, 0}};
  LaneLineStringInstancePtr feat1 =
      std::make_shared<LaneLineStringInstance>(p1, Id(123), LineStringType::Solid, Ids(1234), false);
  BasicLineString3d p2{BasicPoint3d{-5, 0, 0}, BasicPoint3d{0, 0, 0}};
  LaneLineStringInstancePtr feat2 =
      std::make_shared<LaneLineStringInstance>(p2, Id(123), LineStringType::Solid, Ids(1234), false);
  BasicLineString3d p3{BasicPoint3d{0, 0, 0}, BasicPoint3d{5, 0, 0}};
  LaneLineStringInstancePtr feat3 =
      std::make_shared<LaneLineStringInstance>(p3, Id(123), LineStringType::Solid, Ids(1234), false);
  BasicLineString3d p4{BasicPoint3d{5, 0, 0}, BasicPoint3d{10, 0, 0}};
  LaneLineStringInstancePtr feat4 =
      std::make_shared<LaneLineStringInstance>(p4, Id(124), LineStringType::Solid, Ids(1234), false);
  BasicLineString3d p5{BasicPoint3d{10, 0, 0}, BasicPoint3d{20, 0, 0}};
  LaneLineStringInstancePtr feat5 =
      std::make_shared<LaneLineStringInstance>(p5, Id(125), LineStringType::Solid, Ids(1234), false);

  CompoundLaneLineStringInstance cpdFeat(LaneLineStringInstanceList{feat1, feat2, feat3, feat4, feat5},
                                         LineStringType::Solid);

  cpdFeat.process(bbox, ParametrizationType::LineString, 5);
  EXPECT_EQ(cpdFeat.cutAndResampledInstance().size(), 1);
  EXPECT_EQ(cpdFeat.cutAndResampledInstance()[0].size(), 5);
  EXPECT_NEAR(cpdFeat.cutAndResampledInstance()[0][0].x(), -5, 10e-5);
  EXPECT_NEAR(cpdFeat.cutAndResampledInstance()[0][1].x(), 0, 10e-5);
  EXPECT_NEAR(cpdFeat.cutAndResampledInstance()[0][3].x(), 10, 10e-5);

  EXPECT_NEAR(cpdFeat.pathLengthsProcessed().front(), 0, 10e-5);
  EXPECT_EQ(cpdFeat.processedInstancesValid().front(), false);
  double cpdFeatLength =
      boost::geometry::length(cpdFeat.cutInstance()[0], boost::geometry::strategy::distance::pythagoras<double>());
  EXPECT_NEAR(cpdFeatLength, cpdFeat.pathLengthsProcessed().back(), 10e-5);

  std::vector<VectorXd> vec = cpdFeat.computeInstanceVectors(false, true);
  EXPECT_EQ(vec.size(), 1);
  EXPECT_EQ(vec[0].size(), 11);
  EXPECT_NEAR(vec[0][1], 10, 10e-5);
  EXPECT_NEAR(vec[0][2], -5, 10e-5);
  EXPECT_NEAR(vec[0][3], 5, 10e-5);
  EXPECT_NEAR(vec[0][6], -5, 10e-5);
  EXPECT_NEAR(vec[0][7], -5, 10e-5);
  EXPECT_EQ(vec[0][10], 2);

  std::vector<MatrixXd> pointMat = cpdFeat.pointMatrices(false);
  EXPECT_EQ(pointMat.size(), 1);
  EXPECT_EQ(pointMat[0].rows(), 5);
  EXPECT_EQ(pointMat[0].cols(), 2);
  EXPECT_NEAR(pointMat[0](0, 1), 10, 10e-5);
  EXPECT_NEAR(pointMat[0](2, 1), 0, 10e-5);
  EXPECT_NEAR(pointMat[0](3, 1), -5, 10e-5);
  EXPECT_NEAR(pointMat[0](4, 0), -5, 10e-5);
}