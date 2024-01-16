#include "lanelet2_map_learning/MapFeatures.h"

#include <boost/geometry.hpp>
#include <type_traits>

#include "lanelet2_map_learning/Utils.h"

namespace lanelet {
namespace map_learning {

struct LStringProcessResult {
  BasicLineStrings3d cutFeatures;
  BasicLineStrings3d cutAndResampledFeatures;
  BasicLineStrings3d cutResampledAndTransformedFeatures;
  bool wasCut_{false};
  bool valid_{true};
};

LStringProcessResult processLineStringImpl(const BasicLineString3d& lstring, const OrientedRect& bbox,
                                           const ParametrizationType& paramType, int32_t nPoints) {
  LStringProcessResult result;
  if (paramType != ParametrizationType::LineString) {
    throw std::runtime_error("Only polyline parametrization is implemented so far!");
  }
  std::vector<BasicLineString3d> cutLines = cutLineString(bbox, lstring);
  if (cutLines.empty()) {
    result.wasCut_ = true;
    result.valid_ = false;
    return result;
  }

  for (const auto& line : cutLines) {
    result.cutFeatures.push_back(line);
    BasicLineString3d lineResampled = resampleLineString(line, nPoints);
    result.cutAndResampledFeatures.push_back(lineResampled);
    result.cutResampledAndTransformedFeatures.push_back(transformLineString(bbox, lineResampled));
  }

  double lengthOriginal = boost::geometry::length(lstring, boost::geometry::strategy::distance::pythagoras<double>());
  double lengthProcessed = 0;
  for (const auto& line : result.cutResampledAndTransformedFeatures) {
    lengthProcessed =
        lengthProcessed + boost::geometry::length(line, boost::geometry::strategy::distance::pythagoras<double>());
  }

  if (lengthOriginal - lengthProcessed > 1e-2) {
    result.wasCut_ = true;
  }
  return result;
}

bool LaneLineStringFeature::process(const OrientedRect& bbox, const ParametrizationType& paramType, int32_t nPoints) {
  LStringProcessResult result = processLineStringImpl(rawFeature_, bbox, paramType, nPoints);
  if (result.valid_) {
    cutFeatures_ = result.cutFeatures;
    cutAndResampledFeatures_ = result.cutAndResampledFeatures;
    cutResampledAndTransformedFeatures_ = result.cutResampledAndTransformedFeatures;
  } else {
    valid_ = result.valid_;
  }
  wasCut_ = result.wasCut_;
  return result.valid_;
}

std::vector<VectorXd> LaneLineStringFeature::computeFeatureVectors(bool onlyPoints, bool pointsIn2d) const {
  std::vector<VectorXd> featVecs;
  for (const auto& split : cutResampledAndTransformedFeatures_) {
    featVecs.push_back(toFeatureVector(split, typeInt(), onlyPoints, pointsIn2d));
  }
  return featVecs;
}

std::vector<VectorXd> TEFeature::computeFeatureVectors(bool onlyPoints, bool pointsIn2d) const {
  VectorXd vec = pointsIn2d ? VectorXd(2 * rawFeature_.size() + 1)
                            : VectorXd(3 * rawFeature_.size() + 1);  // n points with 2/3 dims + type
  if (pointsIn2d == true) {
    for (size_t i = 0; i < rawFeature_.size(); i++) {
      vec(Eigen::seq(2 * i, 2 * i + 1)) = rawFeature_[i](Eigen::seq(0, 1));
    }
  } else {
    for (size_t i = 0; i < rawFeature_.size(); i++) {
      vec(Eigen::seq(3 * i, 3 * i + 2)) = rawFeature_[i](Eigen::seq(0, 2));
    }
  }
  vec[vec.size() - 1] = static_cast<int>(teType_);
  if (onlyPoints) {
    return std::vector<VectorXd>{vec(Eigen::seq(0, vec.size() - 2))};
  } else {
    return std::vector<VectorXd>{vec};
  }
}

std::vector<MatrixXd> LaneLineStringFeature::pointMatrices(bool pointsIn2d) const {
  std::vector<MatrixXd> pointMatrices;
  for (const auto& split : cutResampledAndTransformedFeatures_) {
    pointMatrices.push_back(toPointMatrix(split, pointsIn2d));
  }
  return pointMatrices;
}

std::vector<MatrixXd> TEFeature::pointMatrices(bool pointsIn2d) const {
  return std::vector<MatrixXd>{toPointMatrix(rawFeature_, pointsIn2d)};
}

LaneletFeature::LaneletFeature(LaneLineStringFeaturePtr leftBoundary, LaneLineStringFeaturePtr rightBoundary,
                               LaneLineStringFeaturePtr centerline, Id mapID)
    : MapFeature(mapID), leftBoundary_{leftBoundary}, rightBoundary_{rightBoundary}, centerline_{centerline} {}

LaneletFeature::LaneletFeature(const ConstLanelet& ll)
    : leftBoundary_{std::make_shared<LaneLineStringFeature>(ll.leftBound3d().basicLineString(), ll.leftBound3d().id(),
                                                            bdTypeToEnum(ll.leftBound3d()), Ids{ll.id()},
                                                            ll.leftBound3d().inverted())},
      rightBoundary_{std::make_shared<LaneLineStringFeature>(ll.rightBound3d().basicLineString(),
                                                             ll.rightBound3d().id(), bdTypeToEnum(ll.rightBound3d()),
                                                             Ids{ll.id()}, ll.rightBound3d().inverted())},
      centerline_{std::make_shared<LaneLineStringFeature>(ll.centerline3d().basicLineString(), ll.centerline3d().id(),
                                                          LineStringType::Centerline, Ids{ll.id()}, false)} {}

bool LaneletFeature::process(const OrientedRect& bbox, const ParametrizationType& paramType, int32_t nPoints) {
  leftBoundary_->process(bbox, paramType, nPoints);
  rightBoundary_->process(bbox, paramType, nPoints);
  centerline_->process(bbox, paramType, nPoints);

  if (leftBoundary_->wasCut() || rightBoundary_->wasCut() || centerline_->wasCut()) {
    wasCut_ = true;
  }
  if (!leftBoundary_->valid() || !rightBoundary_->valid() || !centerline_->valid()) {
    valid_ = false;
  }
  return valid_;
}

VectorXd stackVector(const std::vector<VectorXd>& vec) {
  size_t flattenedLength = 0;
  for (const auto& el : vec) {
    flattenedLength = flattenedLength + el.size();
  }
  VectorXd stacked(flattenedLength);
  size_t currIndex = 0;
  for (const auto& el : vec) {
    stacked(Eigen::seq(currIndex, currIndex + el.size())) = el;
  }
  return stacked;
}

std::vector<VectorXd> LaneletFeature::computeFeatureVectors(bool onlyPoints, bool pointsIn2d) const {
  if (reprType_ == LaneletRepresentationType::Centerline) {
    std::vector<VectorXd> vecCenterlinePts = centerline_->computeFeatureVectors(true, pointsIn2d);
    if (onlyPoints) {
      return vecCenterlinePts;
    }
    std::vector<VectorXd> featureVecs(vecCenterlinePts.size());
    for (size_t i = 0; i < vecCenterlinePts.size(); i++) {
      VectorXd vec(vecCenterlinePts[i].size() + 2);  // pts vec + left and right type
      vec(Eigen::seq(0, vecCenterlinePts.size() - 1)) = vecCenterlinePts[i];
      vec[vec.size() - 2] = leftBoundary_->typeInt();
      vec[vec.size() - 1] = rightBoundary_->typeInt();
      featureVecs[i] = vec;
    }
    return featureVecs;
  } else if (reprType_ == LaneletRepresentationType::Boundaries) {
    std::vector<VectorXd> vecLeftBdPts = leftBoundary_->computeFeatureVectors(true, pointsIn2d);
    VectorXd leftBdPts = stackVector(vecLeftBdPts);
    std::vector<VectorXd> vecRightBdPts = rightBoundary_->computeFeatureVectors(true, pointsIn2d);
    VectorXd rightBdPts = stackVector(vecRightBdPts);

    VectorXd vec(leftBdPts.size() + rightBdPts.size() + 2);  // pts vec + left and right type
    vec(Eigen::seq(0, leftBdPts.size() - 1)) = leftBdPts;
    vec(Eigen::seq(leftBdPts.size(), leftBdPts.size() + rightBdPts.size() - 1)) = rightBdPts;
    vec[vec.size() - 2] = leftBoundary_->typeInt();
    vec[vec.size() - 1] = rightBoundary_->typeInt();

    if (onlyPoints) {
      return std::vector<VectorXd>{vec(Eigen::seq(0, vec.size() - 2))};
    } else {
      return std::vector<VectorXd>{vec};
    }
  } else {
    throw std::runtime_error("Unknown LaneletRepresentationType!");
    return std::vector<VectorXd>();
  }
}

CompoundLaneLineStringFeature::CompoundLaneLineStringFeature(const LaneLineStringFeatureList& features,
                                                             LineStringType compoundType)
    : individualFeatures_{features},
      pathLengthsRaw_{std::vector<double>(features.size())},
      pathLengthsProcessed_{std::vector<double>(features.size())},
      processedFeaturesValid_{std::vector<bool>(features.size())} {
  type_ = compoundType;

  for (size_t i = 0; i < features.size(); i++) {
    if (features[i]->rawFeature().empty()) {
      throw std::runtime_error("Feature with empty rawFeature() supplied!");
    }
    // checking if the linestring is correctly
    if (i == features.size() - 1) {
      rawFeature_.insert(rawFeature_.end(), features[i]->rawFeature().begin(), features[i]->rawFeature().end());
    } else {
      rawFeature_.insert(rawFeature_.end(), features[i]->rawFeature().begin(), features[i]->rawFeature().end() - 1);
    }

    double rawLength =
        boost::geometry::length(features[i]->rawFeature(), boost::geometry::strategy::distance::pythagoras<double>());
    if (i > 0) {
      pathLengthsRaw_[i] = pathLengthsRaw_[i - 1] + rawLength;
    } else {
      pathLengthsRaw_[i] = rawLength;
    }
  }
}

bool CompoundLaneLineStringFeature::process(const OrientedRect& bbox, const ParametrizationType& paramType,
                                            int32_t nPoints) {
  bool valid = false;
  LaneLineStringFeature::process(bbox, paramType, nPoints);
  for (size_t i = 0; i < individualFeatures_.size(); i++) {
    individualFeatures_[i]->process(bbox, paramType, nPoints);

    double processedLength = 0;
    for (const auto& line : individualFeatures_[i]->cutFeature()) {
      processedLength =
          processedLength + boost::geometry::length(line, boost::geometry::strategy::distance::pythagoras<double>());
    }

    if (processedLength > validLengthThresh_) {
      processedFeaturesValid_[i] = true;
      valid = true;
    }

    if (i > 0) {
      pathLengthsProcessed_[i] = pathLengthsProcessed_[i - 1] + processedLength;
    } else {
      pathLengthsProcessed_[i] = processedLength;
    }
  }
  return valid;
}

MatrixXd toPointMatrix(const BasicLineString3d& lString, bool pointsIn2d) {
  MatrixXd mat =
      pointsIn2d ? MatrixXd(lString.size(), 2) : MatrixXd(lString.size(), 3);  // n points with 2/3 dims + type
  if (pointsIn2d == true) {
    for (size_t i = 0; i < lString.size(); i++) {
      mat.row(i) = lString[i](Eigen::seq(0, 1));
    }
  } else {
    for (size_t i = 0; i < lString.size(); i++) {
      mat.row(i) = lString[i](Eigen::seq(0, 2));
    }
  }
  return mat;
}

VectorXd toFeatureVector(const BasicLineString3d& line, int typeInt, bool onlyPoints, bool pointsIn2d) {
  VectorXd vec =
      pointsIn2d ? VectorXd(2 * line.size() + 1) : VectorXd(3 * line.size() + 1);  // n points with 2/3 dims + type
  if (pointsIn2d == true) {
    for (size_t i = 0; i < line.size(); i++) {
      vec(Eigen::seq(2 * i, 2 * i + 1)) = line[i](Eigen::seq(0, 1));
    }
  } else {
    for (size_t i = 0; i < line.size(); i++) {
      vec(Eigen::seq(3 * i, 3 * i + 2)) = line[i](Eigen::seq(0, 2));
    }
  }
  vec[vec.size() - 1] = typeInt;
  if (onlyPoints) {
    return vec(Eigen::seq(0, vec.size() - 2));
  } else {
    return vec;
  }
}

}  // namespace map_learning
}  // namespace lanelet