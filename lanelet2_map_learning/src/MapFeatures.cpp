#include "lanelet2_map_learning/MapFeatures.h"

#include <boost/geometry.hpp>
#include <type_traits>

#include "lanelet2_map_learning/Utils.h"

namespace lanelet {
namespace map_learning {

struct LStringProcessResult {
  BasicLineString3d cutFeature_;
  BasicLineString3d cutAndResampledFeature_;
  bool wasCut_{false};
  bool valid_{true};
};

LStringProcessResult processLineStringImpl(const BasicLineString3d& lstring, const OrientedRect& bbox,
                                           const ParametrizationType& paramType, int32_t nPoints) {
  LStringProcessResult result;
  if (paramType != ParametrizationType::LineString) {
    throw std::runtime_error("Only polyline parametrization is implemented so far!");
  }
  result.cutFeature_ = cutLineString(bbox, lstring);
  if (result.cutFeature_.empty()) {
    result.wasCut_ = true;
    result.valid_ = false;
    return result;
  }
  result.cutAndResampledFeature_ = resampleLineString(result.cutFeature_, nPoints);

  double lengthOriginal = boost::geometry::length(lstring, boost::geometry::strategy::distance::pythagoras<double>());
  double lengthProcessed = boost::geometry::length(result.cutAndResampledFeature_,
                                                   boost::geometry::strategy::distance::pythagoras<double>());

  assert(lengthOriginal - lengthProcessed > -1e-2);
  if (lengthOriginal - lengthProcessed > 1e-2) {
    result.wasCut_ = true;
  }
  return result;
}

bool LaneLineStringFeature::process(const OrientedRect& bbox, const ParametrizationType& paramType, int32_t nPoints) {
  LStringProcessResult result = processLineStringImpl(rawFeature_, bbox, paramType, nPoints);
  if (result.valid_) {
    cutFeature_ = result.cutFeature_;
    cutAndResampledFeature_ = result.cutAndResampledFeature_;
  } else {
    valid_ = result.valid_;
  }
  wasCut_ = result.wasCut_;
  return result.valid_;
}

Eigen::VectorXd LaneLineStringFeature::computeFeatureVector(bool onlyPoints, bool pointsIn2d) const {
  const BasicLineString3d& selectedFeature =
      (cutAndResampledFeature_.size() > 0) ? cutAndResampledFeature_ : rawFeature_;
  Eigen::VectorXd vec = pointsIn2d ? Eigen::VectorXd(2 * selectedFeature.size() + 1)
                                   : Eigen::VectorXd(3 * selectedFeature.size() + 1);  // n points with 2/3 dims + type
  if (pointsIn2d == true) {
    for (size_t i = 0; i < selectedFeature.size(); i++) {
      vec(Eigen::seq(2 * i, 2 * i + 1)) = selectedFeature[i](Eigen::seq(0, 1));
    }
  } else {
    for (size_t i = 0; i < selectedFeature.size(); i++) {
      vec(Eigen::seq(3 * i, 3 * i + 2)) = selectedFeature[i](Eigen::seq(0, 2));
    }
  }

  vec[vec.size() - 1] = typeInt();
  if (onlyPoints) {
    return vec(Eigen::seq(0, vec.size() - 2));
  } else {
    return vec;
  }
}

Eigen::VectorXd TEFeature::computeFeatureVector(bool onlyPoints, bool pointsIn2d) const {
  Eigen::VectorXd vec = pointsIn2d ? Eigen::VectorXd(2 * rawFeature_.size() + 1)
                                   : Eigen::VectorXd(3 * rawFeature_.size() + 1);  // n points with 2/3 dims + type
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
    return vec(Eigen::seq(0, vec.size() - 2));
  } else {
    return vec;
  }
}

Eigen::MatrixXd LaneLineStringFeature::pointMatrix(bool pointsIn2d) const {
  const BasicLineString3d& selectedFeature =
      (cutAndResampledFeature_.size() > 0) ? cutAndResampledFeature_ : rawFeature_;
  Eigen::MatrixXd mat = pointsIn2d ? Eigen::MatrixXd(selectedFeature.size(), 2)
                                   : Eigen::MatrixXd(selectedFeature.size(), 3);  // n points with 2/3 dims + type
  if (pointsIn2d == true) {
    for (size_t i = 0; i < selectedFeature.size(); i++) {
      mat.row(i) = selectedFeature[i](Eigen::seq(0, 1));
    }
  } else {
    for (size_t i = 0; i < selectedFeature.size(); i++) {
      mat.row(i) = selectedFeature[i](Eigen::seq(0, 2));
    }
  }
  return mat;
}

Eigen::MatrixXd TEFeature::pointMatrix(bool pointsIn2d) const {
  Eigen::MatrixXd mat = pointsIn2d ? Eigen::MatrixXd(rawFeature_.size(), 2)
                                   : Eigen::MatrixXd(rawFeature_.size(), 3);  // n points with 2/3 dims + type
  if (pointsIn2d == true) {
    for (size_t i = 0; i < rawFeature_.size(); i++) {
      mat.row(i) = rawFeature_[i](Eigen::seq(0, 1));
    }
  } else {
    for (size_t i = 0; i < rawFeature_.size(); i++) {
      mat.row(i) = rawFeature_[i](Eigen::seq(0, 2));
    }
  }
  return mat;
}

LaneletFeature::LaneletFeature(const LaneLineStringFeature& leftBoundary, const LaneLineStringFeature& rightBoundary,
                               const LaneLineStringFeature& centerline, Id mapID)
    : MapFeature(mapID), leftBoundary_{leftBoundary}, rightBoundary_{rightBoundary}, centerline_{centerline} {}

LaneletFeature::LaneletFeature(const ConstLanelet& ll)
    : leftBoundary_{LaneLineStringFeature(ll.leftBound3d().basicLineString(), ll.leftBound3d().id(),
                                          bdSubtypeToEnum(ll.leftBound3d()))},
      rightBoundary_{LaneLineStringFeature(ll.rightBound3d().basicLineString(), ll.rightBound3d().id(),
                                           bdSubtypeToEnum(ll.rightBound3d()))},
      centerline_{LaneLineStringFeature(ll.centerline3d().basicLineString(), ll.centerline3d().id(),
                                        LineStringType::Centerline)} {}

bool LaneletFeature::process(const OrientedRect& bbox, const ParametrizationType& paramType, int32_t nPoints) {
  leftBoundary_.process(bbox, paramType, nPoints);
  rightBoundary_.process(bbox, paramType, nPoints);
  centerline_.process(bbox, paramType, nPoints);

  if (leftBoundary_.wasCut() || rightBoundary_.wasCut() || centerline_.wasCut()) {
    wasCut_ = true;
  }
  if (!leftBoundary_.valid() || !rightBoundary_.valid() || !centerline_.valid()) {
    valid_ = false;
  }
  return valid_;
}

Eigen::VectorXd LaneletFeature::computeFeatureVector(bool onlyPoints, bool pointsIn2d) const {
  if (!reprType_.has_value()) {
    throw std::runtime_error(
        "You need to set a LaneletRepresentationType with setReprType() before computing the feature vector!");
  } else if (*reprType_ == LaneletRepresentationType::Centerline) {
    Eigen::VectorXd vecCenterlinePts = centerline_.computeFeatureVector(true, pointsIn2d);
    Eigen::VectorXd vec(vecCenterlinePts.size() + 2);  // pts vec + left and right type
    vec(Eigen::seq(0, vecCenterlinePts.size() - 1)) = vecCenterlinePts;
    vec[vec.size() - 2] = leftBoundary_.typeInt();
    vec[vec.size() - 1] = rightBoundary_.typeInt();
    if (onlyPoints) {
      return vec(Eigen::seq(0, vec.size() - 2));
    } else {
      return vec;
    }
  } else if (*reprType_ == LaneletRepresentationType::Boundaries) {
    Eigen::VectorXd vecLeftBdPts = leftBoundary_.computeFeatureVector(true, pointsIn2d);
    Eigen::VectorXd vecRightBdPts = rightBoundary_.computeFeatureVector(true, pointsIn2d);

    Eigen::VectorXd vec(vecLeftBdPts.size() + vecRightBdPts.size() + 2);  // pts vec + left and right type
    vec(Eigen::seq(0, vecLeftBdPts.size() - 1)) = vecLeftBdPts;
    vec(Eigen::seq(vecLeftBdPts.size(), vecLeftBdPts.size() + vecRightBdPts.size() - 1)) = vecRightBdPts;
    vec[vec.size() - 2] = leftBoundary_.typeInt();
    vec[vec.size() - 1] = rightBoundary_.typeInt();
    if (onlyPoints) {
      return vec(Eigen::seq(0, vec.size() - 2));
    } else {
      return vec;
    }
  } else {
    throw std::runtime_error("Unknown LaneletRepresentationType!");
    return Eigen::VectorXd();
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
    assert(features[i].rawFeature().size() > 1);
    if (i == features.size() - 1) {
      rawFeature_.insert(rawFeature_.end(), features[i].rawFeature().begin(), features[i].rawFeature().end());
    } else {
      rawFeature_.insert(rawFeature_.end(), features[i].rawFeature().begin(), features[i].rawFeature().end() - 1);
    }

    double rawLength =
        boost::geometry::length(features[i].rawFeature(), boost::geometry::strategy::distance::pythagoras<double>());
    if (i > 0) {
      pathLengthsRaw_[i] = pathLengthsRaw_[i - 1] + rawLength;
    } else {
      pathLengthsRaw_[i] = rawLength;
    }
  }
}

bool CompoundLaneLineStringFeature::process(const OrientedRect& bbox, const ParametrizationType& paramType,
                                            int32_t nPoints) {
  bool valid = LaneLineStringFeature::process(bbox, paramType, nPoints);
  for (size_t i = 0; i < individualFeatures_.size(); i++) {
    individualFeatures_[i].process(bbox, paramType, nPoints);
    double processedLength = boost::geometry::length(individualFeatures_[i].cutFeature(),
                                                     boost::geometry::strategy::distance::pythagoras<double>());

    if (processedLength > validLengthThresh_) {
      processedFeaturesValid_[i] = true;
    }

    if (i > 0) {
      pathLengthsProcessed_[i] = pathLengthsProcessed_[i - 1] + processedLength;
    } else {
      pathLengthsProcessed_[i] = processedLength;
    }
  }
  return valid;
}

}  // namespace map_learning
}  // namespace lanelet