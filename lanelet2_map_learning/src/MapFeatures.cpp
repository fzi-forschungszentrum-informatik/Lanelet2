#include "lanelet2_map_learning/MapFeatures.h"

#include <boost/geometry.hpp>
#include <type_traits>

#include "lanelet2_map_learning/Utils.h"

namespace lanelet {
namespace map_learning {

struct LStringProcessResult {
  BasicLineString3d cutFeature_;
  BasicLineString3d cutAndResampledFeature_;
  BasicLineString3d cutResampledAndTransformedFeature_;
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
  result.cutFeature_ = std::accumulate(cutLines.begin(), cutLines.end(), BasicLineString3d(),
                                       [](BasicLineString3d a, BasicLineString3d b) {
                                         a.insert(a.end(), b.begin(), b.end());
                                         return a;
                                       });
  if (result.cutFeature_.empty()) {
    result.wasCut_ = true;
    result.valid_ = false;
    return result;
  }
  result.cutAndResampledFeature_ = resampleLineString(result.cutFeature_, nPoints);
  result.cutResampledAndTransformedFeature_ = transformLineString(bbox, result.cutAndResampledFeature_);

  double lengthOriginal = boost::geometry::length(lstring, boost::geometry::strategy::distance::pythagoras<double>());
  double lengthProcessed = boost::geometry::length(result.cutAndResampledFeature_,
                                                   boost::geometry::strategy::distance::pythagoras<double>());

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
    cutResampledAndTransformedFeature_ = result.cutResampledAndTransformedFeature_;
  } else {
    valid_ = result.valid_;
  }
  wasCut_ = result.wasCut_;
  return result.valid_;
}

VectorXd LaneLineStringFeature::computeFeatureVector(bool onlyPoints, bool pointsIn2d) const {
  const BasicLineString3d& selectedFeature = cutResampledAndTransformedFeature_;
  VectorXd vec = pointsIn2d ? VectorXd(2 * selectedFeature.size() + 1)
                            : VectorXd(3 * selectedFeature.size() + 1);  // n points with 2/3 dims + type
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

VectorXd TEFeature::computeFeatureVector(bool onlyPoints, bool pointsIn2d) const {
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
    return vec(Eigen::seq(0, vec.size() - 2));
  } else {
    return vec;
  }
}

MatrixXd LaneLineStringFeature::pointMatrix(bool pointsIn2d) const {
  return toPointMatrix(cutResampledAndTransformedFeature_, pointsIn2d);
}

MatrixXd TEFeature::pointMatrix(bool pointsIn2d) const { return toPointMatrix(rawFeature_, pointsIn2d); }

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

VectorXd LaneletFeature::computeFeatureVector(bool onlyPoints, bool pointsIn2d) const {
  if (reprType_ == LaneletRepresentationType::Centerline) {
    VectorXd vecCenterlinePts = centerline_->computeFeatureVector(true, pointsIn2d);
    VectorXd vec(vecCenterlinePts.size() + 2);  // pts vec + left and right type
    vec(Eigen::seq(0, vecCenterlinePts.size() - 1)) = vecCenterlinePts;
    vec[vec.size() - 2] = leftBoundary_->typeInt();
    vec[vec.size() - 1] = rightBoundary_->typeInt();
    if (onlyPoints) {
      return vec(Eigen::seq(0, vec.size() - 2));
    } else {
      return vec;
    }
  } else if (reprType_ == LaneletRepresentationType::Boundaries) {
    VectorXd vecLeftBdPts = leftBoundary_->computeFeatureVector(true, pointsIn2d);
    VectorXd vecRightBdPts = rightBoundary_->computeFeatureVector(true, pointsIn2d);

    VectorXd vec(vecLeftBdPts.size() + vecRightBdPts.size() + 2);  // pts vec + left and right type
    vec(Eigen::seq(0, vecLeftBdPts.size() - 1)) = vecLeftBdPts;
    vec(Eigen::seq(vecLeftBdPts.size(), vecLeftBdPts.size() + vecRightBdPts.size() - 1)) = vecRightBdPts;
    vec[vec.size() - 2] = leftBoundary_->typeInt();
    vec[vec.size() - 1] = rightBoundary_->typeInt();
    if (onlyPoints) {
      return vec(Eigen::seq(0, vec.size() - 2));
    } else {
      return vec;
    }
  } else {
    throw std::runtime_error("Unknown LaneletRepresentationType!");
    return VectorXd();
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
    double processedLength = boost::geometry::length(individualFeatures_[i]->cutFeature(),
                                                     boost::geometry::strategy::distance::pythagoras<double>());

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

}  // namespace map_learning
}  // namespace lanelet