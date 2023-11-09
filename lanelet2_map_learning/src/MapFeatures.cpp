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

Eigen::VectorXd LaneLineStringFeature::computeFeatureVector(bool onlyPoints) const {
  const BasicLineString3d& selectedFeature =
      (cutAndResampledFeature_.size() > 0) ? cutAndResampledFeature_ : rawFeature_;
  Eigen::VectorXd vec(3 * selectedFeature.size() + 1);  // n points with 3 dims + type
  for (size_t i = 0; i < selectedFeature.size(); i++) {
    vec(Eigen::seq(i, i + 2)) = selectedFeature[i](Eigen::seq(i, i + 2));
  }
  vec[vec.size() - 1] = static_cast<int>(type_);
  if (onlyPoints) {
    return vec(Eigen::seq(0, vec.size() - 1));
  } else {
    return vec;
  }
}

Eigen::VectorXd TEFeature::computeFeatureVector(bool onlyPoints = false) const {
  Eigen::VectorXd vec(3 * rawFeature_.size() + 1);  // n points with 3 dims + type
  for (size_t i = 0; i < rawFeature_.size(); i++) {
    vec(Eigen::seq(i, i + 2)) = rawFeature_[i](Eigen::seq(i, i + 2));
  }
  vec[vec.size() - 1] = static_cast<int>(teType_);
  return vec;
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
}

Eigen::VectorXd LaneletFeature::computeFeatureVector(bool onlyPoints) const {
  if (reprType_ == LaneletRepresentationType::Centerline) {
    Eigen::VectorXd vecCenterlinePts = centerline_.computeFeatureVector(true);
    Eigen::VectorXd vec(vecCenterlinePts.size() + 2);  // pts vec + left and right type
    vec(Eigen::seq(0, vecCenterlinePts.size() - 1)) = vecCenterlinePts;
    vec[vec.size() - 2] = static_cast<int>(leftBoundary_.type());
    vec[vec.size() - 1] = static_cast<int>(rightBoundary_.type());
    if (onlyPoints) {
      return vec(Eigen::seq(0, vec.size() - 2));
    } else {
      return vec;
    }
  } else if (reprType_ == LaneletRepresentationType::Boundaries) {
    Eigen::VectorXd vecLeftBdPts = leftBoundary_.computeFeatureVector(true);
    Eigen::VectorXd vecRightBdPts = rightBoundary_.computeFeatureVector(true);
    Eigen::VectorXd vec(vecLeftBdPts.size() + vecRightBdPts.size() + 2);  // pts vec + left and right type
    vec(Eigen::seq(0, vecLeftBdPts.size() - 1)) = vecLeftBdPts;
    vec(Eigen::seq(vecLeftBdPts.size(), vecLeftBdPts.size() + vecRightBdPts.size() - 1)) = vecRightBdPts;
    vec[vec.size() - 2] = static_cast<int>(leftBoundary_.type());
    vec[vec.size() - 1] = static_cast<int>(rightBoundary_.type());
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

CompoundLaneLineStringFeature::CompoundLaneLineStringFeature(const LaneLineStringFeatures& features,
                                                             LineStringType compoundType)
    : individualFeatures_{features}, pathLengthsRaw_{std::vector<double>(features.size())} {
  type_ = compoundType;

  for (size_t i = 0; i < features.size(); i++) {
    assert(features[i].rawFeature().size() > 1);
    rawFeature_.insert(rawFeature_.end(), features[i].rawFeature().begin(), features[i].rawFeature().end() - 1);

    double rawLength =
        boost::geometry::length(features[i].rawFeature(), boost::geometry::strategy::distance::pythagoras<double>());
    if (features[i].cutFeature().size() > 1)
      if (i > 0) {
        pathLengthsRaw_[i] = pathLengthsRaw_[i - 1] + rawLength;
      } else {
        pathLengthsRaw_[i] = rawLength;
      }
  }
}

}  // namespace map_learning
}  // namespace lanelet