#include "lanelet2_map_learning/MapFeatures.h"

#include <boost/geometry.hpp>
#include <type_traits>

#include "lanelet2_map_learning/Utils.h"

namespace lanelet {
namespace map_learning {

void LaneLineStringFeature::processLineString(const OrientedRect& bbox, const ParametrizationType& paramType,
                                              int32_t nPoints) {
  if (paramType != ParametrizationType::LineString) {
    throw std::runtime_error("Only polyline parametrization is implemented so far!");
  }
  processedFeature_ = cutLineString(bbox, rawFeature_);
  processedFeature_ = resampleLineString(processedFeature_, nPoints);

  double lengthOriginal =
      boost::geometry::length(rawFeature_, boost::geometry::strategy::distance::pythagoras<double>());
  double lengthProcessed =
      boost::geometry::length(processedFeature_, boost::geometry::strategy::distance::pythagoras<double>());

  assert(lengthOriginal - lengthProcessed > -1e-2);
  if (lengthOriginal - lengthProcessed > 1e-2) {
    wasCut_ = true;
  }
}

Eigen::VectorXd LaneLineStringFeature::computeFeatureVector(bool onlyPoints) {
  const BasicLineString3d& selectedFeature = (processedFeature_.size() > 0) ? processedFeature_ : rawFeature_;
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

Eigen::VectorXd TEFeature::computeFeatureVector() {
  Eigen::VectorXd vec(3 * rawFeature_.size() + 1);  // n points with 3 dims + type
  for (size_t i = 0; i < rawFeature_.size(); i++) {
    vec(Eigen::seq(i, i + 2)) = rawFeature_[i](Eigen::seq(i, i + 2));
  }
  vec[vec.size() - 1] = static_cast<int>(teType_);
  return vec;
}

LaneletFeature::LaneletFeature(const LaneLineStringFeature& leftBoundary, const LaneLineStringFeature& rightBoundary,
                               const LaneLineStringFeature& centerline, Id mapID)
    : leftBoundary_{leftBoundary}, rightBoundary_{rightBoundary}, centerline_{centerline}, mapID_{mapID} {}

LaneletFeature::LaneletFeature(const ConstLanelet& ll)
    : leftBoundary_{LaneLineStringFeature(ll.leftBound3d().basicLineString(), ll.leftBound3d().id(),
                                          bdSubtypeToEnum(ll.leftBound3d()))},
      rightBoundary_{LaneLineStringFeature(ll.rightBound3d().basicLineString(), ll.rightBound3d().id(),
                                           bdSubtypeToEnum(ll.rightBound3d()))},
      centerline_{LaneLineStringFeature(ll.centerline3d().basicLineString(), ll.centerline3d().id(),
                                        LineStringType::Centerline)} {}

void LaneletFeature::processLanelet(const OrientedRect& bbox, const ParametrizationType& paramType, int32_t nPoints) {
  leftBoundary_.processLineString(bbox, paramType, nPoints);
  rightBoundary_.processLineString(bbox, paramType, nPoints);
  centerline_.processLineString(bbox, paramType, nPoints);

  if (leftBoundary_.wasCut_ || rightBoundary_.wasCut_ || centerline_.wasCut_) {
    wasCut_ = true;
  }
}

Eigen::VectorXd LaneletFeature::computeFeatureVector(const LaneletRepresentationType& reprType, bool onlyPoints) {
  if (reprType == LaneletRepresentationType::Centerline) {
    Eigen::VectorXd vecCenterlinePts = centerline_.computeFeatureVector(true);
    Eigen::VectorXd vec(vecCenterlinePts.size() + 2);  // pts vec + left and right type
    vec(Eigen::seq(0, vecCenterlinePts.size() - 1)) = vecCenterlinePts;
    vec[vec.size() - 2] = static_cast<int>(leftBoundary_.type_);
    vec[vec.size() - 1] = static_cast<int>(rightBoundary_.type_);
    if (onlyPoints) {
      return vec(Eigen::seq(0, vec.size() - 2));
    } else {
      return vec;
    }
  } else if (reprType == LaneletRepresentationType::Boundaries) {
    Eigen::VectorXd vecLeftBdPts = leftBoundary_.computeFeatureVector(true);
    Eigen::VectorXd vecRightBdPts = rightBoundary_.computeFeatureVector(true);
    Eigen::VectorXd vec(vecLeftBdPts.size() + vecRightBdPts.size() + 2);  // pts vec + left and right type
    vec(Eigen::seq(0, vecLeftBdPts.size() - 1)) = vecLeftBdPts;
    vec(Eigen::seq(vecLeftBdPts.size(), vecLeftBdPts.size() + vecRightBdPts.size() - 1)) = vecRightBdPts;
    vec[vec.size() - 2] = static_cast<int>(leftBoundary_.type_);
    vec[vec.size() - 1] = static_cast<int>(rightBoundary_.type_);
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

}  // namespace map_learning
}  // namespace lanelet