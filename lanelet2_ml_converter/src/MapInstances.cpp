#include <boost/geometry.hpp>
#include <type_traits>

#include "lanelet2_ml_converter/MapInstances.h"
#include "lanelet2_ml_converter/Utils.h"

namespace lanelet {
namespace ml_converter {

struct LStringProcessResult {
  BasicLineStrings3d cutInstances;
  BasicLineStrings3d cutAndResampledInstances;
  BasicLineStrings3d cutResampledAndTransformedInstances;
  bool wasCut_{false};
  bool valid_{true};
};

LStringProcessResult processLineStringImpl(const BasicLineString3d& lstring, const OrientedRect& bbox,
                                           const ParametrizationType& paramType, int32_t nPoints, double pitch,
                                           double roll) {
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

  double lengthOriginal = boost::geometry::length(lstring, boost::geometry::strategy::distance::pythagoras<double>());
  double lengthProcessed = 0;
  for (const auto& line : cutLines) {
    lengthProcessed =
        lengthProcessed + boost::geometry::length(line, boost::geometry::strategy::distance::pythagoras<double>());
  }
  if (lengthProcessed < 1e-1) {
    result.wasCut_ = true;
    result.valid_ = false;
    return result;
  }
  if (lengthOriginal - lengthProcessed > 1e-2) {
    result.wasCut_ = true;
  }

  for (const auto& line : cutLines) {
    result.cutInstances.push_back(line);
    BasicLineString3d lineResampled = resampleLineString(line, nPoints);
    result.cutAndResampledInstances.push_back(lineResampled);
    result.cutResampledAndTransformedInstances.push_back(transformLineString(bbox, lineResampled, pitch, roll));
  }
  return result;
}

bool LaneLineStringInstance::process(const OrientedRect& bbox, const ParametrizationType& paramType, int32_t nPoints,
                                     double pitch, double roll) {
  LStringProcessResult result = processLineStringImpl(rawInstance_, bbox, paramType, nPoints, pitch, roll);
  if (result.valid_) {
    cutInstances_ = result.cutInstances;
    cutAndResampledInstances_ = result.cutAndResampledInstances;
    cutResampledAndTransformedInstances_ = result.cutResampledAndTransformedInstances;
  } else {
    valid_ = result.valid_;
  }
  wasCut_ = result.wasCut_;
  processedFrom2d_ = bbox.from2d;
  return result.valid_;
}

std::vector<VectorXd> LaneLineStringInstance::computeInstanceVectors(bool onlyPoints, bool pointsIn2d) const {
  std::vector<VectorXd> featVecs;
  for (const auto& split : cutResampledAndTransformedInstances_) {
    featVecs.push_back(toInstanceVector(split, typeInt(), onlyPoints, (pointsIn2d || processedFrom2d_)));
  }
  return featVecs;
}

std::vector<VectorXd> TEInstance::computeInstanceVectors(bool onlyPoints, bool pointsIn2d) const {
  VectorXd vec = (pointsIn2d || processedFrom2d_)
                     ? VectorXd(2 * rawInstance_.size() + 1)
                     : VectorXd(3 * rawInstance_.size() + 1);  // n points with 2/3 dims + type
  if (pointsIn2d == true) {
    for (size_t i = 0; i < rawInstance_.size(); i++) {
      vec.segment(2 * i, 2) = rawInstance_[i].segment(0, 2);
    }
  } else {
    for (size_t i = 0; i < rawInstance_.size(); i++) {
      vec.segment(3 * i, 3) = rawInstance_[i].segment(0, 3);
    }
  }
  vec[vec.size() - 1] = static_cast<int>(teType_);
  if (onlyPoints) {
    return std::vector<VectorXd>{vec.segment(0, vec.size() - 1)};
  } else {
    return std::vector<VectorXd>{vec};
  }
}

std::vector<MatrixXd> LaneLineStringInstance::pointMatrices(bool pointsIn2d) const {
  std::vector<MatrixXd> pointMatrices;
  for (const auto& split : cutResampledAndTransformedInstances_) {
    pointMatrices.push_back(toPointMatrix(split, (pointsIn2d || processedFrom2d_)));
  }
  return pointMatrices;
}

std::vector<MatrixXd> TEInstance::pointMatrices(bool pointsIn2d) const {
  return std::vector<MatrixXd>{toPointMatrix(rawInstance_, (pointsIn2d || processedFrom2d_))};
}

bool TEInstance::process(const OrientedRect& bbox, const ParametrizationType& paramType, int32_t /*unused*/,
                         double pitch, double roll) {
  throw std::runtime_error("Not implemented yet!");
}

LaneletInstance::LaneletInstance(LaneLineStringInstancePtr leftBoundary, LaneLineStringInstancePtr rightBoundary,
                                 LaneLineStringInstancePtr centerline, Id mapID)
    : MapInstance(mapID), leftBoundary_{leftBoundary}, rightBoundary_{rightBoundary}, centerline_{centerline} {}

LaneletInstance::LaneletInstance(const ConstLanelet& ll)
    : leftBoundary_{std::make_shared<LaneLineStringInstance>(ll.leftBound3d().basicLineString(), ll.leftBound3d().id(),
                                                             bdTypeToEnum(ll.leftBound3d()), Ids{ll.id()},
                                                             ll.leftBound3d().inverted())},
      rightBoundary_{std::make_shared<LaneLineStringInstance>(ll.rightBound3d().basicLineString(),
                                                              ll.rightBound3d().id(), bdTypeToEnum(ll.rightBound3d()),
                                                              Ids{ll.id()}, ll.rightBound3d().inverted())},
      centerline_{std::make_shared<LaneLineStringInstance>(ll.centerline3d().basicLineString(), ll.centerline3d().id(),
                                                           LineStringType::Centerline, Ids{ll.id()}, false)} {}

bool LaneletInstance::process(const OrientedRect& bbox, const ParametrizationType& paramType, int32_t nPoints,
                              double pitch, double roll) {
  leftBoundary_->process(bbox, paramType, nPoints, pitch, roll);
  rightBoundary_->process(bbox, paramType, nPoints, pitch, roll);
  centerline_->process(bbox, paramType, nPoints, pitch, roll);

  if (leftBoundary_->wasCut() || rightBoundary_->wasCut() || centerline_->wasCut()) {
    wasCut_ = true;
  }
  if (!leftBoundary_->valid() || !rightBoundary_->valid() || !centerline_->valid()) {
    valid_ = false;
  }
  processedFrom2d_ = bbox.from2d;
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
    stacked.segment(currIndex, el.size()) = el;
  }
  return stacked;
}

std::vector<VectorXd> LaneletInstance::computeInstanceVectors(bool onlyPoints, bool pointsIn2d) const {
  if (reprType_ == LaneletRepresentationType::Centerline) {
    std::vector<VectorXd> vecCenterlinePts = centerline_->computeInstanceVectors(true, pointsIn2d);
    if (onlyPoints) {
      return vecCenterlinePts;
    }
    std::vector<VectorXd> featureVecs(vecCenterlinePts.size());
    for (size_t i = 0; i < vecCenterlinePts.size(); i++) {
      VectorXd vec(vecCenterlinePts[i].size() + 2);  // pts vec + left and right type
      vec.segment(0, vecCenterlinePts.size()) = vecCenterlinePts[i];
      vec[vec.size() - 2] = leftBoundary_->typeInt();
      vec[vec.size() - 1] = rightBoundary_->typeInt();
      featureVecs[i] = vec;
    }
    return featureVecs;
  } else if (reprType_ == LaneletRepresentationType::Boundaries) {
    std::vector<VectorXd> vecLeftBdPts = leftBoundary_->computeInstanceVectors(true, pointsIn2d);
    VectorXd leftBdPts = stackVector(vecLeftBdPts);
    std::vector<VectorXd> vecRightBdPts = rightBoundary_->computeInstanceVectors(true, pointsIn2d);
    VectorXd rightBdPts = stackVector(vecRightBdPts);

    VectorXd vec(leftBdPts.size() + rightBdPts.size() + 2);  // pts vec + left and right type
    vec.segment(0, leftBdPts.size()) = leftBdPts;
    vec.segment(leftBdPts.size(), rightBdPts.size()) = rightBdPts;
    vec[vec.size() - 2] = leftBoundary_->typeInt();
    vec[vec.size() - 1] = rightBoundary_->typeInt();

    if (onlyPoints) {
      return std::vector<VectorXd>{vec.segment(0, vec.size() - 2)};
    } else {
      return std::vector<VectorXd>{vec};
    }
  } else {
    throw std::runtime_error("Unknown LaneletRepresentationType!");
    return std::vector<VectorXd>();
  }
}

CompoundLaneLineStringInstance::CompoundLaneLineStringInstance(const LaneLineStringInstanceList& features,
                                                               LineStringType compoundType)
    : individualInstances_{features},
      pathLengthsRaw_{std::vector<double>(features.size())},
      pathLengthsProcessed_{std::vector<double>(features.size())},
      processedInstancesValid_{std::vector<bool>(features.size())} {
  type_ = compoundType;

  for (size_t i = 0; i < features.size(); i++) {
    if (features[i]->rawInstance().empty()) {
      throw std::runtime_error("Instance with empty rawInstance() supplied!");
    }
    // checking if the linestring is correctly
    if (i == features.size() - 1) {
      rawInstance_.insert(rawInstance_.end(), features[i]->rawInstance().begin(), features[i]->rawInstance().end());
    } else {
      rawInstance_.insert(rawInstance_.end(), features[i]->rawInstance().begin(), features[i]->rawInstance().end() - 1);
    }

    double rawLength =
        boost::geometry::length(features[i]->rawInstance(), boost::geometry::strategy::distance::pythagoras<double>());
    if (i > 0) {
      pathLengthsRaw_[i] = pathLengthsRaw_[i - 1] + rawLength;
    } else {
      pathLengthsRaw_[i] = rawLength;
    }
  }
}

bool CompoundLaneLineStringInstance::process(const OrientedRect& bbox, const ParametrizationType& paramType,
                                             int32_t nPoints, double pitch, double roll) {
  bool valid = false;
  LaneLineStringInstance::process(bbox, paramType, nPoints, pitch, roll);
  for (size_t i = 0; i < individualInstances_.size(); i++) {
    individualInstances_[i]->process(bbox, paramType, nPoints, pitch, roll);

    double processedLength = 0;
    for (const auto& line : individualInstances_[i]->cutInstance()) {
      processedLength =
          processedLength + boost::geometry::length(line, boost::geometry::strategy::distance::pythagoras<double>());
    }

    if (processedLength > validLengthThresh_) {
      processedInstancesValid_[i] = true;
      valid = true;
    }

    if (i > 0) {
      pathLengthsProcessed_[i] = pathLengthsProcessed_[i - 1] + processedLength;
    } else {
      pathLengthsProcessed_[i] = processedLength;
    }
  }
  processedFrom2d_ = bbox.from2d;
  return valid;
}

MatrixXd toPointMatrix(const BasicLineString3d& lString, bool pointsIn2d) {
  MatrixXd mat =
      pointsIn2d ? MatrixXd(lString.size(), 2) : MatrixXd(lString.size(), 3);  // n points with 2/3 dims + type
  if (pointsIn2d == true) {
    for (size_t i = 0; i < lString.size(); i++) {
      mat.row(i) = lString[i].segment(0, 2);
    }
  } else {
    for (size_t i = 0; i < lString.size(); i++) {
      mat.row(i) = lString[i].segment(0, 3);
    }
  }
  return mat;
}

VectorXd toInstanceVector(const BasicLineString3d& line, int typeInt, bool onlyPoints, bool pointsIn2d) {
  VectorXd vec =
      pointsIn2d ? VectorXd(2 * line.size() + 1) : VectorXd(3 * line.size() + 1);  // n points with 2/3 dims + type
  if (pointsIn2d == true) {
    for (size_t i = 0; i < line.size(); i++) {
      vec.segment(2 * i, 2) = line[i].segment(0, 2);
    }
  } else {
    for (size_t i = 0; i < line.size(); i++) {
      vec.segment(3 * i, 3) = line[i].segment(0, 3);
    }
  }
  vec[vec.size() - 1] = typeInt;
  if (onlyPoints) {
    return vec.segment(0, vec.size() - 1);
  } else {
    return vec;
  }
}

}  // namespace ml_converter
}  // namespace lanelet