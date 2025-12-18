#pragma once
#include <lanelet2_core/Exceptions.h>
#include <lanelet2_core/Forward.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/utility/Optional.h>

#include <boost/geometry.hpp>
#include <type_traits>

#include "lanelet2_ml_converter/Forward.h"
#include "lanelet2_ml_converter/Types.h"

namespace lanelet {
namespace ml_converter {

using VectorXd = Eigen::Matrix<double, Eigen::Dynamic, 1>;
using MatrixXd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;

using BasicLineStrings3d = std::vector<BasicLineString3d>;

class MapInstance {
 public:
  const Id mapID() const { return mapID_.get_value_or(InvalId); }
  bool initialized() const { return initialized_; }
  bool valid() const { return valid_; }
  bool wasCut() const { return wasCut_; }

  virtual std::vector<VectorXd> computeInstanceVectors(bool onlyPoints, bool pointsIn2d) const = 0;
  virtual bool process(const OrientedRect& bbox, const ParametrizationType& paramType, int32_t nPoints,
                       double pitch = 0, double roll = 0) = 0;

  template <class Archive>
  friend void boost::serialization::serialize(Archive& ar, lanelet::ml_converter::MapInstance& feat,
                                              const unsigned int /*version*/);

  virtual ~MapInstance() noexcept = default;

 protected:
  bool initialized_{false};
  bool valid_{true};
  bool wasCut_{false};
  bool processedFrom2d_{false};
  Optional<Id> mapID_;

  MapInstance() = default;
  MapInstance(Id mapID) : mapID_{mapID}, initialized_{true} {}
};

class LineStringInstance : public MapInstance {
 public:
  const BasicLineString3d& rawInstance() const { return rawInstance_; }

  virtual std::vector<VectorXd> computeInstanceVectors(bool onlyPoints, bool pointsIn2d) const = 0;
  virtual std::vector<MatrixXd> pointMatrices(bool pointsIn2d) const = 0;
  virtual bool process(const OrientedRect& bbox, const ParametrizationType& paramType, int32_t nPoints,
                       double pitch = 0, double roll = 0) = 0;

  template <class Archive>
  friend void boost::serialization::serialize(Archive& ar, lanelet::ml_converter::LineStringInstance& feat,
                                              const unsigned int /*version*/);

  virtual ~LineStringInstance() noexcept = default;

 protected:
  BasicLineString3d rawInstance_;
  LineStringInstance() {}
  LineStringInstance(const BasicLineString3d& feature, Id mapID) : MapInstance(mapID), rawInstance_{feature} {}
};

class LaneLineStringInstance : public LineStringInstance {
 public:
  LaneLineStringInstance() {}
  LaneLineStringInstance(const BasicLineString3d& feature, Id mapID, LineStringType type,
                         const std::vector<Id>& laneletID, bool inverted)
      : LineStringInstance(feature, mapID), type_{type}, laneletIDs_{laneletID}, inverted_{inverted} {}
  virtual ~LaneLineStringInstance() noexcept = default;

  virtual bool process(const OrientedRect& bbox, const ParametrizationType& paramType, int32_t nPoints,
                       double pitch = 0, double roll = 0) override;
  virtual std::vector<VectorXd> computeInstanceVectors(
      bool onlyPoints,
      bool pointsIn2d) const override;  // uses processedInstance_ when available
  virtual std::vector<MatrixXd> pointMatrices(
      bool pointsIn2d) const override;  // uses processedInstance_ when available

  const BasicLineStrings3d& cutInstance() const { return cutInstances_; }
  const BasicLineStrings3d& cutAndResampledInstance() const { return cutAndResampledInstances_; }
  const BasicLineStrings3d& cutResampledAndTransformedInstance() const { return cutResampledAndTransformedInstances_; }
  LineStringType type() const { return type_; }
  bool inverted() const { return inverted_; }
  int typeInt() const { return static_cast<int>(type_); }
  const Ids& laneletIDs() const { return laneletIDs_; }
  void addLaneletID(Id id) { laneletIDs_.push_back(id); }

  template <class Archive>
  friend void boost::serialization::serialize(Archive& ar, lanelet::ml_converter::LaneLineStringInstance& feat,
                                              const unsigned int /*version*/);

 protected:
  BasicLineStrings3d cutInstances_;
  BasicLineStrings3d cutAndResampledInstances_;
  BasicLineStrings3d cutResampledAndTransformedInstances_;
  LineStringType type_;
  bool inverted_{false};  // = inverted compared to element with that Id in lineStringLayer
  Ids laneletIDs_;
};

using LaneLineStringInstancePtr = std::shared_ptr<LaneLineStringInstance>;
using LaneLineStringInstances = std::map<Id, LaneLineStringInstancePtr>;
using LaneLineStringInstanceList = std::vector<LaneLineStringInstancePtr>;

class TEInstance : public LineStringInstance {
 public:
  TEInstance() {}
  TEInstance(const BasicLineString3d& feature, Id mapID, TEType type)
      : LineStringInstance(feature, mapID), teType_{type} {}
  virtual ~TEInstance() noexcept = default;

  bool process(const OrientedRect& bbox, const ParametrizationType& paramType, int32_t /*unused*/, double pitch = 0,
               double roll = 0) override;  // not implemented yet
  std::vector<VectorXd> computeInstanceVectors(bool onlyPoints,
                                               bool pointsIn2d) const override;  // currently uses raw feature only
  virtual std::vector<MatrixXd> pointMatrices(bool pointsIn2d) const override;

  const TEType& teType() { return teType_; }

 private:
  TEType teType_;
};

class LaneletInstance : public MapInstance {
 public:
  LaneletInstance() {}
  LaneletInstance(LaneLineStringInstancePtr leftBoundary, LaneLineStringInstancePtr rightBoundary,
                  LaneLineStringInstancePtr centerline, Id mapID);
  LaneletInstance(const ConstLanelet& ll);
  virtual ~LaneletInstance() noexcept = default;

  bool process(const OrientedRect& bbox, const ParametrizationType& paramType, int32_t nPoints, double pitch = 0,
               double roll = 0) override;
  std::vector<VectorXd> computeInstanceVectors(bool onlyPoints, bool pointsIn2d) const override;

  void setReprType(LaneletRepresentationType reprType) { reprType_ = reprType; }

  LaneLineStringInstancePtr leftBoundary() const { return leftBoundary_; }
  LaneLineStringInstancePtr rightBoundary() const { return rightBoundary_; }
  LaneLineStringInstancePtr centerline() const { return centerline_; }

  template <class Archive>
  friend void boost::serialization::serialize(Archive& ar, lanelet::ml_converter::LaneletInstance& feat,
                                              const unsigned int /*version*/);

 private:
  LaneLineStringInstancePtr leftBoundary_;
  LaneLineStringInstancePtr rightBoundary_;
  LaneLineStringInstancePtr centerline_;
  LaneletRepresentationType reprType_{LaneletRepresentationType::Centerline};
};

class CompoundLaneLineStringInstance : public LaneLineStringInstance {
 public:
  CompoundLaneLineStringInstance() {}
  // features for this constructor are required to be given in already sorted order
  CompoundLaneLineStringInstance(const LaneLineStringInstanceList& features, LineStringType compoundType);

  virtual ~CompoundLaneLineStringInstance() noexcept = default;
  bool process(const OrientedRect& bbox, const ParametrizationType& paramType, int32_t nPoints, double pitch = 0,
               double roll = 0) override;

  LaneLineStringInstanceList features() const { return individualInstances_; }
  const std::vector<double>& pathLengthsRaw() const { return pathLengthsRaw_; }
  const std::vector<double>& pathLengthsProcessed() const { return pathLengthsProcessed_; }
  const std::vector<bool>& processedInstancesValid() const { return processedInstancesValid_; }

  template <class Archive>
  friend void boost::serialization::serialize(Archive& ar, lanelet::ml_converter::CompoundLaneLineStringInstance& feat,
                                              const unsigned int /*version*/);

 private:
  LaneLineStringInstanceList individualInstances_;
  std::vector<double> pathLengthsRaw_;
  std::vector<double> pathLengthsProcessed_;
  std::vector<bool> processedInstancesValid_;
  double validLengthThresh_{0.3};
};

using CompoundLaneLineStringInstancePtr = std::shared_ptr<CompoundLaneLineStringInstance>;
using TEInstancePtr = std::shared_ptr<TEInstance>;
using LaneletInstancePtr = std::shared_ptr<LaneletInstance>;
using CompoundLaneLineStringInstanceList = std::vector<CompoundLaneLineStringInstancePtr>;
using TEInstances = std::map<Id, TEInstancePtr>;
using LaneletInstances = std::map<Id, LaneletInstancePtr>;

template <class T>
MatrixXd getInstanceVectorMatrix(const std::map<Id, std::shared_ptr<T>>& mapInstances, bool onlyPoints,
                                 bool pointsIn2d) {
  std::vector<std::shared_ptr<T>> featList;
  for (const auto& pair : mapInstances) {
    featList.push_back(pair.second);
  }
  return getInstanceVectorMatrix(featList, onlyPoints, pointsIn2d);
}

template <class T>
MatrixXd getInstanceVectorMatrix(const std::vector<std::shared_ptr<T>>& mapInstances, bool onlyPoints,
                                 bool pointsIn2d) {
  std::vector<VectorXd> featureVectors;
  for (const auto& feat : mapInstances) {
    if (!feat->valid()) {
      throw std::runtime_error("Invalid feature in list! This function requires all given features to be valid!");
    }
    std::vector<VectorXd> individualVecs = feat->computeInstanceVectors(onlyPoints, pointsIn2d);
    featureVectors.insert(featureVectors.end(), individualVecs.begin(), individualVecs.end());
  }
  if (std::adjacent_find(featureVectors.begin(), featureVectors.end(),
                         [](const VectorXd& v1, const VectorXd& v2) { return v1.size() != v2.size(); }) ==
          featureVectors.end() &&
      featureVectors.size() > 1) {
    throw std::runtime_error(
        "Unequal length of feature vectors! To create a matrix all feature vectors must have the same length!");
  }
  MatrixXd featureMat(featureVectors.size(), featureVectors[0].size());
  for (size_t i = 0; i < featureVectors.size(); i++) {
    featureMat.row(i) = featureVectors[i];
  }
  return featureMat;
}

template <class T>
std::vector<MatrixXd> getPointMatrices(const std::map<Id, std::shared_ptr<T>>& mapInstances, bool pointsIn2d) {
  std::vector<std::shared_ptr<T>> featList;
  for (const auto& pair : mapInstances) {
    featList.push_back(pair.second);
  }
  return getPointMatrices(featList, pointsIn2d);
}

template <class T>
std::vector<MatrixXd> getPointMatrices(const std::vector<std::shared_ptr<T>>& mapInstances, bool pointsIn2d) {
  std::vector<MatrixXd> pointMatrices;
  for (const auto& feat : mapInstances) {
    if (!feat->valid()) {
      throw std::runtime_error("Invalid feature in list! This function requires all given features to be valid!");
    }
    std::vector<MatrixXd> individualMats = feat->pointMatrices(pointsIn2d);
    pointMatrices.insert(pointMatrices.end(), individualMats.begin(), individualMats.end());
  }
  return pointMatrices;
}

template <typename T>
bool processInstances(std::map<Id, std::shared_ptr<T>>& featMap, const OrientedRect& bbox,
                      const ParametrizationType& paramType, int32_t nPoints, double pitch = 0, double roll = 0) {
  bool allValid = true;
  for (auto& feat : featMap) {
    if (!feat.second->process(bbox, paramType, nPoints, pitch, roll)) {
      allValid = false;
    }
  }
  return allValid;
}

template <typename T>
bool processInstances(std::vector<std::shared_ptr<T>>& featVec, const OrientedRect& bbox,
                      const ParametrizationType& paramType, int32_t nPoints, double pitch = 0, double roll = 0) {
  bool allValid = true;
  for (auto& feat : featVec) {
    if (!feat->process(bbox, paramType, nPoints, pitch, roll)) {
      allValid = false;
    }
  }
  return allValid;
}

MatrixXd toPointMatrix(const BasicLineString3d& lString, bool pointsIn2d);
VectorXd toInstanceVector(const BasicLineString3d& line, int typeInt, bool onlyPoints, bool pointsIn2d);
}  // namespace ml_converter
}  // namespace lanelet