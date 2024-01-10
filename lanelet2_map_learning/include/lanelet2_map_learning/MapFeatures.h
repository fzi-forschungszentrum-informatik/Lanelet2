#pragma once
#include <lanelet2_core/Exceptions.h>
#include <lanelet2_core/Forward.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/utility/Optional.h>

#include <boost/geometry.hpp>
#include <type_traits>

#include "lanelet2_map_learning/Forward.h"
#include "lanelet2_map_learning/Types.h"

namespace lanelet {
namespace map_learning {

using VectorXd = Eigen::Matrix<double, Eigen::Dynamic, 1>;
using MatrixXd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;

class MapFeature {
 public:
  const Id mapID() const { return mapID_.get_value_or(InvalId); }
  bool initialized() const { return initialized_; }
  bool valid() const { return valid_; }
  bool wasCut() const { return wasCut_; }

  virtual VectorXd computeFeatureVector(bool onlyPoints, bool pointsIn2d) const = 0;
  virtual bool process(const OrientedRect& bbox, const ParametrizationType& paramType, int32_t nPoints) = 0;

  template <class Archive>
  friend void boost::serialization::serialize(Archive& ar, lanelet::map_learning::MapFeature& feat,
                                              const unsigned int /*version*/);

  virtual ~MapFeature() noexcept = default;

 protected:
  bool initialized_{false};
  bool valid_{true};
  bool wasCut_{false};
  Optional<Id> mapID_;

  MapFeature() = default;
  MapFeature(Id mapID) : mapID_{mapID}, initialized_{true} {}
};

class LineStringFeature : public MapFeature {
 public:
  const BasicLineString3d& rawFeature() const { return rawFeature_; }

  virtual VectorXd computeFeatureVector(bool onlyPoints, bool pointsIn2d) const = 0;
  virtual MatrixXd pointMatrix(bool pointsIn2d) const = 0;
  virtual bool process(const OrientedRect& bbox, const ParametrizationType& paramType, int32_t nPoints) = 0;

  template <class Archive>
  friend void boost::serialization::serialize(Archive& ar, lanelet::map_learning::LineStringFeature& feat,
                                              const unsigned int /*version*/);

  virtual ~LineStringFeature() noexcept = default;

 protected:
  BasicLineString3d rawFeature_;

  LineStringFeature() {}
  LineStringFeature(const BasicLineString3d& feature, Id mapID) : MapFeature(mapID), rawFeature_{feature} {}
};

class LaneLineStringFeature : public LineStringFeature {
 public:
  LaneLineStringFeature() {}
  LaneLineStringFeature(const BasicLineString3d& feature, Id mapID, LineStringType type, Id laneletID)
      : LineStringFeature(feature, mapID), type_{type}, laneletIDs_{laneletID} {}
  virtual ~LaneLineStringFeature() noexcept = default;

  virtual bool process(const OrientedRect& bbox, const ParametrizationType& paramType, int32_t nPoints) override;
  virtual VectorXd computeFeatureVector(bool onlyPoints,
                                        bool pointsIn2d) const override;  // uses processedFeature_ when available
  virtual MatrixXd pointMatrix(bool pointsIn2d) const override;           // uses processedFeature_ when available

  const BasicLineString3d& cutFeature() const { return cutFeature_; }
  const BasicLineString3d& cutAndResampledFeature() const { return cutAndResampledFeature_; }
  const BasicLineString3d& cutResampledAndTransformedFeature() const { return cutResampledAndTransformedFeature_; }
  LineStringType type() const { return type_; }
  int typeInt() const { return static_cast<int>(type_); }
  const std::vector<Id>& laneletIDs() const { return laneletIDs_; }
  void addLaneletID(Id id) { laneletIDs_.push_back(id); }

  template <class Archive>
  friend void boost::serialization::serialize(Archive& ar, lanelet::map_learning::LaneLineStringFeature& feat,
                                              const unsigned int /*version*/);

 protected:
  BasicLineString3d cutFeature_;
  BasicLineString3d cutAndResampledFeature_;
  BasicLineString3d cutResampledAndTransformedFeature_;
  LineStringType type_;
  std::vector<Id> laneletIDs_;
};

using LaneLineStringFeaturePtr = std::shared_ptr<LaneLineStringFeature>;
using LaneLineStringFeatures = std::map<Id, LaneLineStringFeaturePtr>;
using LaneLineStringFeatureList = std::vector<LaneLineStringFeaturePtr>;

class TEFeature : public LineStringFeature {
 public:
  TEFeature() {}
  TEFeature(const BasicLineString3d& feature, Id mapID, TEType type)
      : LineStringFeature(feature, mapID), teType_{type} {}
  virtual ~TEFeature() noexcept = default;

  bool process(const OrientedRect& bbox, const ParametrizationType& paramType,
               int32_t /*unused*/) override;  // not implemented yet
  VectorXd computeFeatureVector(bool onlyPoints,
                                bool pointsIn2d) const override;  // currently uses raw feature only
  virtual MatrixXd pointMatrix(bool pointsIn2d) const override;

  const TEType& teType() { return teType_; }

 private:
  TEType teType_;
};

class LaneletFeature : public MapFeature {
 public:
  LaneletFeature() {}
  LaneletFeature(LaneLineStringFeaturePtr leftBoundary, LaneLineStringFeaturePtr rightBoundary,
                 LaneLineStringFeaturePtr centerline, Id mapID);
  LaneletFeature(const ConstLanelet& ll);
  virtual ~LaneletFeature() noexcept = default;

  bool process(const OrientedRect& bbox, const ParametrizationType& paramType, int32_t nPoints) override;
  VectorXd computeFeatureVector(bool onlyPoints, bool pointsIn2d) const override;

  void setReprType(LaneletRepresentationType reprType) { reprType_ = reprType; }

  LaneLineStringFeaturePtr leftBoundary() const { return leftBoundary_; }
  LaneLineStringFeaturePtr rightBoundary() const { return rightBoundary_; }
  LaneLineStringFeaturePtr centerline() const { return centerline_; }

  template <class Archive>
  friend void boost::serialization::serialize(Archive& ar, lanelet::map_learning::LaneletFeature& feat,
                                              const unsigned int /*version*/);

 private:
  LaneLineStringFeaturePtr leftBoundary_;
  LaneLineStringFeaturePtr rightBoundary_;
  LaneLineStringFeaturePtr centerline_;
  LaneletRepresentationType reprType_{LaneletRepresentationType::Centerline};
};

class CompoundLaneLineStringFeature : public LaneLineStringFeature {
 public:
  CompoundLaneLineStringFeature() {}
  // features for this constructor are required to be given in already sorted order
  CompoundLaneLineStringFeature(const LaneLineStringFeatureList& features, LineStringType compoundType);

  virtual ~CompoundLaneLineStringFeature() noexcept = default;
  bool process(const OrientedRect& bbox, const ParametrizationType& paramType, int32_t nPoints) override;

  LaneLineStringFeatureList features() const { return individualFeatures_; }
  const std::vector<double>& pathLengthsRaw() const { return pathLengthsRaw_; }
  const std::vector<double>& pathLengthsProcessed() const { return pathLengthsProcessed_; }
  const std::vector<bool>& processedFeaturesValid() const { return processedFeaturesValid_; }

  template <class Archive>
  friend void boost::serialization::serialize(Archive& ar, lanelet::map_learning::CompoundLaneLineStringFeature& feat,
                                              const unsigned int /*version*/);

 private:
  LaneLineStringFeatureList individualFeatures_;
  std::vector<double> pathLengthsRaw_;
  std::vector<double> pathLengthsProcessed_;
  std::vector<bool> processedFeaturesValid_;
  double validLengthThresh_{0.3};
};

using CompoundLaneLineStringFeaturePtr = std::shared_ptr<CompoundLaneLineStringFeature>;
using TEFeaturePtr = std::shared_ptr<TEFeature>;
using LaneletFeaturePtr = std::shared_ptr<LaneletFeature>;
using CompoundLaneLineStringFeatureList = std::vector<CompoundLaneLineStringFeaturePtr>;
using TEFeatures = std::map<Id, TEFeaturePtr>;
using LaneletFeatures = std::map<Id, LaneletFeaturePtr>;

template <class T>
MatrixXd getFeatureVectorMatrix(const std::map<Id, std::shared_ptr<T>>& mapFeatures, bool onlyPoints, bool pointsIn2d) {
  std::vector<std::shared_ptr<T>> featList;
  for (const auto& pair : mapFeatures) {
    featList.push_back(pair.second);
  }
  return getFeatureVectorMatrix(featList, onlyPoints, pointsIn2d);
}

template <class T>
MatrixXd getFeatureVectorMatrix(const std::vector<std::shared_ptr<T>>& mapFeatures, bool onlyPoints, bool pointsIn2d) {
  if (mapFeatures.empty()) {
    throw std::runtime_error("Empty mapFeatures vector supplied!");
  }
  std::vector<VectorXd> featureVectors;
  for (const auto& feat : mapFeatures) {
    if (!feat->valid()) {
      throw std::runtime_error("Invalid feature in list! This function requires all given features to be valid!");
    }
    featureVectors.push_back(feat->computeFeatureVector(onlyPoints, pointsIn2d));
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
std::vector<MatrixXd> getPointMatrices(const std::map<Id, std::shared_ptr<T>>& mapFeatures, bool pointsIn2d) {
  std::vector<std::shared_ptr<T>> featList;
  for (const auto& pair : mapFeatures) {
    featList.push_back(pair.second);
  }
  return getPointMatrices(featList, pointsIn2d);
}

template <class T>
std::vector<MatrixXd> getPointMatrices(const std::vector<std::shared_ptr<T>>& mapFeatures, bool pointsIn2d) {
  if (mapFeatures.empty()) {
    throw std::runtime_error("Empty mapFeatures vector supplied!");
  }
  std::vector<MatrixXd> pointMatrices;
  for (const auto& feat : mapFeatures) {
    if (!feat->valid()) {
      throw std::runtime_error("Invalid feature in list! This function requires all given features to be valid!");
    }
    pointMatrices.push_back(feat->pointMatrix(pointsIn2d));
  }
  return pointMatrices;
}

template <typename T>
bool processFeatures(std::map<Id, std::shared_ptr<T>>& featMap, const OrientedRect& bbox,
                     const ParametrizationType& paramType, int32_t nPoints) {
  bool allValid = true;
  for (auto& feat : featMap) {
    if (!feat.second->process(bbox, paramType, nPoints)) {
      allValid = false;
    }
  }
  return allValid;
}

template <typename T>
bool processFeatures(std::vector<std::shared_ptr<T>>& featVec, const OrientedRect& bbox,
                     const ParametrizationType& paramType, int32_t nPoints) {
  bool allValid = true;
  for (auto& feat : featVec) {
    if (!feat->process(bbox, paramType, nPoints)) {
      allValid = false;
    }
  }
  return allValid;
}

}  // namespace map_learning
}  // namespace lanelet