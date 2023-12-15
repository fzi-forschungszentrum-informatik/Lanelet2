#pragma once

#include <boost/serialization/map.hpp>
#include <boost/serialization/optional.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/vector.hpp>

#include "MapData.h"

namespace boost {
namespace serialization {

template <class Archive, class S, int Rows_, int Cols_, int Ops_, int MaxRows_, int MaxCols_>
inline void save(Archive& ar, const Eigen::Matrix<S, Rows_, Cols_, Ops_, MaxRows_, MaxCols_>& g,
                 const unsigned int /*version*/) {
  int rows = g.rows();
  int cols = g.cols();
  ar & rows;
  ar & cols;
  std::cerr << "Trying to save Eigen Mat!" << std::endl;
  ar& boost::serialization::make_array(g.data(), rows * cols);
  std::cerr << "Saved Eigen Mat!" << std::endl;
}

template <class Archive, class S, int Rows_, int Cols_, int Ops_, int MaxRows_, int MaxCols_>
inline void load(Archive& ar, Eigen::Matrix<S, Rows_, Cols_, Ops_, MaxRows_, MaxCols_>& g,
                 const unsigned int /*version*/) {
  int rows, cols;
  ar & rows;
  ar & cols;
  g.resize(rows, cols);
  ar& boost::serialization::make_array(g.data(), rows * cols);
}

template <class Archive, class S, int Rows_, int Cols_, int Ops_, int MaxRows_, int MaxCols_>
inline void serialize(Archive& ar, Eigen::Matrix<S, Rows_, Cols_, Ops_, MaxRows_, MaxCols_>& g,
                      const unsigned int version) {
  split_free(ar, g, version);
}

template <class Archive>
void serialize(Archive& ar, lanelet::map_learning::LaneletRepresentationType& type, const unsigned int /*version*/) {
  ar & type;
}

template <class Archive>
void serialize(Archive& ar, lanelet::map_learning::ParametrizationType& type, const unsigned int /*version*/) {
  ar & type;
}

template <class Archive>
void serialize(Archive& ar, lanelet::map_learning::LineStringType& type, const unsigned int /*version*/) {
  ar & type;
}

template <class Archive>
void serialize(Archive& ar, lanelet::map_learning::MapFeature& feat, const unsigned int /*version*/) {
  ar & feat.initialized_;
  ar & feat.wasCut_;
  ar & feat.valid_;
  ar & feat.mapID_;
}

template <class Archive>
void serialize(Archive& ar, lanelet::map_learning::LineStringFeature& feat, const unsigned int /*version*/) {
  ar& boost::serialization::base_object<lanelet::map_learning::MapFeature>(feat);
  ar & feat.rawFeature_;
}

template <class Archive>
void serialize(Archive& ar, lanelet::map_learning::LaneLineStringFeature& feat, const unsigned int /*version*/) {
  ar& boost::serialization::base_object<lanelet::map_learning::LineStringFeature>(feat);
  ar & feat.cutFeature_;
  ar & feat.cutAndResampledFeature_;
  ar & feat.wasCut_;
  ar & feat.type_;
  ar & feat.laneletIDs_;
}

template <class Archive>
void serialize(Archive& ar, lanelet::map_learning::CompoundLaneLineStringFeature& feat,
               const unsigned int /*version*/) {
  ar& boost::serialization::base_object<lanelet::map_learning::LaneLineStringFeature>(feat);
  ar & feat.individualFeatures_;
  ar & feat.pathLengthsRaw_;
  ar & feat.pathLengthsProcessed_;
  ar & feat.processedFeaturesValid_;
  ar & feat.validLengthThresh_;
}

template <class Archive>
void serialize(Archive& ar, lanelet::map_learning::LaneletFeature& feat, const unsigned int /*version*/) {
  ar& boost::serialization::base_object<lanelet::map_learning::MapFeature>(feat);
  ar & feat.leftBoundary_;
  ar & feat.rightBoundary_;
  ar & feat.centerline_;
  ar & feat.reprType_;
}

template <class Archive>
void serialize(Archive& ar, lanelet::map_learning::Edge& edge, const unsigned int /*version*/) {
  ar & edge.el1_;
  ar & edge.el2_;
}

template <class Archive>
void serialize(Archive& ar, lanelet::map_learning::LaneData& lData, const unsigned int /*version*/) {
  ar & lData.roadBorders_;
  ar & lData.laneDividers_;
  ar & lData.compoundRoadBorders_;
  ar & lData.compoundLaneDividers_;
  ar & lData.compoundCenterlines_;
  ar & lData.laneletFeatures_;
  ar & lData.associatedCpdRoadBorderIndices_;
  ar & lData.associatedCpdLaneDividerIndices_;
  ar & lData.associatedCpdCenterlineIndices_;
  ar & lData.edges_;
}

}  // namespace serialization
}  // namespace boost