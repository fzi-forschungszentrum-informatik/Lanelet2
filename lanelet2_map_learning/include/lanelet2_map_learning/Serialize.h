#pragma once

#include <boost/serialization/level.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/optional.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>

#include "MapData.h"

namespace boost {
namespace serialization {

// from https://stackoverflow.com/a/35074759
template <class Archive, class S, int Rows_, int Cols_, int Ops_, int MaxRows_, int MaxCols_>
inline void serialize(Archive& ar, Eigen::Matrix<S, Rows_, Cols_, Ops_, MaxRows_, MaxCols_>& matrix,
                      const unsigned int version) {
  int rows = matrix.rows();
  int cols = matrix.cols();
  ar& make_nvp("rows", rows);
  ar& make_nvp("cols", cols);
  matrix.resize(rows, cols);  // no-op if size does not change!

  // always save/load row-major
  for (int r = 0; r < rows; ++r)
    for (int c = 0; c < cols; ++c) ar& make_nvp("val", matrix(r, c));
}

// prevent unneccessary boost serialization xml tags
// template <class S, int Rows_, int Cols_, int Ops_, int MaxRows_, int MaxCols_>
// struct implementation_level_impl<Eigen::Matrix<S, Rows_, Cols_, Ops_, MaxRows_, MaxCols_>> {
//   typedef mpl::integral_c_tag tag;
//   typedef mpl::int_<object_serializable> type;
//   BOOST_STATIC_CONSTANT(int, value = implementation_level_impl::type::value);
// };

template <class Archive>
void serialize(Archive& ar, lanelet::map_learning::LaneletRepresentationType& type, const unsigned int /*version*/) {
  ar& BOOST_SERIALIZATION_NVP(type);
}

template <class Archive>
void serialize(Archive& ar, lanelet::map_learning::ParametrizationType& type, const unsigned int /*version*/) {
  ar& BOOST_SERIALIZATION_NVP(type);
}

template <class Archive>
void serialize(Archive& ar, lanelet::map_learning::LineStringType& type, const unsigned int /*version*/) {
  ar& BOOST_SERIALIZATION_NVP(type);
}

template <class Archive>
void serialize(Archive& ar, lanelet::map_learning::MapFeature& feat, const unsigned int /*version*/) {
  ar& BOOST_SERIALIZATION_NVP(feat.initialized_);
  ar& BOOST_SERIALIZATION_NVP(feat.wasCut_);
  ar& BOOST_SERIALIZATION_NVP(feat.valid_);
  ar& BOOST_SERIALIZATION_NVP(feat.mapID_);
}

template <class Archive>
void serialize(Archive& ar, lanelet::map_learning::LineStringFeature& feat, const unsigned int /*version*/) {
  ar& make_nvp("MapFeature", boost::serialization::base_object<lanelet::map_learning::MapFeature>(feat));
  ar& BOOST_SERIALIZATION_NVP(feat.rawFeature_);
}

template <class Archive>
void serialize(Archive& ar, lanelet::map_learning::LaneLineStringFeature& feat, const unsigned int /*version*/) {
  ar& make_nvp("LineStringFeature", boost::serialization::base_object<lanelet::map_learning::LineStringFeature>(feat));
  ar& BOOST_SERIALIZATION_NVP(feat.cutFeature_);
  ar& BOOST_SERIALIZATION_NVP(feat.cutAndResampledFeature_);
  ar& BOOST_SERIALIZATION_NVP(feat.type_);
  ar& BOOST_SERIALIZATION_NVP(feat.laneletIDs_);
}

template <class Archive>
void serialize(Archive& ar, lanelet::map_learning::CompoundLaneLineStringFeature& feat,
               const unsigned int /*version*/) {
  ar& make_nvp("LaneLineStringFeature",
               boost::serialization::base_object<lanelet::map_learning::LaneLineStringFeature>(feat));
  ar& BOOST_SERIALIZATION_NVP(feat.individualFeatures_);
  ar& BOOST_SERIALIZATION_NVP(feat.pathLengthsRaw_);
  ar& BOOST_SERIALIZATION_NVP(feat.pathLengthsProcessed_);
  ar& BOOST_SERIALIZATION_NVP(feat.processedFeaturesValid_);
  ar& BOOST_SERIALIZATION_NVP(feat.validLengthThresh_);
}

template <class Archive>
void serialize(Archive& ar, lanelet::map_learning::LaneletFeature& feat, const unsigned int /*version*/) {
  ar& make_nvp("MapFeature", boost::serialization::base_object<lanelet::map_learning::MapFeature>(feat));
  ar& BOOST_SERIALIZATION_NVP(feat.leftBoundary_);
  ar& BOOST_SERIALIZATION_NVP(feat.rightBoundary_);
  ar& BOOST_SERIALIZATION_NVP(feat.centerline_);
  ar& BOOST_SERIALIZATION_NVP(feat.reprType_);
}

template <class Archive>
void serialize(Archive& ar, lanelet::map_learning::Edge& edge, const unsigned int /*version*/) {
  ar& BOOST_SERIALIZATION_NVP(edge.el1_);
  ar& BOOST_SERIALIZATION_NVP(edge.el2_);
}

template <class Archive>
void serialize(Archive& ar, lanelet::map_learning::LaneData& lData, const unsigned int /*version*/) {
  ar& BOOST_SERIALIZATION_NVP(lData.roadBorders_);
  ar& BOOST_SERIALIZATION_NVP(lData.laneDividers_);
  ar& BOOST_SERIALIZATION_NVP(lData.compoundRoadBorders_);
  ar& BOOST_SERIALIZATION_NVP(lData.compoundLaneDividers_);
  ar& BOOST_SERIALIZATION_NVP(lData.compoundCenterlines_);
  ar& BOOST_SERIALIZATION_NVP(lData.laneletFeatures_);
  ar& BOOST_SERIALIZATION_NVP(lData.associatedCpdRoadBorderIndices_);
  ar& BOOST_SERIALIZATION_NVP(lData.associatedCpdLaneDividerIndices_);
  ar& BOOST_SERIALIZATION_NVP(lData.associatedCpdCenterlineIndices_);
  ar& BOOST_SERIALIZATION_NVP(lData.edges_);
}

}  // namespace serialization
}  // namespace boost

// prevent unneccessary boost serialization xml tags
// BOOST_CLASS_IMPLEMENTATION(lanelet::map_learning::LaneData, boost::serialization::object_serializable);
// BOOST_CLASS_IMPLEMENTATION(lanelet::map_learning::Edge, boost::serialization::object_serializable);
// BOOST_CLASS_IMPLEMENTATION(lanelet::map_learning::LaneletFeature, boost::serialization::object_serializable);
// BOOST_CLASS_IMPLEMENTATION(lanelet::map_learning::LaneLineStringFeature, boost::serialization::object_serializable);
// BOOST_CLASS_IMPLEMENTATION(lanelet::map_learning::CompoundLaneLineStringFeature,
//                            boost::serialization::object_serializable);
// BOOST_CLASS_IMPLEMENTATION(lanelet::map_learning::LineStringFeature, boost::serialization::object_serializable);
// BOOST_CLASS_IMPLEMENTATION(lanelet::map_learning::MapFeature, boost::serialization::object_serializable);
// BOOST_CLASS_IMPLEMENTATION(lanelet::map_learning::LineStringType, boost::serialization::object_serializable);
// BOOST_CLASS_IMPLEMENTATION(lanelet::map_learning::ParametrizationType, boost::serialization::object_serializable);
// BOOST_CLASS_IMPLEMENTATION(lanelet::map_learning::LaneletRepresentationType,
// boost::serialization::object_serializable);
//
// BOOST_CLASS_IMPLEMENTATION(lanelet::map_learning::LaneLineStringFeatures, boost::serialization::object_serializable);
// BOOST_CLASS_IMPLEMENTATION(lanelet::map_learning::LaneLineStringFeatureList,
// boost::serialization::object_serializable);
// BOOST_CLASS_IMPLEMENTATION(lanelet::map_learning::CompoundLaneLineStringFeatureList,
//                            boost::serialization::object_serializable);
// BOOST_CLASS_IMPLEMENTATION(lanelet::map_learning::LaneletFeatures, boost::serialization::object_serializable);
// BOOST_CLASS_IMPLEMENTATION(lanelet::BasicLineString3d, boost::serialization::object_serializable);
