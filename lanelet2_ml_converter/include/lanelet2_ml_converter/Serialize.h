#pragma once

#include <boost/serialization/level.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/optional.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/shared_ptr.hpp>
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
void serialize(Archive& ar, lanelet::ml_converter::LaneletRepresentationType& type, const unsigned int /*version*/) {
  ar& BOOST_SERIALIZATION_NVP(type);
}

template <class Archive>
void serialize(Archive& ar, lanelet::ml_converter::ParametrizationType& type, const unsigned int /*version*/) {
  ar& BOOST_SERIALIZATION_NVP(type);
}

template <class Archive>
void serialize(Archive& ar, lanelet::ml_converter::LineStringType& type, const unsigned int /*version*/) {
  ar& BOOST_SERIALIZATION_NVP(type);
}

template <class Archive>
void serialize(Archive& ar, lanelet::ml_converter::MapInstance& feat, const unsigned int /*version*/) {
  ar& BOOST_SERIALIZATION_NVP(feat.initialized_);
  ar& BOOST_SERIALIZATION_NVP(feat.wasCut_);
  ar& BOOST_SERIALIZATION_NVP(feat.valid_);
  ar& BOOST_SERIALIZATION_NVP(feat.mapID_);
}

template <class Archive>
void serialize(Archive& ar, lanelet::ml_converter::LineStringInstance& feat, const unsigned int /*version*/) {
  ar& make_nvp("MapInstance", boost::serialization::base_object<lanelet::ml_converter::MapInstance>(feat));
  ar& BOOST_SERIALIZATION_NVP(feat.rawInstance_);
}

template <class Archive>
void serialize(Archive& ar, lanelet::ml_converter::LaneLineStringInstance& feat, const unsigned int /*version*/) {
  ar& make_nvp("LineStringInstance",
               boost::serialization::base_object<lanelet::ml_converter::LineStringInstance>(feat));
  ar& BOOST_SERIALIZATION_NVP(feat.cutInstances_);
  ar& BOOST_SERIALIZATION_NVP(feat.cutAndResampledInstances_);
  ar& BOOST_SERIALIZATION_NVP(feat.cutResampledAndTransformedInstances_);
  ar& BOOST_SERIALIZATION_NVP(feat.type_);
  ar& BOOST_SERIALIZATION_NVP(feat.inverted_);
  ar& BOOST_SERIALIZATION_NVP(feat.laneletIDs_);
}

template <class Archive>
void serialize(Archive& ar, lanelet::ml_converter::CompoundLaneLineStringInstance& feat,
               const unsigned int /*version*/) {
  ar& make_nvp("LaneLineStringInstance",
               boost::serialization::base_object<lanelet::ml_converter::LaneLineStringInstance>(feat));
  ar& BOOST_SERIALIZATION_NVP(feat.individualInstances_);
  ar& BOOST_SERIALIZATION_NVP(feat.pathLengthsRaw_);
  ar& BOOST_SERIALIZATION_NVP(feat.pathLengthsProcessed_);
  ar& BOOST_SERIALIZATION_NVP(feat.processedInstancesValid_);
  ar& BOOST_SERIALIZATION_NVP(feat.validLengthThresh_);
}

template <class Archive>
void serialize(Archive& ar, lanelet::ml_converter::LaneletInstance& feat, const unsigned int /*version*/) {
  ar& make_nvp("MapInstance", boost::serialization::base_object<lanelet::ml_converter::MapInstance>(feat));
  ar& BOOST_SERIALIZATION_NVP(feat.leftBoundary_);
  ar& BOOST_SERIALIZATION_NVP(feat.rightBoundary_);
  ar& BOOST_SERIALIZATION_NVP(feat.centerline_);
  ar& BOOST_SERIALIZATION_NVP(feat.reprType_);
}

template <class Archive>
void serialize(Archive& ar, lanelet::ml_converter::Edge& edge, const unsigned int /*version*/) {
  ar& BOOST_SERIALIZATION_NVP(edge.el1_);
  ar& BOOST_SERIALIZATION_NVP(edge.el2_);
}

template <class Archive>
void serialize(Archive& ar, lanelet::ml_converter::LaneData& lData, const unsigned int /*version*/) {
  ar& BOOST_SERIALIZATION_NVP(lData.roadBorders_);
  ar& BOOST_SERIALIZATION_NVP(lData.laneDividers_);
  ar& BOOST_SERIALIZATION_NVP(lData.compoundRoadBorders_);
  ar& BOOST_SERIALIZATION_NVP(lData.compoundLaneDividers_);
  ar& BOOST_SERIALIZATION_NVP(lData.compoundCenterlines_);
  ar& BOOST_SERIALIZATION_NVP(lData.laneletInstances_);
  ar& BOOST_SERIALIZATION_NVP(lData.associatedCpdRoadBorderIndices_);
  ar& BOOST_SERIALIZATION_NVP(lData.associatedCpdLaneDividerIndices_);
  ar& BOOST_SERIALIZATION_NVP(lData.associatedCpdCenterlineIndices_);
  ar& BOOST_SERIALIZATION_NVP(lData.edges_);
}

}  // namespace serialization
}  // namespace boost

// prevent unneccessary boost serialization xml tags
// BOOST_CLASS_IMPLEMENTATION(lanelet::ml_converter::LaneData, boost::serialization::object_serializable);
// BOOST_CLASS_IMPLEMENTATION(lanelet::ml_converter::Edge, boost::serialization::object_serializable);
// BOOST_CLASS_IMPLEMENTATION(lanelet::ml_converter::LaneletInstance, boost::serialization::object_serializable);
// BOOST_CLASS_IMPLEMENTATION(lanelet::ml_converter::LaneLineStringInstance, boost::serialization::object_serializable);
// BOOST_CLASS_IMPLEMENTATION(lanelet::ml_converter::CompoundLaneLineStringInstance,
//                            boost::serialization::object_serializable);
// BOOST_CLASS_IMPLEMENTATION(lanelet::ml_converter::LineStringInstance, boost::serialization::object_serializable);
// BOOST_CLASS_IMPLEMENTATION(lanelet::ml_converter::MapInstance, boost::serialization::object_serializable);
// BOOST_CLASS_IMPLEMENTATION(lanelet::ml_converter::LineStringType, boost::serialization::object_serializable);
// BOOST_CLASS_IMPLEMENTATION(lanelet::ml_converter::ParametrizationType, boost::serialization::object_serializable);
// BOOST_CLASS_IMPLEMENTATION(lanelet::ml_converter::LaneletRepresentationType,
// boost::serialization::object_serializable);
//
// BOOST_CLASS_IMPLEMENTATION(lanelet::ml_converter::LaneLineStringInstances,
// boost::serialization::object_serializable);
// BOOST_CLASS_IMPLEMENTATION(lanelet::ml_converter::LaneLineStringInstanceList,
// boost::serialization::object_serializable);
// BOOST_CLASS_IMPLEMENTATION(lanelet::ml_converter::CompoundLaneLineStringInstanceList,
//                            boost::serialization::object_serializable);
// BOOST_CLASS_IMPLEMENTATION(lanelet::ml_converter::LaneletInstances, boost::serialization::object_serializable);
// BOOST_CLASS_IMPLEMENTATION(lanelet::BasicLineString3d, boost::serialization::object_serializable);
