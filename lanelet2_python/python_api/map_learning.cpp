#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <map>
#include <vector>

#include "lanelet2_map_learning/MapData.h"
#include "lanelet2_map_learning/MapDataInterface.h"
#include "lanelet2_map_learning/MapFeatures.h"
#include "lanelet2_python/internal/converter.h"
#include "lanelet2_python/internal/eigen_converter.h"

using namespace boost::python;
using namespace lanelet;
using namespace lanelet::map_learning;

/// TODO: ADD VIRTUAL FUNCTIONS, CONSTRUCTORS, EIGEN TO NUMPY
/// TODO: DEBUG EIGEN SEGFAULTS FROM PYTHON

class MapFeatureWrap : public MapFeature, public wrapper<MapFeature> {
 public:
  VectorXd computeFeatureVector(bool onlyPoints, bool pointsIn2d) const {
    return this->get_override("computeFeatureVector")(onlyPoints, pointsIn2d);
  }
  bool process(const OrientedRect &bbox, const ParametrizationType &paramType, int32_t nPoints) {
    return this->get_override("process")(bbox, paramType, nPoints);
  }
};

class LineStringFeatureWrap : public LineStringFeature, public wrapper<LineStringFeature> {
 public:
  VectorXd computeFeatureVector(bool onlyPoints, bool pointsIn2d) const {
    return this->get_override("computeFeatureVector")(onlyPoints, pointsIn2d);
  }
  bool process(const OrientedRect &bbox, const ParametrizationType &paramType, int32_t nPoints) {
    return this->get_override("process")(bbox, paramType, nPoints);
  }
  MatrixXd pointMatrix(bool pointsIn2d) const { return this->get_override("pointMatrix")(pointsIn2d); }
};

class LaneLineStringFeatureWrap : public LaneLineStringFeature, public wrapper<LaneLineStringFeature> {
 public:
  LaneLineStringFeatureWrap() {}

  LaneLineStringFeatureWrap(const BasicLineString3d &feature, Id mapID, LineStringType type, Id laneletID)
      : LaneLineStringFeature(feature, mapID, type, laneletID) {}

  VectorXd computeFeatureVector(bool onlyPoints, bool pointsIn2d) const {
    if (override f = this->get_override("computeFeatureVector")) return f(onlyPoints, pointsIn2d);
    return LaneLineStringFeature::computeFeatureVector(onlyPoints, pointsIn2d);
  }
  VectorXd default_computeFeatureVector(bool onlyPoints, bool pointsIn2d) const {
    return this->LaneLineStringFeature::computeFeatureVector(onlyPoints, pointsIn2d);
  }
  bool process(const OrientedRect &bbox, const ParametrizationType &paramType, int32_t nPoints) {
    if (override f = this->get_override("process")) return f(bbox, paramType, nPoints);
    return LaneLineStringFeature::process(bbox, paramType, nPoints);
  }
  bool default_process(const OrientedRect &bbox, const ParametrizationType &paramType, int32_t nPoints) {
    return this->LaneLineStringFeature::process(bbox, paramType, nPoints);
  }
  MatrixXd pointMatrix(bool pointsIn2d) const {
    if (override f = this->get_override("pointMatrix")) return f(pointsIn2d);
    return LaneLineStringFeature::pointMatrix(pointsIn2d);
  }
  MatrixXd default_pointMatrix(bool pointsIn2d) const { return this->LaneLineStringFeature::pointMatrix(pointsIn2d); }
};

class CompoundLaneLineStringFeatureWrap : public CompoundLaneLineStringFeature,
                                          public wrapper<CompoundLaneLineStringFeature> {
 public:
  CompoundLaneLineStringFeatureWrap() {}

  CompoundLaneLineStringFeatureWrap(const LaneLineStringFeatureList &features, LineStringType compoundType)
      : CompoundLaneLineStringFeature(features, compoundType) {}

  VectorXd computeFeatureVector(bool onlyPoints, bool pointsIn2d) const {
    if (override f = this->get_override("computeFeatureVector")) return f(onlyPoints, pointsIn2d);
    return CompoundLaneLineStringFeature::computeFeatureVector(onlyPoints, pointsIn2d);
  }
  VectorXd default_computeFeatureVector(bool onlyPoints, bool pointsIn2d) const {
    return this->CompoundLaneLineStringFeature::computeFeatureVector(onlyPoints, pointsIn2d);
  }
  bool process(const OrientedRect &bbox, const ParametrizationType &paramType, int32_t nPoints) {
    if (override f = this->get_override("process")) return f(bbox, paramType, nPoints);
    return CompoundLaneLineStringFeature::process(bbox, paramType, nPoints);
  }
  bool default_process(const OrientedRect &bbox, const ParametrizationType &paramType, int32_t nPoints) {
    return this->CompoundLaneLineStringFeature::process(bbox, paramType, nPoints);
  }
  MatrixXd pointMatrix(bool pointsIn2d) const {
    if (override f = this->get_override("pointMatrix")) return f(pointsIn2d);
    return CompoundLaneLineStringFeature::pointMatrix(pointsIn2d);
  }
  MatrixXd default_pointMatrix(bool pointsIn2d) const {
    return this->CompoundLaneLineStringFeature::pointMatrix(pointsIn2d);
  }
};

class LaneletFeatureWrap : public LaneletFeature, public wrapper<LaneletFeature> {
 public:
  LaneletFeatureWrap() {}

  LaneletFeatureWrap(const LaneLineStringFeature &leftBoundary, const LaneLineStringFeature &rightBoundary,
                     const LaneLineStringFeature &centerline, Id mapID)
      : LaneletFeature(leftBoundary, rightBoundary, centerline, mapID) {}
  LaneletFeatureWrap(const ConstLanelet &ll) : LaneletFeature(ll) {}

  VectorXd computeFeatureVector(bool onlyPoints, bool pointsIn2d) const {
    if (override f = this->get_override("computeFeatureVector")) return f(onlyPoints, pointsIn2d);
    return LaneletFeature::computeFeatureVector(onlyPoints, pointsIn2d);
  }
  VectorXd default_computeFeatureVector(bool onlyPoints, bool pointsIn2d) const {
    return this->LaneletFeature::computeFeatureVector(onlyPoints, pointsIn2d);
  }
  bool process(const OrientedRect &bbox, const ParametrizationType &paramType, int32_t nPoints) {
    if (override f = this->get_override("process")) return f(bbox, paramType, nPoints);
    return LaneletFeature::process(bbox, paramType, nPoints);
  }
  bool default_process(const OrientedRect &bbox, const ParametrizationType &paramType, int32_t nPoints) {
    return this->LaneletFeature::process(bbox, paramType, nPoints);
  }
};

BOOST_PYTHON_MODULE(PYTHON_API_MODULE_NAME) {  // NOLINT

  enum_<LaneletRepresentationType>("LaneletRepresentationType")
      .value("Centerline", LaneletRepresentationType::Centerline)
      .value("Boundaries", LaneletRepresentationType::Boundaries);

  enum_<ParametrizationType>("ParametrizationType")
      .value("Bezier", ParametrizationType::Bezier)
      .value("BezierEndpointFixed", ParametrizationType::BezierEndpointFixed)
      .value("LineString", ParametrizationType::LineString);

  enum_<LineStringType>("LineStringType")
      .value("RoadBorder", LineStringType::RoadBorder)
      .value("Dashed", LineStringType::Dashed)
      .value("Solid", LineStringType::Solid)
      .value("Mixed", LineStringType::Mixed)
      .value("Virtual", LineStringType::Virtual)
      .value("Centerline", LineStringType::Centerline)
      .value("Unknown", LineStringType::Unknown);

  class_<MapFeatureWrap, boost::noncopyable>("MapFeature", "Abstract base map feature class", no_init)
      .add_property("wasCut", &MapFeature::wasCut)
      .add_property("mapID", &MapFeature::mapID)
      .add_property("initialized", &MapFeature::initialized)
      .add_property("valid", &MapFeature::valid)
      .def("computeFeatureVector", pure_virtual(&MapFeature::computeFeatureVector))
      .def("process", pure_virtual(&MapFeature::process));

  class_<LineStringFeatureWrap, bases<MapFeature>, boost::noncopyable>("LineStringFeature",
                                                                       "Abstract line string feature class", no_init)
      .add_property("rawFeature",
                    make_function(&LineStringFeature::rawFeature, return_value_policy<copy_const_reference>()))
      .def("computeFeatureVector", pure_virtual(&LineStringFeature::computeFeatureVector))
      .def("process", pure_virtual(&LineStringFeature::process))
      .def("pointMatrix", pure_virtual(&LineStringFeature::pointMatrix));

  class_<LaneLineStringFeatureWrap, bases<LineStringFeature>, boost::noncopyable>(
      "LaneLineStringFeature", "Lane line string feature class", init<BasicLineString3d, Id, LineStringType, Id>())
      .def(init<>())
      .add_property("cutFeature",
                    make_function(&LaneLineStringFeature::cutFeature, return_value_policy<copy_const_reference>()))
      .add_property("cutAndResampledFeature", make_function(&LaneLineStringFeature::cutAndResampledFeature,
                                                            return_value_policy<copy_const_reference>()))
      .add_property("type", &LaneLineStringFeature::type)
      .add_property("typeInt", &LaneLineStringFeature::typeInt)
      .add_property("laneletIDs",
                    make_function(&LaneLineStringFeature::laneletIDs, return_value_policy<copy_const_reference>()))
      .add_property("addLaneletID", &LaneLineStringFeature::addLaneletID)
      .def("computeFeatureVector", &LaneLineStringFeature::computeFeatureVector,
           &LaneLineStringFeatureWrap::default_computeFeatureVector)
      .def("process", &LaneLineStringFeature::process, &LaneLineStringFeatureWrap::default_process)
      .def("pointMatrix", &LaneLineStringFeature::pointMatrix, &LaneLineStringFeatureWrap::default_pointMatrix);

  class_<LaneletFeatureWrap, bases<MapFeature>, boost::noncopyable>(
      "LaneletFeature", "Lanelet feature class that contains lower level LaneLineStringFeatures",
      init<LaneLineStringFeature, LaneLineStringFeature, LaneLineStringFeature, Id>())
      .def(init<ConstLanelet>())
      .def(init<>())
      .add_property("leftBoundary",
                    make_function(&LaneletFeature::leftBoundary, return_value_policy<copy_const_reference>()))
      .add_property("rightBoundary",
                    make_function(&LaneletFeature::rightBoundary, return_value_policy<copy_const_reference>()))
      .add_property("centerline",
                    make_function(&LaneletFeature::centerline, return_value_policy<copy_const_reference>()))
      .add_property("setReprType", &LaneletFeature::setReprType)
      .def("computeFeatureVector", &LaneletFeature::computeFeatureVector,
           &LaneletFeatureWrap::default_computeFeatureVector)
      .def("process", &LaneletFeature::process, &LaneletFeatureWrap::default_process);

  class_<CompoundLaneLineStringFeatureWrap, bases<LaneLineStringFeature>, boost::noncopyable>(
      "CompoundLaneLineStringFeature",
      "Compound lane line string feature class that can trace back the individual features",
      init<LaneLineStringFeatureList, LineStringType>())
      .def(init<>())
      .add_property("features", make_function(&CompoundLaneLineStringFeature::features,
                                              return_value_policy<copy_const_reference>()))
      .add_property("pathLengthsRaw", make_function(&CompoundLaneLineStringFeature::pathLengthsRaw,
                                                    return_value_policy<copy_const_reference>()))
      .add_property("pathLengthsProcessed", make_function(&CompoundLaneLineStringFeature::pathLengthsProcessed,
                                                          return_value_policy<copy_const_reference>()))
      .add_property("processedFeaturesValid", make_function(&CompoundLaneLineStringFeature::processedFeaturesValid,
                                                            return_value_policy<copy_const_reference>()))
      .def("computeFeatureVector", &CompoundLaneLineStringFeature::computeFeatureVector,
           &CompoundLaneLineStringFeatureWrap::default_computeFeatureVector)
      .def("process", &CompoundLaneLineStringFeature::process, &CompoundLaneLineStringFeatureWrap::default_process)
      .def("pointMatrix", &CompoundLaneLineStringFeature::pointMatrix,
           &CompoundLaneLineStringFeatureWrap::default_pointMatrix);

  class_<Edge>("Edge");

  class_<LaneData>("LaneData");

  // Eigen and stl converters
  converters::numpy_array_to_eigen_matrix<MatrixXd>();
  converters::python_list_to_eigen_matrix<MatrixXd>();
  py::to_python_converter<MatrixXd, converters::eigen_matrix_to_numpy_array<MatrixXd>>();

  converters::numpy_array_to_eigen_vector<VectorXd>();
  converters::python_list_to_eigen_vector<VectorXd>();
  py::to_python_converter<VectorXd, converters::eigen_vector_to_numpy_array<VectorXd>>();

  converters::VectorToListConverter<std::vector<MatrixXd>>();
  converters::VectorToListConverter<LaneLineStringFeatureList>();
  converters::VectorToListConverter<CompoundLaneLineStringFeatureList>();
  converters::IterableConverter()
      .fromPython<std::vector<MatrixXd>>()
      .fromPython<BasicLineString3d>()
      .fromPython<LaneLineStringFeatureList>()
      .fromPython<CompoundLaneLineStringFeatureList>();
  class_<std::map<Id, LaneLineStringFeature>>("LaneLineStringFeatures")
      .def(map_indexing_suite<std::map<Id, LaneLineStringFeature>>());
  class_<std::map<Id, LaneletFeature>>("LaneletFeatures").def(map_indexing_suite<LaneletFeatures>());
  converters::VectorToListConverter<std::vector<double>>();
  class_<std::vector<bool>>("BoolList").def(vector_indexing_suite<std::vector<bool>>());

  // overloaded template function instantiations
  def<std::vector<MatrixXd> (*)(const std::map<Id, LaneLineStringFeature> &, bool)>("getPointsMatrices",
                                                                                    &getPointsMatrices);
  def<std::vector<MatrixXd> (*)(const std::vector<LaneLineStringFeature> &, bool)>("getPointsMatrices",
                                                                                   &getPointsMatrices);
  def<std::vector<MatrixXd> (*)(const std::map<Id, CompoundLaneLineStringFeature> &, bool)>("getPointsMatrices",
                                                                                            &getPointsMatrices);
  def<std::vector<MatrixXd> (*)(const std::vector<CompoundLaneLineStringFeature> &, bool)>("getPointsMatrices",
                                                                                           &getPointsMatrices);

  def<MatrixXd (*)(const std::map<Id, LaneLineStringFeature> &, bool, bool)>("getFeatureVectorMatrix",
                                                                             &getFeatureVectorMatrix);
  def<MatrixXd (*)(const std::vector<LaneLineStringFeature> &, bool, bool)>("getFeatureVectorMatrix",
                                                                            &getFeatureVectorMatrix);
  def<MatrixXd (*)(const std::map<Id, CompoundLaneLineStringFeature> &, bool, bool)>("getFeatureVectorMatrix",
                                                                                     &getFeatureVectorMatrix);
  def<MatrixXd (*)(const std::vector<CompoundLaneLineStringFeature> &, bool, bool)>("getFeatureVectorMatrix",
                                                                                    &getFeatureVectorMatrix);
  def<MatrixXd (*)(const std::map<Id, LaneletFeature> &, bool, bool)>("getFeatureVectorMatrix",
                                                                      &getFeatureVectorMatrix);
  def<MatrixXd (*)(const std::vector<LaneletFeature> &, bool, bool)>("getFeatureVectorMatrix", &getFeatureVectorMatrix);

  def<bool (*)(std::map<Id, LaneLineStringFeature> &, const OrientedRect &, const ParametrizationType &, int32_t)>(
      "processFeatures", &processFeatures);
  def<bool (*)(std::vector<LaneLineStringFeature> &, const OrientedRect &, const ParametrizationType &, int32_t)>(
      "processFeatures", &processFeatures);
  def<bool (*)(std::map<Id, CompoundLaneLineStringFeature> &, const OrientedRect &, const ParametrizationType &,
               int32_t)>("processFeatures", &processFeatures);
  def<bool (*)(std::vector<CompoundLaneLineStringFeature> &, const OrientedRect &, const ParametrizationType &,
               int32_t)>("processFeatures", &processFeatures);
  def<bool (*)(std::map<Id, LaneletFeature> &, const OrientedRect &, const ParametrizationType &, int32_t)>(
      "processFeatures", &processFeatures);
  def<bool (*)(std::vector<LaneletFeature> &, const OrientedRect &, const ParametrizationType &, int32_t)>(
      "processFeatures", &processFeatures);
}
