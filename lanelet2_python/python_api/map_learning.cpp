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

class GetPointsMatricesWrapper {
 public:
  template <class T>
  static std::vector<Eigen::MatrixXd> getPointsMatricesMap(const std::map<Id, T>& mapFeatures, bool pointsIn2d) {
    return getPointsMatrices(mapFeatures, pointsIn2d);
  }

  template <class T>
  static std::vector<Eigen::MatrixXd> getPointsMatricesVector(const std::vector<T>& mapFeatures, bool pointsIn2d) {
    return getPointsMatrices(mapFeatures, pointsIn2d);
  }

  static void wrap() {
    // Expose the templated functions to Python
    def("getPointsMatricesMap", &GetPointsMatricesWrapper::getPointsMatricesMap<LaneLineStringFeature>);
    def("getPointsMatricesVector", &GetPointsMatricesWrapper::getPointsMatricesVector<LaneLineStringFeature>);
    def("getPointsMatricesMap", &GetPointsMatricesWrapper::getPointsMatricesMap<CompoundLaneLineStringFeature>);
    def("getPointsMatricesVector", &GetPointsMatricesWrapper::getPointsMatricesVector<CompoundLaneLineStringFeature>);
  }
};

/// TODO: ADD VIRTUAL FUNCTIONS, CONSTRUCTORS, EIGEN TO NUMPY

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

  class_<MapFeature, boost::noncopyable>("MapFeature", "Abstract base map feature class", no_init)
      .add_property("wasCut", &MapFeature::wasCut)
      .add_property("mapID", &MapFeature::mapID)
      .add_property("initialized", &MapFeature::initialized)
      .add_property("valid", &MapFeature::valid)
      .def("computeFeatureVector", pure_virtual(&MapFeature::computeFeatureVector))
      .def("process", pure_virtual(&MapFeature::process));

  class_<LineStringFeature, bases<MapFeature>, boost::noncopyable>("LineStringFeature",
                                                                   "Abstract line string feature class", no_init)
      .add_property("rawFeature",
                    make_function(&LineStringFeature::rawFeature, return_value_policy<reference_existing_object>()))
      .def("computeFeatureVector", pure_virtual(&LineStringFeature::computeFeatureVector))
      .def("process", pure_virtual(&LineStringFeature::process))
      .def("pointMatrix", pure_virtual(&LineStringFeature::pointMatrix));

  class_<LaneLineStringFeature, bases<LineStringFeature>>("LaneLineStringFeature", "Lane line string feature class",
                                                          init<BasicLineString3d, Id, LineStringType, Id>())
      .def(init<>())
      .add_property("cutFeature",
                    make_function(&LaneLineStringFeature::cutFeature, return_value_policy<reference_existing_object>()))
      .add_property("cutAndResampledFeature", make_function(&LaneLineStringFeature::cutAndResampledFeature,
                                                            return_value_policy<reference_existing_object>()))
      .add_property("type", &LaneLineStringFeature::type)
      .add_property("typeInt", &LaneLineStringFeature::typeInt)
      .add_property("laneletIDs",
                    make_function(&LaneLineStringFeature::laneletIDs, return_value_policy<reference_existing_object>()))
      .add_property("addLaneletID", &LaneLineStringFeature::addLaneletID);

  class_<LaneletFeature, bases<MapFeature>>(
      "LaneletFeature", "Lanelet feature class that contains lower level LaneLineStringFeatures",
      init<LaneLineStringFeature, LaneLineStringFeature, LaneLineStringFeature, Id>())
      .def(init<ConstLanelet>())
      .def(init<>())
      .add_property("leftBoundary",
                    make_function(&LaneletFeature::leftBoundary, return_value_policy<reference_existing_object>()))
      .add_property("rightBoundary",
                    make_function(&LaneletFeature::rightBoundary, return_value_policy<reference_existing_object>()))
      .add_property("centerline",
                    make_function(&LaneletFeature::centerline, return_value_policy<reference_existing_object>()))
      .add_property("setReprType", &LaneletFeature::setReprType);

  class_<CompoundLaneLineStringFeature, bases<LaneLineStringFeature>>(
      "CompoundLaneLineStringFeature",
      "Compound lane line string feature class that can trace back the individual features",
      init<LaneLineStringFeatureList, LineStringType>())
      .def(init<>())
      .add_property("features", make_function(&CompoundLaneLineStringFeature::features,
                                              return_value_policy<reference_existing_object>()))
      .add_property("pathLengthsRaw", make_function(&CompoundLaneLineStringFeature::pathLengthsRaw,
                                                    return_value_policy<reference_existing_object>()))
      .add_property("pathLengthsProcessed", make_function(&CompoundLaneLineStringFeature::pathLengthsProcessed,
                                                          return_value_policy<reference_existing_object>()))
      .add_property("processedFeaturesValid", make_function(&CompoundLaneLineStringFeature::processedFeaturesValid,
                                                            return_value_policy<reference_existing_object>()));

  class_<Edge>("Edge");

  class_<LaneData>("LaneData");

  GetPointsMatricesWrapper::wrap();

  converters::numpy_array_to_eigen_matrix<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>();
  converters::python_list_to_eigen_matrix<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>();
  py::to_python_converter<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>,
      converters::eigen_matrix_to_numpy_array<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>>();
  converters::VectorToListConverter<std::vector<Eigen::MatrixXd>>();
  converters::IterableConverter().fromPython<std::vector<Eigen::MatrixXd>>().fromPython<BasicLineString3d>();
  class_<std::map<Id, LaneLineStringFeature>>("IdToLaneLineStringFeatureMap")
      .def(map_indexing_suite<std::map<Id, LaneLineStringFeature>>());
  class_<std::map<Id, LaneletFeature>>("IdToLaneletFeatureMap").def(map_indexing_suite<std::map<Id, LaneletFeature>>());
  class_<std::map<Id, CompoundLaneLineStringFeature>>("IdToCompoundLaneLineStringFeatureMap")
      .def(map_indexing_suite<std::map<Id, CompoundLaneLineStringFeature>>());
}
