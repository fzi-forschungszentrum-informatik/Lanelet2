#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <map>
#include <vector>

#include "lanelet2_ml_converter/MapData.h"
#include "lanelet2_ml_converter/MapDataInterface.h"
#include "lanelet2_ml_converter/MapFeatures.h"
#include "lanelet2_ml_converter/Utils.h"
#include "lanelet2_python/internal/converter.h"
#include "lanelet2_python/internal/eigen_converter.h"

using namespace boost::python;
using namespace lanelet;
using namespace lanelet::ml_converter;

class MapFeatureWrap : public MapFeature, public wrapper<MapFeature> {
 public:
  std::vector<VectorXd> computeFeatureVectors(bool onlyPoints, bool pointsIn2d) const {
    return this->get_override("computeFeatureVectors")(onlyPoints, pointsIn2d);
  }
  bool process(const OrientedRect &bbox, const ParametrizationType &paramType, int32_t nPoints) {
    return this->get_override("process")(bbox, paramType, nPoints);
  }
};

class LineStringFeatureWrap : public LineStringFeature, public wrapper<LineStringFeature> {
 public:
  std::vector<VectorXd> computeFeatureVectors(bool onlyPoints, bool pointsIn2d) const {
    return this->get_override("computeFeatureVectors")(onlyPoints, pointsIn2d);
  }
  bool process(const OrientedRect &bbox, const ParametrizationType &paramType, int32_t nPoints) {
    return this->get_override("process")(bbox, paramType, nPoints);
  }
  std::vector<MatrixXd> pointMatrices(bool pointsIn2d) const { return this->get_override("pointMatrices")(pointsIn2d); }
};

class LaneLineStringFeatureWrap : public LaneLineStringFeature, public wrapper<LaneLineStringFeature> {
 public:
  LaneLineStringFeatureWrap() {}

  LaneLineStringFeatureWrap(const BasicLineString3d &feature, Id mapID, LineStringType type,
                            const std::vector<Id> &laneletID, bool inverted)
      : LaneLineStringFeature(feature, mapID, type, laneletID, inverted) {}

  std::vector<VectorXd> computeFeatureVectors(bool onlyPoints, bool pointsIn2d) const {
    if (override f = this->get_override("computeFeatureVectors")) return f(onlyPoints, pointsIn2d);
    return LaneLineStringFeature::computeFeatureVectors(onlyPoints, pointsIn2d);
  }
  std::vector<VectorXd> default_computeFeatureVectors(bool onlyPoints, bool pointsIn2d) const {
    return this->LaneLineStringFeature::computeFeatureVectors(onlyPoints, pointsIn2d);
  }
  bool process(const OrientedRect &bbox, const ParametrizationType &paramType, int32_t nPoints) {
    if (override f = this->get_override("process")) return f(bbox, paramType, nPoints);
    return LaneLineStringFeature::process(bbox, paramType, nPoints);
  }
  bool default_process(const OrientedRect &bbox, const ParametrizationType &paramType, int32_t nPoints) {
    return this->LaneLineStringFeature::process(bbox, paramType, nPoints);
  }
  std::vector<MatrixXd> pointMatrices(bool pointsIn2d) const {
    if (override f = this->get_override("pointMatrices")) return f(pointsIn2d);
    return LaneLineStringFeature::pointMatrices(pointsIn2d);
  }
  std::vector<MatrixXd> default_pointMatrices(bool pointsIn2d) const {
    return this->LaneLineStringFeature::pointMatrices(pointsIn2d);
  }
};

class CompoundLaneLineStringFeatureWrap : public CompoundLaneLineStringFeature,
                                          public wrapper<CompoundLaneLineStringFeature> {
 public:
  CompoundLaneLineStringFeatureWrap() {}

  CompoundLaneLineStringFeatureWrap(const LaneLineStringFeatureList &features, LineStringType compoundType)
      : CompoundLaneLineStringFeature(features, compoundType) {}

  std::vector<VectorXd> computeFeatureVectors(bool onlyPoints, bool pointsIn2d) const {
    if (override f = this->get_override("computeFeatureVectors")) return f(onlyPoints, pointsIn2d);
    return CompoundLaneLineStringFeature::computeFeatureVectors(onlyPoints, pointsIn2d);
  }
  std::vector<VectorXd> default_computeFeatureVectors(bool onlyPoints, bool pointsIn2d) const {
    return this->CompoundLaneLineStringFeature::computeFeatureVectors(onlyPoints, pointsIn2d);
  }
  bool process(const OrientedRect &bbox, const ParametrizationType &paramType, int32_t nPoints) {
    if (override f = this->get_override("process")) return f(bbox, paramType, nPoints);
    return CompoundLaneLineStringFeature::process(bbox, paramType, nPoints);
  }
  bool default_process(const OrientedRect &bbox, const ParametrizationType &paramType, int32_t nPoints) {
    return this->CompoundLaneLineStringFeature::process(bbox, paramType, nPoints);
  }
  std::vector<MatrixXd> pointMatrices(bool pointsIn2d) const {
    if (override f = this->get_override("pointMatrices")) return f(pointsIn2d);
    return CompoundLaneLineStringFeature::pointMatrices(pointsIn2d);
  }
  std::vector<MatrixXd> default_pointMatrices(bool pointsIn2d) const {
    return this->CompoundLaneLineStringFeature::pointMatrices(pointsIn2d);
  }
};

class LaneletFeatureWrap : public LaneletFeature, public wrapper<LaneletFeature> {
 public:
  LaneletFeatureWrap() {}

  LaneletFeatureWrap(LaneLineStringFeaturePtr leftBoundary, LaneLineStringFeaturePtr rightBoundary,
                     LaneLineStringFeaturePtr centerline, Id mapID)
      : LaneletFeature(leftBoundary, rightBoundary, centerline, mapID) {}
  LaneletFeatureWrap(const ConstLanelet &ll) : LaneletFeature(ll) {}

  std::vector<VectorXd> computeFeatureVectors(bool onlyPoints, bool pointsIn2d) const {
    if (override f = this->get_override("computeFeatureVectors")) return f(onlyPoints, pointsIn2d);
    return LaneletFeature::computeFeatureVectors(onlyPoints, pointsIn2d);
  }
  std::vector<VectorXd> default_computeFeatureVectors(bool onlyPoints, bool pointsIn2d) const {
    return this->LaneletFeature::computeFeatureVectors(onlyPoints, pointsIn2d);
  }
  bool process(const OrientedRect &bbox, const ParametrizationType &paramType, int32_t nPoints) {
    if (override f = this->get_override("process")) return f(bbox, paramType, nPoints);
    return LaneletFeature::process(bbox, paramType, nPoints);
  }
  bool default_process(const OrientedRect &bbox, const ParametrizationType &paramType, int32_t nPoints) {
    return this->LaneletFeature::process(bbox, paramType, nPoints);
  }
};

template <typename T>
struct DictToMapConverter {
  DictToMapConverter() { converter::registry::push_back(&convertible, &construct, type_id<T>()); }
  static void *convertible(PyObject *obj) {
    if (!PyDict_CheckExact(obj)) {  // NOLINT
      return nullptr;
    }
    return obj;
  }
  static void construct(PyObject *obj, converter::rvalue_from_python_stage1_data *data) {
    dict d(borrowed(obj));
    list keys = d.keys();
    list values = d.values();
    T map;
    for (auto i = 0u; i < len(keys); ++i) {
      typename T::key_type key = extract<typename T::key_type>(keys[i]);
      typename T::mapped_type value = extract<typename T::mapped_type>(values[i]);
      map.insert(std::make_pair(key, value));
    }
    using StorageType = converter::rvalue_from_python_storage<T>;
    void *storage = reinterpret_cast<StorageType *>(data)->storage.bytes;  // NOLINT
    new (storage) T(map);
    data->convertible = storage;
  }
};

BOOST_PYTHON_MODULE(PYTHON_API_MODULE_NAME) {  // NOLINT

  Py_Initialize();
  np::initialize();

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

  class_<OrientedRect>("OrientedRect", "Oriented rectangle for feature crop area", no_init)
      .add_property("bounds", make_function(&OrientedRect::bounds_const, return_value_policy<copy_const_reference>()));

  def("getRotatedRect", &getRotatedRect);
  def("extractSubmap", &extractSubmap);
  def("isRoadBorder", &isRoadBorder);
  def("bdTypeToEnum", &bdTypeToEnum);
  def("teTypeToEnum", &teTypeToEnum);
  def("resampleLineString", &resampleLineString);
  def("cutLineString", &cutLineString);
  def("saveLaneData", &saveLaneData);
  def("loadLaneData", &loadLaneData);
  def("saveLaneDataMultiFile", &saveLaneDataMultiFile);
  def("loadLaneDataMultiFile", &loadLaneDataMultiFile);

  class_<MapFeatureWrap, boost::noncopyable>("MapFeature", "Abstract base map feature class", no_init)
      .add_property("wasCut", &MapFeature::wasCut)
      .add_property("mapID", &MapFeature::mapID)
      .add_property("initialized", &MapFeature::initialized)
      .add_property("valid", &MapFeature::valid)
      .def("computeFeatureVectors", pure_virtual(&MapFeature::computeFeatureVectors))
      .def("process", pure_virtual(&MapFeature::process));

  class_<LineStringFeatureWrap, bases<MapFeature>, boost::noncopyable>("LineStringFeature",
                                                                       "Abstract line string feature class", no_init)
      .add_property("rawFeature",
                    make_function(&LineStringFeature::rawFeature, return_value_policy<copy_const_reference>()))
      .def("computeFeatureVectors", pure_virtual(&LineStringFeature::computeFeatureVectors))
      .def("process", pure_virtual(&LineStringFeature::process))
      .def("pointMatrices", pure_virtual(&LineStringFeature::pointMatrices));

  class_<LaneLineStringFeatureWrap, bases<LineStringFeature>, LaneLineStringFeaturePtr, boost::noncopyable>(
      "LaneLineStringFeature", "Lane line string feature class",
      init<BasicLineString3d, Id, LineStringType, Ids, bool>())
      .def(init<>())
      .add_property("cutFeature",
                    make_function(&LaneLineStringFeature::cutFeature, return_value_policy<copy_const_reference>()))
      .add_property("cutAndResampledFeature", make_function(&LaneLineStringFeature::cutAndResampledFeature,
                                                            return_value_policy<copy_const_reference>()))
      .add_property("cutResampledAndTransformedFeature",
                    make_function(&LaneLineStringFeature::cutResampledAndTransformedFeature,
                                  return_value_policy<copy_const_reference>()))
      .add_property("type", &LaneLineStringFeature::type)
      .add_property("inverted", &LaneLineStringFeature::inverted)
      .add_property("typeInt", &LaneLineStringFeature::typeInt)
      .add_property("laneletIDs",
                    make_function(&LaneLineStringFeature::laneletIDs, return_value_policy<copy_const_reference>()))
      .add_property("addLaneletID", &LaneLineStringFeature::addLaneletID)
      .def("computeFeatureVectors", &LaneLineStringFeature::computeFeatureVectors,
           &LaneLineStringFeatureWrap::default_computeFeatureVectors)
      .def("process", &LaneLineStringFeature::process, &LaneLineStringFeatureWrap::default_process)
      .def("pointMatrices", &LaneLineStringFeature::pointMatrices, &LaneLineStringFeatureWrap::default_pointMatrices);

  class_<LaneletFeatureWrap, bases<MapFeature>, LaneletFeaturePtr, boost::noncopyable>(
      "LaneletFeature", "Lanelet feature class that contains lower level LaneLineStringFeatures",
      init<LaneLineStringFeaturePtr, LaneLineStringFeaturePtr, LaneLineStringFeaturePtr, Id>())
      .def(init<ConstLanelet>())
      .def(init<>())
      .add_property("leftBoundary", make_function(&LaneletFeature::leftBoundary))
      .add_property("rightBoundary", make_function(&LaneletFeature::rightBoundary))
      .add_property("centerline", make_function(&LaneletFeature::centerline))
      .def("setReprType", &LaneletFeature::setReprType)
      .def("computeFeatureVectors", &LaneletFeature::computeFeatureVectors,
           &LaneletFeatureWrap::default_computeFeatureVectors)
      .def("process", &LaneletFeature::process, &LaneletFeatureWrap::default_process);

  class_<CompoundLaneLineStringFeatureWrap, bases<LaneLineStringFeature>, CompoundLaneLineStringFeaturePtr,
         boost::noncopyable>("CompoundLaneLineStringFeature",
                             "Compound lane line string feature class that can trace back the individual features",
                             init<LaneLineStringFeatureList, LineStringType>())
      .def(init<>())
      .add_property("features", make_function(&CompoundLaneLineStringFeature::features))
      .add_property("pathLengthsRaw", make_function(&CompoundLaneLineStringFeature::pathLengthsRaw,
                                                    return_value_policy<copy_const_reference>()))
      .add_property("pathLengthsProcessed", make_function(&CompoundLaneLineStringFeature::pathLengthsProcessed,
                                                          return_value_policy<copy_const_reference>()))
      .add_property("processedFeaturesValid", make_function(&CompoundLaneLineStringFeature::processedFeaturesValid,
                                                            return_value_policy<copy_const_reference>()))
      .def("computeFeatureVectors", &CompoundLaneLineStringFeature::computeFeatureVectors,
           &CompoundLaneLineStringFeatureWrap::default_computeFeatureVectors)
      .def("process", &CompoundLaneLineStringFeature::process, &CompoundLaneLineStringFeatureWrap::default_process)
      .def("pointMatrices", &CompoundLaneLineStringFeature::pointMatrices,
           &CompoundLaneLineStringFeatureWrap::default_pointMatrices);

  class_<Edge>("Edge", "Struct of a lane graph edge", init<Id, Id, bool>())
      .def(init<>())
      .def_readwrite("el1", &Edge::el1_)
      .def_readwrite("el2", &Edge::el2_)
      .def_readwrite("isLaneChange", &Edge::isLaneChange_);

  {
    scope inLaneData =
        class_<LaneData, LaneDataPtr>("LaneData", "Class for holding, accessing and processing of lane data")
            .def(init<>())
            .def("build", &LaneData::build)
            .staticmethod("build")
            .def("processAll", &LaneData::processAll)
            .add_property("roadBorders",
                          make_function(&LaneData::roadBorders, return_value_policy<copy_const_reference>()))
            .add_property("laneDividers",
                          make_function(&LaneData::laneDividers, return_value_policy<copy_const_reference>()))
            .add_property("compoundRoadBorders",
                          make_function(&LaneData::compoundRoadBorders, return_value_policy<copy_const_reference>()))
            .add_property("compoundLaneDividers",
                          make_function(&LaneData::compoundLaneDividers, return_value_policy<copy_const_reference>()))
            .add_property("compoundCenterlines",
                          make_function(&LaneData::compoundCenterlines, return_value_policy<copy_const_reference>()))
            .add_property("validRoadBorders", &LaneData::validRoadBorders)
            .add_property("validLaneDividers", &LaneData::validLaneDividers)
            .add_property("validCompoundRoadBorders", &LaneData::validCompoundRoadBorders)
            .add_property("validCompoundLaneDividers", &LaneData::validCompoundLaneDividers)
            .add_property("validCompoundCenterlines", &LaneData::validCompoundCenterlines)
            .def("associatedCpdRoadBorders", &LaneData::associatedCpdRoadBorders)
            .def("associatedCpdLaneDividers", &LaneData::associatedCpdLaneDividers)
            .def("associatedCpdCenterlines", &LaneData::associatedCpdCenterlines)
            .add_property("laneletFeatures",
                          make_function(&LaneData::laneletFeatures, return_value_policy<copy_const_reference>()))
            .add_property("edges", make_function(&LaneData::edges, return_value_policy<copy_const_reference>()))
            .add_property("uuid", make_function(&LaneData::uuid, return_value_policy<copy_const_reference>()))
            .def("getTensorFeatureData", &LaneData::getTensorFeatureData);

    class_<LaneData::TensorFeatureData>("TensorFeatureData", "TensorFeatureData class for LaneData", init<>())
        .add_property("roadBorders", make_function(&LaneData::TensorFeatureData::roadBorders,
                                                   return_value_policy<copy_const_reference>()))
        .add_property("laneDividers", make_function(&LaneData::TensorFeatureData::laneDividers,
                                                    return_value_policy<copy_const_reference>()))
        .add_property("laneDividerTypes", make_function(&LaneData::TensorFeatureData::laneDividerTypes,
                                                        return_value_policy<copy_const_reference>()))
        .add_property("compoundRoadBorders", make_function(&LaneData::TensorFeatureData::compoundRoadBorders,
                                                           return_value_policy<copy_const_reference>()))
        .add_property("compoundLaneDividers", make_function(&LaneData::TensorFeatureData::compoundLaneDividers,
                                                            return_value_policy<copy_const_reference>()))
        .add_property("compoundLaneDividerTypes", make_function(&LaneData::TensorFeatureData::compoundLaneDividerTypes,
                                                                return_value_policy<copy_const_reference>()))
        .add_property("compoundCenterlines", make_function(&LaneData::TensorFeatureData::compoundCenterlines,
                                                           return_value_policy<copy_const_reference>()))
        .add_property("uuid",
                      make_function(&LaneData::TensorFeatureData::uuid, return_value_policy<copy_const_reference>()))
        .def("pointMatrixCpdRoadBorder", &LaneData::TensorFeatureData::pointMatrixCpdRoadBorder)
        .def("pointMatrixCpdLaneDivider", &LaneData::TensorFeatureData::pointMatrixCpdLaneDivider)
        .def("pointMatrixCpdCenterline", &LaneData::TensorFeatureData::pointMatrixCpdCenterline);
  }

  {
    scope inMapDataInterface =
        class_<MapDataInterface>("MapDataInterface", "Main Interface Class for processing of Lanelet maps",
                                 init<LaneletMapConstPtr>())
            .def(init<LaneletMapConstPtr, MapDataInterface::Configuration>())
            .add_property("config",
                          make_function(&MapDataInterface::config, return_value_policy<copy_const_reference>()))
            .def("setCurrPosAndExtractSubmap", &MapDataInterface::setCurrPosAndExtractSubmap)
            .def("laneData", &MapDataInterface::laneData)
            .def("teData", &MapDataInterface::teData)
            .def("laneDataBatch", &MapDataInterface::laneDataBatch)
            .def("laneTEDataBatch", &MapDataInterface::laneTEDataBatch);

    class_<MapDataInterface::Configuration>("Configuration", "Configuration class for MapDataInterface", init<>())
        .def(init<LaneletRepresentationType, ParametrizationType, double, double, int>())
        .def_readwrite("reprType", &MapDataInterface::Configuration::reprType)
        .def_readwrite("paramType", &MapDataInterface::Configuration::paramType)
        .def_readwrite("submapExtentLongitudinal", &MapDataInterface::Configuration::submapExtentLongitudinal)
        .def_readwrite("submapExtentLateral", &MapDataInterface::Configuration::submapExtentLateral)
        .def_readwrite("nPoints", &MapDataInterface::Configuration::nPoints);
  }

  // Eigen, stl etc. converters
  converters::convertMatrix<MatrixXd>(true);
  converters::convertVector<VectorXd>(true);

  converters::VectorToListConverter<std::vector<MatrixXd>>();
  converters::VectorToListConverter<BasicLineStrings3d>();
  converters::VectorToListConverter<LaneLineStringFeatureList>();
  converters::VectorToListConverter<CompoundLaneLineStringFeatureList>();
  converters::VectorToListConverter<
      boost::geometry::model::ring<BasicPoint2d, true, true, std::vector, std::allocator>>();
  converters::VectorToListConverter<std::vector<double>>();
  converters::VectorToListConverter<std::vector<int>>();
  converters::VectorToListConverter<std::vector<LaneDataPtr>>();
  converters::VectorToListConverter<std::vector<Edge>>();
  converters::IterableConverter()
      .fromPython<std::vector<MatrixXd>>()
      .fromPython<BasicLineString3d>()
      .fromPython<BasicLineStrings3d>()
      .fromPython<std::vector<BasicPoint2d>>()
      .fromPython<std::vector<double>>()
      .fromPython<std::vector<std::string>>()
      .fromPython<std::vector<LaneDataPtr>>()
      .fromPython<LaneLineStringFeatureList>()
      .fromPython<CompoundLaneLineStringFeatureList>();
  converters::MapToDictConverter<LaneLineStringFeatures>();
  converters::MapToDictConverter<LaneletFeatures>();
  converters::MapToDictConverter<Edges>();
  DictToMapConverter<LaneLineStringFeatures>();
  DictToMapConverter<LaneletFeatures>();
  class_<std::vector<bool>>("BoolList").def(vector_indexing_suite<std::vector<bool>>());

  def("toPointMatrix", &toPointMatrix);

  implicitly_convertible<routing::RoutingGraphPtr, routing::RoutingGraphConstPtr>();
}
