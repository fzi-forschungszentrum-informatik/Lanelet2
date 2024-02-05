#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <map>
#include <vector>

#include "lanelet2_ml_converter/MapData.h"
#include "lanelet2_ml_converter/MapDataInterface.h"
#include "lanelet2_ml_converter/MapInstances.h"
#include "lanelet2_ml_converter/Utils.h"
#include "lanelet2_python/internal/converter.h"
#include "lanelet2_python/internal/eigen_converter.h"

using namespace boost::python;
using namespace lanelet;
using namespace lanelet::ml_converter;

class MapInstanceWrap : public MapInstance, public wrapper<MapInstance> {
 public:
  std::vector<VectorXd> computeInstanceVectors(bool onlyPoints, bool pointsIn2d) const {
    return this->get_override("computeInstanceVectors")(onlyPoints, pointsIn2d);
  }
  bool process(const OrientedRect &bbox, const ParametrizationType &paramType, int32_t nPoints, double pitch,
               double roll) {
    return this->get_override("process")(bbox, paramType, nPoints, pitch, roll);
  }
};

class LineStringInstanceWrap : public LineStringInstance, public wrapper<LineStringInstance> {
 public:
  std::vector<VectorXd> computeInstanceVectors(bool onlyPoints, bool pointsIn2d) const {
    return this->get_override("computeInstanceVectors")(onlyPoints, pointsIn2d);
  }
  bool process(const OrientedRect &bbox, const ParametrizationType &paramType, int32_t nPoints, double pitch,
               double roll) {
    return this->get_override("process")(bbox, paramType, nPoints, pitch, roll);
  }
  std::vector<MatrixXd> pointMatrices(bool pointsIn2d) const { return this->get_override("pointMatrices")(pointsIn2d); }
};

class LaneLineStringInstanceWrap : public LaneLineStringInstance, public wrapper<LaneLineStringInstance> {
 public:
  LaneLineStringInstanceWrap() {}

  LaneLineStringInstanceWrap(const BasicLineString3d &feature, Id mapID, LineStringType type,
                             const std::vector<Id> &laneletID, bool inverted)
      : LaneLineStringInstance(feature, mapID, type, laneletID, inverted) {}

  std::vector<VectorXd> computeInstanceVectors(bool onlyPoints, bool pointsIn2d) const {
    if (override f = this->get_override("computeInstanceVectors")) return f(onlyPoints, pointsIn2d);
    return LaneLineStringInstance::computeInstanceVectors(onlyPoints, pointsIn2d);
  }
  std::vector<VectorXd> default_computeInstanceVectors(bool onlyPoints, bool pointsIn2d) const {
    return this->LaneLineStringInstance::computeInstanceVectors(onlyPoints, pointsIn2d);
  }
  bool process(const OrientedRect &bbox, const ParametrizationType &paramType, int32_t nPoints, double pitch,
               double roll) {
    if (override f = this->get_override("process")) return f(bbox, paramType, nPoints, pitch, roll);
    return LaneLineStringInstance::process(bbox, paramType, nPoints, pitch, roll);
  }
  bool default_process(const OrientedRect &bbox, const ParametrizationType &paramType, int32_t nPoints, double pitch,
                       double roll) {
    return this->LaneLineStringInstance::process(bbox, paramType, nPoints, pitch, roll);
  }
  std::vector<MatrixXd> pointMatrices(bool pointsIn2d) const {
    if (override f = this->get_override("pointMatrices")) return f(pointsIn2d);
    return LaneLineStringInstance::pointMatrices(pointsIn2d);
  }
  std::vector<MatrixXd> default_pointMatrices(bool pointsIn2d) const {
    return this->LaneLineStringInstance::pointMatrices(pointsIn2d);
  }
};

class CompoundLaneLineStringInstanceWrap : public CompoundLaneLineStringInstance,
                                           public wrapper<CompoundLaneLineStringInstance> {
 public:
  CompoundLaneLineStringInstanceWrap() {}

  CompoundLaneLineStringInstanceWrap(const LaneLineStringInstanceList &features, LineStringType compoundType)
      : CompoundLaneLineStringInstance(features, compoundType) {}

  std::vector<VectorXd> computeInstanceVectors(bool onlyPoints, bool pointsIn2d) const {
    if (override f = this->get_override("computeInstanceVectors")) return f(onlyPoints, pointsIn2d);
    return CompoundLaneLineStringInstance::computeInstanceVectors(onlyPoints, pointsIn2d);
  }
  std::vector<VectorXd> default_computeInstanceVectors(bool onlyPoints, bool pointsIn2d) const {
    return this->CompoundLaneLineStringInstance::computeInstanceVectors(onlyPoints, pointsIn2d);
  }
  bool process(const OrientedRect &bbox, const ParametrizationType &paramType, int32_t nPoints, double pitch,
               double roll) {
    if (override f = this->get_override("process")) return f(bbox, paramType, nPoints, pitch, roll);
    return CompoundLaneLineStringInstance::process(bbox, paramType, nPoints, pitch, roll);
  }
  bool default_process(const OrientedRect &bbox, const ParametrizationType &paramType, int32_t nPoints, double pitch,
                       double roll) {
    return this->CompoundLaneLineStringInstance::process(bbox, paramType, nPoints, pitch, roll);
  }
  std::vector<MatrixXd> pointMatrices(bool pointsIn2d) const {
    if (override f = this->get_override("pointMatrices")) return f(pointsIn2d);
    return CompoundLaneLineStringInstance::pointMatrices(pointsIn2d);
  }
  std::vector<MatrixXd> default_pointMatrices(bool pointsIn2d) const {
    return this->CompoundLaneLineStringInstance::pointMatrices(pointsIn2d);
  }
};

class LaneletInstanceWrap : public LaneletInstance, public wrapper<LaneletInstance> {
 public:
  LaneletInstanceWrap() {}

  LaneletInstanceWrap(LaneLineStringInstancePtr leftBoundary, LaneLineStringInstancePtr rightBoundary,
                      LaneLineStringInstancePtr centerline, Id mapID)
      : LaneletInstance(leftBoundary, rightBoundary, centerline, mapID) {}
  LaneletInstanceWrap(const ConstLanelet &ll) : LaneletInstance(ll) {}

  std::vector<VectorXd> computeInstanceVectors(bool onlyPoints, bool pointsIn2d) const {
    if (override f = this->get_override("computeInstanceVectors")) return f(onlyPoints, pointsIn2d);
    return LaneletInstance::computeInstanceVectors(onlyPoints, pointsIn2d);
  }
  std::vector<VectorXd> default_computeInstanceVectors(bool onlyPoints, bool pointsIn2d) const {
    return this->LaneletInstance::computeInstanceVectors(onlyPoints, pointsIn2d);
  }
  bool process(const OrientedRect &bbox, const ParametrizationType &paramType, int32_t nPoints, double pitch = 0,
               double roll = 0) {
    if (override f = this->get_override("process")) return f(bbox, paramType, nPoints, pitch, roll);
    return LaneletInstance::process(bbox, paramType, nPoints, pitch, roll);
  }
  bool default_process(const OrientedRect &bbox, const ParametrizationType &paramType, int32_t nPoints,
                       double pitch = 0, double roll = 0) {
    return this->LaneletInstance::process(bbox, paramType, nPoints, pitch, roll);
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

  def("getRotatedRect", &getRotatedRect,
      (arg("center"), arg("extentLongitudinal"), arg("extentLateral"), arg("yaw"), arg("from2dPos")));
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

  class_<MapInstanceWrap, boost::noncopyable>("MapInstance", "Abstract base map feature class", no_init)
      .add_property("wasCut", &MapInstance::wasCut)
      .add_property("mapID", &MapInstance::mapID)
      .add_property("initialized", &MapInstance::initialized)
      .add_property("valid", &MapInstance::valid)
      .def("computeInstanceVectors", pure_virtual(&MapInstance::computeInstanceVectors),
           (arg("onlyPoints"), arg("pointsIn2d")))
      .def("process", pure_virtual(&MapInstance::process));

  class_<LineStringInstanceWrap, bases<MapInstance>, boost::noncopyable>("LineStringInstance",
                                                                         "Abstract line string feature class", no_init)
      .add_property("rawInstance",
                    make_function(&LineStringInstance::rawInstance, return_value_policy<copy_const_reference>()))
      .def("computeInstanceVectors", pure_virtual(&LineStringInstance::computeInstanceVectors),
           (arg("onlyPoints"), arg("pointsIn2d")))
      .def("process", pure_virtual(&LineStringInstance::process))
      .def("pointMatrices", pure_virtual(&LineStringInstance::pointMatrices));

  class_<LaneLineStringInstanceWrap, bases<LineStringInstance>, LaneLineStringInstancePtr, boost::noncopyable>(
      "LaneLineStringInstance", "Lane line string feature class",
      init<BasicLineString3d, Id, LineStringType, Ids, bool>())
      .def(init<>())
      .add_property("cutInstance",
                    make_function(&LaneLineStringInstance::cutInstance, return_value_policy<copy_const_reference>()))
      .add_property("cutAndResampledInstance", make_function(&LaneLineStringInstance::cutAndResampledInstance,
                                                             return_value_policy<copy_const_reference>()))
      .add_property("cutResampledAndTransformedInstance",
                    make_function(&LaneLineStringInstance::cutResampledAndTransformedInstance,
                                  return_value_policy<copy_const_reference>()))
      .add_property("type", &LaneLineStringInstance::type)
      .add_property("inverted", &LaneLineStringInstance::inverted)
      .add_property("typeInt", &LaneLineStringInstance::typeInt)
      .add_property("laneletIDs",
                    make_function(&LaneLineStringInstance::laneletIDs, return_value_policy<copy_const_reference>()))
      .add_property("addLaneletID", &LaneLineStringInstance::addLaneletID)
      .def("computeInstanceVectors", &LaneLineStringInstance::computeInstanceVectors,
           &LaneLineStringInstanceWrap::default_computeInstanceVectors, (arg("onlyPoints"), arg("pointsIn2d")))
      .def("process", &LaneLineStringInstance::process, &LaneLineStringInstanceWrap::default_process)
      .def("pointMatrices", &LaneLineStringInstance::pointMatrices, &LaneLineStringInstanceWrap::default_pointMatrices);

  class_<LaneletInstanceWrap, bases<MapInstance>, LaneletInstancePtr, boost::noncopyable>(
      "LaneletInstance", "Lanelet feature class that contains lower level LaneLineStringInstances",
      init<LaneLineStringInstancePtr, LaneLineStringInstancePtr, LaneLineStringInstancePtr, Id>())
      .def(init<ConstLanelet>())
      .def(init<>())
      .add_property("leftBoundary", make_function(&LaneletInstance::leftBoundary))
      .add_property("rightBoundary", make_function(&LaneletInstance::rightBoundary))
      .add_property("centerline", make_function(&LaneletInstance::centerline))
      .def("setReprType", &LaneletInstance::setReprType)
      .def("computeInstanceVectors", &LaneletInstance::computeInstanceVectors,
           &LaneletInstanceWrap::default_computeInstanceVectors, (arg("onlyPoints"), arg("pointsIn2d")))
      .def("process", &LaneletInstance::process, &LaneletInstanceWrap::default_process);

  class_<CompoundLaneLineStringInstanceWrap, bases<LaneLineStringInstance>, CompoundLaneLineStringInstancePtr,
         boost::noncopyable>("CompoundLaneLineStringInstance",
                             "Compound lane line string feature class that can trace back the individual features",
                             init<LaneLineStringInstanceList, LineStringType>())
      .def(init<>())
      .add_property("features", make_function(&CompoundLaneLineStringInstance::features))
      .add_property("pathLengthsRaw", make_function(&CompoundLaneLineStringInstance::pathLengthsRaw,
                                                    return_value_policy<copy_const_reference>()))
      .add_property("pathLengthsProcessed", make_function(&CompoundLaneLineStringInstance::pathLengthsProcessed,
                                                          return_value_policy<copy_const_reference>()))
      .add_property("processedInstancesValid", make_function(&CompoundLaneLineStringInstance::processedInstancesValid,
                                                             return_value_policy<copy_const_reference>()))
      .def("computeInstanceVectors", &CompoundLaneLineStringInstance::computeInstanceVectors,
           &CompoundLaneLineStringInstanceWrap::default_computeInstanceVectors, (arg("onlyPoints"), arg("pointsIn2d")))
      .def("process", &CompoundLaneLineStringInstance::process, &CompoundLaneLineStringInstanceWrap::default_process)
      .def("pointMatrices", &CompoundLaneLineStringInstance::pointMatrices,
           &CompoundLaneLineStringInstanceWrap::default_pointMatrices);

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
            .add_property("laneletInstances",
                          make_function(&LaneData::laneletInstances, return_value_policy<copy_const_reference>()))
            .add_property("edges", make_function(&LaneData::edges, return_value_policy<copy_const_reference>()))
            .add_property("uuid", make_function(&LaneData::uuid, return_value_policy<copy_const_reference>()))
            .def("getTensorInstanceData", &LaneData::getTensorInstanceData);

    class_<LaneData::TensorInstanceData>("TensorInstanceData", "TensorInstanceData class for LaneData", init<>())
        .add_property("roadBorders", make_function(&LaneData::TensorInstanceData::roadBorders,
                                                   return_value_policy<copy_const_reference>()))
        .add_property("laneDividers", make_function(&LaneData::TensorInstanceData::laneDividers,
                                                    return_value_policy<copy_const_reference>()))
        .add_property("laneDividerTypes", make_function(&LaneData::TensorInstanceData::laneDividerTypes,
                                                        return_value_policy<copy_const_reference>()))
        .add_property("compoundRoadBorders", make_function(&LaneData::TensorInstanceData::compoundRoadBorders,
                                                           return_value_policy<copy_const_reference>()))
        .add_property("compoundLaneDividers", make_function(&LaneData::TensorInstanceData::compoundLaneDividers,
                                                            return_value_policy<copy_const_reference>()))
        .add_property("compoundLaneDividerTypes", make_function(&LaneData::TensorInstanceData::compoundLaneDividerTypes,
                                                                return_value_policy<copy_const_reference>()))
        .add_property("compoundCenterlines", make_function(&LaneData::TensorInstanceData::compoundCenterlines,
                                                           return_value_policy<copy_const_reference>()))
        .add_property("uuid",
                      make_function(&LaneData::TensorInstanceData::uuid, return_value_policy<copy_const_reference>()))
        .def("pointMatrixCpdRoadBorder", &LaneData::TensorInstanceData::pointMatrixCpdRoadBorder)
        .def("pointMatrixCpdLaneDivider", &LaneData::TensorInstanceData::pointMatrixCpdLaneDivider)
        .def("pointMatrixCpdCenterline", &LaneData::TensorInstanceData::pointMatrixCpdCenterline);
  }

  {
    void (MapDataInterface::*setCurrPosAndExtractSubmap2d)(const lanelet::BasicPoint2d &, double) =
        &MapDataInterface::setCurrPosAndExtractSubmap2d;
    void (MapDataInterface::*setCurrPosAndExtractSubmap4d)(const lanelet::BasicPoint3d &, double) =
        &MapDataInterface::setCurrPosAndExtractSubmap;
    void (MapDataInterface::*setCurrPosAndExtractSubmap6d)(const lanelet::BasicPoint3d &, double, double, double) =
        &MapDataInterface::setCurrPosAndExtractSubmap;

    std::vector<LaneDataPtr> (MapDataInterface::*laneDataBatch2d)(std::vector<BasicPoint2d>, std::vector<double>) =
        &MapDataInterface::laneDataBatch2d;
    std::vector<LaneDataPtr> (MapDataInterface::*laneDataBatch4d)(std::vector<BasicPoint3d>, std::vector<double>) =
        &MapDataInterface::laneDataBatch;
    std::vector<LaneDataPtr> (MapDataInterface::*laneDataBatch6d)(std::vector<BasicPoint3d>, std::vector<double>,
                                                                  std::vector<double>, std::vector<double>) =
        &MapDataInterface::laneDataBatch;
    scope inMapDataInterface =
        class_<MapDataInterface>("MapDataInterface", "Main Interface Class for processing of Lanelet maps",
                                 init<LaneletMapConstPtr>())
            .def(init<LaneletMapConstPtr, MapDataInterface::Configuration>())
            .add_property("config",
                          make_function(&MapDataInterface::config, return_value_policy<copy_const_reference>()))
            .def("setCurrPosAndExtractSubmap2d", setCurrPosAndExtractSubmap2d)
            .def("setCurrPosAndExtractSubmap", setCurrPosAndExtractSubmap4d)
            .def("setCurrPosAndExtractSubmap", setCurrPosAndExtractSubmap6d)
            .def("laneData", &MapDataInterface::laneData)
            .def("teData", &MapDataInterface::teData)
            .def("laneDataBatch2d", laneDataBatch2d)
            .def("laneDataBatch", laneDataBatch4d)
            .def("laneDataBatch", laneDataBatch6d)
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
  converters::VectorToListConverter<LaneLineStringInstanceList>();
  converters::VectorToListConverter<CompoundLaneLineStringInstanceList>();
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
      .fromPython<LaneLineStringInstanceList>()
      .fromPython<CompoundLaneLineStringInstanceList>();
  converters::MapToDictConverter<LaneLineStringInstances>();
  converters::MapToDictConverter<LaneletInstances>();
  converters::MapToDictConverter<Edges>();
  DictToMapConverter<LaneLineStringInstances>();
  DictToMapConverter<LaneletInstances>();
  class_<std::vector<bool>>("BoolList").def(vector_indexing_suite<std::vector<bool>>());

  def("toPointMatrix", &toPointMatrix);

  implicitly_convertible<routing::RoutingGraphPtr, routing::RoutingGraphConstPtr>();
}
