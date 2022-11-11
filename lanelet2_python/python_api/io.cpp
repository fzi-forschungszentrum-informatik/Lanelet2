#include <lanelet2_io/Io.h>

#include <boost/python.hpp>
#include <boost/python/suite/indexing/indexing_suite.hpp>
#include <boost/python/suite/indexing/map_indexing_suite.hpp>

#include "lanelet2_python/internal/converter.h"

using namespace boost::python;
using namespace lanelet;

struct DictToConfigurationConverter {
  DictToConfigurationConverter() { converter::registry::push_back(&convertible, &construct, type_id<io::Configuration>()); }
  static void* convertible(PyObject* obj) {
    if (!PyDict_CheckExact(obj)) {  // NOLINT
      return nullptr;
    }
    return obj;
  }
  static void construct(PyObject* obj, converter::rvalue_from_python_stage1_data* data) {
    dict d(borrowed(obj));
    list keys = d.keys();
    list values = d.values();
    io::Configuration attributes;
    for (auto i = 0u; i < len(keys); ++i) {
      std::string key = extract<std::string>(keys[i]);
      bool value = extract<bool>(values[i]);
      attributes.insert(std::make_pair(key, value));
    }
    using StorageType = converter::rvalue_from_python_storage<io::Configuration>;
    void* storage = reinterpret_cast<StorageType*>(data)->storage.bytes;  // NOLINT
    new (storage) io::Configuration(attributes);
    data->convertible = storage;
  }
};

std::shared_ptr<LaneletMap> loadWrapper(const std::string& filename, const Origin& origin) {
  return load(filename, origin);
}
std::shared_ptr<LaneletMap> loadProjectorWrapper(const std::string& filename, const Projector& projector) {
  return load(filename, projector);
}
boost::python::tuple loadWithErrorWrapper(const std::string& filename, const Projector& projector) {
  ErrorMessages errs;
  LaneletMapPtr map = load(filename, projector, &errs);
  return boost::python::make_tuple(map, errs);
}

void writeWrapper(const std::string& filename, const LaneletMap& map, const Origin& origin, const Optional<io::Configuration>& params) {
  write(filename, map, origin, nullptr, params.get_value_or(io::Configuration()));
}

void writeProjectorWrapper(const std::string& filename, const LaneletMap& map, const Projector& projector, const Optional<io::Configuration>& params) {
  write(filename, map, projector, nullptr, params.get_value_or(io::Configuration()));
}

ErrorMessages writeWithErrorWrapper(const std::string& filename, const LaneletMap& map, const Projector& projector, const Optional<io::Configuration>& params) {
  ErrorMessages errs;
  write(filename, map, projector, &errs, params.get_value_or(io::Configuration()));
  return errs;
}

BOOST_PYTHON_MODULE(PYTHON_API_MODULE_NAME) {  // NOLINT
  auto core = import("lanelet2.core");
  auto proj = import("lanelet2.projection");

  DictToConfigurationConverter();
  converters::OptionalConverter<io::Configuration>();
  converters::ToOptionalConverter().fromPython<io::Configuration>();

  class_<Origin, std::shared_ptr<Origin>>("Origin", init<>())
      .def(init<GPSPoint>())
      .def("__init__", make_constructor(
                           +[](double lat, double lon, double alt) {
                             return std::make_shared<Origin>(GPSPoint{lat, lon, alt});
                           },
                           default_call_policies(), (arg("lat") = 0., arg("lon") = 0., arg("lon") = 0)))
      .def_readwrite("position", &Origin::position);

  def("load", loadProjectorWrapper, (arg("filename"), arg("projector") = DefaultProjector()));
  def("load", loadWrapper, (arg("filename"), arg("origin")));
  def("loadRobust", loadWithErrorWrapper, arg("filename"),
      "Loads a map robustly. Parser errors are returned as second member of "
      "the tuple. If there are errors, the map will be incomplete somewhere.");

  def("write", writeProjectorWrapper, (arg("filename"), arg("map"), arg("projector"), arg("params") = Optional<io::Configuration>{}),
      "Writes the map to a file. The extension determines which format will "
      "be used (usually .osm)");
  def("write", writeWrapper, (arg("filename"), arg("map"), arg("origin"), arg("params") = Optional<io::Configuration>{}),
      "Writes the map to a file. The extension determines which format will "
      "be used (usually .osm)");
  def("writeRobust", writeWithErrorWrapper, (arg("filename"), arg("map"), arg("projector"), arg("params") = Optional<io::Configuration>{}),
      "Writes a map robustly and returns writer errors. If there are errors, "
      "the map will be incomplete somewhere.");
}
