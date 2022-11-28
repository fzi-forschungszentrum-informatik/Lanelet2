#include <lanelet2_io/Io.h>

#include "lanelet2_python/internal/converter.h"

namespace py = boost::python;
using namespace lanelet;

struct DictToConfigurationConverter {
  DictToConfigurationConverter() { py::converter::registry::push_back(&convertible, &construct, py::type_id<io::Configuration>()); }
  static void* convertible(PyObject* obj) {
    if (!PyDict_CheckExact(obj)) {  // NOLINT
      return nullptr;
    }
    return obj;
  }
  static void construct(PyObject* obj, py::converter::rvalue_from_python_stage1_data* data) {
    py::dict d(py::borrowed(obj));
    py::list keys = d.keys();
    py::list values = d.values();
    io::Configuration attributes;
    for (auto i = 0u; i < py::len(keys); ++i) {
      std::string key = py::extract<std::string>(keys[i]);
      std::string value = py::extract<std::string>(values[i]);
      attributes.insert(std::make_pair(key, value));
    }
    using StorageType = py::converter::rvalue_from_python_storage<io::Configuration>;
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
py::tuple loadWithErrorWrapper(const std::string& filename, const Projector& projector) {
  ErrorMessages errs;
  LaneletMapPtr map = load(filename, projector, &errs);
  return py::make_tuple(map, errs);
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
  auto core = py::import("lanelet2.core");
  auto proj = py::import("lanelet2.projection");

  DictToConfigurationConverter();
  converters::OptionalConverter<io::Configuration>();
  converters::ToOptionalConverter().fromPython<io::Configuration>();

  py::class_<Origin, std::shared_ptr<Origin>>("Origin", py::init<>())
      .def(py::init<GPSPoint>())
      .def("__init__", py::make_constructor(
                           +[](double lat, double lon, double alt) {
                             return std::make_shared<Origin>(GPSPoint{lat, lon, alt});
                           },
                           py::default_call_policies(), (py::arg("lat") = 0., py::arg("lon") = 0., py::arg("lon") = 0)))
      .def_readwrite("position", &Origin::position);

  py::def("load", loadProjectorWrapper, (py::arg("filename"), py::arg("projector") = DefaultProjector()));
  py::def("load", loadWrapper, (py::arg("filename"), py::arg("origin")));
  py::def("loadRobust", loadWithErrorWrapper, py::arg("filename"),
      "Loads a map robustly. Parser errors are returned as second member of "
      "the tuple. If there are errors, the map will be incomplete somewhere.");

  py::def("write", writeProjectorWrapper, (py::arg("filename"), py::arg("map"), py::arg("projector"), py::arg("params") = Optional<io::Configuration>{}),
      "Writes the map to a file. The extension determines which format will "
      "be used (usually .osm)");
  py::def("write", writeWrapper, (py::arg("filename"), py::arg("map"), py::arg("origin"), py::arg("params") = Optional<io::Configuration>{}),
      "Writes the map to a file. The extension determines which format will "
      "be used (usually .osm)");
  py::def("writeRobust", writeWithErrorWrapper, (py::arg("filename"), py::arg("map"), py::arg("projector"), py::arg("params") = Optional<io::Configuration>{}),
      "Writes a map robustly and returns writer errors. If there are errors, "
      "the map will be incomplete somewhere.");
}
