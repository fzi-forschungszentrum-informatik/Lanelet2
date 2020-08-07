#include <lanelet2_io/Io.h>

#include <boost/python.hpp>

namespace py = boost::python;
using namespace lanelet;

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

void writeWrapper(const std::string& filename, const LaneletMap& map, const Origin& origin) {
  write(filename, map, origin);
}

void writeProjectorWrapper(const std::string& filename, const LaneletMap& map, const Projector& projector) {
  write(filename, map, projector);
}

ErrorMessages writeWithErrorWrapper(const std::string& filename, const LaneletMap& map, const Projector& projector) {
  ErrorMessages errs;
  write(filename, map, projector, &errs);
  return errs;
}

BOOST_PYTHON_MODULE(PYTHON_API_MODULE_NAME) {  // NOLINT
  using namespace py;
  auto core = import("lanelet2.core");
  auto proj = import("lanelet2.projection");

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

  def("write", writeProjectorWrapper, (arg("filename"), arg("map"), arg("projector") = DefaultProjector()),
      "Writes the map to a file. The extension determines which format will "
      "be used (usually .osm)");
  def("write", writeWrapper, (arg("filename"), arg("map"), arg("origin")),
      "Writes the map to a file. The extension determines which format will "
      "be used (usually .osm)");
  def("writeRobust", writeWithErrorWrapper, (arg("filename"), arg("map"), arg("projector") = DefaultProjector()),
      "Writes a map robustly and returns writer errors. If there are errors, "
      "the map will be incomplete somewhere.");
}
