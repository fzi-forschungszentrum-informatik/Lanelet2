#include <lanelet2_projection/UTM.h>

#include <boost/python.hpp>

using namespace lanelet;

BOOST_PYTHON_MODULE(PYTHON_API_MODULE_NAME) {  // NOLINT
  using namespace boost::python;
  class_<Projector, boost::noncopyable, std::shared_ptr<Projector>>(  // NOLINT
      "Projector", "Projects point from lat/lon to x/y and back", no_init)
      .def("forward", &Projector::forward, "Convert lat/lon into x/y")
      .def("reverse", &Projector::reverse, "Convert x/y into lat/lon")
      .def("origin", &Projector::origin, "Global origin of the converter", return_internal_reference<>());
  class_<projection::SphericalMercatorProjector, std::shared_ptr<projection::SphericalMercatorProjector>,  // NOLINT
         bases<Projector>>("MercatorProjector", init<Origin>("origin"));
  class_<projection::UtmProjector, std::shared_ptr<projection::UtmProjector>, bases<Projector>>("UtmProjector",
                                                                                                init<Origin>("origin"))
      .def(init<Origin, bool, bool>("UtmProjector(origin, useOffset, throwInPaddingArea)"));
}
