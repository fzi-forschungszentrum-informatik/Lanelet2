#include <lanelet2_projection/UTM.h>
#include <boost/python.hpp>

using namespace lanelet;

BOOST_PYTHON_MODULE(PYTHON_API_MODULE_NAME) {  // NOLINT
  using namespace boost::python;
  class_<Projector, boost::noncopyable, std::shared_ptr<Projector>>(  // NOLINT
      "Projector", "Projects point from lat/lon to x/y and back", no_init);
  class_<projection::SphericalMercatorProjector, std::shared_ptr<projection::SphericalMercatorProjector>,
         bases<Projector>>("MercatorProjector", init<Origin>("origin"));
  class_<projection::UtmProjector, std::shared_ptr<projection::UtmProjector>, bases<Projector>>("UtmProjector",
                                                                                                init<Origin>("origin"));
}
