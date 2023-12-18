#include "lanelet2_map_learning/MapData.h"
#include "lanelet2_map_learning/MapDataInterface.h"
#include "lanelet2_map_learning/MapFeatures.h"
#include "lanelet2_python/internal/converter.h"
#include "lanelet2_python/internal/eigen_converter.h"

using namespace boost::python;
using namespace lanelet;

/// TODO: ADD VIRTUAL FUNCTIONS, CONSTRUCTORS

BOOST_PYTHON_MODULE(PYTHON_API_MODULE_NAME) {  // NOLINT
  class_<map_learning::MapFeature>("MapFeature", "Abstract base map feature class", no_init)
      .add_property("initialized", &map_learning::MapFeature::initialized)
      .add_property("valid", &map_learning::MapFeature::valid)
      .add_property("mapID", &map_learning::MapFeature::mapID);

  class_<map_learning::LineStringFeature, bases<map_learning::MapFeature>>(
      "LineStringFeature", "Abstract line string feature class", no_init)
      .add_property("rawFeature", &map_learning::LineStringFeature::rawFeature);

  class_<map_learning::LaneLineStringFeature, bases<map_learning::LineStringFeature>>(
      "LaneLineStringFeature", "Lane line string feature class",
      init<BasicLineString3d, Id, map_learning::LineStringType, Id>())
      .add_property("wasCut", &map_learning::LaneLineStringFeature::wasCut)
      .add_property("cutFeature", &map_learning::LaneLineStringFeature::cutFeature)
      .add_property("cutAndResampledFeature", &map_learning::LaneLineStringFeature::cutAndResampledFeature)
      .add_property("type", &map_learning::LaneLineStringFeature::type)
      .add_property("typeInt", &map_learning::LaneLineStringFeature::typeInt)
      .add_property("laneletIDs", &map_learning::LaneLineStringFeature::laneletIDs)
      .add_property("addLaneletID", &map_learning::LaneLineStringFeature::addLaneletID);

  class_<map_learning::LaneletFeature, bases<map_learning::MapFeature>>(
      "LaneletFeature", "Lanelet feature class that contains lower level LaneLineStringFeatures",
      init<map_learning::LaneLineStringFeature, map_learning::LaneLineStringFeature,
           map_learning::LaneLineStringFeature, Id>())
      .def(init<ConstLanelet>())
      .add_property("leftBoundary", &map_learning::LaneletFeature::leftBoundary)
      .add_property("rightBoundary", &map_learning::LaneletFeature::rightBoundary)
      .add_property("centerline", &map_learning::LaneletFeature::centerline)
      .add_property("setReprType", &map_learning::LaneletFeature::setReprType);

  class_<map_learning::CompoundLaneLineStringFeature, bases<map_learning::LaneLineStringFeature>>(
      "CompoundLaneLineStringFeature",
      "Compound lane line string feature class that can trace back the individual features",
      init<map_learning::LaneLineStringFeatureList, map_learning::LineStringType>());

  class_<map_learning::Edge>("Edge");

  class_<map_learning::LaneData>("MapFeature");
}