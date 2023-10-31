#include "lanelet2_map_learning/MapFeatures.h"

#include <boost/geometry.hpp>
#include <type_traits>

#include "lanelet2_map_learning/Utils.h"

namespace lanelet {
namespace map_learning {

void PolylineFeature::processPolyline(const OrientedRect& bbox, const ParametrizationType& paramType, int32_t nPoints) {
  if (paramType != ParametrizationType::Polyline) {
    throw std::runtime_error("Only polyline parametrization is implemented so far!");
  }
  processedFeature_ = cutPolyline(bbox, originalFeature_);
  processedFeature_ = resamplePolyline(originalFeature_, nPoints);

  double lengthOriginal =
      boost::geometry::length(originalFeature_, boost::geometry::strategy::distance::pythagoras<double>());
  double lengthProcessed =
      boost::geometry::length(originalFeature_, boost::geometry::strategy::distance::pythagoras<double>());

  if (lengthOriginal - lengthProcessed > 1e-2) {
    wasCut_ = true;
  }
}

LaneletFeature::LaneletFeature(const PolylineFeature& leftBoundary, const PolylineFeature& rightBoundary,
                               const PolylineFeature& centerline, Id mapID)
    : leftBoundary_{leftBoundary}, rightBoundary_{rightBoundary}, centerline_{centerline}, mapID_{mapID} {}

LaneletFeature::LaneletFeature(const ConstLanelet& ll)
    : leftBoundary_{PolylineFeature(ll.leftBound3d().basicLineString(), ll.leftBound3d().id())},
      rightBoundary_{PolylineFeature(ll.rightBound3d().basicLineString(), ll.rightBound3d().id())},
      centerline_{PolylineFeature(ll.centerline3d().basicLineString(), ll.centerline3d().id())},
      bdTypeLeft_{bdSubtypeToInt(ll.leftBound3d())},
      bdTypeRight_{bdSubtypeToInt(ll.rightBound3d())} {}

void LaneletFeature::processLanelet(const OrientedRect& bbox, const ParametrizationType& paramType, int32_t nPoints) {
  leftBoundary_.processPolyline(bbox, paramType, nPoints);
  rightBoundary_.processPolyline(bbox, paramType, nPoints);
  centerline_.processPolyline(bbox, paramType, nPoints);

  if (leftBoundary_.wasCut_ || rightBoundary_.wasCut_ || centerline_.wasCut_) {
    wasCut_ = true;
  }
}

}  // namespace map_learning
}  // namespace lanelet