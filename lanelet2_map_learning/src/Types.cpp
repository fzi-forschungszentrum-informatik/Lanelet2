#include "lanelet2_map_learning/Types.h"

#include <boost/geometry.hpp>
#include <type_traits>

namespace lanelet {
namespace map_learning {

void MapFeature::computePolyline(int32_t nPoints) {
  double length = boost::geometry::length(originalFeature_, boost::geometry::strategy::distance::pythagoras<double>());
  double dist = length / static_cast<double>(nPoints);
  boost::geometry::model::multi_point<BasicPoint3d> bdInterp;
  boost::geometry::line_interpolate(originalFeature_, dist, bdInterp);
  assert(bdInterp.size() == nPoints);
  processedFeature_ = bdInterp;
}

}  // namespace map_learning
}  // namespace lanelet