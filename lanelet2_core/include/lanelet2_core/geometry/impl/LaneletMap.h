#pragma once
#include <boost/geometry/algorithms/buffer.hpp>
#include <boost/geometry/algorithms/distance.hpp>

#include "lanelet2_core/LaneletMap.h"
#include "lanelet2_core/primitives/BoundingBox.h"
#include "lanelet2_core/primitives/Lanelet.h"
#include "lanelet2_core/utility/Utilities.h"

namespace lanelet {
namespace geometry {

template <typename LayerT, typename GeometryT>
auto findWithin2d(LayerT& layer, const GeometryT& geometry, double maxDist)
    -> std::vector<std::pair<double, traits::LayerPrimitiveType<LayerT>>> {
  static_assert(std::is_same<traits::TwoD<GeometryT>, GeometryT>::value,
                "Call this function with a 2d type as geometry!");
  using RetT = std::vector<std::pair<double, traits::LayerPrimitiveType<LayerT>>>;

  BoundingBox2d boundingBox = boundingBox2d(geometry);
  if (maxDist > 0.) {
    boost::geometry::buffer(boundingBox, boundingBox, maxDist);
  }
  const auto bboxApproximation = layer.search(boundingBox);

  auto withinVec = utils::createReserved<RetT>(bboxApproximation.size());
  for (auto const& elem : bboxApproximation) {
    const double dist = distance2d(geometry, elem);
    if (dist <= maxDist) {
      withinVec.push_back(std::make_pair(dist, elem));
    }
  }
  std::sort(withinVec.begin(), withinVec.end(), [](auto& v1, auto& v2) { return v1.first < v2.first; });
  return withinVec;
}

template <typename LayerT, typename GeometryT>
auto findWithin3d(LayerT& layer, const GeometryT& geometry, double maxDist)
    -> std::vector<std::pair<double, traits::LayerPrimitiveType<LayerT>>> {
  using RetT = std::vector<std::pair<double, traits::LayerPrimitiveType<LayerT>>>;

  BoundingBox2d boundingBox = boundingBox2d(traits::to2D(geometry));
  if (maxDist > 0.) {
    boost::geometry::buffer(boundingBox, boundingBox, maxDist);
  }
  const auto bboxApproximation = layer.search(boundingBox);

  auto withinVec = utils::createReserved<RetT>(bboxApproximation.size());
  for (auto const& elem : bboxApproximation) {
    const double dist = distance3d(geometry, elem);
    if (dist <= maxDist) {
      withinVec.push_back(std::make_pair(dist, elem));
    }
  }
  std::sort(withinVec.begin(), withinVec.end(), [](auto& v1, auto& v2) { return v1.first < v2.first; });
  return withinVec;
}
}  // namespace geometry
}  // namespace lanelet
