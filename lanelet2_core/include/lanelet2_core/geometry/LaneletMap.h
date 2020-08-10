#pragma once

#include "lanelet2_core/LaneletMap.h"
#include "lanelet2_core/geometry/BoundingBox.h"
#include "lanelet2_core/geometry/Lanelet.h"
#include "lanelet2_core/geometry/LineString.h"
#include "lanelet2_core/primitives/BoundingBox.h"
#include "lanelet2_core/primitives/Lanelet.h"

namespace lanelet {
namespace geometry {
/**
 * @brief Returns all elements that are closer than maxDist to a geometry in 2d
 * @param layer for the check (a layer of LaneletMap)
 * @param geometry to check (any 2d geometry)
 * @param maxDist maximum distance to the input geometry. If zero, only primitives containing the element are returned.
 * Be aware that rounding errors can affect the result for primitives directly on (or very close to) the boundary.
 * @return vector of pairs: <actual distance, element> of all elements of a layer which are closer than maxDist to the
 * geometry. The return type differs depending on if the layer is const or not. Result sorted in ascending distance.
 * @see findNearest
 */
template <typename LayerT, typename GeometryT>
auto findWithin2d(LayerT& layer, const GeometryT& geometry, double maxDist = 0.)
    -> std::vector<std::pair<double, traits::LayerPrimitiveType<LayerT>>>;

/**
 * @brief Returns all elements that are closer than maxDist to a geometry in 3d
 * @param layer for the check (a layer of LaneletMap)
 * @param geometry to check (any 3d geometry)
 * @param maxDist maximum distance to the input geometry. If zero, only primitives containing the element are returned.
 * Be aware that rounding errors can affect the result for primitives directly on (or very close to) the boundary.
 * @return vector of pairs: <actual distance, element> of all elements of a layer which are closer than maxDist to the
 * geometry. The return type differs depending on if the layer is const or not. Result sorted in ascending distance.
 */
template <typename LayerT, typename GeometryT>
auto findWithin3d(LayerT& layer, const GeometryT& geometry, double maxDist = 0.)
    -> std::vector<std::pair<double, traits::LayerPrimitiveType<LayerT>>>;
}  // namespace geometry
}  // namespace lanelet

#include "impl/LaneletMap.h"
