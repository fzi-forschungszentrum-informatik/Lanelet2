#pragma once
#include <boost/geometry/geometries/register/box.hpp>

#include "lanelet2_core/geometry/Point.h"
#include "lanelet2_core/primitives/BoundingBox.h"

// registrations for use with boost::geometry
BOOST_GEOMETRY_REGISTER_BOX(lanelet::BoundingBox2d, lanelet::BasicPoint2d, min(), max())
BOOST_GEOMETRY_REGISTER_BOX(lanelet::BoundingBox3d, lanelet::BasicPoint3d, min(), max())

namespace lanelet {
namespace geometry {
/**
 * @brief trivial overload for boundingBoxes
 * @param b
 */
inline BoundingBox2d boundingBox2d(const BoundingBox2d& b) { return b; }
inline BoundingBox3d boundingBox3d(const BoundingBox2d& b) { return traits::to3D(b); }

/**
 * @brief trivial overload for boundingBoxes
 * @param b
 */
inline BoundingBox3d boundingBox3d(const BoundingBox3d& b) { return b; }
inline BoundingBox2d boundingBox2d(const BoundingBox3d& b) { return traits::to2D(b); }

/**
 * @brief calculates a (very small) 2d bounding box around a point
 * @param p point to to this for
 * @return the bounding box
 */
inline BoundingBox2d boundingBox2d(const ConstPoint2d& p) { return {p.basicPoint2d(), p.basicPoint2d()}; }
inline BoundingBox2d boundingBox2d(const ConstPoint3d& p) { return {p.basicPoint2d(), p.basicPoint2d()}; }
inline BoundingBox2d boundingBox2d(const BasicPoint2d& p) { return {p, p}; }
inline BoundingBox2d boundingBox2d(const BasicPoint3d& p) { return {traits::to2D(p), traits::to2D(p)}; }

/**
 * @brief calculates a (very small) 3d bounding box around a point
 * @param p point to to this for
 * @return the bounding box
 */
inline BoundingBox3d boundingBox3d(const ConstPoint3d& p) { return BoundingBox3d(p.basicPoint(), p.basicPoint()); }
inline BoundingBox3d boundingBox3d(const ConstPoint2d& p) { return boundingBox3d(utils::to3D(p)); }
inline BoundingBox3d boundingBox3d(const BasicPoint3d& p) { return {p, p}; }
inline BoundingBox3d boundingBox3d(const BasicPoint2d& p) { return {traits::to3D(p), traits::to3D(p)}; }
}  // namespace geometry
}  // namespace lanelet
