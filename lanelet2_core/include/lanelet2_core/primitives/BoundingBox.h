#pragma once

#pragma GCC diagnostic push
#if defined __GNUC__ && (__GNUC__ >= 6)
#pragma GCC diagnostic ignored "-Wignored-attributes"
#pragma GCC diagnostic ignored "-Wint-in-bool-context"
#endif
#include <Eigen/Dense>
#pragma GCC diagnostic pop

namespace lanelet {

/**
 * @brief Convenience type for an axis aligned bounding box in 2d.
 *
 * Can be used as Eigen::AlignedBox2d and as boost::geometry::Box.
 */
using BoundingBox2d = Eigen::AlignedBox2d;
using BoundingBox2d = Eigen::AlignedBox<double, 2>;

/**
 * @brief Convenience type for an axis aligned bounding box in 3d.
 *
 * Can be used as Eigen::AlignedBox2d and as boost::geometry::Box.
 */
using BoundingBox3d = Eigen::AlignedBox3d;
}  // namespace lanelet
