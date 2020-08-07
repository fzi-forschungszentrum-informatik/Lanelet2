#pragma once
#include <boost/version.hpp>
#if BOOST_VERSION < 106300 && BOOST_VERSION >= 106200
// Boost 1.62 is missing an iostream include...
#include <iostream>
#endif
#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/strategies/strategies.hpp>

#include "lanelet2_core/primitives/Point.h"

/***********************************************************************
 *                   BOOST GEOMETRY REGISTRATIONS                      *
 ***********************************************************************/
BOOST_GEOMETRY_REGISTER_POINT_3D(lanelet::BasicPoint3d, double, cs::cartesian, x(), y(), z())
BOOST_GEOMETRY_REGISTER_POINT_2D(lanelet::BasicPoint2d, double, cs::cartesian, x(), y())
BOOST_GEOMETRY_REGISTER_POINT_2D(Eigen::Vector2d, double, cs::cartesian, x(), y())
BOOST_GEOMETRY_REGISTER_POINT_2D(lanelet::Point2d, double, cs::cartesian, x(), y())
BOOST_GEOMETRY_REGISTER_POINT_2D_CONST(lanelet::ConstPoint2d, double, cs::cartesian, x(), y())
BOOST_GEOMETRY_REGISTER_POINT_3D(lanelet::Point3d, double, cs::cartesian, x(), y(), z())
BOOST_GEOMETRY_REGISTER_POINT_3D_CONST(lanelet::ConstPoint3d, double, cs::cartesian, x(), y(), z())

namespace boost {
namespace geometry {
// Help boost with type deduction for proxies
template <typename Policy>
struct robust_point_type<const lanelet::BasicPoint2d, Policy> {
  using type = lanelet::BasicPoint2d;  // NOLINT
};

template <>
struct robust_point_type<const lanelet::BasicPoint2d, detail::no_rescale_policy> {
  using type = lanelet::BasicPoint2d;  // NOLINT
};

}  // namespace geometry
}  // namespace boost

namespace lanelet {
namespace geometry {
namespace internal {
template <typename T, typename Enable = void>
struct GetGeometry {
  static inline auto twoD(const T& geometry) { return geometry; }
  static inline auto threeD(const T& geometry) { return geometry; }
};

template <typename T>
struct GetGeometry<T, IfPT<T, void>> {
  static inline auto twoD(const T& geometry) { return traits::to2D(geometry); }
  static inline auto threeD(const T& geometry) { return traits::to3D(geometry); }
};

template <typename T1, typename T2>
constexpr bool isTrivialDistance() {
  return traits::noRegelem<T1>() && traits::noRegelem<T2>() &&
         !(traits::isCategory<T1, traits::LineStringTag>() && traits::isCategory<T2, traits::LineStringTag>() &&
           traits::is3D<T1>());
}
}  // namespace internal
using boost::geometry::distance;

//! Calculates the distance of two lanelet2 geometries in 2D, converting them if necessary
template <typename Geometry1T, typename Geometry2T>
auto distance2d(const Geometry1T& p1, const Geometry2T& p2)
    -> std::enable_if_t<internal::isTrivialDistance<Geometry1T, Geometry2T>(), double> {
  return distance(internal::GetGeometry<Geometry1T>::twoD(p1), internal::GetGeometry<Geometry2T>::twoD(p2));
}

//! Calculates the distance of two lanelet2 geometries in 3D, converting them if necessary
template <typename Geometry1T, typename Geometry2T>
auto distance3d(const Geometry1T& p1, const Geometry2T& p2)
    -> std::enable_if_t<internal::isTrivialDistance<Geometry1T, Geometry2T>(), double> {
  return distance(internal::GetGeometry<Geometry1T>::threeD(p1), internal::GetGeometry<Geometry2T>::threeD(p2));
}
}  // namespace geometry
}  // namespace lanelet
