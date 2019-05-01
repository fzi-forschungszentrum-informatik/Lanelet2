#pragma once
#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/strategies/strategies.hpp>
#include "../primitives/Point.h"

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
using boost::geometry::distance;

template <typename Point1T, typename Point2T>
IfPT<Point1T, double> distance2d(const Point1T& p1, const Point2T& p2) {
  return distance(traits::to2D(p1), traits::to2D(p2));
}

template <typename Point1T, typename Point2T>
IfPT<Point1T, double> distance3d(const Point1T& p1, const Point2T& p2) {
  return distance(traits::to3D(p1), traits::to3D(p2));
}
}  // namespace geometry
}  // namespace lanelet
