#pragma once
#include <boost/geometry/geometries/register/ring.hpp>
#include "../primitives/CompoundPolygon.h"
#include "../primitives/Polygon.h"
#include "LineString.h"

/***********************************************************************
 *                   BOOST GEOMETRY REGISTRATIONS                      *
 ***********************************************************************/

// Register our Polygons
BOOST_GEOMETRY_REGISTER_RING(lanelet::Polygon3d)
BOOST_GEOMETRY_REGISTER_RING(lanelet::ConstPolygon3d)
BOOST_GEOMETRY_REGISTER_RING(lanelet::Polygon2d)
BOOST_GEOMETRY_REGISTER_RING(lanelet::ConstPolygon2d)
BOOST_GEOMETRY_REGISTER_RING(lanelet::BasicPolygon2d)
BOOST_GEOMETRY_REGISTER_RING(lanelet::BasicPolygon3d)
BOOST_GEOMETRY_REGISTER_RING(lanelet::ConstHybridPolygon2d)
BOOST_GEOMETRY_REGISTER_RING(lanelet::ConstHybridPolygon3d)
BOOST_GEOMETRY_REGISTER_RING(lanelet::CompoundPolygon2d)
BOOST_GEOMETRY_REGISTER_RING(lanelet::CompoundPolygon3d)
BOOST_GEOMETRY_REGISTER_RING(lanelet::CompoundHybridPolygon2d)
BOOST_GEOMETRY_REGISTER_RING(lanelet::CompoundHybridPolygon3d)

// traits for boost::geometry. All lanelet polygons are open and clockwise.
namespace boost {
namespace geometry {
namespace traits {
// traits for lanelet::Polygon
template <>
struct point_order<lanelet::Polygon3d> {
  static const order_selector value = clockwise;
};
template <>
struct closure<lanelet::Polygon3d> {
  static const closure_selector value = open;
};

// traits for lanelet::ConstPolygon
template <>
struct point_order<lanelet::ConstPolygon3d> {
  static const order_selector value = clockwise;
};
template <>
struct closure<lanelet::ConstPolygon3d> {
  static const closure_selector value = open;
};

// traits for lanelet::Polygon2d
template <>
struct point_order<lanelet::Polygon2d> {
  static const order_selector value = clockwise;
};
template <>
struct closure<lanelet::Polygon2d> {
  static const closure_selector value = open;
};

// traits for lanelet::ConstPolygon2d
template <>
struct point_order<lanelet::ConstPolygon2d> {
  static const order_selector value = clockwise;
};
template <>
struct closure<lanelet::ConstPolygon2d> {
  static const closure_selector value = open;
};

// traits for lanelet::BasicPolygon2d
template <>
struct point_order<lanelet::BasicPolygon2d> {
  static const order_selector value = clockwise;
};
template <>
struct closure<lanelet::BasicPolygon2d> {
  static const closure_selector value = open;
};

// traits for lanelet::BasicPolygon3d
template <>
struct point_order<lanelet::BasicPolygon3d> {
  static const order_selector value = clockwise;
};
template <>
struct closure<lanelet::BasicPolygon3d> {
  static const closure_selector value = open;
};

// traits for lanelet::ConstHybridPolygon2d
template <>
struct point_order<lanelet::ConstHybridPolygon2d> {
  static const order_selector value = clockwise;
};
template <>
struct closure<lanelet::ConstHybridPolygon2d> {
  static const closure_selector value = open;
};

// traits for lanelet::ConstHybridPolygon
template <>
struct point_order<lanelet::ConstHybridPolygon3d> {
  static const order_selector value = clockwise;
};
template <>
struct closure<lanelet::ConstHybridPolygon3d> {
  static const closure_selector value = open;
};
// traits for lanelet::CompoundHybridPolygon2d
template <>
struct point_order<lanelet::CompoundHybridPolygon2d> {
  static const order_selector value = clockwise;
};
template <>
struct closure<lanelet::CompoundHybridPolygon2d> {
  static const closure_selector value = open;
};

// traits for lanelet::CompoundHybridPolygon3d
template <>
struct point_order<lanelet::CompoundHybridPolygon3d> {
  static const order_selector value = clockwise;
};
template <>
struct closure<lanelet::CompoundHybridPolygon3d> {
  static const closure_selector value = open;
};

// traits for lanelet::CompoundPolygon2d
template <>
struct point_order<lanelet::CompoundPolygon2d> {
  static const order_selector value = clockwise;
};
template <>
struct closure<lanelet::CompoundPolygon2d> {
  static const closure_selector value = open;
};

// traits for lanelet::CompoundPolygon3d
template <>
struct point_order<lanelet::CompoundPolygon3d> {
  static const order_selector value = clockwise;
};
template <>
struct closure<lanelet::CompoundPolygon3d> {
  static const closure_selector value = open;
};

// traits for BasicPolygonWithHoles3d to register as a boost::polygon (taken
// from
// http://www.boost.org/doc/libs/1_58_0/libs/geometry/doc/html/geometry/examples/example_source_code__adapting_a_legacy_geometry_object_model.html)
using LLPolygon3d = lanelet::BasicPolygonWithHoles3d;
template <>
struct tag<LLPolygon3d> {
  using type = polygon_tag;  // NOLINT
};
template <>
struct ring_const_type<LLPolygon3d> {
  using type = const lanelet::BasicPolygon3d&;  // NOLINT
};
template <>
struct ring_mutable_type<LLPolygon3d> {
  using type = lanelet::BasicPolygon3d&;  // NOLINT
};
template <>
struct interior_const_type<LLPolygon3d> {
  using type = const lanelet::BasicPolygons3d&;  // NOLINT
};
template <>
struct interior_mutable_type<LLPolygon3d> {
  using type = lanelet::BasicPolygons3d&;  // NOLINT
};

template <>
struct exterior_ring<LLPolygon3d> {
  static lanelet::BasicPolygon3d& get(LLPolygon3d& p) { return p.outer; }
  static const lanelet::BasicPolygon3d& get(LLPolygon3d const& p) { return p.outer; }
};

template <>
struct interior_rings<LLPolygon3d> {
  static lanelet::BasicPolygons3d get(LLPolygon3d& p) { return p.inner; }
  static const lanelet::BasicPolygons3d& get(LLPolygon3d const& p) { return p.inner; }
};

// traits for BasicPolygonWithHoles2d to register as a boost::polygon (taken
// from
// http://www.boost.org/doc/libs/1_58_0/libs/geometry/doc/html/geometry/examples/example_source_code__adapting_a_legacy_geometry_object_model.html)
using LLPolygon2d = lanelet::BasicPolygonWithHoles2d;
template <>
struct tag<LLPolygon2d> {
  using type = polygon_tag;  // NOLINT
};
template <>
struct ring_const_type<LLPolygon2d> {
  using type = const lanelet::BasicPolygon2d&;  // NOLINT
};
template <>
struct ring_mutable_type<LLPolygon2d> {
  using type = lanelet::BasicPolygon2d&;  // NOLINT
};
template <>
struct interior_const_type<LLPolygon2d> {
  using type = const lanelet::BasicPolygons2d&;  // NOLINT
};
template <>
struct interior_mutable_type<LLPolygon2d> {
  using type = lanelet::BasicPolygons2d&;  // NOLINT
};

template <>
struct exterior_ring<LLPolygon2d> {
  static lanelet::BasicPolygon2d& get(LLPolygon2d& p) { return p.outer; }
  static const lanelet::BasicPolygon2d& get(LLPolygon2d const& p) { return p.outer; }
};

template <>
struct interior_rings<LLPolygon2d> {
  static lanelet::BasicPolygons2d get(LLPolygon2d& p) { return p.inner; }
  static const lanelet::BasicPolygons2d& get(LLPolygon2d const& p) { return p.inner; }
};
}  // namespace traits
}  // namespace geometry
}  // namespace boost

/***********************************************************************
 *                   ALGORITHMS                                        *
 ***********************************************************************/
namespace lanelet {
namespace geometry {
using boost::geometry::area;
using boost::geometry::within;

//! Computes the distance of a polygon and a point in 2d (ie ignoring z)
template <typename PolygonT>
IfPoly<PolygonT, double> distance2d(const PolygonT& poly, const BasicPoint2d& p);

/**
 * @brief computes the distance of the outer bounds of two polygons in 3d.
 *
 * There is no such implementation in boost::geometry, so we implemented our own
 * solution using an RTree for more efficient computation.
 */
template <typename Polygon3dT>
IfPoly<Polygon3dT, double> distanceToBorder3d(const Polygon3dT& poly1, const Polygon3dT& poly2);

/**
 * Get the surrounding bounding box
 *
 * @return The enclosing axis aligned bounding box of all points.
 */
template <typename Polygon3dT>
IfPoly<Polygon3dT, BoundingBox3d> boundingBox3d(const Polygon3dT& polygon);

template <typename Polygon2dT>
IfPoly<Polygon2dT, BoundingBox2d> boundingBox2d(const Polygon2dT& polygon);

// implementations
template <typename PolygonT>
IfPoly<PolygonT, double> distance2d(const PolygonT& poly1, const PolygonT& poly2) {
  return distance(traits::toHybrid(traits::to2D(poly1)), traits::toHybrid(traits::to2D(poly2)));
}

template <typename PolygonT, typename LineStringT>
IfPoly<PolygonT, IfLS<LineStringT, double>> distance2d(const PolygonT& poly, const LineStringT& ls) {
  return distance(traits::toHybrid(traits::to2D(poly)), traits::toHybrid(traits::to2D(ls)));
}

template <typename PolygonT>
IfPoly<PolygonT, double> distance2d(const PolygonT& poly, const BasicPoint2d& p) {
  return distance(traits::to2D(traits::toConst(poly)), p);
}

template <typename Polygon3dT>
IfPoly<Polygon3dT, BoundingBox3d> boundingBox3d(const Polygon3dT& polygon) {
  static_assert(traits::is3D<Polygon3dT>(), "Please call this function with a 3D type!");
  BoundingBox3d bb;
  for (const auto& p : polygon) {
    bb.extend(traits::toBasicPoint(p));
  }
  return bb;
}

template <typename Polygon2dT>
IfPoly<Polygon2dT, BoundingBox2d> boundingBox2d(const Polygon2dT& polygon) {
  static_assert(traits::is2D<Polygon2dT>(), "Please call this function with a 2D type!");
  BoundingBox2d bb;
  for (const auto& p : polygon) {
    bb.extend(traits::toBasicPoint(p));
  }
  return bb;
}

template <typename Polygon3dT>
IfPoly<Polygon3dT, std::pair<BasicPoint3d, BasicPoint3d>> projectedBorderPoint3d(const Polygon3dT& l1,
                                                                                 const Polygon3dT& l2);

template <typename PolygonT>
IfPoly<PolygonT, bool> touches2d(const PolygonT& poly1, const PolygonT& poly2);

template <typename Polygon2dT>
IfPoly<Polygon2dT, bool> overlaps2d(const Polygon2dT& poly1, const Polygon2dT& poly2);

template <typename Polygon3dT>
IfPoly<Polygon3dT, bool> overlaps3d(const Polygon3dT& poly1, const Polygon3dT& poly2, double heightTolerance);
}  // namespace geometry
}  // namespace lanelet

#include "impl/Polygon.h"
