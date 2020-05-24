#pragma once

#include "lanelet2_core/Forward.h"
#include "lanelet2_core/primitives/LineString.h"

namespace lanelet {

/**
 * @brief Primitive 2d polygon with basic points
 * @ingroup PolygonPrimitives
 *
 *
 * This just a normal vector with normal points. We inherit from it, because we
 * need to create a new type that will be recognized as polygon by
 * boost::geometry (and not as linestring).
 */
class BasicPolygon2d : public BasicLineString2d {
 public:
  explicit BasicPolygon2d(const BasicLineString2d& other) : BasicLineString2d(other) {}
  using TwoDType = BasicPolygon2d;
  using ThreeDType = BasicPolygon3d;
  using HybridType = BasicPolygon2d;
  using PointType = BasicPoint2d;
  using BasicLineString2d::BasicLineString2d;
  BasicPolygon2d() = default;
};

/**
 * @brief Primitive 3d polygon with basic points
 * @ingroup PolygonPrimitives
 *
 *
 * This just a normal vector with normal points. We inherit from it, because we
 * need to create a new type that will be recognized as polygon by
 * boost::geometry (and not as linestring).
 */
class BasicPolygon3d : public BasicLineString3d {
 public:
  explicit BasicPolygon3d(const BasicLineString3d& other) : BasicLineString3d(other) {}
  using TwoDType = BasicPolygon2d;
  using ThreeDType = BasicPolygon3d;
  using HybridType = BasicPolygon3d;
  using PointType = BasicPoint3d;
  using BasicLineString3d::BasicLineString3d;
  BasicPolygon3d() = default;
};

//! @defgroup PolygonPrimitives Polygon
//! @ingroup Primitives
//!
//! ## General
//! Polygons are very similar to LineStringPrimitives. The only difference is
//! their interpretation: LineStrings are a one-dimensional structure, Polygons
//! are two-dimensional. They are assumed to be in clockwise order. An implicit
//! edge is assumend between the last point and the first point of the polygon.
//!
//! These polygons are second-class citisens of lanelet2. Most of the time, you
//! will use Area instead. Their only role is for an annotation of a region in
//! the map.
//!
//! ## Design
//! The design of a Polygon is completely identical to LineStrings. They even
//! share the same data structures and are directily convertible. The only
//! difference is that polygons can not be inverted. This would not make sense.
//!
//! As LineStrings and Polygons share the same data object, converting between
//! the two is as cheap as copying a shared_ptr.
//!
//! ## Compound versions
//! Similar to the CompoundLineString3d / CompoundLineString2d / ... classes,
//! CompoundPolygon2d / CompoundPolygon3d exists that allows to combine multiple
//! LineStrings3d to one closed polygon. They act like a normal polygon but
//! internally access the data of multiple line strings.

/**
 * @brief An immutable *clockwise* oriented, *open* (ie start point != end
 * point) polygon in 2d
 * @ingroup PolygonPrimitives
 * @ingroup ConstPrimitives
 */
class ConstPolygon2d : public ConstLineStringImpl<Point2d> {
 public:
  using ConstLineStringImpl::ConstLineStringImpl;
  using ConstType = ConstPolygon2d;
  using MutableType = Polygon2d;
  using TwoDType = ConstPolygon2d;
  using ThreeDType = ConstPolygon3d;
  using Category = traits::PolygonTag;
  using HybridType = ConstHybridPolygon2d;

  /**
   * @brief Conversion from/to ConstLineString2d
   *
   * This does not make sure that the polygon is valid and not
   * self-intersecting! If you want to ensure this, use boost::polygon's
   * is_valid.
   */
  explicit ConstPolygon2d(const ConstLineString2d& other) : ConstLineStringImpl(other) {}
  explicit operator ConstLineString2d() const { return ConstLineString2d(constData(), inverted()); }

  ConstPolygon2d() = default;

  //! create a simple 2d polygon from this (just a vector)
  BasicPolygon2d basicPolygon() const { return {basicBegin(), basicEnd()}; }

  //! implicit conversion to a basic polygon in 2d
  operator BasicPolygon2d() const {  // NOLINT
    return {basicBegin(), basicEnd()};
  }

  //! Returns the number of (geometrically valid) segments.
  size_t numSegments() const noexcept { return size() < 2 ? 0 : size(); }
};

/**
 * @brief An immutable *clockwise* oriented, *open* (ie start point != end
 * point) polygon in 3d
 * @ingroup PolygonPrimitives
 * @ingroup ConstPrimitives
 */
class ConstPolygon3d : public ConstLineStringImpl<Point3d> {
 public:
  using ConstLineStringImpl::ConstLineStringImpl;
  using ConstType = ConstPolygon3d;
  using MutableType = Polygon3d;
  using TwoDType = ConstPolygon2d;
  using ThreeDType = ConstPolygon3d;
  using Category = traits::PolygonTag;
  using HybridType = ConstHybridPolygon3d;

  ConstPolygon3d() = default;

  //! Conversion from ConstLineString3d. Does not ensure validity of the
  //! polygon!
  explicit ConstPolygon3d(const ConstLineString3d& other) : ConstLineStringImpl(other) {}
  explicit operator ConstLineString3d() const { return ConstLineString3d(constData(), inverted()); }

  //! create a simple 3d polygon from this (just a vector)
  BasicPolygon3d basicPolygon() const { return {basicBegin(), basicEnd()}; }

  //! implicit conversion to a basic polygon
  operator BasicPolygon3d() const {  // NOLINT
    return {basicBegin(), basicEnd()};
  }

  //! Returns the number of (geometrically valid) segments.
  size_t numSegments() const noexcept { return size() < 2 ? 0 : size(); }
};

/**
 * @brief A mutable *clockwise* oriented, *open* (ie start point != end point)
 * polygon in 3d
 * @ingroup PolygonPrimitives
 * @ingroup Primitives
 */
class Polygon3d : public LineStringImpl<ConstPolygon3d> {
 public:
  using LineStringImpl::LineStringImpl;
  using ConstType = ConstPolygon3d;
  using MutableType = Polygon3d;
  using TwoDType = Polygon2d;
  using ThreeDType = Polygon3d;

  Polygon3d() = default;  // gcc5 needs this

  //! Conversion from LineString3d. Does not ensure validity of the polygon!
  explicit Polygon3d(const LineString3d& other) : LineStringImpl(other) {}
  explicit operator LineString3d() const { return LineString3d(data(), inverted()); }

  friend class Polygon2d;

  //! create a simple 3d polygon from this (just a vector)
  BasicPolygon3d basicPolygon() const { return {basicBegin(), basicEnd()}; }

  //! implicit conversion to a basic polygon
  operator BasicPolygon3d() const {  // NOLINT
    return {basicBegin(), basicEnd()};
  }
};

/**
 * @brief An mutable *clockwise* oriented, *open* (ie start point != end
 * point) polygon in 2d
 * @ingroup PolygonPrimitives
 * @ingroup Primitives
 */
class Polygon2d : public LineStringImpl<ConstPolygon2d> {
 public:
  using LineStringImpl::LineStringImpl;
  using ConstType = ConstPolygon2d;
  using MutableType = Polygon2d;
  using TwoDType = Polygon2d;
  using ThreeDType = Polygon3d;

  Polygon2d() = default;

  //! Conversion from LineString2d. Does not ensure validity of the polygon!
  explicit Polygon2d(const LineString2d& other) : LineStringImpl(other) {}
  explicit operator LineString2d() const { return LineString2d(data(), inverted()); }

  /**
   * @brief create a simple 2d polygon from this (just a vector)
   */
  inline BasicPolygon2d basicPolygon() const {
    BasicPolygon2d::allocator_type alloc;  // gcc produces undef refs otherwise
    return BasicPolygon2d(basicBegin(), basicEnd(), alloc);
  }
  inline operator BasicPolygon2d() const {  // NOLINT
    return BasicPolygon2d(basicBegin(), basicEnd());
  }
  friend class Polygon3d;
};

/**
 * @brief Polygon with access to primitive points
 * @ingroup PolygonPrimitives
 * @ingroup ConstPrimitives
 *
 *
 * This polygon tries to behave as close to a BasicPolygon3d as possible
 * while still keeping all properties (id, attributes) of a ConstPolygon. This
 * is important as some functions of boost::geometry do not like our ConstPoint
 * type.
 *
 * As with all lanelet primitives, conversion from other primitives is cheap.
 */
class ConstHybridPolygon3d : public ConstPolygon3d {
 public:
  using const_iterator = BasicIterator;  // NOLINT
  using iterator = BasicIterator;        // NOLINT

  using ConstPolygon3d::ConstPolygon3d;
  using ConstType = ConstHybridPolygon3d;
  using TwoDType = ConstHybridPolygon2d;
  using ThreeDType = ConstHybridPolygon3d;
  using PointType = BasicPoint3d;

  ConstHybridPolygon3d() = default;

  //! Conversion from ConstPolygon3d
  explicit ConstHybridPolygon3d(const ConstPolygon3d& poly) : ConstPolygon3d(poly) {}

  //! BasicPoint3d Iterator to begin
  BasicIterator begin() const noexcept { return basicBegin(); }

  //! BasicPoint3d Iterator to past-the-end
  BasicIterator end() const noexcept { return basicEnd(); }

  //! Get first BasicPoint3d
  const BasicPointType& front() const noexcept { return ConstPolygon3d::front().basicPoint(); }

  //! Get last BasicPoint3d
  const BasicPointType& back() const noexcept { return ConstPolygon3d::back().basicPoint(); }

  //! access BasicPoint3d at specific position
  const BasicPointType& operator[](size_t idx) const noexcept { return ConstPolygon3d::operator[](idx).basicPoint(); }
};

/**
 * @brief Polygon with access to primitive points
 * @ingroup PolygonPrimitives
 * @ingroup ConstPrimitives
 *
 *
 * This polygon tries to behave as close to a BasicPolygon2d as possible
 * while still keeping all properties (id, attributes) of a ConstPolygon2d. This
 * is important as some functions of boost::geometry do not like our
 * ConstPoint2d type.
 *
 * As with all lanelet primitives, conversion from other primitives is cheap.
 */
class ConstHybridPolygon2d : public ConstPolygon2d {
 public:
  using const_iterator = BasicIterator;  // NOLINT
  using iterator = BasicIterator;        // NOLINT

  using ConstPolygon2d::ConstPolygon2d;
  using ConstType = ConstHybridPolygon2d;
  using TwoDType = ConstHybridPolygon2d;
  using ThreeDType = ConstHybridPolygon3d;
  using PointType = BasicPoint2d;
  using HybridType = ConstHybridPolygon2d;

  ConstHybridPolygon2d() = default;

  //! Conversion from ConstPolygon2d
  explicit ConstHybridPolygon2d(const ConstPolygon2d& poly) : ConstPolygon2d(poly) {}

  //! BasicPoint2d Iterator to begin
  BasicIterator begin() const noexcept { return basicBegin(); }

  //! BasicPoint2d Iterator to past-the-end
  BasicIterator end() const noexcept { return basicEnd(); }

  //! Get first BasicPoint2d
  BasicPointType front() const noexcept { return ConstPolygon2d::front().basicPoint(); }

  //! Get last BasicPoint2d
  BasicPointType back() const noexcept { return ConstPolygon2d::back().basicPoint(); }

  //! access element at specific position
  BasicPointType operator[](size_t idx) const noexcept { return ConstPolygon2d::operator[](idx).basicPoint(); }
};

/**
 * @brief A (basic) 2d polygon with holes inside
 * @ingroup PolygonPrimitives
 *
 * This class is thought for geometry calculations,
 * it has no properties of a normal lanelet primitive.
 */
class BasicPolygonWithHoles3d {
 public:
  BasicPolygon3d outer;
  BasicPolygons3d inner;
};

/**
 * @brief A (basic) 2d polygon with holes inside
 * @ingroup PolygonPrimitives
 *
 * This class is thought for geometry calculations,
 * it has no properties of a normal lanelet primitive.
 */
class BasicPolygonWithHoles2d {
 public:
  BasicPolygon2d outer;
  BasicPolygons2d inner;
};

inline std::ostream& operator<<(std::ostream& stream, const ConstPolygon2d& obj) {
  return stream << ConstLineString2d(obj);
}

inline std::ostream& operator<<(std::ostream& stream, const ConstPolygon3d& obj) {
  return stream << ConstLineString2d(obj);
}

namespace internal {

template <>
class Converter<const ConstPolygon3d> {
 public:
  template <typename T>
  const ConstPolygon3d& convert(const T& t) const {
    val_ = t;
    return val_;
  }

 private:
  mutable ConstPolygon3d val_;
};
template <>
class Converter<const ConstPolygon2d> {
 public:
  template <typename T>
  const ConstPolygon2d& convert(const T& t) const {
    val_ = t;
    return val_;
  }

 private:
  mutable ConstPolygon2d val_;
};
}  // namespace internal

namespace traits {
template <typename T>
constexpr bool isPolygonT() {
  return isCategory<T, traits::PolygonTag>();
}

template <>
struct PrimitiveTraits<BasicPolygon2d> {
  using ConstType = BasicPolygon2d;
  using MutableType = BasicPolygon2d;
  using TwoDType = BasicPolygon2d;
  using ThreeDType = BasicPolygon3d;
  using Category = PolygonTag;
};
template <>
struct PrimitiveTraits<BasicPolygon3d> {
  using ConstType = BasicPolygon3d;
  using MutableType = BasicPolygon3d;
  using TwoDType = BasicPolygon2d;
  using ThreeDType = BasicPolygon3d;
  using Category = PolygonTag;
};
template <>
inline BasicPolygon2d to2D<BasicPolygon3d>(const BasicPolygon3d& primitive) {
  BasicPolygon2d p2d(primitive.size());
  std::transform(primitive.begin(), primitive.end(), p2d.begin(), utils::to2D<BasicPoint3d>);
  return p2d;
}
}  // namespace traits
template <typename T, typename RetT>
using IfPoly = std::enable_if_t<traits::isPolygonT<T>(), RetT>;

}  // namespace lanelet

// Hash function for usage in containers
namespace std {
template <>
struct hash<lanelet::Polygon3d> : public lanelet::HashBase<lanelet::Polygon3d> {};
template <>
struct hash<lanelet::ConstPolygon3d> : public lanelet::HashBase<lanelet::ConstPolygon3d> {};
template <>
struct hash<lanelet::Polygon2d> : public lanelet::HashBase<lanelet::Polygon2d> {};
template <>
struct hash<lanelet::ConstPolygon2d> : public lanelet::HashBase<lanelet::ConstPolygon2d> {};
}  // namespace std
