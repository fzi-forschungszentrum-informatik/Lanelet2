#pragma once
#pragma GCC diagnostic push
#if defined __GNUC__ && (__GNUC__ >= 6)
#pragma GCC diagnostic ignored "-Wignored-attributes"
#pragma GCC diagnostic ignored "-Wint-in-bool-context"
#endif
#include <Eigen/Core>
#pragma GCC diagnostic pop
#include <utility>

#include "lanelet2_core/Forward.h"
#include "lanelet2_core/primitives/Primitive.h"
#include "lanelet2_core/primitives/Traits.h"
#include "lanelet2_core/utility/TransformIterator.h"
#include "lanelet2_core/utility/Utilities.h"

namespace lanelet {

using BasicPoint3d = Eigen::Vector3d;                                //!< a simple 3d-point
using BasicPoint2d = Eigen::Matrix<double, 2, 1, Eigen::DontAlign>;  //!< a simple 2d-point
using BasicPoints2d = std::vector<Eigen::Vector2d,
                                  Eigen::aligned_allocator<Eigen::Vector2d>>;  //!< multiple simple 2d-points
using BasicPoints3d = std::vector<BasicPoint3d>;                               //!< multiple simple 3d-points

namespace internal {
template <>
class Converter<const BasicPoint2d> {
 public:
  template <typename InType>
  const BasicPoint2d& convert(const InType& t) const {
    return t.basicPoint2d();
  }
};
}  // namespace internal

namespace traits {
//! used for templates that require info about this point
template <>
struct PrimitiveTraits<BasicPoint3d> {
  using ConstType = BasicPoint3d;
  using MutableType = BasicPoint3d;
  using TwoDType = BasicPoint2d;
  using ThreeDType = BasicPoint3d;
  using Category = PointTag;
};
template <>
struct PrimitiveTraits<BasicPoint2d> {
  using ConstType = BasicPoint2d;
  using MutableType = BasicPoint2d;
  using TwoDType = BasicPoint2d;
  using ThreeDType = BasicPoint3d;
  using Category = PointTag;
};
template <>
struct PrimitiveTraits<Eigen::Vector2d> {
  using ConstType = BasicPoint2d;
  using MutableType = BasicPoint2d;
  using TwoDType = BasicPoint2d;
  using ThreeDType = BasicPoint3d;
  using Category = PointTag;
};
template <>
struct PointTraits<BasicPoint3d> : PrimitiveTraits<BasicPoint3d> {
  using BasicPoint = BasicPoint3d;
  using ConstPoint = typename PrimitiveTraits<BasicPoint3d>::ConstType;
  using MutablePoint = typename PrimitiveTraits<BasicPoint3d>::MutableType;
  static constexpr bool IsPrimitive = false;
  static constexpr Dimensions Dimension = Dimensions::Three;
};
template <>
struct PointTraits<BasicPoint2d> : PrimitiveTraits<BasicPoint2d> {
  using BasicPoint = BasicPoint2d;
  using ConstPoint = typename PrimitiveTraits<BasicPoint2d>::ConstType;
  using MutablePoint = typename PrimitiveTraits<BasicPoint2d>::MutableType;
  static constexpr bool IsPrimitive = false;
  static constexpr Dimensions Dimension = Dimensions::Three;
};
template <>
struct PointTraits<Eigen::Vector2d> : PrimitiveTraits<Eigen::Vector2d> {
  using BasicPoint = BasicPoint2d;
  using ConstPoint = typename PrimitiveTraits<BasicPoint2d>::ConstType;
  using MutablePoint = typename PrimitiveTraits<BasicPoint2d>::MutableType;
  static constexpr bool IsPrimitive = false;
  static constexpr Dimensions Dimension = Dimensions::Three;
};

template <>
inline BasicPoint3d to3D(const BasicPoint2d& primitive) {
  return {primitive.x(), primitive.y(), 0};
}
template <>
inline BasicPoint2d to2D(const BasicPoint3d& primitive) {
  return {primitive.x(), primitive.y()};
}
}  // namespace traits

//! Describes a position in linestring coordinates
struct ArcCoordinates {
  double length{0.};    //!< length along linestring
  double distance{0.};  //!< signed distance to linestring (left=positive)
};

//! Common data management class for all Point primitives.
//! @ingroup DataObjects
class PointData : public PrimitiveData {  // NOLINT
 public:
  PointData(Id id, BasicPoint3d point, const AttributeMap& attributes = AttributeMap())
      : PrimitiveData(id, attributes), point(point), point2d_{point.x(), point.y()} {}
  PointData(const PointData&) = delete;
  PointData& operator=(const LineStringData&) = delete;
  PointData(PointData&&) = default;
  PointData& operator=(PointData&&) = default;
  ~PointData() = default;
  const BasicPoint2d& point2d() const {
    if (BOOST_UNLIKELY(point.head<2>() != point2d_)) {
      point2d_ = point.head<2>();
    }
    return point2d_;
  }

  BasicPoint3d point;  // NOLINT(cppcoreguidelines-non-private-member-variables-in-classes)

 private:
  mutable BasicPoint2d point2d_;
};

//! @defgroup PointPrimitives Point
//! @ingroup Primitives
//!
//! # Design
//! Points are the most basic data structure of lanelet2. They are
//! characterized by their coordinates and, as all primitives, an ID and
//! attributes.
//!
//! The exist in a 2D and 3D and in a mutable/immutable version.
//!
//! # Geometry
//! Points can be used for geometry calculations in boost::geometry. If you need
//! more basic operations, you can convert them to their respective basic point
//! type, e.g. BasicPoint2d, which is an Eigen object that supports all usual
//! math operations and is mutable.
//!
//! All coordinates are in *meters*.

/**
 * @brief An immutable 2d point
 * @ingroup PointPrimitives
 * @ingroup ConstPrimitives
 */
class ConstPoint2d : public ConstPrimitive<PointData> {
 public:
  using BasicPoint = BasicPoint2d;  //!< returned basic point type
  using ConstType = ConstPoint2d;
  using MutableType = Point2d;
  using TwoDType = ConstPoint2d;
  using ThreeDType = ConstPoint3d;
  using Category = traits::PointTag;
  static constexpr traits::Dimensions Dimension = traits::Dimensions::Two;

  using ConstPrimitive::ConstPrimitive;

  //! @brief Construct from an id and a point
  ConstPoint2d(Id id, const BasicPoint3d& point, const AttributeMap& attributes = AttributeMap())
      : ConstPrimitive<PointData>{std::make_shared<PointData>(id, point, attributes)} {}  // NOLINT

  //! @brief Construct from an id and coordinates
  /**
   * The z coordinate is required because this point can be converted to a 3d
   * point, where z matters.
   */
  explicit ConstPoint2d(Id id = InvalId, double x = 0., double y = 0., double z = 0.,
                        const AttributeMap& attributes = AttributeMap())
      : ConstPrimitive<PointData>{std::make_shared<PointData>(  // NOLINT
            id, BasicPoint3d(x, y, z), attributes)} {}

  //! A ConstPoint 2d is implicitly convertible to a normal 2d point
  operator BasicPoint2d() const noexcept {  // NOLINT
    return point2d();
  }

  //! Get an Eigen point for faster calculations
  const BasicPoint& basicPoint() const noexcept { return point2d(); }

  //! Get a 2d Eigen point. Does not make sense for this particular class, but
  //! for its children.
  const BasicPoint2d& basicPoint2d() const noexcept { return point2d(); }

  //! gets x coordinate
  double x() const noexcept { return point().x(); }

  //! gets y coordinate
  double y() const noexcept { return point().y(); }

 protected:
  const BasicPoint3d& point() const { return constData()->point; }
  const BasicPoint2d& point2d() const { return constData()->point2d(); }
};

//! @brief An immutable 3d point.
//! @ingroup PointPrimitives
//! @ingroup ConstPrimitives
class ConstPoint3d : public ConstPoint2d {
 public:
  using BasicPoint = BasicPoint3d;
  using ConstType = ConstPoint3d;
  using MutableType = Point3d;
  static constexpr traits::Dimensions Dimension = traits::Dimensions::Three;

  using ConstPoint2d::ConstPoint2d;
  ConstPoint3d() = default;
  explicit ConstPoint3d(const ConstPoint2d& other) : ConstPoint2d(other) {}

  operator BasicPoint2d() const noexcept = delete;  // NOLINT

  //! Implicit cast to a basic 3d point
  operator const BasicPoint3d&() const noexcept {  // NOLINT
    return point();
  }

  //! get a basic 3d point
  const BasicPoint& basicPoint() const noexcept { return point(); }

  //! get the z coordinate
  double z() const noexcept { return point().z(); }
};

//! @brief A mutable 2d point.
//! @ingroup PointPrimitives
//! @ingroup Primitives
class Point2d : public Primitive<ConstPoint3d> {
 public:
  using BasicPoint = BasicPoint2d;
  using ConstType = ConstPoint2d;
  using MutableType = Point2d;
  using TwoDType = Point2d;
  using ThreeDType = Point3d;
  static constexpr traits::Dimensions Dimension = traits::Dimensions::Two;

  using Primitive::Primitive;
  using Primitive::x;
  using Primitive::y;
  Point2d() = default;
  operator const BasicPoint3d&() const = delete;

  //! Cast to a basic 2d point
  operator BasicPoint2d() const noexcept {  // NOLINT
    return point().head<2>();
  }

  //! get a mutable reference to the 2d point data
  auto basicPoint() noexcept { return point().head<2>(); }

  //! get a basic 2d point
  const BasicPoint2d& basicPoint() const noexcept { return point2d(); }

  //! gets a mutable reference to the x coordinate
  double& x() noexcept { return point().x(); }

  //! gets a mutable reference to the y coordinate
  double& y() noexcept { return point().y(); }
  double z() const noexcept = delete;

 protected:
  using Primitive::point;
  BasicPoint3d& point() { return data()->point; }
};

//! @brief A mutable 3d point.
//! @ingroup PointPrimitives
//! @ingroup Primitives
class Point3d : public Point2d {
 public:
  using BasicPoint = BasicPoint3d;
  using ConstType = ConstPoint3d;
  using MutableType = Point3d;
  Point3d() = default;
  static constexpr traits::Dimensions Dimension = traits::Dimensions::Three;

  using Point2d::Point2d;

  //! constructs a 3D from a 2D point. No data is lost in that process.
  explicit Point3d(const Point2d& other) : Point2d(other) {}

  using Primitive::x;
  using Primitive::y;
  using Primitive::z;

  operator BasicPoint2d() const noexcept = delete;

  //! Implicit cast to a mutable BasicPoint3d
  operator BasicPoint3d&() noexcept { return point(); }  // NOLINT

  //! Implicit cast to an immutable basicPoint3d
  operator const BasicPoint3d&() const noexcept { return point(); }  // NOLINT

  //! Get a mutable reference to the internal 3d point
  BasicPoint& basicPoint() noexcept { return point(); }

  //! Get an immutable reference to the internal 3d point
  const BasicPoint& basicPoint() const noexcept { return point(); }

  //! gets a mutable reference to the x coordinate
  double& x() noexcept { return point().x(); }

  //! gets a mutable reference to the y coordinate
  double& y() noexcept { return point().y(); }

  //! gets a mutable reference to the z coordinate
  double& z() noexcept { return point().z(); }
};

inline std::ostream& operator<<(std::ostream& stream, const ConstPoint2d& obj) {
  return stream << "[id: " << obj.id() << " x: " << obj.x() << " y: " << obj.y() << "]";
}

inline std::ostream& operator<<(std::ostream& stream, const ConstPoint3d& obj) {
  return stream << "[id: " << obj.id() << " x: " << obj.x() << " y: " << obj.y() << " z: " << obj.z() << "]";
}

inline std::ostream& operator<<(std::ostream& stream, const Point2d& obj) {
  return stream << "[id: " << obj.id() << " x: " << obj.x() << " y: " << obj.y() << "]";
}

inline std::ostream& operator<<(std::ostream& stream, const Point3d& obj) {
  return stream << "[id: " << obj.id() << " x: " << obj.x() << " y: " << obj.y() << " z: " << obj.z() << "]";
}

namespace traits {
template <typename T>
constexpr bool isPointT() {
  return isCategory<T, traits::PointTag>();
}
}  // namespace traits
template <typename T, typename RetT>
using IfPT = std::enable_if_t<traits::isPointT<T>(), RetT>;

}  // namespace lanelet

// Hash function for usage in containers
namespace std {
template <>
struct hash<lanelet::Point3d> : public lanelet::HashBase<lanelet::Point3d> {};
template <>
struct hash<lanelet::ConstPoint3d> : public lanelet::HashBase<lanelet::ConstPoint3d> {};
template <>
struct hash<lanelet::Point2d> : public lanelet::HashBase<lanelet::Point2d> {};
template <>
struct hash<lanelet::ConstPoint2d> : public lanelet::HashBase<lanelet::ConstPoint2d> {};
}  // namespace std
