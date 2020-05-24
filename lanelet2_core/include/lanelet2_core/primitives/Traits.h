#pragma once
#include "lanelet2_core/Forward.h"

namespace lanelet {
namespace traits {
struct PointTag {};              //! Identifies PointPrimitives
struct BoundingBoxTag {};        //! Identifies bounding boxes
struct LineStringTag {};         //! Identifies LineStringPrimitives
struct PolygonTag {};            //! Identifies PolygonPrimitives
struct LaneletTag {};            //! Identifies LaneletPrimitives
struct AreaTag {};               //! Identifies AreaPrimitives
struct RegulatoryElementTag {};  //! Identifies RegulatoryElementPrimitives

/**
 *  @brief Trait class for querying information on a lanelet primitive
 *
 * All lanelet primitives are compatible with this trait class.
 */
template <typename T>
struct PrimitiveTraits {
  using DataType = typename T::DataType;
  using ConstType = typename T::ConstType;
  using MutableType = typename T::MutableType;
  using TwoDType = typename T::TwoDType;
  using ThreeDType = typename T::ThreeDType;
  using Category = typename T::Category;
};

// specialization for RegulatoryElementPtr
template <>
struct PrimitiveTraits<RegulatoryElementPtr> {
  using DataType = RegulatoryElementData;
  using ConstType = RegulatoryElementConstPtr;
  using MutableType = RegulatoryElementPtr;
  using TwoDType = RegulatoryElementPtr;
  using ThreeDType = RegulatoryElementPtr;
  using Category = RegulatoryElementTag;
};
template <>
struct PrimitiveTraits<RegulatoryElementConstPtr> {
  using DataType = RegulatoryElementData;
  using ConstType = RegulatoryElementConstPtr;
  using MutableType = RegulatoryElementPtr;
  using TwoDType = RegulatoryElementPtr;
  using ThreeDType = RegulatoryElementPtr;
  using Category = RegulatoryElementTag;
};

//! Can be used to determine which primitive type is managed by a primitive
template <typename PrimitiveT>
struct Owned {
  using Type = PrimitiveT;
};

template <>
struct Owned<LineString3d> {
  using Type = Point3d;
};

template <>
struct Owned<Polygon3d> {
  using Type = Point3d;
};

template <>
struct Owned<Lanelet> {
  using Type = LineString3d;
};

template <>
struct Owned<Area> {
  using Type = LineString3d;
};

template <typename PrimitiveT>
using OwnedT = typename Owned<PrimitiveT>::Type;

//! Utility for determinig the matching const type for a primitive
template <typename PrimitiveT>
using ConstPrimitiveType = typename PrimitiveTraits<PrimitiveT>::ConstType;

//! Utility for determinig the matching mutable type for a primitive
template <typename PrimitiveT>
using MutablePrimitiveType = typename PrimitiveTraits<PrimitiveT>::MutableType;

//! Utility for determinig the matching two dimensional type for a primitive
template <typename PrimitiveT>
using TwoD = typename PrimitiveTraits<PrimitiveT>::TwoDType;

//! f Utility for determinig the matching three dimensional type for a primitive
template <typename PrimitiveT>
using ThreeD = typename PrimitiveTraits<PrimitiveT>::ThreeDType;

//! Converts a primitive to its matching threeD type
template <typename PrimitiveT>
auto to3D(const PrimitiveT& primitive) -> ThreeD<PrimitiveT> {
  return ThreeD<PrimitiveT>(primitive);
}

//! Converts a primitive to its matching twoD type
template <typename PrimitiveT>
auto to2D(const PrimitiveT& primitive) -> TwoD<PrimitiveT> {
  return TwoD<PrimitiveT>(primitive);
}

//! Converts a primitive to its matching const type
template <typename PrimitiveT>
ConstPrimitiveType<PrimitiveT> toConst(const PrimitiveT& primitive) {
  return ConstPrimitiveType<PrimitiveT>(primitive);
}

template <typename PrimitiveT>
constexpr bool isConst() {
  return std::is_same<PrimitiveT, ConstPrimitiveType<PrimitiveT>>::value;
}

//! Checks (at compile time) whether a primitive is two dimensional
template <typename PrimitiveT>
constexpr bool is2D() {
  return std::is_same<std::decay_t<PrimitiveT>, TwoD<PrimitiveT>>::value;
}

//! Checks (at compile time) whether a primitive is three dimensional
template <typename PrimitiveT>
constexpr bool is3D() {
  return std::is_same<std::decay_t<PrimitiveT>, ThreeD<PrimitiveT>>::value;
}

enum class Dimensions { Two = 2, Three = 3 };

//! Specialization of traits for points
template <typename PointT>
struct PointTraits : PrimitiveTraits<PointT> {
  using BasicPoint = typename PointT::BasicPoint;
  using ConstPoint = typename PrimitiveTraits<PointT>::ConstType;
  using MutablePoint = typename PrimitiveTraits<PointT>::MutableType;
  static constexpr bool IsPrimitive = true;
  static constexpr Dimensions Dimension = PointT::Dimension;
};

//! Specialization of traits for linestrings
template <typename LineStringT>
struct LineStringTraits : PrimitiveTraits<LineStringT> {
  using PointType = typename LineStringT::PointType;
  using HybridType = typename LineStringT::HybridType;
};

//! Specialization of traits for polygons
template <typename PolygonT>
struct PolygonTraits : PrimitiveTraits<PolygonT> {
  using PointType = typename PolygonT::PointType;
  using HybridType = typename PolygonT::HybridType;
};

template <typename PointT>
using BasicPointT = typename PointTraits<PointT>::BasicPoint;

template <typename PointT>
using ConstPointT = typename PointTraits<PointT>::ConstPoint;

template <typename PointT>
using MutablePointT = typename PointTraits<PointT>::MutablePoint;

template <typename PointT>
auto toBasicPoint(const PointT& point) -> std::enable_if_t<PointTraits<PointT>::IsPrimitive, BasicPointT<PointT>> {
  return point.basicPoint();
}
template <typename PointT>
auto toBasicPoint(const PointT& point)
    -> std::enable_if_t<!PointTraits<PointT>::IsPrimitive && std::is_same<PointT, BasicPointT<PointT>>::value, PointT> {
  return point;
}
template <typename PointT>
auto toBasicPoint(const PointT& point)
    -> std::enable_if_t<!PointTraits<PointT>::IsPrimitive && !std::is_same<PointT, BasicPointT<PointT>>::value,
                        BasicPointT<PointT>> {
  return BasicPointT<PointT>(point);
}

template <typename PrimT, typename TagT>
constexpr bool isCategory() {
  return std::is_same<typename PrimitiveTraits<std::decay_t<PrimT>>::Category, TagT>::value;
}

template <typename T>
constexpr bool noRegelem() {
  return !traits::isCategory<T, traits::RegulatoryElementTag>();
}

template <typename LineStringT>
using PointType = typename LineStringTraits<LineStringT>::PointType;

template <typename PrimT>
Id getId(const PrimT& prim) {
  return prim.id();
}
}  // namespace traits

namespace utils {
// make trait based helper functions available in utils namespace
using traits::getId;
using traits::to2D;
using traits::to3D;
using traits::toBasicPoint;
using traits::toConst;
}  // namespace utils
}  // namespace lanelet
