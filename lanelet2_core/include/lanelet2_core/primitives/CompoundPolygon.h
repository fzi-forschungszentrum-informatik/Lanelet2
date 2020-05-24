#pragma once
#include "lanelet2_core/primitives/CompoundLineString.h"
#include "lanelet2_core/primitives/Polygon.h"

namespace lanelet {

/**
 * @brief Combines multiple linestrings to one polygon in 2d
 * @ingroup PolygonPrimitives
 */
class CompoundPolygon2d : public CompoundLineStringImpl<ConstPoint2d> {
 public:
  using TwoDType = CompoundPolygon2d;
  using ThreeDType = CompoundPolygon3d;
  using ConstType = CompoundPolygon2d;
  using HybridType = CompoundHybridPolygon2d;
  using MutableType = void;
  using Category = traits::PolygonTag;

  using CompoundLineStringImpl::CompoundLineStringImpl;
  explicit CompoundPolygon2d(const CompoundPolygons2d& other)
      : CompoundPolygon2d(utils::concatenate(other, [](auto& cls) { return cls.lineStrings(); })) {}
  explicit CompoundPolygon2d(const CompoundLineStrings2d& other)
      : CompoundPolygon2d(utils::concatenate(other, [](auto& cls) { return cls.lineStrings(); })) {}

  CompoundPolygon2d() = default;
  explicit CompoundPolygon2d(const CompoundLineString2d& other) : CompoundLineStringImpl(other) {}
  explicit operator CompoundLineString2d() const { return CompoundLineString2d(constData(), inverted()); }
  //! Returns the number of (geometrically valid) segments.
  size_t numSegments() const noexcept { return std::max(size_t(1), size()); }

  BasicLineString basicLineString() = delete;
  BasicPolygon2d basicPolygon() const { return {basicBegin(), basicEnd()}; }
};

/**
 * @brief Combines multiple linestrings to one polygon in 3d
 * @ingroup PolygonPrimitives
 */
class CompoundPolygon3d : public CompoundLineStringImpl<ConstPoint3d> {
 public:
  using TwoDType = CompoundPolygon2d;
  using ThreeDType = CompoundPolygon3d;
  using ConstType = CompoundPolygon3d;
  using HybridType = CompoundHybridPolygon3d;
  using MutableType = void;
  using CompoundLineStringImpl::CompoundLineStringImpl;
  using Category = traits::PolygonTag;
  explicit CompoundPolygon3d(const CompoundLineStrings3d& other)
      : CompoundPolygon3d(utils::concatenate(other, [](auto& cls) { return cls.lineStrings(); })) {}
  explicit CompoundPolygon3d(const CompoundPolygons3d& other)
      : CompoundPolygon3d(utils::concatenate(other, [](auto& cls) { return cls.lineStrings(); })) {}

  CompoundPolygon3d() = default;
  explicit CompoundPolygon3d(const CompoundLineString3d& other) : CompoundLineStringImpl(other) {}
  explicit operator CompoundLineString3d() const { return CompoundLineString3d(constData(), inverted()); }
  //! Returns the number of (geometrically valid) segments.
  size_t numSegments() const noexcept { return std::max(size_t(1), size()); }

  BasicLineString basicLineString() = delete;
  BasicPolygon3d basicPolygon() const {
    return BasicPolygon3d{basicBegin(), basicEnd(), BasicPolygon3d::allocator_type()};
  }
};

/**
 * @brief Combines multiple linestrings to one polygon in 2d that returns
 * BasicPoint2d
 * @ingroup PolygonPrimitives
 */
class CompoundHybridPolygon2d : public CompoundLineStringImpl<BasicPoint2d> {
 public:
  using CompoundLineStringImpl::CompoundLineStringImpl;
  using TwoDType = CompoundHybridPolygon2d;
  using ThreeDType = CompoundHybridPolygon3d;
  using HybridType = CompoundHybridPolygon2d;
  using ConstType = CompoundHybridPolygon2d;
  using MutableType = void;
  using Category = traits::PolygonTag;

  explicit CompoundHybridPolygon2d(const CompoundHybridPolygons2d& other)
      : CompoundHybridPolygon2d(utils::concatenate(other, [](auto& cls) { return cls.lineStrings(); })) {}
  explicit CompoundHybridPolygon2d(const CompoundHybridLineStrings2d& other)
      : CompoundHybridPolygon2d(utils::concatenate(other, [](auto& cls) { return cls.lineStrings(); })) {}

  CompoundHybridPolygon2d() = default;
  explicit CompoundHybridPolygon2d(const CompoundHybridLineString2d& other) : CompoundLineStringImpl(other) {}
  explicit operator CompoundHybridLineString2d() const { return CompoundHybridLineString2d(constData(), inverted()); }
  //! Returns the number of (geometrically valid) segments.
  size_t numSegments() const noexcept { return std::max(size_t(1), size()); }
  BasicLineString basicLineString() = delete;
  BasicPolygon2d basicPolygon() const { return {basicBegin(), basicEnd()}; }
};

/**
 * @brief Combines multiple linestrings to one polygon in 3d that returns
 * BasicPoint3d
 * @ingroup PolygonPrimitives
 */
class CompoundHybridPolygon3d : public CompoundLineStringImpl<BasicPoint3d> {
 public:
  using CompoundLineStringImpl::CompoundLineStringImpl;
  using TwoDType = CompoundHybridPolygon2d;
  using ThreeDType = CompoundHybridPolygon3d;
  using ConstType = CompoundHybridPolygon3d;
  using HybridType = CompoundHybridPolygon3d;
  using MutableType = void;
  using Category = traits::PolygonTag;

  explicit CompoundHybridPolygon3d(const CompoundHybridLineStrings3d& other)
      : CompoundHybridPolygon3d(utils::concatenate(other, [](auto& cls) { return cls.lineStrings(); })) {}
  explicit CompoundHybridPolygon3d(const CompoundHybridPolygons3d& other)
      : CompoundHybridPolygon3d(utils::concatenate(other, [](auto& cls) { return cls.lineStrings(); })) {}
  CompoundHybridPolygon3d() = default;
  explicit CompoundHybridPolygon3d(const CompoundHybridLineString3d& other) : CompoundLineStringImpl(other) {}
  explicit operator CompoundHybridLineString3d() const { return CompoundHybridLineString3d(constData(), inverted()); }
  //! Returns the number of (geometrically valid) segments.
  size_t numSegments() const noexcept { return std::max(size_t(1), size()); }
  BasicLineString basicLineString() = delete;
  BasicPolygon3d basicPolygon() const { return {basicBegin(), basicEnd()}; }
};
}  // namespace lanelet
