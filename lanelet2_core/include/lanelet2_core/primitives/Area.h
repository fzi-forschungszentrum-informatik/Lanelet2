// this is for emacs file handling -*- mode: c++; c-basic-offset: 2;
// indent-tabs-mode: nil -*-

#pragma once

#include <memory>
#include <utility>

#include "lanelet2_core/Forward.h"
#include "lanelet2_core/primitives/CompoundPolygon.h"
#include "lanelet2_core/primitives/Point.h"
#include "lanelet2_core/primitives/Polygon.h"
#include "lanelet2_core/primitives/Primitive.h"
#include "lanelet2_core/utility/Optional.h"

namespace lanelet {
//! @defgroup AreaPrimitives Area
//! @ingroup Primitives
//!
//! ## General
//! While \ref LaneletPrimitives represet a road section with directed motion,
//! Areas represent regions outside of the normal lanes where either no motion
//! occures (as in green stripes) or undirected motion occurs (e.g. on parking
//! spaces). Areas represent a polygon that can have holes.
//!
//! ## Design
//! Areas are similar to \ref LaneletPrimitives in that they can have
//! RegulatoryElements that represent traffic rules.
//! From a geometrical point of view, they are polygons with an outer contour
//! and multiple inner contours. Every contour can be composed of multiple \ref
//! LineStringPrimitives that together form a closed ring.
//!
//! ## Geometry
//! Areas can be used direcly for some geometry calculations (see
//! lanelet::geometry). They can also be interpreted as a polygon to make use of
//! all polygon operations that boost::polygon supports.

using InnerBounds = std::vector<LineStrings3d>;
using ConstInnerBounds = std::vector<ConstLineStrings3d>;

//! @brief Common data management class for all Area-Typed objects.
//! @ingroup DataObjects
class AreaData : public PrimitiveData {
 public:
  //! Constructs a new, AreaData object
  AreaData(Id id, LineStrings3d outerBound, std::vector<LineStrings3d> innerBounds = std::vector<LineStrings3d>(),
           AttributeMap attributes = AttributeMap(), RegulatoryElementPtrs regulatoryElements = RegulatoryElementPtrs())
      : PrimitiveData(id, std::move(attributes)),
        outerBound_{std::move(outerBound)},
        innerBounds_{std::move(innerBounds)},
        regulatoryElements_{std::move(regulatoryElements)} {
    resetCache();
  }

  ConstLineStrings3d outerBound() const {
    return utils::transform(outerBound_, [](const auto& elem) { return ConstLineString3d(elem); });
  }
  ConstInnerBounds innerBounds() const {
    return utils::transform(innerBounds_, [](const auto& elem) {
      return utils::transform(elem, [](const auto& ls) { return ConstLineString3d(ls); });
    });
  }
  const LineStrings3d& outerBound() { return outerBound_; }
  const InnerBounds& innerBounds() { return innerBounds_; }

  const CompoundPolygon3d& outerBoundPolygon() const { return outerBoundPolygon_; }

  const CompoundPolygons3d& innerBoundPolygons() const { return innerBoundPolygons_; }

  RegulatoryElementConstPtrs regulatoryElements() const {
    return utils::transform(regulatoryElements_, [](const auto& elem) { return RegulatoryElementConstPtr(elem); });
  }
  RegulatoryElementPtrs& regulatoryElements() { return regulatoryElements_; }

  template <typename RegElemT>
  std::vector<std::shared_ptr<const RegElemT>> regulatoryElementsAs() const {
    return utils::transformSharedPtr<const RegElemT>(regulatoryElements_);
  }
  template <typename RegElemT>
  std::vector<std::shared_ptr<RegElemT>> regulatoryElementsAs() {
    return utils::transformSharedPtr<RegElemT>(regulatoryElements_);
  }

  //! sets a new outer bound.
  void setOuterBound(const LineStrings3d& bound) {
    outerBound_ = bound;
    resetCache();
  }

  void setInnerBounds(const std::vector<LineStrings3d>& bounds) {
    innerBounds_ = bounds;
    resetCache();
  }

  void resetCache() {
    outerBoundPolygon_ = CompoundPolygon3d(utils::addConst(*this).outerBound());
    innerBoundPolygons_ = utils::transform(utils::addConst(*this).innerBounds(),
                                           [](const auto& elem) { return CompoundPolygon3d(elem); });
  }

 private:
  LineStrings3d outerBound_;                  //!< linestrings that together form the outer bound
  std::vector<LineStrings3d> innerBounds_;    //!< vector of ls for inner bounds
  RegulatoryElementPtrs regulatoryElements_;  //!< regulatory elements that apply

  // caches
  CompoundPolygon3d outerBoundPolygon_;    //!< represents the outer bounds of the area
  CompoundPolygons3d innerBoundPolygons_;  //!< represents the inner bounds of the area
};

//! @brief A const (i.e. immutable) Area.
//! @ingroup AreaPrimitives
//! @ingroup ConstPrimitives
class ConstArea : public ConstPrimitive<AreaData> {
 public:
  using ConstType = ConstArea;
  using MutableType = Area;
  using TwoDType = ConstArea;
  using ThreeDType = ConstArea;
  using Category = traits::AreaTag;

  /**
   * @brief Constructs an empty or invalid area.
   * @param id id of this area or InvalId
   *
   * This construction is only useful for copying.
   */
  explicit ConstArea(Id id = InvalId) : ConstArea(std::make_shared<AreaData>(id, LineStrings3d())) {}

  /**
   * @brief Constructs a new area
   * @param id id of the area
   * @param outerBound outer Boundary of area. Must be in clockwise order.
   * @param innerBounds inner Boundaries of area. Must be in counter-clockwise
   * order.
   * @param attributes attributes of this area
   * @param regulatoryElements regulatoryElements that apply to this area
   *
   * This is the constructor that you most probably want to use.
   */
  ConstArea(Id id, const LineStrings3d& outerBound, const InnerBounds& innerBounds = InnerBounds(),
            const AttributeMap& attributes = AttributeMap(),
            const RegulatoryElementPtrs& regulatoryElements = RegulatoryElementPtrs())
      : ConstPrimitive{std::make_shared<AreaData>(id, outerBound, innerBounds, attributes, regulatoryElements)} {}

  //! Constructor to construct from the data of a different Area
  explicit ConstArea(const std::shared_ptr<const AreaData>& data) : ConstPrimitive<AreaData>(data) {}

  //! Get linestrings that form the outer bound
  ConstLineStrings3d outerBound() const { return constData()->outerBound(); }

  //! get the linestrings that form the inner bounds
  ConstInnerBounds innerBounds() const { return constData()->innerBounds(); }

  /**
   * @brief get the outer bound as polygon
   * @return the polygon
   */
  CompoundPolygon3d outerBoundPolygon() const { return constData()->outerBoundPolygon(); }

  //! get the inner bounds as polygon
  CompoundPolygons3d innerBoundPolygons() const { return constData()->innerBoundPolygons(); }

  /**
   * @brief generates a basic polygon in 2d with holes for this area
   *
   * The generated polygon only has the points, no ids or attributes. It is
   * thought for geometry calculations.
   */
  BasicPolygonWithHoles3d basicPolygonWithHoles3d() const {
    return {outerBoundPolygon().basicPolygon(),
            utils::transform(innerBoundPolygons(), [](const auto& poly) { return poly.basicPolygon(); })};
  }

  /**
   * @brief generates a basic polygon in 2d with holes for this area
   *
   * The generated polygon only has the points, no ids or attributes. It is
   * thought for geometry calculations.
   */
  BasicPolygonWithHoles2d basicPolygonWithHoles2d() const {
    return {traits::to2D(outerBoundPolygon()).basicPolygon(),
            utils::transform(innerBoundPolygons(), [](const auto& poly) { return traits::to2D(poly).basicPolygon(); })};
  }

  //! get a list of regulatory elements that affect this area
  RegulatoryElementConstPtrs regulatoryElements() const { return constData()->regulatoryElements(); }

  /**
   * @brief get the regulatoryElements that could be converted to a type
   * @return the matching regulatory elements, if any
   * @tparam RegElemT the type to covert to *without cv qualifiers*
   *
   * E.g. regulatoryElementsAs<TrafficLight>() will return all regulatory
   * elements that are traffic lights.
   */
  template <typename RegElemT>
  std::vector<std::shared_ptr<const RegElemT>> regulatoryElementsAs() const {
    return constData()->regulatoryElementsAs<RegElemT>();
  }
};

//! @brief Famous Area class that represents a basic area as element of the map.
//! @ingroup AreaPrimitives
//! @ingroup Primitives
class Area : public Primitive<ConstArea> {
 public:
  using Primitive::Primitive;
  using TwoDType = Area;
  using ThreeDType = Area;
  friend class ConstWeakArea;
  Area() = default;
  Area(const AreaDataConstPtr&) = delete;
  explicit Area(const AreaDataPtr& data) : Primitive{data} {}

  using Primitive::innerBounds;
  using Primitive::outerBound;
  using Primitive::regulatoryElements;
  using Primitive::regulatoryElementsAs;

  //! Get linestrings that form the outer bound
  const LineStrings3d& outerBound() { return data()->outerBound(); }

  //! Get the linestrings that form the inner bound
  const InnerBounds& innerBounds() { return data()->innerBounds(); }

  //! Sets a new outer bound and resets the cache
  void setOuterBound(const LineStrings3d& bound) { data()->setOuterBound(bound); }

  //! sets a new inner bound and resets the cache
  void setInnerBounds(const std::vector<LineStrings3d>& bounds) { data()->setInnerBounds(bounds); }

  //! return regulatoryElements that affect this area.
  const RegulatoryElementPtrs& regulatoryElements() { return data()->regulatoryElements(); }

  /**
   * @brief adds a new regulatory element
   * @throws nullptr error if regElem is a nullptr
   */
  void addRegulatoryElement(RegulatoryElementPtr regElem) {
    if (regElem == nullptr) {
      throw NullptrError("regulatory element is a nullptr.");
    }
    data()->regulatoryElements().push_back(std::move(regElem));
  }

  //! removes a regulatory element, returns true on success
  bool removeRegulatoryElement(const RegulatoryElementPtr& regElem) {
    auto& regElems = data()->regulatoryElements();
    auto remove = std::find(regElems.begin(), regElems.end(), regElem);
    if (remove != regElems.end()) {
      regElems.erase(remove);
      return true;
    }
    return false;
  }

  //! get the regulatoryElements that could be converted to RegElemT
  template <typename RegElemT>
  std::vector<std::shared_ptr<RegElemT>> regulatoryElementsAs() {
    return data()->regulatoryElementsAs<RegElemT>();
  }
};

/**
 * @brief used internally by RegulatoryElements to avoid cyclic dependencies.
 *
 * It works conceptually similar to a std::weak_ptr. This is the immutable
 * version of a WeakArea and returns a ConstArea when lock() is called.
 */
class ConstWeakArea {
 public:
  using ConstType = ConstWeakArea;
  using MutableType = WeakArea;
  using TwoDType = ConstWeakArea;
  using ThreeDType = ConstWeakArea;
  ConstWeakArea() = default;
  ConstWeakArea(ConstArea area)  // NOLINT
      : areaData_(area.constData()) {}

  /**
   * @brief Obtains the original ConstArea.
   * @throws NullptrError if the managed lanelet expired.
   */
  ConstArea lock() const { return ConstArea(areaData_.lock()); }

  //! tests whether the WeakArea is still valid
  bool expired() const noexcept { return areaData_.expired(); }

 protected:
  std::weak_ptr<const AreaData> areaData_;  // NOLINT
};

/**
 * @brief used internally by RegulatoryElements to avoid cyclic dependencies.
 *
 * It works conceptually similar to a std::weak_ptr. This is the mutable
 * version of a WeakArea.
 */
class WeakArea final : public ConstWeakArea {
 public:
  WeakArea() = default;
  WeakArea(Area llet) : ConstWeakArea(std::move(llet)) {}  // NOLINT

  /**
   * @brief Obtains the original ConstArea
   * @throws NullptrError if the managed lanelet expired.
   */
  Area lock() const { return Area(std::const_pointer_cast<AreaData>(areaData_.lock())); }
};

namespace utils {
/**
 * @brief returns true if element of a regulatory element has a matching Id
 * @param ll the element holding other primitives
 * @param id id to look for
 * @return true if the primitive has such an element
 *
 * This function does not look for the id of the element, only its members
 * Works for linestrings and polylines. Does NOT check members in regulatory
 * elements, only ids of regulatory elements itself.
 * A similar implementation exists for linestrings and regulatory elements.
 */
inline bool has(const ConstArea& ll, Id id) { return has(ll.outerBoundPolygon(), id); }
}  // namespace utils

inline bool operator==(const ConstArea& lhs, const ConstArea& rhs) { return lhs.constData() == rhs.constData(); }
inline bool operator!=(const ConstArea& lhs, const ConstArea& rhs) { return !(lhs == rhs); }
inline bool operator==(const ConstWeakArea& lhs, const ConstWeakArea& rhs) {
  return !lhs.expired() && !rhs.expired() && lhs.lock() == rhs.lock();
}
inline bool operator!=(const ConstWeakArea& lhs, const ConstWeakArea& rhs) { return !(lhs == rhs); }
inline std::ostream& operator<<(std::ostream& stream, const ConstArea& obj) {
  stream << "[id: ";
  stream << obj.id();
  auto obId = obj.outerBoundPolygon().ids();
  if (!obId.empty()) {
    stream << " outer: [";
    std::copy(obId.begin(), obId.end(), std::ostream_iterator<Id>(stream, ","));
    stream << "]";
  }
  auto ibs = obj.innerBoundPolygons();
  if (!ibs.empty()) {
    stream << " inner: ";
    for (const auto& ib : ibs) {
      stream << "[";
      auto ibId = ib.ids();
      std::copy(ibId.begin(), ibId.end(), std::ostream_iterator<Id>(stream, ","));
      stream << "]";
    }
  }
  stream << "]";
  return stream;
}

namespace traits {
template <typename T>
constexpr bool isAreaT() {
  return isCategory<T, traits::AreaTag>();
}
}  // namespace traits

template <typename T, typename RetT>
using IfAr = std::enable_if_t<traits::isAreaT<T>(), RetT>;

}  // namespace lanelet

// Hash function for usage in containers
namespace std {
template <>
struct hash<lanelet::Area> : public lanelet::HashBase<lanelet::Area> {};
template <>
struct hash<lanelet::ConstArea> : public lanelet::HashBase<lanelet::ConstArea> {};
}  // namespace std
