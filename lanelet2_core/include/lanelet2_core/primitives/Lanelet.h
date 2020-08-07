// this is for emacs file handling -*- mode: c++; c-basic-offset: 2;
// indent-tabs-mode: nil -*-

#pragma once

#include <functional>
#include <memory>
#include <utility>

#include "lanelet2_core/Forward.h"
#include "lanelet2_core/primitives/LineString.h"
#include "lanelet2_core/primitives/Primitive.h"
#include "lanelet2_core/utility/Optional.h"

namespace lanelet {
enum class LaneletType { OneWay, Bidirectional };

/**
 * @brief Common data management class for all Lanelet-Typed objects.
 * @ingroup DataObjects
 */
class LaneletData : public PrimitiveData {
 public:
  /**
   * @brief Constructs a new, valid LaneletData object
   * @see ConstLanelet::ConstLanelet
   */
  LaneletData(Id id, LineString3d leftBound, LineString3d rightBound, const AttributeMap& attributes = AttributeMap(),
              RegulatoryElementPtrs regulatoryElements = RegulatoryElementPtrs())
      : PrimitiveData(id, attributes),
        leftBound_{std::move(leftBound)},
        rightBound_{std::move(rightBound)},
        regulatoryElements_{std::move(regulatoryElements)} {}

  const ConstLineString3d& leftBound() const { return leftBound_; }
  const LineString3d& leftBound() { return leftBound_; }

  const ConstLineString3d& rightBound() const { return rightBound_; }
  const LineString3d& rightBound() { return rightBound_; }

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

  //! Sets a new left bound. Resets all cached data of this object
  void setLeftBound(const LineString3d& bound);

  //! Sets a new right bound. Resets all cached data of this object
  void setRightBound(const LineString3d& bound);

  //! Sets a new right bound. Resets all cached data of this object
  void setCenterline(const LineString3d& centerline);

  //! Returns whether the centerline has been overridden by setCenterline
  bool hasCustomCenterline() const;

  //! call this to indicate that the objects data has been modified.
  void resetCache() const;

  //! Returns centerline by computing it, if necessary. Result is cached.
  ConstLineString3d centerline() const;

  //! Get the bounding polygon of this lanelet. Result is cached.
  CompoundPolygon3d polygon() const;

 private:
  LineString3d leftBound_;                    //!< represents the left bound
  LineString3d rightBound_;                   //!< represents the right bound
  RegulatoryElementPtrs regulatoryElements_;  //!< regulatory elements

  // Cached data
  mutable std::shared_ptr<ConstLineString3d> centerline_;
};

/**
 * @defgroup LaneletPrimitives Lanelets
 * @ingroup Primitives
 *
 * ## General
 * Lanelets represent an atomic section of a lane.
 *
 * A lanelet consists of two bounds, attributes and regulatory elements.
 * The orientation of the bounds is important and must be consistent as it
 * indicates the driving direction. The attributes can define simple constaints
 * for a lanelet and regulatory elements impose traffic regulations (right of
 * way, speed limits).
 *
 * ## Centerline
 * You can get the centerline of a Lanelet. This computation is a bit costly but
 * this is necessary to guarantee that the centerline never violates the bounds
 * of its polygon. To speed up subsequend calls, the result is cached. However,
 * if points are modified, the Lanelet does not get notified and the cached
 * centerline will be wrong. To solve that, manually reset the cache of the
 * affected lanelets when changing point coordinates.
 *
 * You can override the centerline by setting your own. If you do that, you take
 * resposability of the validity of the centerline. If the bounds of lanelet
 * with such a centerline is updated, the centerline will stay unchanged.
 *
 * ## Geometry
 * Lanelets can directly be used in some geometry calculations. see
 * lanelet::geometry. If this does not satisfy your needs, you can try the
 * Lanelet::polygon2d or Lanelet::polygon3d instead.
 *
 * ## Regulatory Elements
 * Regulatory Elements represent traffic rules that affect a lanelet. Instead of
 * trying to interpret the traffic rules by hand, use the lanelet2_traffic_rules
 * package. It is designed to yield senseful results for all commmon traffic
 * rules. If they do not fit your needs, you can extend them easily.
 *
 * ## Inverting
 * A lanelet implicitly stands for a certain driving direction. To get the
 * opposite direction, Lanelets can be inverted in O(0). An inverted Lanelet
 * shares its data with all normal lanelets, it just returns its bounds with
 * left and right flipped.
 */

//! @brief An immutable lanelet.
//! @ingroup LaneletPrimitives
//! @ingroup ConstPrimitives
class ConstLanelet : public ConstPrimitive<LaneletData> {
 public:
  using ConstType = ConstLanelet;
  using MutableType = Lanelet;
  using TwoDType = ConstLanelet;
  using ThreeDType = ConstLanelet;
  using Category = traits::LaneletTag;

  //! Constructs an empty or invalid lanelet.
  explicit ConstLanelet(Id id = InvalId)
      : ConstLanelet(std::make_shared<LaneletData>(id, LineString3d(), LineString3d()), false) {}

  //! Constructs a lanelet from id, attributes, regulatoryElements and bounds.
  ConstLanelet(Id id, const LineString3d& leftBound, const LineString3d& rightBound,
               const AttributeMap& attributes = AttributeMap(),
               const RegulatoryElementPtrs& regulatoryElements = RegulatoryElementPtrs())
      : ConstPrimitive{std::make_shared<LaneletData>(id, leftBound, rightBound, attributes, regulatoryElements)} {}

  //! Construct from the data of a different Lanelet
  explicit ConstLanelet(const std::shared_ptr<const LaneletData>& data, bool inverted = false)
      : ConstPrimitive<lanelet::LaneletData>(data), inverted_{inverted} {}

  //! returns if this is an inverted lanelet
  bool inverted() const { return inverted_; }

  //! returns the inverted lanelet
  ConstLanelet invert() const { return ConstLanelet{constData(), !inverted()}; }

  //! get the left bound.
  ConstLineString3d leftBound() const { return leftBound3d(); }
  //! get the left bound in 2d. To be used over leftBound where geometric calculations are required.
  ConstLineString2d leftBound2d() const { return utils::to2D(leftBound3d()); }
  //! get the left bound in 3d. To be used over leftBound where geometric calculations are required.
  ConstLineString3d leftBound3d() const {
    return inverted() ? constData()->rightBound().invert() : constData()->leftBound();
  }

  //! get the right bound.
  ConstLineString3d rightBound() const { return rightBound3d(); }
  //! get the right bound in 2d. To be used over rightBound where geometric calculations are required.
  ConstLineString2d rightBound2d() const { return utils::to2D(rightBound3d()); }
  //! get the right bound in 3d. To be used over rightBound where geometric calculations are required.
  ConstLineString3d rightBound3d() const {
    return inverted() ? constData()->leftBound().invert() : constData()->rightBound();
  }

  //! get a list of regulatory elements that affect this lanelet
  RegulatoryElementConstPtrs regulatoryElements() const { return constData()->regulatoryElements(); }

  //! get the regulatoryElements that could be converted to RegElemT
  template <typename RegElemT>
  std::vector<std::shared_ptr<const RegElemT>> regulatoryElementsAs() const {
    return constData()->regulatoryElementsAs<RegElemT>();
  }

  //! get the (cached) centerline of this lanelet.
  ConstLineString3d centerline() const { return centerline3d(); }

  //! get the (cached) centerline of this lanelet in 2d. To be used in context of geometric calculations.
  ConstLineString2d centerline2d() const { return utils::to2D(centerline3d()); }

  //! get the (cached) centerline of this lanelet in 3d. To be used in context of geometric calculations.
  ConstLineString3d centerline3d() const {
    return inverted() ? constData()->centerline().invert() : constData()->centerline();
  }

  //! Returns whether the centerline has been overridden by setCenterline
  bool hasCustomCenterline() const { return constData()->hasCustomCenterline(); }

  //! returns the surface covered by this lanelet as 3-dimensional polygon.
  CompoundPolygon3d polygon3d() const;

  //! returns the surface covered by this lanelet as 2-dimensional polygon.
  CompoundPolygon2d polygon2d() const;

  /**
   * @brief resets the internal cache of the centerline
   *
   * this can be necessary if an element of the linestring was modified
   * somewhere else.
   */
  void resetCache() const { constData()->resetCache(); }

 private:
  bool inverted_{false};  //!< indicates if this lanelet is inverted
};

//! @brief The famous (mutable) lanelet class
//! @ingroup LaneletPrimitives
//! @ingroup Primitives
class Lanelet : public Primitive<ConstLanelet> {
 public:
  using Primitive::Primitive;
  using TwoDType = Lanelet;
  using ThreeDType = Lanelet;
  friend class ConstWeakLanelet;
  Lanelet() = default;
  Lanelet(const std::shared_ptr<const LaneletData>&, bool inverted) = delete;
  Lanelet(const LaneletDataPtr& data, bool inverted) : Primitive{data, inverted} {}

  //! non-const version to invert a lanelet
  Lanelet invert() const { return Lanelet{data(), !inverted()}; }

  using Primitive::leftBound;
  using Primitive::leftBound2d;
  using Primitive::leftBound3d;
  //! Get the left bound
  LineString3d leftBound() { return leftBound3d(); }

  //! get the left bound in 2d. To be used over leftBound where geometric calculations are required.
  LineString2d leftBound2d() { return utils::to2D(leftBound3d()); }

  //! get the left bound in 3d. To be used over leftBound where geometric calculations are required.
  LineString3d leftBound3d() { return inverted() ? data()->rightBound().invert() : data()->leftBound(); }

  using Primitive::rightBound;
  using Primitive::rightBound2d;
  using Primitive::rightBound3d;
  //! Get the right bound
  LineString3d rightBound() { return rightBound3d(); }

  //! get the right bound in 2d. To be used over rightBound where geometric calculations are required.
  LineString2d rightBound2d() { return utils::to2D(rightBound3d()); }

  //! get the right bound in 3d. To be used over rightBound where geometric calculations are required.
  LineString3d rightBound3d() { return inverted() ? data()->leftBound().invert() : data()->rightBound(); }

  //! get the right bound.
  ConstLineString3d rightBound() const { return rightBound3d(); }

  //! Sets a new left bound and resets the cached centerline
  void setLeftBound(const LineString3d& bound) {
    inverted() ? data()->setRightBound(bound.invert()) : data()->setLeftBound(bound);
  }

  //! Sets a new right bound and resets the cached centerline
  void setRightBound(const LineString3d& bound) {
    inverted() ? data()->setLeftBound(bound.invert()) : data()->setRightBound(bound);
  }

  //! Forcefully set a new centerline. Must have a valid Id.
  //!
  //! This overrides the default behaviour where the centerline is automatically
  //! calculated by the library. The centerline will not be updated if the
  //! bounds of the lanelet are modified.
  //!
  //! There is no check whether the centerline is actually valid.
  void setCenterline(const LineString3d& centerline) {
    inverted() ? data()->setCenterline(centerline.invert()) : data()->setCenterline(centerline);
  }

  //! Return regulatoryElements that affect this lanelet.
  const RegulatoryElementPtrs& regulatoryElements() { return data()->regulatoryElements(); }

  //! Add a new regulatory element
  void addRegulatoryElement(RegulatoryElementPtr regElem) {
    if (regElem == nullptr) {
      throw NullptrError("regulatory element is a nullptr.");
    }
    data()->regulatoryElements().push_back(std::move(regElem));
  }

  //! Removes a regulatory element, returns true on success
  bool removeRegulatoryElement(const RegulatoryElementPtr& regElem) {
    auto& regElems = data()->regulatoryElements();
    auto remove = std::find(regElems.begin(), regElems.end(), regElem);
    if (remove != regElems.end()) {
      regElems.erase(remove);
      return true;
    }
    return false;
  }

  using Primitive::regulatoryElements;
  using Primitive::regulatoryElementsAs;

  //! Get the regulatoryElements that could be converted to RegElemT
  template <typename RegElemT>
  std::vector<std::shared_ptr<RegElemT>> regulatoryElementsAs() {
    return data()->regulatoryElementsAs<RegElemT>();
  }
};

//! Used internally by RegulatoryElements to avoid cyclic dependencies.
//! Conceptually similar to a std::weak_ptr.
class ConstWeakLanelet {
 public:
  using ConstType = ConstWeakLanelet;
  using MutableType = WeakLanelet;
  using TwoDType = ConstWeakLanelet;
  using ThreeDType = ConstWeakLanelet;
  ConstWeakLanelet() = default;
  ConstWeakLanelet(const ConstLanelet& llet)  // NOLINT
      : laneletData_{llet.constData()}, inverted_{llet.inverted()} {}

  /**
   * @brief Obtains the original ConstLanelet.
   * @throws NullptrError if the managed lanelet expired.
   */
  ConstLanelet lock() const { return ConstLanelet(laneletData_.lock(), inverted_); }

  //! tests whether the WeakLanelet is still valid
  bool expired() const noexcept { return laneletData_.expired(); }

 protected:
  std::weak_ptr<const LaneletData> laneletData_;  // NOLINT
  bool inverted_{false};                          // NOLINT
};

//! Used internally by RegulatoryElements to avoid cyclic dependencies.
//! Conceptually similar to a std::weak_ptr.
class WeakLanelet final : public ConstWeakLanelet {
 public:
  WeakLanelet() = default;
  WeakLanelet(const Lanelet& llet) : ConstWeakLanelet(llet) {}  // NOLINT

  /**
   * @brief Obtains the original Lanelet.
   * @throws NullptrError if the managed lanelet expired.
   */
  Lanelet lock() const { return Lanelet(std::const_pointer_cast<LaneletData>(laneletData_.lock()), inverted_); }
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
bool has(const ConstLanelet& ll, Id id);
}  // namespace utils

inline bool operator==(const ConstLanelet& lhs, const ConstLanelet& rhs) {
  return lhs.constData() == rhs.constData() && lhs.inverted() == rhs.inverted();
}
inline bool operator!=(const ConstLanelet& lhs, const ConstLanelet& rhs) { return !(lhs == rhs); }
inline bool operator==(const ConstWeakLanelet& lhs, const ConstWeakLanelet& rhs) {
  return !lhs.expired() && !rhs.expired() && lhs.lock() == rhs.lock();
}
inline bool operator!=(const ConstWeakLanelet& lhs, const ConstWeakLanelet& rhs) { return !(lhs == rhs); }
std::ostream& operator<<(std::ostream& stream, const ConstLanelet& obj);

namespace traits {
template <typename T>
constexpr bool isLaneletT() {
  return isCategory<T, traits::LaneletTag>();
}
}  // namespace traits

template <typename T, typename RetT>
using IfLL = std::enable_if_t<traits::isLaneletT<T>(), RetT>;

}  // namespace lanelet

// Hash function for usage in containers
namespace std {
template <>
struct hash<lanelet::Lanelet> : public lanelet::HashBase<lanelet::Lanelet> {};
template <>
struct hash<lanelet::ConstLanelet> : public lanelet::HashBase<lanelet::ConstLanelet> {};

template <typename T, typename U>
struct hash<std::pair<U, T>> {
  size_t operator()(const std::pair<U, T> x) const noexcept {
    return std::hash<U>()(x.first) ^ std::hash<T>()(x.second);
  }
};

}  // namespace std
