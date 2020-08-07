// this is for emacs file handling -*- mode: c++; c-basic-offset: 2;
// indent-tabs-mode: nil -*-

#pragma once
#include <limits>
#include <utility>

#include "lanelet2_core/Attribute.h"
#include "lanelet2_core/Exceptions.h"
#include "lanelet2_core/primitives/Traits.h"

namespace lanelet {
//! @defgroup DataObjects lanelet data management
//! All the data of lanelets are managed by these objects. Usually you should
//! not get in contact with them, as the lanelet Primitives and ConstPrimitives
//! hide them.
//!
//! All data classes represent a unique part of the map and are therefore
//! noncopyable.

/**
 * @brief Common data class for all lanelet primitives
 * @ingroup DataObjects
 *
 * This class provides the data that all lanelet primitives have in common: id
 * and attributes. It is inherited by all other data classes.
 *
 * This is only an implementation class and only meant to be derived from, never
 * instanciated.
 */
class PrimitiveData {
 public:
  PrimitiveData() noexcept = default;
  PrimitiveData(PrimitiveData&& rhs) noexcept = default;
  PrimitiveData& operator=(PrimitiveData&& rhs) noexcept = default;
  PrimitiveData(const PrimitiveData& rhs) = default;
  PrimitiveData& operator=(const PrimitiveData& rhs) = default;

  /**
   * @brief Constructs a PrimitiveData object
   */
  explicit PrimitiveData(Id id, AttributeMap attributes = AttributeMap()) : id{id}, attributes{std::move(attributes)} {}

  Id id{InvalId};           //!< Id of this primitive (unique across one map)
  AttributeMap attributes;  //!< attributes of this primitive
 protected:
  ~PrimitiveData() = default;
};  // class PrimitiveData

//! @defgroup ConstPrimitives Immutable lanelet primitives
//! All lanelet primitives are defined in a mutable and an immutable version
//! (see also @ref Primitives). All primitives can be converted to their const
//! version but not vice versa. Each primitive represents one of lanelet2's
//! basic types.
//!
//! All ConstPrimitives have in common that they usually only have one data
//! member: a shared pointer to the actual data of the primitve. Copying them is
//! therefore extremely cheap. This also ensures that if a primitive is
//! modified, all existing copies will also be affected.
//!
//! They have in common that they have a unique ID and attributes in the form of
//! key-value pairs. They all derive from the ConstPrimitive template class.

/**
 * @brief Basic Primitive class for all primitives of lanelet2
 * @tparam Data Type of the data object that this ConstPrimitive holds
 * @ingroup ConstPrimitive
 */
template <typename Data>
class ConstPrimitive {
 public:
  using DataType = Data;
  static constexpr bool IsConst = true;
  /**
   * @brief Construct from a pointer to the data
   * @param data internal data for this primitive. Must not be null.
   */
  explicit ConstPrimitive(const std::shared_ptr<const Data>& data) : constData_{data} {
    if (!data) {
      throw lanelet::NullptrError("Nullptr passed to constructor!");
    }
  }

  // Comparison
  bool operator==(const ConstPrimitive& rhs) const { return this->constData_ == rhs.constData_; }
  bool operator!=(const ConstPrimitive& rhs) const { return !(*this == rhs); }

  //! get the attributes of this primitive
  const AttributeMap& attributes() const { return constData_->attributes; }

  //! get the unique id of this primitive
  /**
   * Keep in mind that the Id can also be InvalId, if the element is a temporary
   * structure that is not a part of the map.
   */
  Id id() const noexcept { return constData_->id; }

  //! check whether this primitive has a specific attribute
  bool hasAttribute(const std::string& name) const noexcept { return attributes().find(name) != attributes().end(); }

  //! check for an attribute (enum version)
  /**
   * The enum version provides a much more efficient access to the data. It is
   * basically the difference between access via a vector and a std::map lookup.
   * However, this kind of access only works for the most common attribte names.
   */
  bool hasAttribute(AttributeName name) const noexcept { return attributes().find(name) != attributes().end(); }

  /**
   * @brief retrieve an attribute
   * @throws NoSuchAttributeError if it does not exist
   */
  const Attribute& attribute(const std::string& name) const {
    try {
      return attributes().at(name);
    } catch (std::out_of_range& err) {
      throw NoSuchAttributeError(err.what());
    }
  }

  /**
   * @brief retrieve an attribute (enum version)
   * @throws NoSuchAttributeError if it does not exist
   */
  const Attribute& attribute(AttributeName name) const {
    try {
      return attributes().at(name);
    } catch (std::out_of_range& err) {
      throw NoSuchAttributeError(err.what());
    }
  }

  /**
   * @brief retrieve an attribute (string version)
   * @param name name of the attribute
   * @param defaultVal value returned if not existing
   * @tparam T return type (must be one of Attribute::as<T>())
   *
   * T can also be an Optional so that you get an empty optional if the value
   * did not exist or could not be converted.
   */
  template <typename T>
  T attributeOr(const std::string& name, T defaultVal) const noexcept {
    auto elem = attributes().find(name);
    if (elem == attributes().end()) {
      return defaultVal;
    }
    auto val = elem->second.template as<T>();
    if (!!val) {
      return *val;
    }
    return defaultVal;
  }

  /**
   * @brief retrieve an attribute (enum version)
   * @param name name of the attribute
   * @param defaultVal value returned if not existing
   * @tparam T return type (must be one of Attribute::as<T>())
   *
   * T can also be an Optional so that you get an empty optional if the value
   * did not exist or could not be converted.
   */
  template <typename T>
  T attributeOr(AttributeName name, T defaultVal) const {
    auto elem = attributes().find(name);
    if (elem == attributes().end()) {
      return defaultVal;
    }
    auto val = elem->second.template as<T>();
    if (!!val) {
      return *val;
    }
    return defaultVal;
  }

  //! get the internal data of this primitive
  const std::shared_ptr<const Data>& constData() const { return constData_; }

 protected:
  // This class is only an implementation class and should never be instanciated
  ConstPrimitive(ConstPrimitive&& rhs) noexcept = default;
  ConstPrimitive& operator=(ConstPrimitive&& rhs) noexcept = default;
  ConstPrimitive(const ConstPrimitive& rhs) = default;
  ConstPrimitive& operator=(const ConstPrimitive& rhs) = default;
  ~ConstPrimitive() noexcept = default;

 private:
  std::shared_ptr<const Data> constData_;  //!< the data this primitive holds

};  // class ConstPrimitive

//! @defgroup Primitives Lanelet Primitives
//!
//! All primitives are first-class citizens of lanelet2. Unlike \ref
//! ConstPrimitives they provide functionality to not only view the data but
//! also modify it.
//!
//! ## Design concepts
//! All primitives are split in a const and a non-const version that both have a
//! std::shared_ptr to the actual data. This means that a primitive itself is
//! very lightweight. It also means that all copys of a primitive share the same
//! data and are therefore
//!
//! ## Geometry
//! All primitives are registered with boost::geometry (see also the
//! lanelet::geometry). This means that they can be passed directly to
//! boost::geometry functions.
//!
//! With some exceptions: Some of boost::geometry's functions require objects to
//! contain mutable data. This is not possible for lanelet primitives. If you
//! see errors complaining about a missing "set" function, you hit one of those
//! functions. For these functions, you have to pass *hybrid* primitives (see
//! ConstHybridLineString3d).
//!
//! All primitives exist in a 2d and a 3d version and can be converted cheaply
//! and without data loss in both directions (see also lanelet::traits::to2D and
//! lanelet::traits::to3d). This works thanks to the fact that they still
//! reference the same PrimitiveData object.
//!
//! ## Modification
//! Be aware that some parts of lanelet2 cache data about others, e.g. Lanelet
//! and LaneletMap. Modification is not propagated upwards in the lanelet
//! hierarchy. E.g. If you modify the coordinates of a Point, the centerline of
//! a lanelet relying on this point will not be updated. You have to reset these
//! caches by yourself.
//!
//! If you intend to never modify parts of a LaneletMap, better work with
//! \ref ConstPrimitives. All primitives can be converted to a const version.
//! See also lanelet::traits::toConst.
//!
//! ## Thread safety
//! No thread safety is guaranteed when primitives are modified. You have to
//! ensure thread safety yourself. Concurrent reads are always thread safe.

/**
 * @brief Base class for all mutable Primitives of lanelet2.
 * @tparam DerivedConstPrimitive the ConstPrimitive class that this class
 * derives from
 * @ingroup Primitive
 */
template <typename DerivedConstPrimitive>
class Primitive : public DerivedConstPrimitive {
 public:
  Primitive() = default;  // not inherited
  using DataType = typename DerivedConstPrimitive::DataType;
  using DerivedConstPrimitive::DerivedConstPrimitive;
  static constexpr bool IsConst = false;

  // needs a non-const shared_ptr to construct
  Primitive(const std::shared_ptr<const DataType>&) = delete;

  //! Construct a new primitive from shared_ptr to its data
  explicit Primitive(const std::shared_ptr<DataType>& data) : DerivedConstPrimitive(data) {
    if (!data) {
      throw lanelet::NullptrError("Nullptr passed to constructor!");
    }
  }

  //! Construct from another primitive. Only works if both share the same
  template <typename OtherT>
  explicit Primitive(const Primitive<OtherT>& rhs) : DerivedConstPrimitive(rhs) {}

  /**
   * @brief set a new id for this primitive
   *
   * This is the best way to corrupt a map, because all primitives are
   * identified by their id. Make sure you know what you are doing!
   */
  void setId(Id id) noexcept { data()->id = id; }

  //! @brief set or overwrite an attribute
  void setAttribute(const std::string& name, const Attribute& attribute) { attributes()[name] = attribute; }

  //! set or overwrite an attribute (enum version)
  void setAttribute(AttributeName name, const Attribute& attribute) { attributes()[name] = attribute; }

  // need this for copy construction to work
  template <typename>
  friend class Primitive;

  using DerivedConstPrimitive::attributes;
  //! get the attributes in a mutable way
  AttributeMap& attributes() noexcept { return data()->attributes; }

 protected:
  std::shared_ptr<DataType> data() const {
    // const_pointer_cast is ok. Non-const primitives cannot be created without
    // a non-const pointer.
    return std::const_pointer_cast<DataType>(this->constData());
  }

  // This class is only an implementation class and should never be instanciated
  Primitive(Primitive&& rhs) noexcept;
  Primitive& operator=(Primitive&& rhs) noexcept;
  Primitive(const Primitive& rhs) noexcept;
  Primitive& operator=(const Primitive& rhs) noexcept;
  ~Primitive() noexcept = default;  // NOLINT
};

namespace traits {

namespace internal {
template <typename PrimitiveT>
class IsPrimitiveHelper : public std::false_type {};
template <typename PrimitiveT>
class IsPrimitiveHelper<ConstPrimitive<PrimitiveT>> : public std::true_type {};

template <typename DataT>
bool isLaneletPrimitiveHelper(ConstPrimitive<DataT>* /*unused*/, int /*unused*/) {
  return true;
}

template <typename NotPrimitive>
bool isLaneletPrimitiveHelper(NotPrimitive* /*unused*/, long /*unused*/) {  // NOLINT
  return false;
}
}  // namespace internal

template <typename PrimitiveT>
constexpr bool isLaneletPrimitive() {
  PrimitiveT* v = nullptr;
  return IsLaneletPrimitiveHelper(v, 0);
}
}  // namespace traits

template <typename PrimitiveT>
struct HashBase {
  size_t operator()(const PrimitiveT& x) const noexcept { return std::hash<Id>()(x.id()); }
};

template <typename DerivedConstPrimitive>
Primitive<DerivedConstPrimitive>::Primitive(Primitive&& rhs) noexcept = default;

template <typename DerivedConstPrimitive>
Primitive<DerivedConstPrimitive>& Primitive<DerivedConstPrimitive>::operator=(Primitive&& rhs) noexcept = default;

template <typename DerivedConstPrimitive>
Primitive<DerivedConstPrimitive>::Primitive(const Primitive& rhs) noexcept = default;

template <typename DerivedConstPrimitive>
Primitive<DerivedConstPrimitive>& Primitive<DerivedConstPrimitive>::operator=(const Primitive& rhs) noexcept = default;

}  // namespace lanelet
