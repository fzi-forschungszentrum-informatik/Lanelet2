#pragma once

#include <boost/variant.hpp>
#include <utility>

#include "lanelet2_core/Forward.h"
#include "lanelet2_core/primitives/Area.h"
#include "lanelet2_core/primitives/Lanelet.h"
#include "lanelet2_core/primitives/LineString.h"
#include "lanelet2_core/primitives/Point.h"
#include "lanelet2_core/primitives/Polygon.h"
#include "lanelet2_core/primitives/Primitive.h"
#include "lanelet2_core/utility/HybridMap.h"

namespace lanelet {
//! @defgroup RegulatoryElementPrimitives Regulatory Element
//! @ingroup Primitives
//!
//! ## General
//! Regulatory Elements define any kind of traffic rule that affects a Lanelet
//! or an Area. Traffic rules are usually composed by the thing that defined the
//! rule (usually a traffic sign), and other things that are relevant for the
//! rule, e.g. a StopLine. These things are called RuleParameter.
//!
//! ## Design
//! Every regulatoryElement has a number of RuleParameters that are implemented
//! as boost::variants. A RuleParameter can be any lanelet primitive. For
//! techical reasons (to avoid cyclic shared_ptr issues), Area and Lanelet is
//! stored using WeakPtrs. If Lanelets go out of scope, this weak ptr will
//! become invalid. Usually this will not be an issue because LaneletMap still
//! holds the Lanelets.
//!
//! RegulatoryElements are the only primitives that should be referenced by a
//! shared_ptr because they make use of dynamic inheritance.
//!
//! ### RoleNames
//! For faster access, roles can be queried using the RoleName enum (this works
//! similarly to Attribute). Instead of a std::map lookup, this is only a
//! std::vector operation.
//!
//! ## Adding new RegulatoryElements
//! All RegulatoryElements inherit from a common, abstract, immutable base
//! class: The RegulatoryElement. New regulatory elements can be added by
//! inhering from it, implementing whatever needs to be implemented and
//! registering it with the RegulatoryElementFactory.
//! Concrete RegulatoryElements are identified by their subtype attribute.
//!
//! ## Generic rules
//! For corner cases, a GenericRegulatoryElement is offered which is mutable and
//! can be used to model any yet unknown traffic rule.
//! Traffic rules should be interpreted by extending the lanele2_traffic_rules
//! package, preferably for all countries that have this rule.

//! Typical role names within lanelet (for faster lookup)
enum class RoleName {
  Refers,      //!< The primitive(s) that are the origin of this rule (ie signs)
  RefLine,     //!< The referring line where the rule becomes active
  RightOfWay,  //!< A lanelet that has right of way in a relation
  Yield,       //!< A lanelet that has to yield
  Cancels,     //!< primitive(s) that invalidate a rule (eg end of speed zone)
  CancelLine,  //!< The line from which a rule is invalidated
};

//! Lists which role strings are mapped to which enum value
struct RoleNameString {
  static constexpr const char Refers[] = "refers";
  static constexpr const char RefLine[] = "ref_line";
  static constexpr const char Yield[] = "yield";
  static constexpr const char RightOfWay[] = "right_of_way";
  static constexpr const char Cancels[] = "cancels";
  static constexpr const char CancelLine[] = "cancel_line";

  // roles not included in fast lookup
  static constexpr const char Left[] = "left";
  static constexpr const char Right[] = "right";
  static constexpr const char Centerline[] = "centerline";
  static constexpr const char Inner[] = "inner";
  static constexpr const char Outer[] = "outer";
  static constexpr const char Lanelet[] = "lanelet";
  static constexpr const char RegulatoryElement[] = "regulatory_element";

  using RoleNamesItem = std::pair<const char*, const RoleName>;
  static constexpr RoleNamesItem Map[]{{Refers, RoleName::Refers},   {RefLine, RoleName::RefLine},
                                       {Yield, RoleName::Yield},     {RightOfWay, RoleName::RightOfWay},
                                       {Cancels, RoleName::Cancels}, {CancelLine, RoleName::CancelLine}};
};

//! We call every element of a rule a "parameter"
//! A parameter can be any primitive in lanelet2
using RuleParameter = boost::variant<Point3d, LineString3d, Polygon3d, WeakLanelet, WeakArea>;

//! Const-version of the parameters
using ConstRuleParameter =
    boost::variant<ConstPoint3d, ConstLineString3d, ConstPolygon3d, ConstWeakLanelet, ConstWeakArea>;

namespace traits {
template <>
struct PrimitiveTraits<RuleParameter> {
  using DataType = void;
  using ConstType = ConstRuleParameter;
  using MutableType = RuleParameter;
  using TwoDType = void;
  using ThreeDType = void;
  using Category = void;
};
template <>
struct PrimitiveTraits<ConstRuleParameter> {
  using DataType = void;
  using ConstType = ConstRuleParameter;
  using MutableType = RuleParameter;
  using TwoDType = void;
  using ThreeDType = void;
  using Category = void;
};

template <>
ConstRuleParameter toConst<RuleParameter>(const RuleParameter& primitive);
}  // namespace traits

//! Multiple parameters can have the same role in a rule (eg traffic_lights)
using RuleParameters = std::vector<RuleParameter>;

//! Const version for a range of rule parameters
using ConstRuleParameters = std::vector<ConstRuleParameter>;

//! Rules are stored in a map internally
using RuleParameterMap = HybridMap<RuleParameters, decltype(RoleNameString::Map)&, RoleNameString::Map>;

//! Rules are stored in a map internally (const version)
using ConstRuleParameterMap = HybridMap<ConstRuleParameters, decltype(RoleNameString::Map)&, RoleNameString::Map>;

//! @brief Data container for all RegulatoryElement types
//! @ingroup DataObjects
class RegulatoryElementData : public PrimitiveData {
 public:
  explicit RegulatoryElementData(Id id, RuleParameterMap parameters = RuleParameterMap(),
                                 const AttributeMap& attributes = AttributeMap())
      : PrimitiveData(id, attributes), parameters{std::move(parameters)} {}
  RuleParameterMap parameters;
};

/**
 * @brief You can inherit from this visitor to perform an operation on each
 * parameter of a regulatory element
 * @see RegulatoryElement::applyVisitor
 */
class RuleParameterVisitor : public boost::static_visitor<void> {  // NOLINT
 public:
  virtual void operator()(const ConstPoint3d& /*unused*/) {}
  virtual void operator()(const ConstLineString3d& /*unused*/) {}
  virtual void operator()(const ConstPolygon3d& /*unused*/) {}
  virtual void operator()(const ConstWeakLanelet& /*unused*/) {}
  virtual void operator()(const ConstWeakArea& /*unused*/) {}
  virtual ~RuleParameterVisitor() = default;
  std::string role;  //!< applyVisitor will set the current role here
};

namespace internal {
class MutableParameterVisitor : public boost::static_visitor<void> {  // NOLINT
 public:
  virtual void operator()(const Point3d /*unused*/&) = 0;
  virtual void operator()(const LineString3d& /*unused*/) = 0;
  virtual void operator()(const Polygon3d& /*unused*/) = 0;
  virtual void operator()(const WeakLanelet& /*unused*/) = 0;
  virtual void operator()(const WeakArea& /*unused*/) = 0;
  virtual ~MutableParameterVisitor() = default;
  std::string role;  //!< applyVisitor will set the current role here
};
}  // namespace internal

//! @brief A general rule or limitation for a lanelet (abstract base class)
//! @ingroup RegulatoryElementPrimitives
//! @ingroup ConstPrimitives
class RegulatoryElement  // NOLINT
    : public ConstPrimitive<RegulatoryElementData>,
      private boost::noncopyable {
 public:
  using ConstType = RegulatoryElement;
  using MutableType = GenericRegulatoryElement;
  using TwoDType = RegulatoryElement;
  using ThreeDType = RegulatoryElement;
  using Category = traits::RegulatoryElementTag;
  using const_iterator = RuleParameterMap::const_iterator;  // NOLINT
  using iterator = RuleParameterMap::iterator;              // NOLINT
  static constexpr char RuleName[] = "basic_regulatory_element";

  virtual ~RegulatoryElement();

  /*
   * @brief set a new id for this primitive
   *
   * This is the best way to corrupt a map, because all primitives are
   * identified by their id. Make sure you know what you are doing!
   */
  void setId(Id id) noexcept { data()->id = id; }

  //! Returns all parameters as const object (coversion overhead for const)
  ConstRuleParameterMap getParameters() const;

  //! Returns a vector of all RuleParameters that could be converted to T.
  template <typename T>
  std::vector<T> getParameters(const std::string& role) const {
    static_assert(traits::isConst<T>(), "You must pass a const primitive type");
    auto it = constData()->parameters.find(role);
    if (it == constData()->parameters.end()) {
      return {};
    }
    return utils::getVariant<T>(it->second);
  }

  //! Returns a vector of all RuleParameters that could be converted to T (enum
  //! version).
  template <typename T>
  std::vector<T> getParameters(RoleName role) const {
    static_assert(traits::isConst<T>(), "You must pass a const primitive type");
    auto it = constData()->parameters.find(role);
    if (it == constData()->parameters.end()) {
      return {};
    }
    return utils::getVariant<T>(it->second);
  }

  //! returns all the roles this regulatory element has
  std::vector<std::string> roles() const {
    return utils::transform(parameters(), [](const auto& elem) { return elem.first; });
  }

  //! Finds a parameter by its id, independent of the role
  template <typename T>
  Optional<T> find(Id id) const;

  //! returns true if this object contains no parameters
  bool empty() const { return constData()->parameters.empty(); }

  //! get the number of roles in this regulatoryElement
  size_t size() const { return constData()->parameters.size(); }

  //! applies a visitor to every parameter in the regulatory element
  void applyVisitor(RuleParameterVisitor& visitor) const;

 protected:
  const_iterator begin() const { return constData()->parameters.begin(); }
  const_iterator end() const { return constData()->parameters.end(); }
  void applyVisitor(internal::MutableParameterVisitor& visitor) const;
  const RuleParameterMap& parameters() const { return constData()->parameters; }
  RuleParameterMap& parameters() { return std::const_pointer_cast<RegulatoryElementData>(constData())->parameters; }
  RegulatoryElementDataPtr data() { return std::const_pointer_cast<RegulatoryElementData>(constData()); }
  template <typename T>
  std::vector<T> getParameters(RoleName role) {
    auto it = data()->parameters.find(role);
    if (it == data()->parameters.end()) {
      return {};
    }
    return utils::getVariant<T>(it->second);
  }

  friend class RegulatoryElementFactory;
  friend class LaneletMap;  // Needs access to add all parameters of a regElem
  explicit RegulatoryElement(Id id = InvalId, const RuleParameterMap& members = RuleParameterMap(),
                             const AttributeMap& attributes = AttributeMap())
      : ConstPrimitive(std::make_shared<RegulatoryElementData>(id, members, attributes)) {}
  explicit RegulatoryElement(const RegulatoryElementDataPtr& data) : ConstPrimitive(data) {}
};

/**
 * @brief A GenericRegulatoryElement can hold any parameters.
 * @ingroup RegulatoryElementPrimitives
 * @ingroup Primitives
 */
class GenericRegulatoryElement final : public Primitive<RegulatoryElement> {
 public:
  static constexpr char RuleName[] = "regulatory_element";
  explicit GenericRegulatoryElement(const RegulatoryElementDataPtr& data)
      : Primitive<lanelet::RegulatoryElement>(data) {}

  //! Construct generically from id, parameters and attributes
  explicit GenericRegulatoryElement(Id id = InvalId, const RuleParameterMap& parameters = RuleParameterMap(),
                                    const AttributeMap& attributes = AttributeMap())
      : Primitive<lanelet::RegulatoryElement>(id, parameters, attributes) {}

  //! Add a (mutable) primitive to the regulatory element
  template <typename PrimitiveT>
  void addParameter(const std::string& role, const PrimitiveT& primitive);

  //! Add a (mutable) primitive (RoleName version)
  template <typename PrimitiveT>
  void addParameter(RoleName role, const PrimitiveT& primitive);

  using Primitive<RegulatoryElement>::getParameters;

  //! getter for all parameters of a regulatory element.
  RuleParameterMap& parameters() noexcept { return data()->parameters; }
};

namespace traits {
template <>
inline Id getId<RegulatoryElementPtr>(const RegulatoryElementPtr& prim) {
  return prim->id();
}

template <>
inline Id getId<RegulatoryElementConstPtr>(const RegulatoryElementConstPtr& prim) {
  return prim->id();
}

//! Extracts the id of a rule parameter
template <>
Id getId<RuleParameter>(const RuleParameter& prim);

template <>
Id getId<ConstRuleParameter>(const ConstRuleParameter& prim);
}  // namespace traits

namespace utils {
/**
 * @brief returns true if element of a regulatory element has a matching Id
 * @param regElem the element holding other primitives
 * @param id id to look for
 * @return true if the primitive has such an element
 *
 * This function does not look for the id of the element, only its members
 * Works for linestrings and polylines.
 * A similar implementation exists for linestrings and lanelets.
 */
bool has(const RegulatoryElement& regElem, Id id);
inline bool has(const RegulatoryElementConstPtr& ls, Id id) { return has(*ls, id); }
}  // namespace utils

/**
 * @brief Creates regulatory elements based on their type
 */
class RegulatoryElementFactory {
 public:
  using FactoryFcn = std::function<RegulatoryElementPtr(const RegulatoryElementDataPtr&)>;
  void registerStrategy(const std::string& strategy, const FactoryFcn& factoryFunction) {
    registry_[strategy] = factoryFunction;
  }

  /**
   * @brief create a regulatory element based on the name of the rule
   * @param ruleName RuleName of the regulatory element to create. If empty, a
   * GenericRegulatoryElement will be returned.
   * @param data the data with which to create the RegulatoryElement
   * @throws InvalidInputError if the object could not be created (e.g. if
   * requisites for the specific type are not met or ruleName does not exist)
   * @return a matching instance of the regulatory element.
   *
   * The factory will make sure that the subtype tag of the returned object
   * matches the ruleName as is required by liblanelet.
   */
  static RegulatoryElementPtr create(std::string ruleName, const RegulatoryElementDataPtr& data);

  static RegulatoryElementPtr create(const std::string& ruleName, Id id, const RuleParameterMap& map,
                                     const AttributeMap& attributes = AttributeMap()) {
    return create(ruleName, std::make_shared<RegulatoryElementData>(id, map, attributes));
  }

  //! returns regulatory element names that this factory can handle
  static std::vector<std::string> availableRules();

  static RegulatoryElementFactory& instance();

 private:
  RegulatoryElementFactory() = default;
  std::map<std::string, FactoryFcn> registry_;
};

/**
 * @brief template class for registering new RegulatoryElements.
 *
 * The parser will determine which rule to pick based on the "rule" tag of a
 * RegulatoryElement. If no rule matches, a GenericRegulatoryElement will be
 * created.
 *
 * To register a class, put
 * RegisterRegulatoryElement<MyClass> regMyClass;
 * somewhere in your cpp file.
 *
 * Your class is required to have a constructor that takes a
 * RegulatoryElementDataPtr as argument. The constructor of this class is
 * allowed to throw when the data passed to it is invalid.
 */
template <class T>
class RegisterRegulatoryElement {
 public:
  RegisterRegulatoryElement() {
    static_assert(!utils::strequal(T::RuleName, "basic_regulatory_element"),
                  "You did not provide a RuleName for your regulatoryElement!");
    RegulatoryElementFactory::instance().registerStrategy(
        T::RuleName, [](const RegulatoryElementDataPtr& data) -> RegulatoryElementPtr {
          return std::shared_ptr<T>(new T(data));  // have to use new because of friendship
        });
  }
};

template <typename PrimitiveT>
void GenericRegulatoryElement::addParameter(const std::string& role, const PrimitiveT& primitive) {
  parameters()[role].push_back(primitive);
}

template <typename PrimitiveT>
void GenericRegulatoryElement::addParameter(RoleName role, const PrimitiveT& primitive) {
  parameters()[role].push_back(primitive);
}

template <typename T>
Optional<T> RegulatoryElement::find(Id id) const {
  static_assert(traits::isConst<T>(), "You must pass a const primitive type!");
  using MutableT = traits::MutablePrimitiveType<T>;
  for (const auto& params : parameters()) {
    for (const auto& elem : params.second) {
      auto telem = boost::get<MutableT>(&elem);
      if (telem && telem->id() == id) {
        return *telem;
      }
    }
  }
  return {};
}

template <>
inline Optional<ConstRuleParameter> RegulatoryElement::find(Id id) const {
  for (const auto& params : parameters()) {
    for (const auto& elem : params.second) {
      if (utils::getId(elem) == id) {
        return traits::toConst(elem);
      }
    }
  }
  return {};
}

template <>
inline boost::optional<ConstLanelet> RegulatoryElement::find<ConstLanelet>(Id id) const {
  for (const auto& params : parameters()) {
    for (const auto& elem : params.second) {
      const auto* telem = boost::get<WeakLanelet>(&elem);
      if (telem != nullptr && !telem->expired() && telem->lock().id() == id) {
        return telem->lock();
      }
    }
  }
  return {};
}

template <>
inline boost::optional<ConstArea> RegulatoryElement::find<ConstArea>(Id id) const {
  for (const auto& params : parameters()) {
    for (const auto& elem : params.second) {
      const auto* telem = boost::get<WeakArea>(&elem);
      if (telem != nullptr && !telem->expired() && telem->lock().id() == id) {
        return telem->lock();
      }
    }
  }
  return {};
}

std::ostream& operator<<(std::ostream& stream, const RegulatoryElement& obj);

template <>
inline std::vector<ConstLanelet> RegulatoryElement::getParameters(const std::string& role) const {
  auto it = parameters().find(role);
  if (it == parameters().end()) {
    return {};
  }
  return utils::strong(utils::getVariant<ConstWeakLanelet>(it->second));
}
template <>
inline std::vector<ConstLanelet> RegulatoryElement::getParameters(RoleName role) const {
  auto it = parameters().find(role);
  if (it == parameters().end()) {
    return {};
  }
  return utils::strong(utils::getVariant<ConstWeakLanelet>(it->second));
}

template <>
inline std::vector<ConstArea> RegulatoryElement::getParameters(const std::string& role) const {
  auto it = parameters().find(role);
  if (it == parameters().end()) {
    return {};
  }
  return utils::strong(utils::getVariant<ConstWeakArea>(it->second));
}
template <>
inline std::vector<ConstArea> RegulatoryElement::getParameters(RoleName role) const {
  auto it = parameters().find(role);
  if (it == parameters().end()) {
    return {};
  }
  return utils::strong(utils::getVariant<ConstWeakArea>(it->second));
}

namespace traits {
template <typename T>
constexpr bool isRegulatoryElementT() {
  using DataT = typename PrimitiveTraits<T>::DataType;
  return std::is_same<DataT, RegulatoryElementData>::value;
}
template <>
struct Owned<RegulatoryElementPtr> {
  using Type = RuleParameter;
};
}  // namespace traits

template <typename T, typename RetT>
using IfRE = std::enable_if_t<traits::isRegulatoryElementT<T>(), RetT>;

}  // namespace lanelet

// Hash function for usage in containers
namespace std {
template <>
struct hash<lanelet::RegulatoryElement> : public lanelet::HashBase<lanelet::RegulatoryElement> {};
}  // namespace std
