#include "lanelet2_core/primitives/RegulatoryElement.h"

#include "lanelet2_core/primitives/Area.h"
#include "lanelet2_core/primitives/BasicRegulatoryElements.h"
#include "lanelet2_core/primitives/Lanelet.h"
#include "lanelet2_core/primitives/LineString.h"
#include "lanelet2_core/primitives/Point.h"

namespace lanelet {
namespace {
class HasIdVisitor : public RuleParameterVisitor {
 public:
  explicit HasIdVisitor(Id id) : id_{id} {}
  void operator()(const ConstPoint3d& p) override { found_ |= p.id() == id_; }
  void operator()(const ConstLineString3d& l) override { found_ |= l.id() == id_ || utils::has(l, id_); }
  void operator()(const ConstPolygon3d& p) override { found_ |= p.id() == id_ || utils::has(p, id_); }
  void operator()(const ConstWeakLanelet& ll) override {
    if (ll.expired()) {
      return;
    }
    ConstLanelet llet(ll.lock());
    found_ |= llet.id() == id_ || utils::has(llet, id_);
  }
  void operator()(const ConstWeakArea& ar) override {
    if (ar.expired()) {
      return;
    }
    ConstArea area(ar.lock());
    found_ |= area.id() == id_ || utils::has(area, id_);
  }
  bool operator!() const { return !found_; }

 private:
  Id id_;
  bool found_{false};
};

class GetIdVisitor : public RuleParameterVisitor {
 public:
  static Id id(const ConstRuleParameter& param) {
    GetIdVisitor visitor;
    boost::apply_visitor(visitor, param);
    return visitor.id_;
  }
  template <typename PrimT>
  void appendID(const PrimT& p) {
    id_ = p.id();
  }

  void operator()(const ConstPoint3d& p) override { appendID(p); }
  void operator()(const ConstLineString3d& l) override { appendID(l); }
  void operator()(const ConstPolygon3d& p) override { appendID(p); }
  void operator()(const ConstWeakLanelet& ll) override {
    if (!ll.expired()) {
      appendID(ll.lock());
    }
  }
  void operator()(const ConstWeakArea& ar) override {
    if (!ar.expired()) {
      appendID(ar.lock());
    }
  }

 private:
  Id id_{};
};

class ToConstVisitor : public RuleParameterVisitor {
 public:
  void operator()(const ConstPoint3d& p) override { param_ = p; }
  void operator()(const ConstLineString3d& l) override { param_ = l; }
  void operator()(const ConstPolygon3d& p) override { param_ = p; }
  void operator()(const ConstWeakLanelet& ll) override { param_ = ll; }
  void operator()(const ConstWeakArea& ar) override { param_ = ar; }

  ConstRuleParameter apply(const RuleParameter& rule) {
    boost::apply_visitor(*this, rule);
    return param_;
  }

 private:
  ConstRuleParameter param_;
};

}  // namespace

static RegisterRegulatoryElement<GenericRegulatoryElement> genRegelem;
#if __cplusplus < 201703L
constexpr char GenericRegulatoryElement::RuleName[];

constexpr const char RoleNameString::Refers[];
constexpr const char RoleNameString::RefLine[];
constexpr const char RoleNameString::Yield[];
constexpr const char RoleNameString::RightOfWay[];
constexpr const char RoleNameString::Cancels[];
constexpr const char RoleNameString::CancelLine[];

// roles not included in fast lookup
constexpr const char RoleNameString::Left[];
constexpr const char RoleNameString::Right[];
constexpr const char RoleNameString::Centerline[];
constexpr const char RoleNameString::Inner[];
constexpr const char RoleNameString::Outer[];
constexpr const char RoleNameString::Lanelet[];
constexpr const char RoleNameString::RegulatoryElement[];

constexpr char RegulatoryElement::RuleName[];

constexpr RoleNameString::RoleNamesItem RoleNameString::Map[];
#endif

RegulatoryElement::~RegulatoryElement() = default;

RegulatoryElementPtr RegulatoryElementFactory::create(std::string ruleName, const RegulatoryElementDataPtr& data) {
  if (ruleName.empty()) {
    ruleName = GenericRegulatoryElement::RuleName;
  }
  data->attributes[AttributeNamesString::Subtype] = ruleName;
  auto& inst = RegulatoryElementFactory::instance();
  auto it = inst.registry_.find(ruleName);
  if (it != inst.registry_.end()) {
    return it->second(data);
  }
  throw InvalidInputError("No regulatory element found that implements rule " + ruleName);
}

std::vector<std::string> RegulatoryElementFactory::availableRules() {
  auto& registry = RegulatoryElementFactory::instance().registry_;
  return utils::transform(registry, [](const auto& elem) { return elem.first; });
}

RegulatoryElementFactory& RegulatoryElementFactory::instance() {
  static RegulatoryElementFactory factory;
  return factory;
}

ConstRuleParameterMap RegulatoryElement::getParameters() const {
  ConstRuleParameterMap params;
  for (const auto& param : parameters()) {
    params.insert(std::make_pair(
        param.first, utils::transform(param.second, [](const auto& elem) { return traits::toConst(elem); })));
  }
  return params;
}

void RegulatoryElement::applyVisitor(RuleParameterVisitor& visitor) const {
  for (const auto& elems : parameters()) {
    visitor.role = elems.first;
    for (const auto& elem : elems.second) {
      boost::apply_visitor(visitor, elem);
    }
  }
}

void RegulatoryElement::applyVisitor(lanelet::internal::MutableParameterVisitor& visitor) const {
  for (const auto& elems : parameters()) {
    visitor.role = elems.first;
    for (const auto& elem : elems.second) {
      boost::apply_visitor(visitor, elem);
    }
  }
}

bool utils::has(const RegulatoryElement& regElem, Id id) {
  HasIdVisitor hasId(id);
  regElem.applyVisitor(hasId);
  return !!hasId;
}

template <>
ConstRuleParameter traits::toConst<RuleParameter>(const RuleParameter& primitive) {
  return ToConstVisitor().apply(primitive);
}

std::ostream& operator<<(std::ostream& stream, const RegulatoryElement& obj) {
  stream << "[id: " << obj.id();
  if (!obj.empty()) {
    stream << ", parameters: ";
    for (auto& param : obj.getParameters()) {
      stream << '{' << param.first << ':' << ' ';
      for (auto& rule : param.second) {
        stream << GetIdVisitor::id(rule) << ' ';
      }
      stream << '}';
    }
  }
  return stream << ']';
}

template <>
Id traits::getId<RuleParameter>(const RuleParameter& prim) {
  return GetIdVisitor::id(prim);
}

template <>
Id traits::getId<ConstRuleParameter>(const ConstRuleParameter& prim) {
  return GetIdVisitor::id(prim);
}

}  // namespace lanelet
