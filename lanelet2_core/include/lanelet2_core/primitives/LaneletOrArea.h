#pragma once
#include <boost/variant.hpp>

#include "lanelet2_core/primitives/Area.h"
#include "lanelet2_core/primitives/Lanelet.h"

namespace lanelet {

//! An object that can either refer to a lanelet or an area
class ConstLaneletOrArea {
 public:
  ConstLaneletOrArea() = default;
  ConstLaneletOrArea(ConstLaneletOrArea&& rhs) = default;
  ConstLaneletOrArea& operator=(ConstLaneletOrArea&& rhs) = default;
  ConstLaneletOrArea(const ConstLaneletOrArea& rhs) = default;
  ConstLaneletOrArea& operator=(const ConstLaneletOrArea& rhs) = default;
  ~ConstLaneletOrArea() noexcept = default;

  ConstLaneletOrArea(ConstLanelet lanelet)  // NOLINT
      : laneletOrArea_{std::move(lanelet)} {}
  ConstLaneletOrArea(ConstArea area)  // NOLINT
      : laneletOrArea_{std::move(area)} {}
  ConstLaneletOrArea& operator=(ConstLanelet lanelet) {
    laneletOrArea_ = std::move(lanelet);
    return *this;
  }
  ConstLaneletOrArea& operator=(ConstArea area) {
    laneletOrArea_ = std::move(area);
    return *this;
  }

  //! true if this object holds an area
  bool isArea() const { return laneletOrArea_.which() == 1; }

  //! true if this objct holds a lanelet
  bool isLanelet() const { return laneletOrArea_.which() == 0; }

  //! convert to lanelet (type is not checked)
  explicit operator const ConstLanelet&() const { return boost::get<ConstLanelet>(laneletOrArea_); }

  //! convert to area (type is not checked)
  explicit operator const ConstArea&() const { return boost::get<ConstArea>(laneletOrArea_); }

  //! apply a generic visitor
  template <typename VisitorT>
  decltype(auto) applyVisitor(VisitorT visitor) const {
    return boost::apply_visitor(visitor, laneletOrArea_);
  }

  //! get the id of the lanelet or area
  Id id() const {
    return applyVisitor([](auto& elem) { return elem.id(); });
  }

  //! get the attributes of the lanelet or area
  const AttributeMap& attributes() const {
    return applyVisitor([](auto& elem) -> const AttributeMap& { return elem.attributes(); });
  }

  RegulatoryElementConstPtrs regulatoryElements() const {
    return applyVisitor([](auto& elem) { return elem.regulatoryElements(); });
  }

  template <typename T>
  std::vector<std::shared_ptr<const T>> regulatoryElementsAs() const {
    return applyVisitor([](auto& elem) { return elem.template regulatoryElementsAs<T>(); });
  }

  //! return the managed lanelet
  Optional<ConstLanelet> lanelet() const {
    const auto* ll = boost::get<ConstLanelet>(&laneletOrArea_);
    if (ll != nullptr) {
      return *ll;
    }
    return {};
  }

  //! get the managed area
  Optional<ConstArea> area() const {
    const auto* ar = boost::get<ConstArea>(&laneletOrArea_);
    if (ar != nullptr) {
      return *ar;
    }
    return {};
  }
  //! compares this lanelet or area
  bool equals(const ConstLaneletOrArea& other) const { return laneletOrArea_ == other.laneletOrArea_; }

  //! returns the outer bound if it is an area or the polygon made of the lanelet bounds if it's a lanelet
  CompoundPolygon3d boundingPolygon() const {
    if (isArea()) {
      return area()->outerBoundPolygon();
    }
    return lanelet()->polygon3d();
  }

 private:
  boost::variant<ConstLanelet, ConstArea> laneletOrArea_;
};

inline bool operator==(const ConstLaneletOrArea& lhs, const ConstLaneletOrArea& rhs) { return lhs.equals(rhs); }
inline bool operator!=(const ConstLaneletOrArea& lhs, const ConstLaneletOrArea& rhs) { return !(lhs == rhs); }

inline std::ostream& operator<<(std::ostream& stream, const ConstLaneletOrArea& obj) {
  if (obj.isArea()) {
    stream << *obj.area();
  }
  if (obj.isLanelet()) {
    stream << *obj.lanelet();
  }
  return stream;
}

namespace utils {
inline ConstLanelets getAllLanelets(const ConstLaneletOrAreas& lars) {
  ConstLanelets lanelets;
  lanelets.reserve(lars.size());
  for (const auto& lar : lars) {
    if (lar.isLanelet()) {
      lanelets.push_back(static_cast<const ConstLanelet&>(lar));
    }
  };
  return lanelets;
}
inline ConstAreas getAllAreas(const ConstLaneletOrAreas& lars) {
  ConstAreas lanelets;
  lanelets.reserve(lars.size());
  for (const auto& lar : lars) {
    if (lar.isArea()) {
      lanelets.push_back(static_cast<const ConstArea&>(lar));
    }
  };
  return lanelets;
}
}  // namespace utils

}  // namespace lanelet

// Hash function for usage in containers
namespace std {
template <>
struct hash<lanelet::ConstLaneletOrArea> : public lanelet::HashBase<lanelet::ConstLaneletOrArea> {};

}  // namespace std
