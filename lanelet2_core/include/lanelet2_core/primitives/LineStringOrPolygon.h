#pragma once
#include <boost/variant.hpp>

#include "lanelet2_core/primitives/LineString.h"
#include "lanelet2_core/primitives/Polygon.h"
#include "lanelet2_core/primitives/RegulatoryElement.h"

namespace lanelet {
//! Base class for objects that can either refer to linestrings or polygons
template <typename LineStringT, typename PolygonT>
class LineStringOrPolygonBase {
 public:
  LineStringOrPolygonBase() = default;
  LineStringOrPolygonBase(LineStringOrPolygonBase&& rhs) = default;             // NOLINT
  LineStringOrPolygonBase& operator=(LineStringOrPolygonBase&& rhs) = default;  // NOLINT
  LineStringOrPolygonBase(const LineStringOrPolygonBase& rhs) = default;
  LineStringOrPolygonBase& operator=(const LineStringOrPolygonBase& rhs) = default;
  ~LineStringOrPolygonBase() noexcept = default;

  LineStringOrPolygonBase(LineStringT linestring)  // NOLINT
      : lsOrPoly_{std::move(linestring)} {}
  LineStringOrPolygonBase(PolygonT polygon)  // NOLINT
      : lsOrPoly_{std::move(polygon)} {}
  LineStringOrPolygonBase& operator=(LineStringT linestring) {
    lsOrPoly_ = std::move(linestring);
    return *this;
  }
  LineStringOrPolygonBase& operator=(PolygonT poly) {
    lsOrPoly_ = std::move(poly);
    return *this;
  }

  //! true if this object holds an polygon
  bool isPolygon() const { return lsOrPoly_.which() == 1; }

  //! true if this objct holds a lineString
  bool isLineString() const { return lsOrPoly_.which() == 0; }

  //! convert to linestring (type is not checked)
  explicit operator const LineStringT&() const { return boost::get<LineStringT>(lsOrPoly_); }

  //! convert to polygon (type is not checked)
  explicit operator const PolygonT&() const { return boost::get<PolygonT>(lsOrPoly_); }

  //! apply a generic visitor
  template <typename VisitorT>
  decltype(auto) applyVisitor(VisitorT visitor) const {
    return boost::apply_visitor(visitor, lsOrPoly_);
  }

  //! get the id of the linestring or polygon
  Id id() const {
    return applyVisitor([](auto& elem) { return elem.id(); });
  }

  //! get the attributes of the linestring or polygon
  const AttributeMap& attributes() const {
    return applyVisitor([](auto& elem) -> const AttributeMap& { return elem.attributes(); });
  }

  //! return the managed linestring
  Optional<LineStringT> lineString() const {
    const auto* ls = boost::get<LineStringT>(&lsOrPoly_);
    if (ls != nullptr) {
      return *ls;
    }
    return {};
  }

  //! get the managed polygon
  Optional<PolygonT> polygon() const {
    const auto* poly = boost::get<PolygonT>(&lsOrPoly_);
    if (poly != nullptr) {
      return *poly;
    }
    return {};
  }

  bool equals(const LineStringOrPolygonBase& other) const { return lsOrPoly_ == other.lsOrPoly_; }

 protected:
  boost::variant<LineStringT, PolygonT> lsOrPoly_;  // NOLINT
};

//! This class holds either a LineString3d or a Polygon3d
class LineStringOrPolygon3d : public LineStringOrPolygonBase<LineString3d, Polygon3d> {
 public:
  using Base = LineStringOrPolygonBase<LineString3d, Polygon3d>;
  using Base::LineStringOrPolygonBase;
  operator RuleParameter() const { return asRuleParameter(); }  // NOLINT
  RuleParameter asRuleParameter() const {
    return applyVisitor([](auto& prim) { return RuleParameter(prim); });
  }

  using Base::applyVisitor;
  //! apply a generic visitor
  template <typename VisitorT>
  decltype(auto) applyVisitor(VisitorT visitor) {
    return boost::apply_visitor(visitor, lsOrPoly_);
  }
};

//! This class holds either a ConstLineString3d or a ConstPolygon3d
class ConstLineStringOrPolygon3d : public LineStringOrPolygonBase<ConstLineString3d, ConstPolygon3d> {
 public:
  using Base = LineStringOrPolygonBase<ConstLineString3d, ConstPolygon3d>;
  using Base::LineStringOrPolygonBase;
  ConstLineStringOrPolygon3d(const LineStringOrPolygon3d& lsOrPoly) {  // NOLINT
    if (lsOrPoly.isLineString()) {
      *this = *lsOrPoly.lineString();
    }
    if (lsOrPoly.isPolygon()) {
      *this = *lsOrPoly.polygon();
    }
  }
  operator ConstRuleParameter() const { return asRuleParameter(); }  // NOLINT
  ConstRuleParameter asRuleParameter() const {
    return applyVisitor([](auto& prim) { return ConstRuleParameter(prim); });
  }
};

inline bool operator==(const LineStringOrPolygon3d& lhs, const LineStringOrPolygon3d& rhs) { return lhs.equals(rhs); }
inline bool operator!=(const LineStringOrPolygon3d& lhs, const LineStringOrPolygon3d& rhs) { return !(lhs == rhs); }

inline bool operator==(const ConstLineStringOrPolygon3d& lhs, const ConstLineStringOrPolygon3d& rhs) {
  return lhs.equals(rhs);
}
inline bool operator!=(const ConstLineStringOrPolygon3d& lhs, const ConstLineStringOrPolygon3d& rhs) {
  return !(lhs == rhs);
}

using LineStringsOrPolygons3d = std::vector<LineStringOrPolygon3d>;
using ConstLineStringsOrPolygons3d = std::vector<ConstLineStringOrPolygon3d>;

inline std::ostream& operator<<(std::ostream& stream, const LineStringOrPolygon3d& obj) {
  obj.applyVisitor([&stream](auto& prim) { stream << prim; });
  return stream;
}
inline std::ostream& operator<<(std::ostream& stream, const ConstLineStringOrPolygon3d& obj) {
  obj.applyVisitor([&stream](auto& prim) { stream << prim; });
  return stream;
}
}  // namespace lanelet
