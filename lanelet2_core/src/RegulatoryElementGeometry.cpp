#include <boost/variant.hpp>

#include "lanelet2_core/geometry/Area.h"
#include "lanelet2_core/geometry/BoundingBox.h"
#include "lanelet2_core/geometry/Lanelet.h"
#include "lanelet2_core/geometry/LineString.h"
#include "lanelet2_core/geometry/RegulatoryElement.h"
#include "lanelet2_core/primitives/Traits.h"

namespace lanelet {
namespace geometry {
namespace {
using traits::to2D;
struct Bbox2dVisitor : public RuleParameterVisitor {  // NOLINT
  void operator()(const ConstPoint3d& p) override { bbox.extend(boundingBox2d(to2D(p))); }
  void operator()(const ConstLineString3d& l) override { bbox.extend(boundingBox2d(to2D(l))); }
  void operator()(const ConstPolygon3d& p) override { bbox.extend(boundingBox2d(to2D(p))); }
  void operator()(const ConstWeakLanelet& ll) override {
    if (ll.expired()) {
      return;
    }
    bbox.extend(boundingBox2d(ll.lock()));
  }
  void operator()(const ConstWeakArea& ar) override {
    if (ar.expired()) {
      return;
    }
    bbox.extend(boundingBox2d(ar.lock()));
  }
  BoundingBox2d bbox{};
};
struct Bbox3dVisitor : public RuleParameterVisitor {  // NOLINT
  void operator()(const ConstPoint3d& p) override { bbox.extend(boundingBox3d(p)); }
  void operator()(const ConstLineString3d& l) override { bbox.extend(boundingBox3d(l)); }
  void operator()(const ConstPolygon3d& p) override { bbox.extend(boundingBox3d(p)); }
  void operator()(const ConstWeakLanelet& ll) override {
    if (ll.expired()) {
      return;
    }
    bbox.extend(boundingBox3d(ll.lock()));
  }
  void operator()(const ConstWeakArea& ar) override {
    if (ar.expired()) {
      return;
    }
    bbox.extend(boundingBox3d(ar.lock()));
  }
  BoundingBox3d bbox;
};

struct DistanceVisitor : public RuleParameterVisitor {  // NOLINT
  explicit DistanceVisitor(BasicPoint2d p) : pdist{std::move(p)} {}
  void operator()(const ConstPoint3d& p) override { d = std::min(d, distance(traits::to2D(p), pdist)); }
  void operator()(const ConstLineString3d& l) override { d = std::min(d, distance2d(l, pdist)); }
  void operator()(const ConstPolygon3d& p) override { d = std::min(d, distance2d(p, pdist)); }
  void operator()(const ConstWeakLanelet& ll) override {
    if (ll.expired()) {
      return;
    }
    d = std::min(d, distance2d(ll.lock(), pdist));
  }
  void operator()(const ConstWeakArea& ar) override {
    if (ar.expired()) {
      return;
    }
    d = std::min(d, distance2d(ar.lock(), pdist));
  }
  BasicPoint2d pdist;
  double d{std::numeric_limits<double>::infinity()};
};
}  // namespace

BoundingBox2d boundingBox2d(const RegulatoryElement& regElem) {
  Bbox2dVisitor visitor;
  regElem.applyVisitor(visitor);
  return visitor.bbox;
}

BoundingBox3d boundingBox3d(const RegulatoryElement& regElem) {
  Bbox3dVisitor visitor;
  regElem.applyVisitor(visitor);
  return visitor.bbox;
}

double distance2d(const RegulatoryElement& regElem, const BasicPoint2d& p) {
  DistanceVisitor visitor(p);
  regElem.applyVisitor(visitor);
  return visitor.d;
}

}  // namespace geometry
}  // namespace lanelet
