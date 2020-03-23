#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Area.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/geometry/RegulatoryElement.h>
#include <boost/geometry/geometries/register/multi_linestring.hpp>
#include <boost/python.hpp>
#include "internal/converter.h"

BOOST_GEOMETRY_REGISTER_MULTI_LINESTRING(lanelet::LineStrings2d);
BOOST_GEOMETRY_REGISTER_MULTI_LINESTRING(lanelet::ConstLineStrings2d);
BOOST_GEOMETRY_REGISTER_MULTI_LINESTRING(lanelet::ConstHybridLineStrings2d);

using namespace boost::python;
using namespace lanelet;

using HybridLs3d = ConstHybridLineString3d;
using HybridLs2d = ConstHybridLineString2d;

template <typename PrimT>
auto wrapFindNearest() {
  using ResultT = std::vector<std::pair<double, PrimT>>;
  using Sig = ResultT (*)(PrimitiveLayer<PrimT>&, const BasicPoint2d&, unsigned);
  auto func = static_cast<Sig>(lanelet::geometry::findNearest);
  converters::PairConverter<std::pair<double, PrimT>>();
  converters::VectorToListConverter<ResultT>();
  return def("findNearest", func);
}

std::vector<BasicPoint2d> toBasicVector(const BasicPoints2d& pts) {
  return utils::transform(pts, [](auto& p) { return BasicPoint2d(p.x(), p.y()); });
}

template <typename T>
lanelet::BoundingBox2d boundingBox2dFor(const T& t) {
  return lanelet::geometry::boundingBox2d(t);
}

template <typename T>
lanelet::BoundingBox3d boundingBox3dFor(const T& t) {
  return lanelet::geometry::boundingBox3d(t);
}

// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define TO2D_AS(X)                        \
  if (extract<X>(o).check()) {            \
    return object(to2D(extract<X>(o)())); \
  }

// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define TO3D_AS(X)                        \
  if (extract<X>(o).check()) {            \
    return object(to3D(extract<X>(o)())); \
  }

template <typename PtT>
double distancePointToLss(const PtT& p, const object& lss) {
  auto distance = [](PtT p, auto range) {
    return boost::geometry::distance(p, utils::transform(range, [](auto& v) { return utils::toHybrid(v); }));
  };
  if (extract<ConstLineStrings2d>(lss).check()) {
    return distance(p, extract<ConstLineStrings2d>(lss)());
  }
  return distance(p, extract<LineStrings2d>(lss)());
}

object to2D(object o) {
  using utils::to2D;
  TO2D_AS(Point3d)
  TO2D_AS(BasicPoint3d)
  TO2D_AS(ConstPoint3d)
  TO2D_AS(LineString3d)
  TO2D_AS(LineString3d)
  TO2D_AS(ConstLineString3d)
  TO2D_AS(Polygon3d)
  TO2D_AS(ConstPolygon3d)
  return o;
}

object to3D(object o) {
  using utils::to3D;
  TO3D_AS(Point2d)
  TO3D_AS(BasicPoint2d)
  TO3D_AS(ConstPoint2d)
  TO3D_AS(LineString2d)
  TO3D_AS(LineString2d)
  TO3D_AS(ConstLineString2d)
  TO3D_AS(Polygon2d)
  TO3D_AS(ConstPolygon2d)
  return o;
}
#undef TO2D_AS
#undef TO3D_AS

BOOST_PYTHON_MODULE(PYTHON_API_MODULE_NAME) {  // NOLINT
  namespace lg = lanelet::geometry;

  def("to2D", to2D);
  def("to3D", to3D);

  // p2p
  def("distance", lg::distance<BasicPoint2d, BasicPoint2d>);
  def("distance", lg::distance<ConstPoint2d, ConstPoint2d>);
  def("distance", lg::distance<ConstPoint2d, BasicPoint2d>);
  def("distance", lg::distance<BasicPoint2d, ConstPoint2d>);

  // p2l
  def("distance", lg::distance<ConstPoint2d, HybridLs2d>);
  def("distance", lg::distance<HybridLs2d, ConstPoint2d>);
  def("distance", lg::distance<ConstPoint2d, ConstLineString2d>);
  def("distance", lg::distance<ConstLineString2d, ConstPoint2d>);
  def("distance", lg::distance<BasicPoint2d, ConstLineString2d>);
  def("distance", lg::distance<ConstLineString2d, BasicPoint2d>);

  // l2l
  def("distance", +[](const ConstLineString2d& ls1, const ConstLineString2d& ls2) { return lg::distance2d(ls1, ls2); });
  def("distance", lg::distance<ConstHybridLineString2d, ConstHybridLineString2d>);

  // poly2p
  def("distance", lg::distance<ConstHybridPolygon2d, BasicPoint2d>);
  def("distance", +[](const ConstPolygon2d& p1, const BasicPoint2d& p2) { return lg::distance2d(p1, p2); });
  def("distance",
      +[](const ConstPolygon2d& p1, const ConstPoint2d& p2) { return lg::distance2d(p1, p2.basicPoint()); });

  // poly2ls
  def("distance", +[](const ConstPolygon2d& p1, const ConstLineString2d& p2) { return lg::distance2d(p1, p2); });
  def("distance", lg::distance<ConstHybridPolygon2d, ConstHybridLineString2d>);

  // poly2poly
  def("distance", lg::distance<ConstHybridPolygon2d, ConstHybridPolygon2d>);
  def("distance", +[](const ConstPolygon2d& p1, const ConstPolygon2d& p2) { return lg::distance2d(p1, p2); });
  def("distance", +[](const ConstHybridPolygon2d& p1, const ConstPolygon2d& p2) { return lg::distance2d(p1, p2); });
  def("distance", +[](const CompoundPolygon2d& p1, const ConstPolygon2d& p2) { return lg::distance2d(p1, p2); });
  def("distance", +[](const ConstPolygon2d& p1, const ConstHybridPolygon2d& p2) { return lg::distance2d(p1, p2); });

  // p2llt
  def("distance", +[](const ConstLanelet& llt, const BasicPoint2d& p) { return lg::distance2d(llt, p); });
  def("distance", +[](const ConstLanelet& llt1, const ConstLanelet& llt2) { return lg::distance2d(llt1, llt2); });

  // p2area
  def("distance", +[](const ConstArea& llt, const BasicPoint2d& p) { return lg::distance2d(llt, p); });

  // 3d
  // p2p
  def("distance", lg::distance<ConstPoint3d, ConstPoint3d>);
  def("distance", lg::distance<ConstPoint3d, BasicPoint3d>);
  def("distance", lg::distance<BasicPoint3d, ConstPoint3d>);
  def("distance", lg::distance<BasicPoint3d, BasicPoint3d>);

  // p2l
  def("distance", lg::distance<ConstPoint3d, HybridLs3d>);
  def("distance", lg::distance<HybridLs3d, ConstPoint3d>);
  def("distance", lg::distance<ConstPoint3d, ConstLineString3d>);
  def("distance", lg::distance<ConstLineString3d, ConstPoint3d>);

  // p2lines
  def("distanceToLines", distancePointToLss<ConstPoint2d>);
  def("distanceToLines", distancePointToLss<BasicPoint2d>);
  def("distanceToLines", distancePointToLss<ConstPoint2d>);
  def("distanceToLines", distancePointToLss<BasicPoint2d>);

  // l2l
  def("distance", +[](const ConstLineString3d& ls1, const ConstLineString3d& ls2) { return lg::distance3d(ls1, ls2); });
  def("distance", +[](const HybridLs3d& ls1, const HybridLs3d& ls2) { return lg::distance3d(ls1, ls2); });

  // p2llt
  def("distance", lg::distance3d<ConstLanelet, BasicPoint3d>);

  // p2area
  def("distance", +[](const ConstArea& llt, const BasicPoint3d& p) { return lg::distance3d(llt, p); });

  // equals 2d
  def("equals", boost::geometry::equals<BasicPoint2d, BasicPoint2d>);
  def("equals", boost::geometry::equals<ConstPoint2d, ConstPoint2d>);
  def("equals", boost::geometry::equals<ConstPoint2d, BasicPoint2d>);
  def("equals", boost::geometry::equals<BasicPoint2d, ConstPoint2d>);

  // equals 3d
  def("equals", boost::geometry::equals<BasicPoint3d, BasicPoint3d>);
  def("equals", boost::geometry::equals<ConstPoint3d, ConstPoint3d>);
  def("equals", boost::geometry::equals<ConstPoint3d, BasicPoint3d>);
  def("equals", boost::geometry::equals<BasicPoint3d, ConstPoint3d>);

  def("boundingBox2d", boundingBox2dFor<ConstPoint2d>, "Get the surrounding axis-aligned bounding box in 2d");
  def("boundingBox2d", boundingBox2dFor<ConstLineString2d>);
  def("boundingBox2d", boundingBox2dFor<ConstHybridLineString2d>);
  def("boundingBox2d", boundingBox2dFor<ConstPolygon2d>);
  def("boundingBox2d", boundingBox2dFor<ConstHybridPolygon2d>);
  def("boundingBox2d", boundingBox2dFor<ConstLanelet>);
  def("boundingBox2d", boundingBox2dFor<ConstArea>);
  def("boundingBox2d", boundingBox2dFor<RegulatoryElementPtr>);
  def("boundingBox2d", boundingBox2dFor<RegulatoryElementConstPtr>);

  def("boundingBox3d", boundingBox3dFor<ConstPoint3d>, "Get the surrounding axis-aligned bounding box in 3d");
  def("boundingBox3d", boundingBox3dFor<ConstLineString3d>);
  def("boundingBox3d", boundingBox3dFor<ConstHybridLineString3d>);
  def("boundingBox3d", boundingBox3dFor<ConstPolygon3d>);
  def("boundingBox3d", boundingBox3dFor<ConstHybridPolygon3d>);
  def("boundingBox3d", boundingBox3dFor<ConstLanelet>);
  def("boundingBox3d", boundingBox3dFor<ConstArea>);
  def("boundingBox3d", boundingBox3dFor<RegulatoryElementPtr>);
  def("boundingBox3d", boundingBox3dFor<RegulatoryElementConstPtr>);

  // area
  def("area", boost::geometry::area<BasicPolygon2d>);
  def("area", boost::geometry::area<ConstHybridPolygon2d>);

  class_<ArcCoordinates>("ArcCoordinates", "Coordinates along an arc", init<>())
      .add_property("length", &ArcCoordinates::length, "lenght along arc")
      .add_property("distance", &ArcCoordinates::distance, "signed distance to arc (left is positive");

  def("toArcCoordinates", lg::toArcCoordinates<ConstLineString2d>,
      "Project a point into arc coordinates of the linestring");

  def("length", lg::length<ConstLineString2d>);
  def("length", lg::length<ConstLineString3d>);

  def("interpolatedPointAtDistance", lg::interpolatedPointAtDistance<ConstLineString2d>);
  def("interpolatedPointAtDistance", lg::interpolatedPointAtDistance<ConstLineString3d>);

  def("nearestPointAtDistance", lg::nearestPointAtDistance<ConstLineString2d>);
  def("nearestPointAtDistance", lg::nearestPointAtDistance<ConstLineString3d>);

  def("project", lg::project<ConstLineString2d>, "Project a point onto the linestring");
  def("project", lg::project<ConstLineString3d>, "Project a point onto the linestring");

  def("projectedPoint3d", lg::projectedPoint3d<ConstLineString3d>,
      "Returns the respective projected points of the closest distance of two "
      "linestrings");
  def("projectedPoint3d", lg::projectedPoint3d<ConstHybridLineString3d>,
      "Returns the respective projected points of the closest distance of two "
      "linestrings");

  def("intersects2d", +[](const ConstLineString2d& ls1, const ConstLineString2d& ls2) {
    return lg::intersects(utils::toHybrid(ls1), utils::toHybrid(ls2));
  });
  def("intersects2d", lg::intersects<ConstHybridLineString2d, ConstHybridLineString2d>);
  def("intersects2d", +[](const ConstPolygon2d& ls1, const ConstPolygon2d& ls2) {
    return lg::intersects(utils::toHybrid(ls1), utils::toHybrid(ls2));
  });
  def("intersects2d", lg::intersects<ConstHybridPolygon2d, ConstHybridPolygon2d>);
  def("intersects2d", lg::intersects<BoundingBox3d, BoundingBox3d>);
  def("intersects2d", lg::intersects2d<ConstLanelet, ConstLanelet>);
  def("intersects2d", lg::intersects2d<ConstArea, ConstArea>);

  def("intersects3d", lg::intersects3d<ConstLineString3d>);
  def("intersects3d", lg::intersects<BoundingBox3d, BoundingBox3d>);
  def("intersects3d", lg::intersects3d<ConstHybridLineString3d>);
  def("intersects3d", lg::intersects3d<ConstLanelet, ConstLanelet>,
      "Approximates if two lanelets intersect (touch or area  >0) in 3d",
      (arg("lanelet1"), arg("lanelet2"), arg("heightTolerance") = 3.));

  def("inside", lg::inside<ConstLanelet>, "tests whether a point is within a lanelet");
  def("length2d", lg::length2d<ConstLanelet>, "calculate length of centerline");
  def("approximatedLength2d", lg::approximatedLength2d<ConstLanelet>,
      "approximates length by sampling points along left bound");
  def("length3d", lg::length3d<ConstLanelet>, "calculate length of centerline in 3d");
  def("distanceToCenterline2d", lg::distanceToCenterline2d<ConstLanelet>);
  def("distanceToCenterline3d", lg::distanceToCenterline3d<ConstLanelet>);
  def("overlaps2d", lg::overlaps2d<ConstLanelet, ConstLanelet>, "Returns true if shared area of two lanelets is >0");
  def("overlaps3d", lg::overlaps3d<ConstLanelet, ConstLanelet>, "Approximates if two lanelets overlap (area  >0) in 3d",
      (arg("lanelet1"), arg("lanelet2"), arg("heightTolerance") = 3.));
  def("intersectCenterlines2d", +[](const ConstLanelet& ll1, const ConstLanelet& ll2) {
    return toBasicVector(lg::intersectCenterlines2d(ll1, ll2));
  });

  def("leftOf", lg::leftOf<ConstLanelet, ConstLanelet>, "Returns if first lanelet is directly left of second");
  def("rightOf", lg::rightOf<ConstLanelet, ConstLanelet>, "Returns if first lanelet is directly right of second");
  def("follows", lg::follows<ConstLanelet, ConstLanelet>, "Returns if first lanelet precedes the second");

  wrapFindNearest<Point3d>();
  wrapFindNearest<LineString3d>();
  wrapFindNearest<Polygon3d>();
  wrapFindNearest<Lanelet>();
  wrapFindNearest<Area>();
  wrapFindNearest<RegulatoryElementPtr>();
}
