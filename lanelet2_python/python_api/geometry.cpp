#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Area.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LaneletMap.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/geometry/RegulatoryElement.h>

#include <boost/geometry/geometries/register/multi_linestring.hpp>
#include <boost/python.hpp>

#include "lanelet2_python/internal/converter.h"

BOOST_GEOMETRY_REGISTER_MULTI_LINESTRING(lanelet::LineStrings2d)
BOOST_GEOMETRY_REGISTER_MULTI_LINESTRING(lanelet::ConstLineStrings2d)
BOOST_GEOMETRY_REGISTER_MULTI_LINESTRING(lanelet::ConstHybridLineStrings2d)
BOOST_GEOMETRY_REGISTER_MULTI_LINESTRING(lanelet::CompoundLineStrings2d)

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
template <typename PrimT, typename GeometryT>
auto wrapFindWithin2d() {
  using ResultT = std::vector<std::pair<double, PrimT>>;
  using Sig = ResultT (*)(PrimitiveLayer<PrimT>&, const GeometryT&, double);
  auto func = static_cast<Sig>(lanelet::geometry::findWithin2d);
  return def("findWithin2d", func, (arg("layer"), arg("geometry"), arg("maxDist") = 0),
             "returns all elements that are closer than maxDist to a geometry in 2d");
}
template <typename PrimT, typename GeometryT>
auto wrapFindWithin3d() {
  using ResultT = std::vector<std::pair<double, PrimT>>;
  using Sig = ResultT (*)(PrimitiveLayer<PrimT>&, const GeometryT&, double);
  auto func = static_cast<Sig>(lanelet::geometry::findWithin3d);
  return def("findWithin3d", func, (arg("layer"), arg("geometry"), arg("maxDist") = 0),
             "returns all elements that are closer than maxDist to a geometry in 3d");
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
  TO2D_AS(CompoundLineString3d)
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
  TO3D_AS(CompoundLineString2d)
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
  def("distance", lg::distance<ConstPoint2d, CompoundLineString2d>);
  def("distance", lg::distance<CompoundLineString2d, ConstPoint2d>);
  def("distance", lg::distance<ConstPoint2d, ConstLineString2d>);
  def("distance", lg::distance<ConstLineString2d, ConstPoint2d>);
  def("distance", lg::distance<ConstPoint2d, CompoundLineString2d>);
  def("distance", lg::distance<CompoundLineString2d, ConstPoint2d>);
  def("distance", lg::distance<BasicPoint2d, ConstLineString2d>);
  def("distance", lg::distance<ConstLineString2d, BasicPoint2d>);
  def("distance", lg::distance<BasicPoint2d, CompoundLineString2d>);
  def("distance", lg::distance<CompoundLineString2d, BasicPoint2d>);

  // l2l
  def(
      "distance", +[](const ConstLineString2d& ls1, const ConstLineString2d& ls2) { return lg::distance2d(ls1, ls2); });
  def("distance", lg::distance<ConstHybridLineString2d, ConstHybridLineString2d>);
  def(
      "distance",
      +[](const CompoundLineString2d& ls1, const CompoundLineString2d& ls2) { return lg::distance2d(ls1, ls2); });
  def(
      "distance",
      +[](const ConstLineString2d& ls1, const CompoundLineString2d& ls2) { return lg::distance2d(ls1, ls2); });
  def(
      "distance",
      +[](const CompoundLineString2d& ls1, const ConstLineString2d& ls2) { return lg::distance2d(ls1, ls2); });

  // poly2p
  def("distance", lg::distance<ConstHybridPolygon2d, BasicPoint2d>);
  def(
      "distance", +[](const ConstPolygon2d& p1, const BasicPoint2d& p2) { return lg::distance2d(p1, p2); });
  def(
      "distance",
      +[](const ConstPolygon2d& p1, const ConstPoint2d& p2) { return lg::distance2d(p1, p2.basicPoint()); });

  // poly2ls
  def(
      "distance", +[](const ConstPolygon2d& p1, const ConstLineString2d& p2) { return lg::distance2d(p1, p2); });
  def("distance", lg::distance<ConstHybridPolygon2d, ConstHybridLineString2d>);
  def(
      "distance", +[](const ConstLineString2d& p2, const ConstPolygon2d& p1) { return lg::distance2d(p1, p2); });
  def("distance", lg::distance<ConstHybridLineString2d, ConstHybridPolygon2d>);
  def(
      "distance", +[](const ConstPolygon2d& p1, const CompoundLineString2d& p2) { return lg::distance2d(p1, p2); });
  def(
      "distance", +[](const CompoundLineString2d& p1, const ConstPolygon2d& p2) { return lg::distance2d(p1, p2); });

  // poly2poly
  def("distance", lg::distance<ConstHybridPolygon2d, ConstHybridPolygon2d>);
  def(
      "distance", +[](const ConstPolygon2d& p1, const ConstPolygon2d& p2) { return lg::distance2d(p1, p2); });
  def(
      "distance", +[](const ConstHybridPolygon2d& p1, const ConstPolygon2d& p2) { return lg::distance2d(p1, p2); });
  def(
      "distance", +[](const CompoundPolygon2d& p1, const ConstPolygon2d& p2) { return lg::distance2d(p1, p2); });
  def(
      "distance", +[](const ConstPolygon2d& p1, const ConstHybridPolygon2d& p2) { return lg::distance2d(p1, p2); });

  // p2llt
  def(
      "distance", +[](const ConstLanelet& llt, const BasicPoint2d& p) { return lg::distance2d(llt, p); });
  def(
      "distance", +[](const ConstLanelet& llt1, const ConstLanelet& llt2) { return lg::distance2d(llt1, llt2); });
  def(
      "distance", +[](const BasicPoint2d& p, const ConstLanelet& llt) { return lg::distance2d(llt, p); });
  def(
      "distance", +[](const ConstLanelet& llt2, const ConstLanelet& llt1) { return lg::distance2d(llt1, llt2); });

  // p2area
  def(
      "distance", +[](const ConstArea& llt, const BasicPoint2d& p) { return lg::distance2d(llt, p); });
  def(
      "distance", +[](const BasicPoint2d& p, const ConstArea& llt) { return lg::distance2d(llt, p); });

  // 3d
  // p2p
  def("distance", lg::distance<ConstPoint3d, ConstPoint3d>);
  def("distance", lg::distance<ConstPoint3d, BasicPoint3d>);
  def("distance", lg::distance<BasicPoint3d, ConstPoint3d>);
  def("distance", lg::distance<BasicPoint3d, BasicPoint3d>);

  // p2l
  def("distance", lg::distance<ConstPoint3d, HybridLs3d>);
  def("distance", lg::distance<HybridLs3d, ConstPoint3d>);
  def("distance", lg::distance<ConstPoint3d, CompoundLineString3d>);
  def("distance", lg::distance<CompoundLineString3d, ConstPoint3d>);
  def("distance", lg::distance<ConstPoint3d, ConstLineString3d>);
  def("distance", lg::distance<ConstLineString3d, ConstPoint3d>);
  def("distance", lg::distance<ConstPoint3d, CompoundLineString3d>);
  def("distance", lg::distance<CompoundLineString3d, ConstPoint3d>);

  // p2lines
  def("distanceToLines", distancePointToLss<ConstPoint2d>);
  def("distanceToLines", distancePointToLss<BasicPoint2d>);
  def("distanceToLines", distancePointToLss<ConstPoint2d>);
  def("distanceToLines", distancePointToLss<BasicPoint2d>);

  // l2l
  def(
      "distance", +[](const ConstLineString3d& ls1, const ConstLineString3d& ls2) { return lg::distance3d(ls1, ls2); });
  def(
      "distance", +[](const HybridLs3d& ls1, const HybridLs3d& ls2) { return lg::distance3d(ls1, ls2); });
  def(
      "distance",
      +[](const CompoundLineString3d& ls1, const CompoundLineString3d& ls2) { return lg::distance3d(ls1, ls2); });
  def(
      "distance",
      +[](const ConstLineString3d& ls1, const CompoundLineString3d& ls2) { return lg::distance3d(ls1, ls2); });
  def(
      "distance",
      +[](const CompoundLineString3d& ls1, const ConstLineString3d& ls2) { return lg::distance3d(ls1, ls2); });

  // p2llt
  def("distance", lg::distance3d<ConstLanelet, BasicPoint3d>);
  def("distance", lg::distance3d<BasicPoint3d, ConstLanelet>);

  // p2area
  def(
      "distance", +[](const ConstArea& llt, const BasicPoint3d& p) { return lg::distance3d(llt, p); });
  def(
      "distance", +[](const BasicPoint3d& p, const ConstArea& llt) { return lg::distance3d(llt, p); });

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
  def("boundingBox2d", boundingBox2dFor<CompoundLineString2d>);

  def("boundingBox3d", boundingBox3dFor<ConstPoint3d>, "Get the surrounding axis-aligned bounding box in 3d");
  def("boundingBox3d", boundingBox3dFor<ConstLineString3d>);
  def("boundingBox3d", boundingBox3dFor<ConstHybridLineString3d>);
  def("boundingBox3d", boundingBox3dFor<ConstPolygon3d>);
  def("boundingBox3d", boundingBox3dFor<ConstHybridPolygon3d>);
  def("boundingBox3d", boundingBox3dFor<ConstLanelet>);
  def("boundingBox3d", boundingBox3dFor<ConstArea>);
  def("boundingBox3d", boundingBox3dFor<RegulatoryElementPtr>);
  def("boundingBox3d", boundingBox3dFor<RegulatoryElementConstPtr>);
  def("boundingBox3d", boundingBox3dFor<CompoundLineString3d>);

  // area
  def("area", boost::geometry::area<BasicPolygon2d>);
  def("area", boost::geometry::area<ConstHybridPolygon2d>);

  class_<ArcCoordinates>("ArcCoordinates", "Coordinates along an arc", init<>())
      .def_readwrite("length", &ArcCoordinates::length, "length along arc")
      .def_readwrite("distance", &ArcCoordinates::distance, "signed distance to arc (left is positive");

  def("toArcCoordinates", lg::toArcCoordinates<ConstLineString2d>,
      "Project a point into arc coordinates of the linestring");
  def("toArcCoordinates", lg::toArcCoordinates<CompoundLineString2d>,
      "Project a point into arc coordinates of the linestring");

  def("length", lg::length<ConstLineString2d>);
  def("length", lg::length<ConstLineString3d>);
  def("length", lg::length<CompoundLineString2d>);
  def("length", lg::length<CompoundLineString3d>);

  def("interpolatedPointAtDistance", lg::interpolatedPointAtDistance<BasicLineString2d>);
  def("interpolatedPointAtDistance", lg::interpolatedPointAtDistance<BasicLineString3d>);
  def("interpolatedPointAtDistance", lg::interpolatedPointAtDistance<ConstLineString2d>);
  def("interpolatedPointAtDistance", lg::interpolatedPointAtDistance<ConstLineString3d>);
  def("interpolatedPointAtDistance", lg::interpolatedPointAtDistance<CompoundLineString2d>);
  def("interpolatedPointAtDistance", lg::interpolatedPointAtDistance<CompoundLineString3d>);

  def("nearestPointAtDistance", lg::nearestPointAtDistance<ConstLineString2d>);
  def("nearestPointAtDistance", lg::nearestPointAtDistance<ConstLineString3d>);
  def("nearestPointAtDistance", lg::nearestPointAtDistance<CompoundLineString2d>);
  def("nearestPointAtDistance", lg::nearestPointAtDistance<CompoundLineString3d>);

  def("project", lg::project<ConstLineString2d>, "Project a point onto the linestring");
  def("project", lg::project<ConstLineString3d>, "Project a point onto the linestring");
  def("project", lg::project<CompoundLineString2d>, "Project a point onto the linestring");
  def("project", lg::project<CompoundLineString3d>, "Project a point onto the linestring");

  def("projectedPoint3d", lg::projectedPoint3d<ConstLineString3d>,
      "Returns the respective projected points of the closest distance of two "
      "linestrings");
  def("projectedPoint3d", lg::projectedPoint3d<ConstHybridLineString3d>,
      "Returns the respective projected points of the closest distance of two "
      "linestrings");
  def("projectedPoint3d", lg::projectedPoint3d<CompoundLineString3d>,
      "Returns the respective projected points of the closest distance of two "
      "compound linestrings");

  def(
      "intersects2d", +[](const ConstLineString2d& ls1, const ConstLineString2d& ls2) {
        return lg::intersects(utils::toHybrid(ls1), utils::toHybrid(ls2));
      });
  def("intersects2d", lg::intersects<ConstHybridLineString2d, ConstHybridLineString2d>);
  def(
      "intersects2d", +[](const CompoundLineString2d& ls1, const CompoundLineString2d& ls2) {
        return lg::intersects(utils::toHybrid(ls1), utils::toHybrid(ls2));
      });
  def(
      "intersects2d", +[](const ConstPolygon2d& ls1, const ConstPolygon2d& ls2) {
        return lg::intersects(utils::toHybrid(ls1), utils::toHybrid(ls2));
      });
  def("intersects2d", lg::intersects<ConstHybridPolygon2d, ConstHybridPolygon2d>);
  def("intersects2d", lg::intersects<BoundingBox3d, BoundingBox3d>);
  def("intersects2d", lg::intersects2d<ConstLanelet, ConstLanelet>);
  def("intersects2d", lg::intersects2d<ConstArea, ConstArea>);

  def("intersects3d", lg::intersects3d<ConstLineString3d>);
  def("intersects3d", lg::intersects<BoundingBox3d, BoundingBox3d>);
  def("intersects3d", lg::intersects3d<ConstHybridLineString3d>);
  def("intersects3d", lg::intersects3d<CompoundLineString3d>);
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
  def(
      "intersectCenterlines2d", +[](const ConstLanelet& ll1, const ConstLanelet& ll2) {
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

  // find within, point layer
  wrapFindWithin2d<Point3d, Point2d>();
  wrapFindWithin2d<Point3d, BasicPoint2d>();
  wrapFindWithin2d<Point3d, BoundingBox2d>();
  wrapFindWithin2d<Point3d, Polygon2d>();
  wrapFindWithin2d<Point3d, BasicPolygon2d>();
  wrapFindWithin2d<Point3d, LineString2d>();
  wrapFindWithin2d<Point3d, BasicLineString2d>();
  wrapFindWithin2d<Point3d, CompoundLineString2d>();
  wrapFindWithin2d<Point3d, Lanelet>();
  wrapFindWithin2d<Point3d, Area>();
  wrapFindWithin3d<Point3d, Point3d>();
  wrapFindWithin3d<Point3d, BasicPoint3d>();
  wrapFindWithin3d<Point3d, BoundingBox3d>();
  wrapFindWithin3d<Point3d, Polygon3d>();
  wrapFindWithin3d<Point3d, BasicPolygon3d>();
  wrapFindWithin3d<Point3d, LineString3d>();
  wrapFindWithin3d<Point3d, BasicLineString3d>();
  wrapFindWithin3d<Point3d, CompoundLineString3d>();
  wrapFindWithin3d<Point3d, Lanelet>();
  wrapFindWithin3d<Point3d, Area>();

  // linestring layer
  wrapFindWithin2d<LineString3d, Point2d>();
  wrapFindWithin2d<LineString3d, BasicPoint2d>();
  wrapFindWithin2d<LineString3d, BoundingBox2d>();
  wrapFindWithin2d<LineString3d, Polygon2d>();
  wrapFindWithin2d<LineString3d, BasicPolygon2d>();
  wrapFindWithin2d<LineString3d, LineString2d>();
  wrapFindWithin2d<LineString3d, Lanelet>();
  wrapFindWithin2d<LineString3d, Area>();
  wrapFindWithin2d<LineString3d, BasicLineString2d>();
  wrapFindWithin2d<LineString3d, CompoundLineString2d>();
  wrapFindWithin3d<LineString3d, Point3d>();
  wrapFindWithin3d<LineString3d, BasicPoint3d>();

  // polygon layer
  wrapFindWithin2d<Polygon3d, Point2d>();
  wrapFindWithin2d<Polygon3d, BasicPoint2d>();
  wrapFindWithin2d<Polygon3d, BoundingBox2d>();
  wrapFindWithin2d<Polygon3d, Polygon2d>();
  wrapFindWithin2d<Polygon3d, BasicPolygon2d>();
  wrapFindWithin2d<Polygon3d, LineString2d>();
  wrapFindWithin2d<Polygon3d, BasicLineString2d>();
  wrapFindWithin2d<Polygon3d, CompoundLineString2d>();
  wrapFindWithin2d<Polygon3d, Lanelet>();
  wrapFindWithin2d<Polygon3d, Area>();
  wrapFindWithin3d<Polygon3d, Point3d>();
  wrapFindWithin3d<Polygon3d, BasicPoint3d>();

  // lanelet layer
  wrapFindWithin2d<Lanelet, Point2d>();
  wrapFindWithin2d<Lanelet, BasicPoint2d>();
  wrapFindWithin2d<Lanelet, BoundingBox2d>();
  wrapFindWithin2d<Lanelet, Polygon2d>();
  wrapFindWithin2d<Lanelet, BasicPolygon2d>();
  wrapFindWithin2d<Lanelet, LineString2d>();
  wrapFindWithin2d<Lanelet, BasicLineString2d>();
  wrapFindWithin2d<Lanelet, CompoundLineString2d>();
  wrapFindWithin2d<Lanelet, Lanelet>();
  wrapFindWithin2d<Lanelet, Area>();
  wrapFindWithin3d<Lanelet, Point3d>();
  wrapFindWithin3d<Lanelet, BasicPoint3d>();

  // area layer
  wrapFindWithin2d<Area, Point2d>();
  wrapFindWithin2d<Area, BasicPoint2d>();
  wrapFindWithin2d<Area, BoundingBox2d>();
  wrapFindWithin2d<Area, Polygon2d>();
  wrapFindWithin2d<Area, BasicPolygon2d>();
  wrapFindWithin2d<Area, LineString2d>();
  wrapFindWithin2d<Area, BasicLineString2d>();
  wrapFindWithin2d<Area, CompoundLineString2d>();
  wrapFindWithin2d<Area, Lanelet>();
  wrapFindWithin2d<Area, Area>();
  wrapFindWithin3d<Area, Point3d>();
  wrapFindWithin3d<Area, BasicPoint3d>();
}
