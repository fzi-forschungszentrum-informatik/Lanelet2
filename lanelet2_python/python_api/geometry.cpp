#include <boost/python.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Area.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/geometry/RegulatoryElement.h>

using namespace boost::python;
using namespace lanelet;

using HybridLs3d = ConstHybridLineString3d;
using HybridLs2d = ConstHybridLineString2d;

template <typename PrimT>
auto wrapFindNearest() {
  using Sig = std::vector<std::pair<double, PrimT>> (*)(PrimitiveLayer<PrimT>&, const BasicPoint2d&, unsigned);
  auto func = static_cast<Sig>(lanelet::geometry::findNearest);
  return def("findNearest", func);
}

template <typename T>
lanelet::BoundingBox2d boundingBox2dFor(const T& t) {
  return lanelet::geometry::boundingBox2d(t);
}

template <typename T>
lanelet::BoundingBox3d boundingBox3dFor(const T& t) {
  return lanelet::geometry::boundingBox3d(t);
}

BOOST_PYTHON_MODULE(PYTHON_API_MODULE_NAME) {  // NOLINT
  namespace lg = lanelet::geometry;

  def("to2D", utils::to2D<Point3d>);
  def("to2D", utils::to2D<ConstPoint3d>);
  def("to2D", utils::to2D<LineString3d>);
  def("to2D", utils::to2D<ConstLineString3d>);
  def("to2D", utils::to2D<Polygon3d>);
  def("to2D", utils::to2D<ConstPolygon3d>);

  def("to3D", utils::to3D<Point2d>);
  def("to3D", utils::to3D<ConstPoint2d>);
  def("to3D", utils::to3D<LineString2d>);
  def("to3D", utils::to3D<ConstLineString2d>);
  def("to3D", utils::to3D<Polygon2d>);
  def("to3D", utils::to3D<ConstPolygon2d>);

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

  // l2l
  def("distance", +[](const ConstLineString2d& ls1, const ConstLineString2d& ls2) { return lg::distance2d(ls1, ls2); });
  def("distance", lg::distance<ConstHybridLineString2d, ConstHybridLineString2d>);

  // poly2p
  def("distance", lg::distance<ConstHybridPolygon2d, BasicPoint2d>);
  def("distance", +[](const ConstPolygon2d& p1, const BasicPoint2d& p2) { return lg::distance2d(p1, p2); });

  // poly2ls
  def("distance", +[](const ConstPolygon2d& p1, const ConstLineString2d& p2) { return lg::distance2d(p1, p2); });
  def("distance", lg::distance<ConstHybridPolygon2d, ConstHybridLineString2d>);

  // poly2poly
  def("distance", lg::distance<ConstHybridPolygon2d, ConstHybridPolygon2d>);
  def("distance", +[](const ConstPolygon2d& p1, const ConstPolygon2d& p2) { return lg::distance2d(p1, p2); });

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

  // l2l
  def("distance", +[](const ConstLineString3d& ls1, const ConstLineString3d& ls2) { return lg::distance3d(ls1, ls2); });
  def("distance", +[](const HybridLs3d& ls1, const HybridLs3d& ls2) { return lg::distance3d(ls1, ls2); });

  // p2llt
  def("distance", lg::distance3d<ConstLanelet>);

  // p2area
  def("distance", +[](const ConstArea& llt, const BasicPoint3d& p) { return lg::distance3d(llt, p); });

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

  class_<ArcCoordinates>("ArcCoordinates", "Coordinates along an arc", init<>())
      .add_property("length", &ArcCoordinates::length, "lenght along arc")
      .add_property("distance", &ArcCoordinates::distance, "signed distance to arc (left is positive");

  def("toArcCoordinates", lg::toArcCoordinates<ConstLineString2d>,
      "Project a point into arc coordinates of the linestring");

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
  def("intersects2d", lg::intersects2d<ConstLanelet, ConstLanelet>);
  def("intersects2d", lg::intersects2d<ConstArea, ConstArea>);

  def("intersects3d", lg::intersects3d<ConstLineString3d>);
  def("intersects3d", lg::intersects3d<ConstHybridLineString3d>);
  def("intersects3d", lg::intersects3d<ConstLanelet, ConstLanelet>,
      "Approximates if two lanelets intersect (touch or area  >0) in 3d",
      (arg("lanelet1"), arg("lanelet2"), arg("heightTolerance") = 3.));

  def("inside", lg::inside<ConstLanelet>, "tests whether a point is within a lanelet");
  def("length2d", lg::length2d<ConstLanelet>, "calculate length of centerline");
  def("length3d", lg::length3d<ConstLanelet>, "calculate length of centerline in 3d");
  def("distanceToCenterline2d", lg::distanceToCenterline2d<ConstLanelet>);
  def("distanceToCenterline3d", lg::distanceToCenterline3d<ConstLanelet>);
  def("overlaps2d", lg::overlaps2d<ConstLanelet, ConstLanelet>, "Returns true if shared area of two lanelets is >0");
  def("overlaps3d", lg::overlaps3d<ConstLanelet, ConstLanelet>, "Approximates if two lanelets overlap (area  >0) in 3d",
      (arg("lanelet1"), arg("lanelet2"), arg("heightTolerance") = 3.));
  def("intersectCenterlines2d", lg::intersectCenterlines2d<ConstLanelet, ConstLanelet>);

  def("leftOf", lg::leftOf<ConstLanelet, ConstLanelet>, "Returns if first lanelet is directly left of second");
  def("rightOf", lg::rightOf<ConstLanelet, ConstLanelet>, "Returns if first lanelet is directly right of second");
  def("follows", lg::follows<ConstLanelet, ConstLanelet>, "Returns if first lanelet precedes the second");

  wrapFindNearest<Point3d>();
  wrapFindNearest<LineString3d>();
  wrapFindNearest<Lanelet>();
  wrapFindNearest<Area>();
  wrapFindNearest<RegulatoryElementPtr>();
}
