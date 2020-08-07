#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Area.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/Area.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_core/primitives/Polygon.h>
#include <lanelet2_core/utility/Units.h>

#include "lanelet2_examples/internal/ExampleHelpers.h"

// we want assert statements to work in release mode
#undef NDEBUG

#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#pragma GCC diagnostic ignored "-Wunused-variable"

// we compare floats a couple of times and know this is save in this context
#pragma GCC diagnostic ignored "-Wfloat-equal"

void part0Primitives();
void part1Points();
void part2LineStrings();
void part3Polygons();
void part4Lanelets();
void part5Areas();
void part6Geometry();

int main() {
  // this tutorial is divided into 6 parts that teach you about all the primitives, one by one and finally how to use
  // them for geometry calculations.
  part0Primitives();
  part1Points();
  part2LineStrings();
  part3Polygons();
  part4Lanelets();
  part5Areas();
  part6Geometry();
  return 0;
}

void part0Primitives() {
  using namespace lanelet;
  using namespace lanelet::units::literals;  // this enables us to use _kmh for velocities
  // there are many concepts that are similar across all primitives of lanelet2. Once you have understood one of
  // them, you can also understand the rest.

  // Here is how you create a point in 3d with an Id and the x, y and z coordinates. We will go into details on points
  // later. The getId() function gives us a unique id so that we can not accidentally have the same id twice. This
  // would have strange effects once we add these points to a LaneletMap. Every lanelet object has such a unique Id.
  Point3d p(utils::getId(), 0, 0, 0);

  // all primitives can easily be copied. Since Lanelet2 data is unique, these copies simply share the same data.
  Point3d pCopy = p;
  assert(p == pCopy);

  // p is mutable, meaning we can change everything we passed at construction time.
  p.z() = 2;
  p.setId(utils::getId());

  // since pCopy shares the data, we modified it as well. This is one of Lanelet2s basic principles to keep data
  // consistent across the map.
  assert(p.id() == pCopy.id());
  assert(p.z() == pCopy.z());

  // all primitives have attributes in form of key-value pairs. These are of course also shared between copies.
  // attributes can be strings, doubles, ids or even velocities (with units). In the background lanelet2 tries to
  // convert them to the respective type and returns it if that was succssful.
  p.attributes()["type"] = "point";
  p.attributes()["pi"] = 3.14;
  p.attributes()["velocity"] = "5 kmh";
  assert(p.attributes() == pCopy.attributes());
  assert(p.attribute("type") == "point");
  assert(!!p.attribute("pi").asDouble());  // returns an optional value that is only valid if conversion was sucessful
  assert(p.attribute("velocity").asVelocity() == 5_kmh);

  // when you are not sure an attribute exists, you can use "attributeOr" for convenience. When the attribute does not
  // exist or can not be converted to the type you passed as default value, the default value is returned:
  assert(p.attributeOr("nonexistent", -1) == -1);
  assert(p.attributeOr("type", 0) == 0);  // "point" can not be converted to a number
  assert(p.attributeOr("velocity", 0_kmh) == 5_kmh);

  // all primitives have a const version, where the data can only be read but not modified. Similar to the copies,
  // they share a view on the actual data. The operation to create them is very cheap, because this actually only
  // copies a shared pointer. The const version for our Point3d is ConstPoint3d.
  ConstPoint3d pConst = p;

  // the const point has the same functions for accessing data, but can not modify it
  assert(pConst.z() == 2);
  // pConst.z() = 3; // impossible

  // still the const version can observe changes:
  p.z() = 3;
  assert(pConst.z() == 3);
  // actually, and this is more advanced, we do not even have to make a copy, because internally Point3d inherits
  // from ConstPoint3d
  ConstPoint3d& pConstRef = p;
  // pConstRef is now a reference to the const part of the point:
  assert(pConstRef.z() == 3);

  // we can convert things to const, but we can not get back. Once it is const, there is no way to get the const away.
  // This is important, because any function that takes a const object is guaranteed to be unable to modify it,
  // no matter what.

  // Point3d pNonConst = pConst; // not possible

  // we can also access the underlying data (this is the thing that the point copies share). However this is not
  // really meant for daily usage:
  assert(p.constData() == pConst.constData());
}

void part1Points() {
  using namespace lanelet;
  // this is a normal point with x=1, y=2, z=3.
  Point3d p3d(utils::getId(), 1, 2, 3);

  // we can access the indices as usual for a point:
  assert(p3d.x() == 1);
  assert(p3d.y() == 2);
  assert(p3d.z() == 3);

  // we can also get access to a BasicPoint which is actually an Eigen point and can be used in time critical
  // computations:
  BasicPoint3d& p3dBasic = p3d.basicPoint();

  // modifying this reference to the basic point also modifies the actual point data:
  p3dBasic.z() = 4;
  assert(p3d.z() == 4);
  // with this eigen point we can do the usual computations:
  BasicPoint3d pTwice = p3d.basicPoint() * 2;
  assert(pTwice.z() == 8);

  // now the interesting part, where we convert to 2d. There is a utility function that generically gives us the
  // matching 2D type for any lanelet primitive:
  Point2d p2d = utils::to2D(p3d);

  // a 2d point works in the same way like a 3d point, but no longer has a z-coordinate. But the important thing is:
  // it still shares the same data as p3d. The conversion is as efficient as any copy in lanelet2, because we just
  // copy the data pointer. If we now modify something in 2d we also modify it in p3d:
  p2d.x() = 3;
  assert(p3d.x() == 3);
  assert(p3d.constData() == p2d.constData());  // this is because they share the same underlying data.

  // the conversion works also from 2d to 3d without losing anything. The z coordinate is always there in the
  // underlying data:
  Point3d p3dNew = utils::to3D(p2d);
  assert(p3dNew.z() == p3d.z());
  assert(p3dNew.constData() == p3d.constData());  // still the same underlying data
}

void part2LineStrings() {
  using namespace lanelet;
  // LineStrings are created from a set of points with linear interpolation between them.
  Point3d p1{utils::getId(), 0, 0, 0};
  Point3d p2{utils::getId(), 1, 0, 0};
  Point3d p3{utils::getId(), 2, 0, 0};
  LineString3d ls(utils::getId(), {p1, p2, p3});

  // conceptually, linestrings are similar to an std::vector. You can access individal points or loop over them.
  assert(ls[1] == p2);
  assert(ls.size() == 3);
  for (Point3d& p : ls) {
    assert(p.y() == 0);
  }
  ls.push_back(Point3d(utils::getId(), 3, 0, 0));
  assert(ls.size() == 4);
  ls.pop_back();
  assert(ls.size() == 3);

  // of course, linestrings also have attributes. This will become interesting if we infer traffic rules. The most
  // common tags and values are predefined so that you don't have to fear typos:
  ls.attributes()[AttributeName::Type] = AttributeValueString::LineThin;
  ls.attributes()[AttributeName::Subtype] = AttributeValueString::Dashed;

  // instead of looping over individual points, you can also loop over individual segments. A segment consists of two
  // neighbouring points in the LineString:
  assert(ls.numSegments() >= 2);
  Segment3d segment = ls.segment(1);
  assert(segment.first == p2);
  assert(segment.second == p3);

  // you can also invert a linestring. Inverted linestrings still share the same data but points are returned in
  // reversed order:
  LineString3d lsInv = ls.invert();
  assert(lsInv.front() == p3);
  assert(lsInv.inverted());  // inverted is the only way to find out we are actually dealing with an inverted ls
  assert(lsInv.constData() == ls.constData());  // still the same data

  // inverting an inverted linestring gives you the original one:
  assert(lsInv != ls);
  LineString3d lsInvInv = lsInv.invert();
  assert(lsInvInv == ls);
  assert(lsInvInv.front() == p1);

  // there is also a const version
  ConstLineString3d lsConst = ls;
  // lsConst.pop_back() // not possible
  ConstPoint3d p1Const = ls[0];  // const linestrings return const points
  assert(lsConst.constData() == ls.constData());

  // and a 2d version. It behaves exactly as the 3d version, but returns 2d points:
  LineString2d ls2d = utils::to2D(ls);
  Point2d front2d = ls2d.front();
  assert(front2d == utils::to2D(p1));

  // there is also a hybrid version. This is a bit special, because it returns BasicPoints (aka Eigen points). This
  // makes it the perfect fit for geometry calculations because they work well with boost.geometry (we will look into
  // that later)
  ConstHybridLineString3d lsHybrid = utils::toHybrid(ls);
  BasicPoint3d p1Basic = lsHybrid[0];
  assert(p1Basic.x() == p1.x());

  // you can also get a BasicLineString, which is just a vector of Eigen points. These are also usable within
  // boost.geometry. This is the only conversion operation where actual copying is done in the background:
  BasicLineString3d lsBasic = ls.basicLineString();
  assert(lsBasic.front() == lsHybrid.front());
}

void part3Polygons() {
  using namespace lanelet;
  // polygons are very, very similar to linestrings. the only actual difference is that there is a last, implicit
  // segment between the last and the first point that closes the polygon. The interface is basically the same:
  Point3d p1{utils::getId(), 0, 0, 0};
  Point3d p2{utils::getId(), 1, 0, 0};
  Point3d p3{utils::getId(), 2, -1, 0};
  Polygon3d poly(utils::getId(), {p1, p2, p3});

  assert(poly.size() == 3);
  assert(poly.numSegments() == 3);  // not 2, see?
  Segment3d lastSegment = poly.segment(2);
  assert(lastSegment.first == p3);
  assert(lastSegment.second == p1);

  // one more difference: you cannot invert polygons. That would not make sense. Polygons are always in clockwise
  // orientation.

  // poly.invert(); // no.

  // there are all the types that you already know from linestrings, const, 2d, 3d, hybrid:
  ConstPolygon3d polyConst = poly;
  ConstHybridPolygon3d polyHybrid = utils::toHybrid(poly);
  ConstPolygon2d poly2dConst = utils::to2D(polyConst);
  assert(polyHybrid.constData() == poly2dConst.constData());
}

void part4Lanelets() {
  using namespace lanelet;
  // Technically, lanelets are not very special, they have a left and a right bound. You already know how to create
  // them, so lets skip that.
  LineString3d left = examples::getLineStringAtY(1);
  LineString3d right = examples::getLineStringAtY(0);

  Lanelet lanelet(utils::getId(), left, right);
  assert(lanelet.leftBound() == left);
  assert(lanelet.rightBound() == right);
  lanelet.setLeftBound(left);  // we can also change that linestring

  // lanelets also have a centerline. By default, the lanelet computes it for you. It is const because it should not
  // be modified.
  ConstLineString3d centerline = lanelet.centerline();

  // the centerline is cached, because computation is not so cheap. When we set a new boundary, the cache is cleared
  ConstLineString3d centerline2 = lanelet.centerline();  // from the cache
  assert(centerline2 == centerline);
  lanelet.setLeftBound(examples::getLineStringAtY(2));
  ConstLineString3d centerline3 = lanelet.centerline();
  assert(centerline3 != centerline);  // new centerline

  // however there is one limitation. If you modify one of the bounds, the lanelet does not notice it. You have to
  // reset the cache yourself.
  right.push_back(Point3d(utils::getId(), 4, 0, 0));
  assert(centerline3 == lanelet.centerline());  // centerline is still the same, which is wrong.
  lanelet.resetCache();
  assert(centerline3 != lanelet.centerline());  // new centerline is computed
  right.pop_back();
  lanelet.resetCache();

  // lanelets can also be inverted. This is used to deal with lanelets that are drivable in both ways. Inverted
  // lanelets have left and right bounds flipped and inverted. Shared data is still the same:
  Lanelet laneletInv = lanelet.invert();
  assert(laneletInv.leftBound().front() == lanelet.rightBound().back());
  assert(laneletInv.constData() == lanelet.constData());

  // when you deal with lanelets geometrically, you need their outline (a polygon). You haven't met the
  // CompoundPolygon type yet, but it behaves like a single polygon while it is actually composed of multiple
  // linestrings that together form the bound (in clockwise order again).
  CompoundPolygon3d polygon = lanelet.polygon3d();
  assert(polygon.size() == 6);                             // both boundaries have 3 points
  assert(polygon[0] == lanelet.leftBound().front());       // the polygon starts at the first point of the left bound
  assert(polygon.back() == lanelet.rightBound().front());  // and ends at the start of the right bound

  // there is also a const version, but no 3d or 2d version (lanelets are "dimensionless").
  ConstLanelet laneletConst = lanelet;
  assert(laneletConst.constData() == lanelet.constData());
  ConstLineString3d leftConst = lanelet.leftBound();  // the bounds are now const as well
  // laneletConst.setLeftBound(left); // no

  // lanelets hold regulatory elements but we will get to that in the next example
  assert(lanelet.regulatoryElements().empty());
}

void part5Areas() {
  using namespace lanelet;
  // areas are similar to lanelets in their design. But instead of left and right bound they have set of lineStrings
  // that are connected and together form the outer bound. They must be in clockwise order
  LineString3d top = examples::getLineStringAtY(2);
  LineString3d right = examples::getLineStringAtX(2).invert();
  LineString3d bottom = examples::getLineStringAtY(0).invert();
  LineString3d left = examples::getLineStringAtY(0);
  Area area(utils::getId(), {top, right, bottom, left});

  // you can get the outer bounds
  LineStrings3d outer = area.outerBound();
  // or set them
  area.setOuterBound(outer);

  // you can also get a polygon for the outer bounds, similar to a lanelet:
  CompoundPolygon3d outerPolygon = area.outerBoundPolygon();

  // areas can also have holes (they work similar to the outer bounds but are in counter-clockwise order). Hovewer
  // they are rarely used.
  assert(area.innerBounds().empty());

  // areas can be const
  ConstArea areaConst = area;
  ConstLineStrings3d outerConst = areaConst.outerBound();  // now the outer bound linestrings are also const

  // areas also hold regulatory elements
  assert(area.regulatoryElements().empty());
}

void part6Geometry() {
  using namespace lanelet;
  // lanelet2 allows lots of geometry calculations with all kinds of primitives. This tutorial only shows some of them
  // to show you the basic idea. There are many more algorithms that you can find in the geometry folder in
  // lanelet2_core.

  // the primitives we are going to work with
  Point3d point(utils::getId(), 1, 4, 1);
  LineString3d ls = examples::getLineStringAtY(2);  // linestring that goes from (0,2,0) to (2,2,0)
  Polygon3d poly = examples::getAPolygon();         // polygon with points (0,0,0), (2,0,0), (2, -2, 0)
  Lanelet lanelet = examples::getALanelet();        // lanelet that goes from x=0 to x=3 and extends from y=2 to y=0
  Area area = examples::getAnArea();                // quadratic area with the edge points (0,0,0) and (2,2,0)

  // most computations with linestrings and polygons are better done with ther hybrid types (see the doc for more info
  // on this).
  ConstHybridLineString3d lsHybrid = utils::toHybrid(ls);
  ConstHybridPolygon3d polyHybrid = utils::toHybrid(poly);

  // the points, linestrings and polygons directly interface with boost.geometry:
  auto dP2Line3d = geometry::distance(point, lsHybrid);
  // we used 3d objects so the result will also be in 3d.
  assert(dP2Line3d > 2);
  // to calculate in 2d, transform the objects to 2d
  auto dP2Line2d = geometry::distance(utils::to2D(point), utils::to2D(lsHybrid));
  assert(dP2Line2d == 2);

  // other algorithms would be the length or the area
  auto l3d = geometry::length(lsHybrid);
  assert(l3d == 2);
  auto ar = geometry::area(utils::to2D(polyHybrid));  // not defined in 3d
  assert(ar == 2);

  // lanelet2 provides more algorithms where boost.geometry lacks inplementations. For those implementations you don't
  // have to use the hybrid type:
  BasicPoint3d pProj = geometry::project(ls, point);  // gets the projecion of point on ls
  assert(pProj.y() == 2);

  ArcCoordinates arcCoordinates =
      geometry::toArcCoordinates(utils::to2D(ls), utils::to2D(point));  // transforms the point into arc coordinates
  assert(arcCoordinates.distance == 2);                                 // signed distance to linestring
  assert(arcCoordinates.length == 1);                                   // length along linestring

  // bounding boxes can be calculated for all types:
  BoundingBox3d pointBox = geometry::boundingBox3d(point);  // trivial box for points
  BoundingBox3d lsBox = geometry::boundingBox3d(ls);
  BoundingBox2d laneletBox = geometry::boundingBox2d(lanelet);
  BoundingBox2d areaBox = geometry::boundingBox2d(area);

  // we can now use these boxes for efficient intersection estimation. Intersects returns true also if the boxes only
  // touch but the shared area is still 0.
  assert(geometry::intersects(laneletBox, areaBox));
  assert(!geometry::intersects(pointBox, lsBox));

  // if you want more than an estimation you can use the primitives directly. Overlaps only returns true if the shared
  // area of the two primitives is >0. In 3d, the distance in z must also be smaller than a margin.
  assert(geometry::overlaps3d(area, lanelet, 3));
}
