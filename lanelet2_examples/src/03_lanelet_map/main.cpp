#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include "lanelet2_examples/internal/ExampleHelpers.h"

#pragma GCC diagnostic ignored "-Wunused-variable"

// we want assert statements to work in release mode
#undef NDEBUG

void part1AboutLaneletMaps();
void part2CreatingLaneletMaps();
void part3QueryingInformation();
void part4LaneletSubmaps();

int main() {
  // this tutorial shows you what LaneletMaps are and how they are supposed to be used.
  part1AboutLaneletMaps();
  part2CreatingLaneletMaps();
  part3QueryingInformation();
  part4LaneletSubmaps();
  return 0;
}

void part1AboutLaneletMaps() {
  using namespace lanelet;
  // lanelet maps are actually simply a container for all of lanelets primitives. They are the core object of
  // exchanging whole maps or parts of it. For that, they offer different ways of querying data (we will get to that).

  LaneletMap map = examples::getALaneletMap();  // we talk about creation later

  // Lanelet maps are arranged into layers, one for each primitive type:
  PointLayer& points = map.pointLayer;
  LineStringLayer& linestrings = map.lineStringLayer;

  // every layer behaves similar to an unordered map: we can iterate over the primitives or look them up by their id:
  assert(points.size() > 1);
  Point3d aPoint = *points.begin();
  Point3d samePoint = *points.find(aPoint.id());
  assert(samePoint == aPoint);
  assert(points.exists(aPoint.id()));

  // we can do the same thing for the other layers, but it is completely the same thing
  assert(!linestrings.empty());

  // maps can not be copied (they are too big for that), but they can be moved:
  LaneletMap newMap = std::move(map);
  // map.exists(aPoint.id()); // map is no longer valid after move!
  assert(newMap.pointLayer.exists(aPoint.id()));

  // or be passed around as a pointer:
  LaneletMapUPtr mapPtr = std::make_unique<LaneletMap>(std::move(newMap));

  // there is also the concept of constness for lanelet maps: const maps return const primitives.
  const LaneletMap& constMap = *mapPtr;
  ConstPoint3d aConstPoint = *constMap.pointLayer.begin();
  assert(aConstPoint == aPoint);
}

void part2CreatingLaneletMaps() {
  using namespace lanelet;
  // Maps can be created from in two ways: either by adding data to them one by one, or by creating them directly from
  // multiple objects:
  LaneletMap laneletMap;

  Lanelet lanelet = examples::getALanelet();
  laneletMap.add(lanelet);
  assert(laneletMap.laneletLayer.size() == 1);

  // when you add an object, all the things that belong to it (points, linestrings, regulatory elements) are added as
  // well:
  assert(!laneletMap.pointLayer.empty());
  assert(laneletMap.pointLayer.exists(lanelet.leftBound().front().id()));

  // we have not introduced InvalId (=0) yet. This id can be used to indicate that a primitive is not part of a map
  // yet. When you add objects with InvalId, LaneletMap will automatically assign an id for them.
  Point3d invalPoint1(InvalId, 0, 0, 0);
  Point3d invalPoint2(InvalId, 1, 0, 0);
  LineString3d invalLs(InvalId, {invalPoint1, invalPoint2});
  laneletMap.add(invalLs);
  assert(invalPoint1.id() != InvalId);

  // however if you want to create a new map from lots of primitives, the add function is not the right choice. The
  // underlying trees have to be updated (and potentially rebalanced) each time something is added. For efficiently
  // creating a map from multiple objects, you can use the createMap utility function:
  Lanelets lanelets{examples::getALanelet(), examples::getALanelet(), examples::getALanelet()};
  LaneletMapUPtr laneletsMap = utils::createMap(lanelets);
  assert(laneletsMap->laneletLayer.exists(lanelets.front().id()));
}

void part3QueryingInformation() {
  using namespace lanelet;
  // apart from getting primitives by their ID, a lanelet map offers two more ways of querying data: By their
  // relation and by their position.
  LaneletMap laneletMap = examples::getALaneletMap();
  Lanelet mapLanelet = *laneletMap.laneletLayer.begin();
  TrafficLight::Ptr trafficLight = mapLanelet.regulatoryElementsAs<TrafficLight>().front();

  // by their relation means we can query elements based on the things that they own. Linestrings by their points,
  // lanelets and areas by their linestrings or regulatory elements, regulatory elements by anything of the above:
  // find lanelets with linestrings
  auto laneletsOwningLinestring = laneletMap.laneletLayer.findUsages(mapLanelet.leftBound());
  assert(laneletsOwningLinestring.size() == 1 && laneletsOwningLinestring.front() == mapLanelet);
  // find regelems with linestrings
  auto regelemsOwningLs =
      laneletMap.regulatoryElementLayer.findUsages(*trafficLight->trafficLights().front().lineString());
  assert(regelemsOwningLs.size() == 1 && regelemsOwningLs.front() == trafficLight);
  // find lanelets with regelems
  auto laneletsOwningRegelems = laneletMap.laneletLayer.findUsages(trafficLight);
  assert(!laneletsOwningRegelems.empty());

  // spacially means we can find primitives with geometrical queries. Because internally all primitives are stored as
  // bounding boxes, these queries only return the primitive with respect to their *bounding box*.
  Lanelets lanelets = laneletMap.laneletLayer.nearest(BasicPoint2d(0, 0), 1);  // 1 means the nearest "1" lanelets
  assert(!lanelets.empty());  // lanelets holds the lanelet with the closest bounding box

  // to get the actually closest lanelets use this utility function:
  std::vector<std::pair<double, Lanelet>> actuallyNearestLanelets =
      geometry::findNearest(laneletMap.laneletLayer, BasicPoint2d(0, 0), 1);
  assert(!actuallyNearestLanelets.empty());

  // finally we can get primitives using a search region (this also runs on the bounding boxes):
  Lanelets inRegion = laneletMap.laneletLayer.search(BoundingBox2d(BasicPoint2d(0, 0), BasicPoint2d(10, 10)));
  assert(!inRegion.empty());  // inRegion contains all lanelets whose bounding boxes intersect with the query

  // for advanced usage, there are the searchUntil and nearestUntil functions. You pass it a function that is called
  // with primitives with increasing bounding box distance until the function returns true. This is then the returned
  // primitive:

  // in this example we get the first lanelet whose bounding box distance is >3m distance to the query point
  BasicPoint2d searchPoint = BasicPoint2d(10, 10);
  // the search func is called with the bounding box of a primitive and the primitive itself
  auto searchFunc = [&searchPoint](const BoundingBox2d& lltBox, const Lanelet& /*llt*/) {
    return geometry::distance(searchPoint, lltBox) > 3;
  };
  Optional<Lanelet> lanelet = laneletMap.laneletLayer.nearestUntil(searchPoint, searchFunc);
  assert(!!lanelet && geometry::distance(geometry::boundingBox2d(*lanelet), searchPoint) > 3);
}

void part4LaneletSubmaps() {
  using namespace lanelet;
  // While LaneletMap has the property that when an element is added, all the things referenced by it are added as well,
  // LaneletSubmap does not have this property. This can be useful if you want to avoid that if you add a Lanelet, all
  // Lanelets referenced by its RegulatoryElements are added as well. Apart from that, basically everything you can do
  // with a LaneletMap can also be done with a LaneletSubmap:
  LaneletSubmap submap{examples::getALaneletMap()};  // it can be constructed (moved) from an existing map

  // you can search its layers
  Lanelets inRegion = submap.laneletLayer.search(BoundingBox2d(BasicPoint2d(0, 0), BasicPoint2d(10, 10)));

  // you can create new submaps from some elements
  LaneletSubmapUPtr newSubmap = utils::createSubmap(inRegion);

  // but this submap will not contain any elements except for the ones you explicitly added
  assert(newSubmap->pointLayer.empty());
  assert(newSubmap->size() == inRegion.size());

  // ... unless you convert back into a laneletMap. This will again contain all primitives:
  LaneletMapUPtr newMap = newSubmap->laneletMap();
  assert(!newMap->pointLayer.empty());
}
