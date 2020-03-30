#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/LineString.h>

namespace lanelet {
namespace examples {
inline LineString3d getLineStringAtX(double x) {
  return LineString3d(utils::getId(), {Point3d{utils::getId(), x, 0, 0}, Point3d{utils::getId(), x, 1, 0},
                                       Point3d{utils::getId(), x, 2, 0}});
}

inline LineString3d getLineStringAtY(double y) {
  return LineString3d(utils::getId(), {Point3d{utils::getId(), 0, y, 0}, Point3d{utils::getId(), 1, y, 0},
                                       Point3d{utils::getId(), 2, y, 0}});
}

inline Polygon3d getAPolygon() {
  Point3d p1{utils::getId(), 0, 0, 0};
  Point3d p2{utils::getId(), 2, 0, 0};
  Point3d p3{utils::getId(), 2, -2, 0};
  return Polygon3d(utils::getId(), {p1, p2, p3});
}

inline Area getAnArea() {
  LineString3d top = examples::getLineStringAtY(2);
  LineString3d right = examples::getLineStringAtX(2).invert();
  LineString3d bottom = examples::getLineStringAtY(0).invert();
  LineString3d left = examples::getLineStringAtY(0);
  return Area(utils::getId(), {top, right, bottom, left});
}

inline Lanelet getALanelet() {
  LineString3d left = examples::getLineStringAtY(2);
  LineString3d right = examples::getLineStringAtY(0);
  return Lanelet(utils::getId(), left, right);
}

inline RegulatoryElementPtr getARegulatoryElement() {
  LineString3d trafficLight = examples::getLineStringAtX(3);
  return TrafficLight::make(utils::getId(), {}, {trafficLight});
}

inline LaneletMap getALaneletMap() {
  auto area = getAnArea();
  auto lanelet = getALanelet();
  lanelet.addRegulatoryElement(getARegulatoryElement());
  return std::move(*utils::createMap({lanelet}, {area}));
}
}  // namespace examples
}  // namespace lanelet
