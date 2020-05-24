#pragma once
#include "lanelet2_core/LaneletMap.h"

namespace lanelet {
namespace test_cases {

using utils::getId;

template <typename T>
std::unordered_map<Id, T> makeMap(const std::initializer_list<T>& vec) {
  std::unordered_map<Id, T> map;
  for (const auto& elem : vec) {
    map.insert(std::make_pair(elem.id(), elem));
  }
  return map;
}

template <>
inline std::unordered_map<Id, RegulatoryElementPtr> makeMap(const std::initializer_list<RegulatoryElementPtr>& vec) {
  std::unordered_map<Id, RegulatoryElementPtr> map;
  for (const auto& elem : vec) {
    map.insert(std::make_pair(elem->id(), elem));
  }
  return map;
}

class LaneletMapTestCase {
 public:
  LaneletMapTestCase() {
    /**
   p1------left-------p2
   |front             |
   p5------p6--other--p7
   |                  | rear
   p3------right------p4


   p8-----outside-----p9

   */
    using namespace std::string_literals;
    p1 = Point3d(getId(), 0., 1., 1.);
    p2 = Point3d(getId(), 1., 1., 1.);
    p3 = Point3d(getId(), 0., 0., 0.);
    p4 = Point3d(getId(), 1., 0., 0.);
    p5 = Point3d(getId(), 0., 0.5, 0.5);
    p6 = Point3d(getId(), 0.5, 0.5, 0.5);
    p7 = Point3d(getId(), 1., 0.5, 0.);
    p8 = Point3d(getId(), 0., -1., 0.);
    p9 = Point3d(getId(), 1., -1., 0.);
    left = LineString3d(getId(), Points3d{p1, p2});
    right = LineString3d(getId(), Points3d{p3, p4});
    front = LineString3d(getId(), Points3d{p3, p1});
    rear = LineString3d(getId(), Points3d{p4, p2});
    other = LineString3d(getId(), Points3d{p5, p6, p7});
    outside = LineString3d(getId(), Points3d{p8, p9});

    ll1 = Lanelet(getId(), left, right);
    ll2 = Lanelet(getId(), other, outside);

    poly1 = Polygon3d(getId(), Points3d{p1, p2, p3, p4});
    RuleParameterMap rules{{"test"s, {ll1}}, {"point"s, {p1, p9}}};

    regelem1 = std::make_shared<GenericRegulatoryElement>(getId(), rules);

    ar1 = Area(getId(), {left, rear.invert(), right.invert(), front});

    map = std::make_shared<LaneletMap>(makeMap({ll1}), makeMap({ar1}), makeMap({regelem1}), makeMap({poly1}),
                                       makeMap({left, right, front, rear}), makeMap({p1, p2, p3, p4}));
  }

  template <typename Test>
  void testConstAndNonConst(Test&& test) {
    test(map);
    LaneletMapConstPtr cMap(map);
    test(cMap);
  }

  Point3d p1, p2, p3, p4, p5, p6, p7, p8, p9;
  LineString3d left, right, front, rear, other, outside;
  Polygon3d poly1;
  RegulatoryElementPtr regelem1;
  Lanelet ll1, ll2;
  Area ar1;
  LaneletMapPtr map;
};

}  // namespace test_cases
}  // namespace lanelet
