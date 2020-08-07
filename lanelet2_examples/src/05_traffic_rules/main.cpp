#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/utility/Units.h>
#include <lanelet2_traffic_rules/TrafficRules.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include "lanelet2_examples/internal/ExampleHelpers.h"

// we want assert statements to work in release mode
#undef NDEBUG

void part1UsingTrafficRules();

int main() {
  // this tutorial shows how to use traffic rule objects to interpret map data
  part1UsingTrafficRules();
  return 0;
}

void part1UsingTrafficRules() {
  using namespace lanelet;
  using namespace lanelet::units::literals;

  // first lets construct a sequence of lanelets. left is left of right, next follows right.
  LineString3d leftLs{examples::getLineStringAtY(3)};
  LineString3d middleLs{examples::getLineStringAtY(2)};
  LineString3d rightLs{examples::getLineStringAtY(0)};
  LineString3d nextLeftLs{utils::getId(),
                          {middleLs.back(), Point3d(utils::getId(), middleLs.back().x() + 1., middleLs.back().y())}};
  LineString3d nextRightLs{utils::getId(),
                           {rightLs.back(), Point3d(utils::getId(), rightLs.back().x() + 1, rightLs.back().y())}};
  Lanelet left{utils::getId(), leftLs, middleLs};
  Lanelet right{utils::getId(), middleLs, rightLs};
  Lanelet next{utils::getId(), nextLeftLs, nextRightLs};

  // to get information where we can drive, we need a traffic rule object o do the interpretation for us.
  // we use the traffic rules factory to get an traffic rules object that interprets the lanelets from the perspective
  // of a german vehicle.
  traffic_rules::TrafficRulesPtr trafficRules =
      traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Vehicle);
  traffic_rules::TrafficRulesPtr pedestrianRules =
      traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Pedestrian);

  // now we can ask the traffic rules specific things about the lanelet. For now, we have not added any tags, so we
  // get a default interpretation of the lanelet:
  assert(trafficRules->canPass(right));      // lanelets are by default "road" lanelets
  assert(!pedestrianRules->canPass(right));  // and not passable for pedestrians.
  // by default, lanelets are one-directional, so inverted lanelets are also not passable
  assert(!trafficRules->canPass(right.invert()));

  // also: no tags, no lane changes
  assert(!trafficRules->canPass(right, left));

  // we can also query the speed limit. Without tags we get the speed limit for urban roads
  traffic_rules::SpeedLimitInformation limit = trafficRules->speedLimit(right);
  assert(limit.speedLimit == 50_kmh);
  assert(limit.isMandatory);  // mandatory means we must not exceed the speed limit

  // now lets start to add some tags to get more meaningful results:
  middleLs.attributes()[AttributeName::Type] = AttributeValueString::LineThin;
  middleLs.attributes()[AttributeName::Subtype] = AttributeValueString::Dashed;
  right.attributes()[AttributeName::Type] = AttributeValueString::Lanelet;
  right.attributes()[AttributeName::Subtype] = AttributeValueString::Road;
  right.attributes()[AttributeName::Location] = AttributeValueString::Nonurban;
  left.attributes() = right.attributes();
  next.attributes() = right.attributes();
  left.attributes()[AttributeName::OneWay] = false;

  // now we can see that lane changes are possible
  assert(trafficRules->canChangeLane(right, left));
  assert(trafficRules->canChangeLane(left, right));

  // and left is no drivable in inverted direction:
  assert(trafficRules->canPass(left.invert()));
  // but not do lane change to right (because right is not drivable in inverted direction)
  assert(!trafficRules->canChangeLane(left, right.invert()));
  assert(!trafficRules->canChangeLane(left.invert(), right.invert()));

  // also the reported speed limit is different (because lanelets are now nonurban)
  limit = trafficRules->speedLimit(right);
  assert(limit.speedLimit == 100_kmh);  // on german nonurban roads

  // if we now add a speed limit regulatory element, the speed limit changes
  LineString3d sign = examples::getLineStringAtX(3);
  SpeedLimit::Ptr speedLimit =
      SpeedLimit::make(utils::getId(), {}, {{sign}, "de274-60"});  // id of a speed limit 60 sign in germany
  right.addRegulatoryElement(speedLimit);
  assert(trafficRules->speedLimit(right).speedLimit == 60_kmh);

  // if the type of the lanelet is changed from road to walkway, it is no longer drivable for vehicles:
  right.attributes()[AttributeName::Subtype] = AttributeValueString::Crosswalk;
  assert(!trafficRules->canPass(right));

  // but instead, it is passable for pedestrians
  assert(pedestrianRules->canPass(right));
}
