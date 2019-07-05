# Lanelet2 Traffic Rules

This package provides functionality for interpreting the traffic rules in a lanelet map depending on a country and a traffic participant.

It contains functionality to determine right of way, speed limits and legally reachable lanelets for a certain participant.

## Usage

```c++
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
lanelet::traffic_rules::TrafficRulesPtr trafficRulesPtr =
  lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany, lanelet::Participants::Vehicle);
bool passable = trafficRulesPtr->canPass(myLanelet);
lanelet::traffic_rules::SpeedLimitInformation speedLimit = trafficRulesPtr->speedLimit(myLanelet);
```

## Structure

This package offers the abstract `TrafficRules` class that is used as an interface to interpret the data in the map. It provides information whether a lanelet is usable/drivable, where lane changes are possible and what the speed limits are.

Derived from this is the abstract `GenericTrafficRules` class that implements the traffic rules based on the [tagging specification](../lanelet2_core/doc). This class can then be derived to create traffic rules for individual countries and participants. This is especially important for the speed limits and interpreting country specific traffic signs.

After registering this class using `RegisterTrafficRules`, Lanelet2 is able to create instances of this traffic rule using the `TrafficRuleFactory`.

## Hierarchical Structure of Participants

Some classes of participants, especially vehicles, follow a hierarchical structure. A traffic rule class that is registered for an upper level of this class is expected to handle all specializations for this class. For example if a `TrafficRules` implementation is registered for *vehicle*, it can also be instanciated for *vehicle:car*, but not vice versa.

When lanelets/areas have tags for specialized participants, e.g. have "participant:vehicle:car=no", the `TrafficRule` class must handle this appropriately. If these tags are too specialized for an instance, the result should be as conservative as possible. This means that if a `TrafficRule` instance for *vehicle* finds a `participants:vehicle:car=no` and `participants:vehicle:bus=yes`, the lanelet is not passable.

However if it was instanciated for `vehicle:bus` instead of just `vehicle` the lanelet would be reported as passable.
