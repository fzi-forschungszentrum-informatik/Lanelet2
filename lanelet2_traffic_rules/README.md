# Lanelet2 Traffic Rules

This package provides functionality for interpreting the traffic rules in a lanelet map depending on a country and a traffic participant.

It contains functionality to determine right of way, speed limits and legally reachable lanelets for a certain participant.

## Usage

```c++
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
lanelet::TrafficRulesPtr trafficRules = lanelet::TrafficRulesFactors::create(Locations::Germany, Participants::Vehicle);
bool passable = trafficRules.canPass(myLanelet);
lanelet::SpeedLimitInformation speedLimit = trafficRules.speedLimit(myLanelet);
```
