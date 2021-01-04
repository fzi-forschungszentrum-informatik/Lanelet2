# Lanelet2 Matching

[![Build Status](https://api.travis-ci.org/coincar-sim/lanelet2_matching.svg)](https://travis-ci.org/coincar-sim/lanelet2_matching)

The matching module for [lanelet2](https://github.com/fzi-forschungszentrum-informatik/Lanelet2). This module provides functions to determine in which lanelet an object is/could be currently located.

## Matching functions

* **Deterministic matching**: Find all lanelets to which an object has less than a certain Euclidean distance.
  * as lanelets have an orientation, every lanelet is considered twice: regular and inverted, but with the same distance
* **Probabilistic matching**: Compute the squared Mahalanobis distance of the object pose to the lanelet to reason about the probability of that match, as suggested by Petrich et al. ([DOI:0.1109/ITSC.2013.6728549](https://doi.org/10.1109/ITSC.2013.6728549))
  * this distance includes the orientation, thus every lanelet is considered twice: regular and inverted, but with different distances
* **Accounting for traffic rules**: In case you expect traffic rule compliant behavior, you can remove non compliant matches by providing a traffic rules element for the participant you were matching (such as pedestrian lanelets for vehicles or inverted one way lanelets, which means driving in the wrong direction).

## Usage

### C++

```cpp
#include "LaneletMatching.h"

// create objects to match
lanelet::matching::Object2d obj; // deterministic
lanelet::matching::ObjectWithCovariance2d obj2; // considering uncertainty

// retrieve lanelet matches from map
auto detMatches = getDeterministicMatches(laneletMap, obj, 4.); // max distance = 4m
auto probMatches = getProbabilisticMatches(laneletMap, obj2, 4.); // max distance = 4m

// remove non-compliant matches (such as driving in the wrong direction)
auto compliantDetMatches = removeNonRuleCompliantMatches(detMatches, trafficRulesPtr);
auto compliantProbMatches = removeNonRuleCompliantMatches(probMatches, trafficRulesPtr);
```

have a look at the C++ unittests for more examples

### Python

```python
import lanelet2
import lanelet2_matching

# create objects to match
obj = lanelet2_matching.Object2d()

# retrieve lanelet matches from map
matches = lanelet2_matching.getDeterministicMatches(lanelet_map, obj, 4.)  # max distance = 4m

# remove non-compliant matches (such as driving in the wrong direction)
compliant_matches = lanelet2_matching.removeNonRuleCompliantMatches(matches, traffic_rules)

```

have a look at the python unittests for more examples, also supports uncertainty


## License
This package is distributed under the 3-Clause BSD License, see [LICENSE](LICENSE).
