#include <lanelet2_io/Io.h>
#include <lanelet2_matching/LaneletMatching.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_traffic_rules/TrafficRules.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

// we want assert statements to work in release mode
#undef NDEBUG

namespace {
std::string exampleMapPath = std::string(PKG_DIR) + "/../lanelet2_maps/res/mapping_example.osm";

template <typename NumberT>
void assertNear(NumberT number1, NumberT number2, NumberT maxDifference) {
  assert(std::abs(number1 - number2) < maxDifference);
}

template <typename MatchVectorT>
std::string toString(const MatchVectorT& matchVector) {
  std::string out;
  for (auto& match : matchVector) {
    out += std::to_string(match.lanelet.id());
    if (match.lanelet.inverted()) {
      out += "inv";
    }
    out += " ";
  }
  return out;
}

using namespace lanelet;
BasicPolygon2d absoluteHull(const BasicPolygon2d& relativeHull, const matching::Pose2d& pose) {
  BasicPolygon2d hullPoints;
  hullPoints.reserve(relativeHull.size());
  for (const auto& hullPt : relativeHull) {
    hullPoints.push_back(pose * hullPt);
  }
  return hullPoints;
}

struct MapAndObject {
  LaneletMapPtr map;
  matching::ObjectWithCovariance2d obj;
};

MapAndObject getSampleMapAndObject() {
  // load the map
  projection::UtmProjector projector{Origin({49, 8.4})};
  LaneletMapPtr map = load(exampleMapPath, projector);

  // create an object
  matching::ObjectWithCovariance2d obj;
  obj.pose.translation() = map->pointLayer.get(41656).basicPoint2d();
  obj.pose.linear() = Eigen::Rotation2D<double>(150. / 180. * M_PI).matrix();
  obj.absoluteHull = absoluteHull(
      matching::Hull2d{BasicPoint2d{-1, -0.9}, BasicPoint2d{2, -0.9}, BasicPoint2d{2, 0.9}, BasicPoint2d{1, 0.9}},
      obj.pose);
  obj.positionCovariance = matching::PositionCovariance2d::Identity() * 2.;
  obj.vonMisesKappa = 1. / (10. / 180. * M_PI);  // covariance of 10 degrees

  return MapAndObject{map, obj};
}
}  // namespace

void part1testSetup();
void part2deterministicMatching();
void part3probabilisticMatching();
void part4matchingUtils();

int main() {
  // this tutorial shows you how to use the matching module.
  part1testSetup();
  part2deterministicMatching();
  part3probabilisticMatching();
  part4matchingUtils();
  return 0;
}

void part1testSetup() {
  using namespace lanelet;
  MapAndObject mao = getSampleMapAndObject();
  LaneletMapPtr map = mao.map;
  matching::ObjectWithCovariance2d obj = mao.obj;
  assert(map->laneletLayer.exists(42440));
}

void part2deterministicMatching() {
  using namespace lanelet;

  MapAndObject mao = getSampleMapAndObject();
  LaneletMapPtr map = mao.map;
  matching::ObjectWithCovariance2d obj = mao.obj;

  // get matches based on a deterministic assignment
  auto matches = matching::getDeterministicMatches(*map, obj, 4.);

  // matches are ordered by distance
  for (size_t i = 1; i < matches.size(); i++) {
    assert(matches.at(i).distance >= matches.at(i - 1).distance);
  }

  // some checks that the found matches are correct
  assert(14ul == matches.size());
  assertNear(0.69, matches.at(8).distance, 0.1);
  assert(45330 == matches.at(8).lanelet.id());
  assertNear(0.69, matches.at(9).distance, 0.1);
  assert(45330 == matches.at(9).lanelet.id());

  // filter non-rule-compliant matches by providing traffic rules
  traffic_rules::TrafficRulesPtr trafficRulesPtr =
      traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Vehicle);
  auto compliantMatches = matching::removeNonRuleCompliantMatches(matches, trafficRulesPtr);
  assert(8ul == compliantMatches.size());  // see part3 for details on the compliant matches
}

void part3probabilisticMatching() {
  using namespace lanelet;

  MapAndObject mao = getSampleMapAndObject();
  LaneletMapPtr map = mao.map;
  matching::ObjectWithCovariance2d obj = mao.obj;

  // get matches based on a probabilistic assignment
  auto matches = matching::getProbabilisticMatches(*map, obj, 4.);

  // matches are ordered by distance
  // NOLINTNEXTLINE(clang-analyzer-cplusplus.NewDelete)
  for (size_t i = 1; i < matches.size(); i++) {
    assert(matches.at(i).mahalanobisDistSq >= matches.at(i - 1).mahalanobisDistSq);
  }

  // some checks that the found matches are correct
  assertNear(0.288177, matches.at(0).mahalanobisDistSq, 0.001);
  assert(45334 == matches.at(0).lanelet.id());
  assert(!matches.at(0).lanelet.inverted());  // best match must be non inverted 45334
  assert(14ul == matches.size());

  assert(14ul == matches.size());
  assert("45334 45356inv 45358inv 45328inv 45332 45344 45330 45344inv 45330inv 45332inv 45328 45356 45358 45334inv " ==
         toString(matches));

  // filter non-rule-compliant matches by providing traffic rules
  traffic_rules::TrafficRulesPtr trafficRulesPtr =
      traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Vehicle);
  auto compliantMatches = matching::removeNonRuleCompliantMatches(matches, trafficRulesPtr);
  assert(8ul ==
         compliantMatches
             .size());  // see list below: should exclude zebra crossing (pedestrian only) and inverted one way lanelets
  assert("45334 45356inv 45358inv 45332 45330 45328 45356 45358 " == toString(compliantMatches));
}

void part4matchingUtils() {
  using namespace lanelet;
  using namespace lanelet::matching;

  MapAndObject mao = getSampleMapAndObject();
  LaneletMapPtr map = mao.map;
  matching::ObjectWithCovariance2d obj = mao.obj;

  LaneletLayer emptyLayer;

  // check whether an object is close to any element of a layer
  assert(isCloseTo(map->laneletLayer, obj, 4.));
  assert(!isCloseTo(emptyLayer, obj, 4.));

  // check whether an object is within any element of a layer
  assert(isWithin(map->laneletLayer, obj));
  assert(!isWithin(emptyLayer, obj));

  // check that this also works based on the position if no hull is provided
  Object2d objWithEmptyHull;
  objWithEmptyHull.pose.translation() = map->pointLayer.get(41656).basicPoint2d();
  assert(matching::isCloseTo(map->laneletLayer, objWithEmptyHull, 1.));
}
