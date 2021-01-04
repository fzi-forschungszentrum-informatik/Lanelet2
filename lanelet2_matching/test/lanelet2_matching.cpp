/*
 * Copyright (c) 2019
 * FZI Forschungszentrum Informatik, Karlsruhe, Germany (www.fzi.de)
 * KIT, Institute of Measurement and Control, Karlsruhe, Germany (www.mrt.kit.edu)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_traffic_rules/TrafficRules.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include "LaneletMatching.h"
#include "gtest/gtest.h"

using namespace lanelet;

namespace {
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

inline BasicPolygon2d absoluteHull(const BasicPolygon2d& relativeHull, const matching::Pose2d& pose) {
  BasicPolygon2d hullPoints;
  hullPoints.reserve(relativeHull.size());
  for (const auto& hullPt : relativeHull) {
    hullPoints.push_back(pose * hullPt);
  }
  return hullPoints;
}
}  // namespace

class MatchingUtils : public ::testing::Test {
 public:
  MatchingUtils() { map = load(exampleMapPath, projector); }
  std::string exampleMapPath = std::string(PKG_DIR) + "/test/mapping_example.osm";
  projection::UtmProjector projector{Origin({49, 8.4})};
  LaneletMapPtr map;
};

TEST_F(MatchingUtils, fixtureSetupSuccessful) {  // NOLINT
  EXPECT_TRUE(map->laneletLayer.exists(42440));
}

TEST_F(MatchingUtils, absoluteHull) {  // NOLINT
  matching::Object2d obj;

  obj.pose.translation() = BasicPoint2d{10, 0};                         //!< at point x=10 y=0
  obj.pose.linear() = Eigen::Rotation2D<double>(-1.5 * M_PI).matrix();  //!< rotated by pi/2 (=-3pi/2)

  obj.absoluteHull = absoluteHull(matching::Hull2d{BasicPoint2d{-0.5, -1}, BasicPoint2d{2, 1}}, obj.pose);

  EXPECT_DOUBLE_EQ(10 + 1, obj.absoluteHull.at(0).x());
  EXPECT_DOUBLE_EQ(0 - 0.5, obj.absoluteHull.at(0).y());

  EXPECT_DOUBLE_EQ(10 - 1, obj.absoluteHull.at(1).x());
  EXPECT_DOUBLE_EQ(0 + 2, obj.absoluteHull.at(1).y());
}

TEST_F(MatchingUtils, findWithin) {  // NOLINT
  ASSERT_TRUE(map->pointLayer.exists(41656));
  matching::Object2d obj;
  obj.pose.translation() = map->pointLayer.get(41656).basicPoint2d();
  obj.pose.linear() = Eigen::Rotation2D<double>(150. / 180. * M_PI).matrix();

  EXPECT_EQ(4ul, matching::utils::findWithin(map->laneletLayer, obj, 0.1).size());

  obj.absoluteHull = absoluteHull(
      matching::Hull2d{BasicPoint2d{-1, -0.9}, BasicPoint2d{2, -0.9}, BasicPoint2d{2, 0.9}, BasicPoint2d{1, 0.9}},
      obj.pose);

  std::vector<std::pair<double, Lanelet>> laneletsWithDistance =
      matching::utils::findWithin(map->laneletLayer, obj, 0.1);

  EXPECT_EQ(4ul, laneletsWithDistance.size());

  Ids laneletIds;
  std::transform(laneletsWithDistance.begin(), laneletsWithDistance.end(), std::back_inserter(laneletIds),
                 [](const auto& elem) { return elem.second.id(); });

  EXPECT_TRUE(std::find(std::begin(laneletIds), std::end(laneletIds), 45356) != std::end(laneletIds))
      << "Lanelet before the roundabout";
  EXPECT_TRUE(std::find(std::begin(laneletIds), std::end(laneletIds), 45328) != std::end(laneletIds))
      << "Lanelet leaving the roundabout";
  EXPECT_TRUE(std::find(std::begin(laneletIds), std::end(laneletIds), 45334) != std::end(laneletIds))
      << "Lanelet entering the roundabout";
  EXPECT_TRUE(std::find(std::begin(laneletIds), std::end(laneletIds), 45344) != std::end(laneletIds))
      << "Zebra crossing before the roundabout";
  EXPECT_FALSE(std::find(std::begin(laneletIds), std::end(laneletIds), 45332) != std::end(laneletIds))
      << "Lanelet inside the roundabout";
}

TEST_F(MatchingUtils, getMahalanobisDistSq) {  // NOLINT
  ASSERT_TRUE(map->pointLayer.exists(41656));
  matching::ObjectWithCovariance2d obj;
  obj.pose.translation() = map->pointLayer.get(41656).basicPoint2d();
  obj.pose.linear() = Eigen::Rotation2D<double>(150. / 180. * M_PI).matrix();
  obj.absoluteHull = absoluteHull(
      matching::Hull2d{BasicPoint2d{-1, -0.9}, BasicPoint2d{2, -0.9}, BasicPoint2d{2, 0.9}, BasicPoint2d{1, 0.9}},
      obj.pose);

  using namespace matching::utils;
  obj.vonMisesKappa = 0.5;
  EXPECT_THROW(getMahalanobisDistSq(map->laneletLayer.get(45356), obj), MatchingError)  // NOLINT
      << "should throw on covariance = zero";

  obj.positionCovariance = matching::PositionCovariance2d::Ones();
  EXPECT_THROW(getMahalanobisDistSq(map->laneletLayer.get(45356), obj), MatchingError)  // NOLINT
      << "should throw on determinant = zero";

  obj.positionCovariance = matching::PositionCovariance2d::Identity() * 2.;
  obj.vonMisesKappa = 1. / (10. / 180. * M_PI);  // covariance of 10 degrees

  double mahaDist45334 = getMahalanobisDistSq(map->laneletLayer.get(45334), obj);
  EXPECT_NEAR(0.28817716004330124, mahaDist45334, 10e-6) << "Lanelet entering the roundabout";

  double mahaDist45356 = getMahalanobisDistSq(map->laneletLayer.get(45356).invert(), obj);
  EXPECT_NEAR(3.2487924075660697, mahaDist45356, 10e-6) << "Lanelet before the roundabout";

  double mahaDist45328 = getMahalanobisDistSq(map->laneletLayer.get(45328).invert(), obj);
  EXPECT_NEAR(11.919014409926168, mahaDist45328, 10e-6) << "Lanelet leaving the roundabout";

  double mahaDist45344 = getMahalanobisDistSq(map->laneletLayer.get(45344), obj);
  EXPECT_NEAR(57.419088438833477, mahaDist45344, 10e-6) << "Zebra crossing before the roundabout";

  double mahaDist45332 = getMahalanobisDistSq(map->laneletLayer.get(45332), obj);
  EXPECT_NEAR(47.481075513744784, mahaDist45332, 10e-6) << "Lanelet inside the roundabout";
}

class Matching : public MatchingUtils {
 public:
  Matching() {
    obj.pose.translation() = map->pointLayer.get(41656).basicPoint2d();
    obj.pose.linear() = Eigen::Rotation2D<double>(150. / 180. * M_PI).matrix();
    obj.absoluteHull = absoluteHull(
        matching::Hull2d{BasicPoint2d{-1, -0.9}, BasicPoint2d{2, -0.9}, BasicPoint2d{2, 0.9}, BasicPoint2d{1, 0.9}},
        obj.pose);
    obj.positionCovariance = matching::PositionCovariance2d::Identity() * 2.;
    obj.vonMisesKappa = 1. / (10. / 180. * M_PI);  // covariance of 10 degrees
  }
  matching::ObjectWithCovariance2d obj;
};

TEST_F(Matching, deterministicNonConst) {  // NOLINT
  using namespace lanelet::matching;
  auto matches = getDeterministicMatches(*map, obj, 4.);
  for (size_t i = 1; i < matches.size(); i++) {
    EXPECT_TRUE(matches.at(i).distance >= matches.at(i - 1).distance)
        << "Not sorted: at i=" << i - 1 << " dist = " << matches.at(i - 1).distance << "at i=" << i
        << " dist = " << matches.at(i).distance;
  }
  EXPECT_EQ(14ul, matches.size());
  EXPECT_NEAR(0.69, matches.at(8).distance, 0.1);
  EXPECT_EQ(45330, matches.at(8).lanelet.id());
  EXPECT_NEAR(0.69, matches.at(9).distance, 0.1);
  EXPECT_EQ(45330, matches.at(9).lanelet.id());
}

TEST_F(Matching, deterministicConst) {  // NOLINT
  using namespace lanelet::matching;
  const LaneletMap& constMap = *map;
  auto matches = getDeterministicMatches(constMap, obj, 4.);
  EXPECT_EQ(14ul, matches.size());
}

TEST_F(Matching, probabilisticNonConst) {  // NOLINT
  using namespace lanelet::matching;
  auto matches = getProbabilisticMatches(*map, obj, 4.);
  // NOLINTNEXTLINE(clang-analyzer-cplusplus.NewDelete)
  for (size_t i = 1; i < matches.size(); i++) {
    EXPECT_TRUE(matches.at(i).mahalanobisDistSq >= matches.at(i - 1).mahalanobisDistSq) << "Not sorted at i=" << i;
  }
  EXPECT_NEAR(0.288177, matches.at(0).mahalanobisDistSq, 0.001);
  EXPECT_EQ(45334, matches.at(0).lanelet.id());
  EXPECT_FALSE(matches.at(0).lanelet.inverted()) << "best match must be non inverted 45334";
  EXPECT_EQ(14ul, matches.size());
}

TEST_F(Matching, probabilisticConst) {  // NOLINT
  using namespace lanelet::matching;
  const LaneletMap& constMap = *map;
  auto matches = getProbabilisticMatches(constMap, obj, 4.);
  EXPECT_EQ(14ul, matches.size());
}

TEST_F(Matching, isCloseTo) {  // NOLINT
  using namespace lanelet::matching;

  LaneletLayer emptyLayer;
  EXPECT_TRUE(isCloseTo(map->laneletLayer, obj, 4.));
  EXPECT_FALSE(isCloseTo(emptyLayer, obj, 4.));

  EXPECT_TRUE(isWithin(map->laneletLayer, obj));
  EXPECT_FALSE(isWithin(emptyLayer, obj));

  Object2d objWithEmptyHull;
  objWithEmptyHull.pose.translation() = map->pointLayer.get(41656).basicPoint2d();
  EXPECT_TRUE(matching::isCloseTo(map->laneletLayer, objWithEmptyHull, 1.));
}

TEST_F(Matching, filterNonCompliantProbabilistic) {  // NOLINT
  using namespace lanelet::matching;
  auto matches = getProbabilisticMatches(*map, obj, 4.);
  EXPECT_EQ(14ul, matches.size());
  EXPECT_EQ("45334 45356inv 45358inv 45328inv 45332 45344 45330 45344inv 45330inv 45332inv 45328 45356 45358 45334inv ",
            toString(matches));

  lanelet::traffic_rules::TrafficRulesPtr trafficRulesPtr =
      lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany, lanelet::Participants::Vehicle);
  auto compliantMatches = removeNonRuleCompliantMatches(matches, trafficRulesPtr);
  EXPECT_EQ(8ul, compliantMatches.size())
      << "see list below: should exclude zebra crossing (pedestrian only) and inverted one way lanelets";
  EXPECT_EQ("45334 45356inv 45358inv 45332 45330 45328 45356 45358 ", toString(compliantMatches));
}

TEST_F(Matching, filterNonCompliantDeterminstic) {  // NOLINT
  using namespace lanelet::matching;
  auto matches = getDeterministicMatches(*map, obj, 4.);
  EXPECT_EQ(14ul, matches.size());

  lanelet::traffic_rules::TrafficRulesPtr trafficRulesPtr =
      lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany, lanelet::Participants::Vehicle);
  auto compliantMatches = removeNonRuleCompliantMatches(matches, trafficRulesPtr);
  EXPECT_EQ(8ul, compliantMatches.size()) << "filterNonCompliantProbabilistic for details";
}
