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

class MatchingIntegrationTest : public ::testing::Test {
 public:
  MatchingIntegrationTest() {
    map = load(exampleMapPath, projector);

    obj.pose.translation() = map->pointLayer.get(41656).basicPoint2d();
    obj.pose.linear() = Eigen::Rotation2D<double>(150. / 180. * M_PI).matrix();
    obj.absoluteHull = absoluteHull(
        matching::Hull2d{BasicPoint2d{-1, -0.9}, BasicPoint2d{2, -0.9}, BasicPoint2d{2, 0.9}, BasicPoint2d{1, 0.9}},
        obj.pose);
    obj.positionCovariance = matching::PositionCovariance2d::Identity() * 2.;
    obj.vonMisesKappa = 1. / (10. / 180. * M_PI);  // covariance of 10 degrees
  }
  matching::ObjectWithCovariance2d obj;
  std::string exampleMapPath = std::string(PKG_DIR) + "/test/mapping_example.osm";
  projection::UtmProjector projector{Origin({49, 8.4})};
  LaneletMapPtr map;
};

TEST_F(MatchingIntegrationTest, fixtureSetupSuccessful) {  // NOLINT
  EXPECT_TRUE(map->laneletLayer.exists(42440));
}

TEST_F(MatchingIntegrationTest, deterministicNonConst) {  // NOLINT
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

TEST_F(MatchingIntegrationTest, deterministicConst) {  // NOLINT
  using namespace lanelet::matching;
  const LaneletMap& constMap = *map;
  auto matches = getDeterministicMatches(constMap, obj, 4.);
  EXPECT_EQ(14ul, matches.size());
}

TEST_F(MatchingIntegrationTest, probabilisticNonConst) {  // NOLINT
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

TEST_F(MatchingIntegrationTest, probabilisticConst) {  // NOLINT
  using namespace lanelet::matching;
  const LaneletMap& constMap = *map;
  auto matches = getProbabilisticMatches(constMap, obj, 4.);
  EXPECT_EQ(14ul, matches.size());
}

TEST_F(MatchingIntegrationTest, isCloseTo) {  // NOLINT
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

TEST_F(MatchingIntegrationTest, filterNonCompliantProbabilistic) {  // NOLINT
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

TEST_F(MatchingIntegrationTest, filterNonCompliantDeterminstic) {  // NOLINT
  using namespace lanelet::matching;
  auto matches = getDeterministicMatches(*map, obj, 4.);
  EXPECT_EQ(14ul, matches.size());

  lanelet::traffic_rules::TrafficRulesPtr trafficRulesPtr =
      lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany, lanelet::Participants::Vehicle);
  auto compliantMatches = removeNonRuleCompliantMatches(matches, trafficRulesPtr);
  EXPECT_EQ(8ul, compliantMatches.size()) << "filterNonCompliantProbabilistic for details";
}
