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

class MatchingUtilitiesBase : public ::testing::Test {
  using Attr = lanelet::AttributeName;
  using AttrStr = lanelet::AttributeNamesString;
  using Value = lanelet::AttributeValueString;
  using Participants = lanelet::Participants;
  /* looks like this:
   *
   * p1----ls11->---p2                  p5----ls15->--p6
   * |              |
   * |              |
   * ls13           ls14      ...
   * |              |
   * v              v
   * |              |
   * p3----ls12->---p4                  p7----ls16->--p8
   *
   * right: ll21                        right: ll23
   * down: ll22 (pedestrian)
   *
   */
 public:
  MatchingUtilitiesBase() {
    map = std::make_shared<LaneletMap>();
    map->add(ll21);
    map->add(ll22);
    map->add(ll23);
  }
  LaneletMapPtr map;

  lanelet::Point3d p1{1, 0, 0}, p2{2, 2, 0}, p3{3, 0, 2}, p4{4, 2, 2};
  lanelet::Point3d p5{1, 100, 0}, p6{2, 102, 0}, p7{3, 100, 2}, p8{4, 102, 2};
  lanelet::LineString3d ls11{11, {p1, p2}}, ls12{12, {p3, p4}}, ls13{13, {p1, p3}}, ls14{14, {p2, p4}};
  lanelet::LineString3d ls15{15, {p5, p6}}, ls16{16, {p7, p8}};
  lanelet::AttributeMap vehicleAttr{{AttrStr::Subtype, Value::Road}, {AttrStr::Location, Value::Urban}};
  lanelet::AttributeMap pedestrianAttr{{AttrStr::Subtype, Value::Walkway}, {AttrStr::Location, Value::Urban}};
  lanelet::Lanelet ll21{21, ls11, ls12, vehicleAttr}, ll22{22, ls14, ls13, pedestrianAttr};
  lanelet::Lanelet ll23{23, ls15, ls16, vehicleAttr};
};

TEST_F(MatchingUtilitiesBase, fixtureSetupSuccessful) {  // NOLINT
  EXPECT_TRUE(map->laneletLayer.exists(21));
  EXPECT_TRUE(map->laneletLayer.exists(22));
  EXPECT_TRUE(map->laneletLayer.exists(23));
  EXPECT_EQ(3ul, map->laneletLayer.size());
}

TEST_F(MatchingUtilitiesBase, absoluteHull) {  // NOLINT
  matching::Object2d obj;

  obj.pose.translation() = BasicPoint2d{10, 0};                         //!< at point x=10 y=0
  obj.pose.linear() = Eigen::Rotation2D<double>(-1.5 * M_PI).matrix();  //!< rotated by pi/2 (=-3pi/2)

  obj.absoluteHull = absoluteHull(matching::Hull2d{BasicPoint2d{-0.5, -1}, BasicPoint2d{2, 1}}, obj.pose);

  EXPECT_DOUBLE_EQ(10 + 1, obj.absoluteHull.at(0).x());
  EXPECT_DOUBLE_EQ(0 - 0.5, obj.absoluteHull.at(0).y());

  EXPECT_DOUBLE_EQ(10 - 1, obj.absoluteHull.at(1).x());
  EXPECT_DOUBLE_EQ(0 + 2, obj.absoluteHull.at(1).y());
}

TEST_F(MatchingUtilitiesBase, findWithin) {  // NOLINT
  matching::Object2d obj;
  obj.pose.translation() = lanelet::BasicPoint2d(2.05, 1.);
  obj.pose.linear() = Eigen::Rotation2D<double>(150. / 180. * M_PI).matrix();

  EXPECT_EQ(2ul, matching::utils::findWithin(map->laneletLayer, obj, 0.1).size());

  obj.absoluteHull = absoluteHull(
      matching::Hull2d{BasicPoint2d{-1, -0.9}, BasicPoint2d{2, -0.9}, BasicPoint2d{2, 0.9}, BasicPoint2d{1, 0.9}},
      obj.pose);

  std::vector<std::pair<double, Lanelet>> laneletsWithDistance =
      matching::utils::findWithin(map->laneletLayer, obj, 100);

  EXPECT_EQ(3ul, laneletsWithDistance.size());
}

TEST_F(MatchingUtilitiesBase, getMahalanobisDistSq) {  // NOLINT
  matching::ObjectWithCovariance2d obj;
  obj.pose.translation() = lanelet::BasicPoint2d(2.05, 1.);
  obj.pose.linear() = Eigen::Rotation2D<double>(1. / 180. * M_PI).matrix();  // one degree orientation
  obj.absoluteHull = absoluteHull(
      matching::Hull2d{BasicPoint2d{-1, -0.9}, BasicPoint2d{2, -0.9}, BasicPoint2d{2, 0.9}, BasicPoint2d{1, 0.9}},
      obj.pose);

  using namespace matching::utils;
  obj.vonMisesKappa = 0.5;
  EXPECT_THROW(getMahalanobisDistSq(map->laneletLayer.get(21), obj), MatchingError)  // NOLINT
      << "should throw on covariance = zero";

  obj.positionCovariance = matching::PositionCovariance2d::Ones();
  EXPECT_THROW(getMahalanobisDistSq(map->laneletLayer.get(21), obj), MatchingError)  // NOLINT
      << "should throw on determinant = zero";

  obj.positionCovariance = matching::PositionCovariance2d::Identity() * 2.;
  obj.vonMisesKappa = 1. / (10. / 180. * M_PI);  // covariance of 10 degrees

  double mahaDist21 = getMahalanobisDistSq(map->laneletLayer.get(21), obj);
  EXPECT_NEAR(0.011, mahaDist21, 10e-2);

  double mahaDist21inv = getMahalanobisDistSq(map->laneletLayer.get(21).invert(), obj);
  EXPECT_NEAR(320.411, mahaDist21inv, 10e-2);
}

class MatchingBase : public MatchingUtilitiesBase {
 public:
  MatchingBase() {
    obj.pose.translation() = lanelet::BasicPoint2d(1., 1.);
    obj.pose.linear() = Eigen::Rotation2D<double>(90.1 / 180. * M_PI).matrix();
    obj.absoluteHull = absoluteHull(
        matching::Hull2d{BasicPoint2d{-1, -0.9}, BasicPoint2d{2, -0.9}, BasicPoint2d{2, 0.9}, BasicPoint2d{1, 0.9}},
        obj.pose);
    obj.positionCovariance = matching::PositionCovariance2d::Identity() * 2.;
    obj.vonMisesKappa = 1. / (10. / 180. * M_PI);  // covariance of 10 degrees
  }
  matching::ObjectWithCovariance2d obj;
};

TEST_F(MatchingBase, deterministicNonConst) {  // NOLINT
  using namespace lanelet::matching;
  auto matches = getDeterministicMatches(*map, obj, 4.);
  for (size_t i = 1; i < matches.size(); i++) {
    EXPECT_TRUE(matches.at(i).distance >= matches.at(i - 1).distance)
        << "Not sorted: at i=" << i - 1 << " dist = " << matches.at(i - 1).distance << "at i=" << i
        << " dist = " << matches.at(i).distance;
  }
  EXPECT_EQ(4ul, matches.size());
  EXPECT_NEAR(0., matches.at(0).distance, 0.1);
}

TEST_F(MatchingBase, deterministicConst) {  // NOLINT
  using namespace lanelet::matching;
  const LaneletMap& constMap = *map;
  auto matches = getDeterministicMatches(constMap, obj, 4.);
  EXPECT_EQ(4ul, matches.size());
}

TEST_F(MatchingBase, probabilisticNonConst) {  // NOLINT
  using namespace lanelet::matching;
  auto matches = getProbabilisticMatches(*map, obj, 4.);
  // NOLINTNEXTLINE(clang-analyzer-cplusplus.NewDelete)
  for (size_t i = 1; i < matches.size(); i++) {
    EXPECT_TRUE(matches.at(i).mahalanobisDistSq >= matches.at(i - 1).mahalanobisDistSq) << "Not sorted at i=" << i;
  }
  EXPECT_NEAR(0.0, matches.at(0).mahalanobisDistSq, 0.001);
  EXPECT_EQ(22, matches.at(0).lanelet.id());
  EXPECT_FALSE(matches.at(0).lanelet.inverted()) << "best match must be non inverted 21";
  EXPECT_EQ(4ul, matches.size());
}

TEST_F(MatchingBase, probabilisticConst) {  // NOLINT
  using namespace lanelet::matching;
  const LaneletMap& constMap = *map;
  auto matches = getProbabilisticMatches(constMap, obj, 4.);
  EXPECT_EQ(4ul, matches.size());
}

TEST_F(MatchingBase, isCloseTo) {  // NOLINT
  using namespace lanelet::matching;

  LaneletLayer emptyLayer;
  EXPECT_TRUE(isCloseTo(map->laneletLayer, obj, 4.));
  EXPECT_FALSE(isCloseTo(emptyLayer, obj, 4.));

  EXPECT_TRUE(isWithin(map->laneletLayer, obj));
  EXPECT_FALSE(isWithin(emptyLayer, obj));

  Object2d objWithEmptyHull;
  objWithEmptyHull.pose.translation() = obj.pose.translation();
  EXPECT_TRUE(matching::isCloseTo(map->laneletLayer, objWithEmptyHull, 1.));
}

TEST_F(MatchingBase, filterNonCompliantProbabilistic) {  // NOLINT
  using namespace lanelet::matching;
  auto matches = getProbabilisticMatches(*map, obj, 4.);
  EXPECT_EQ(4ul, matches.size());
  EXPECT_EQ("22 21inv 21 22inv ", toString(matches));

  lanelet::traffic_rules::TrafficRulesPtr trafficRulesPtr =
      lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany, lanelet::Participants::Vehicle);
  auto compliantMatches = removeNonRuleCompliantMatches(matches, trafficRulesPtr);
  EXPECT_EQ(1ul, compliantMatches.size()) << "should exclude pedestrian lanelets and inverted one-way lanelets";
  EXPECT_EQ("21 ", toString(compliantMatches));
}

TEST_F(MatchingBase, filterNonCompliantDeterminstic) {  // NOLINT
  using namespace lanelet::matching;
  auto matches = getDeterministicMatches(*map, obj, 4.);
  EXPECT_EQ(4ul, matches.size());

  lanelet::traffic_rules::TrafficRulesPtr trafficRulesPtr =
      lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany, lanelet::Participants::Vehicle);
  auto compliantMatches = removeNonRuleCompliantMatches(matches, trafficRulesPtr);
  EXPECT_EQ(1ul, compliantMatches.size()) << "filterNonCompliantProbabilistic for details";
}
