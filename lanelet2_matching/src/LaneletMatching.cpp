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

#include "lanelet2_matching/LaneletMatching.h"

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/Polygon.h>

#include "lanelet2_matching/Utilities.h"

namespace {
template <typename LaneletT, typename MatchT>
std::vector<MatchT> toMatchVector(std::vector<std::pair<double, LaneletT>> pairVec) {
  std::vector<MatchT> matchVec;
  matchVec.reserve(2 * pairVec.size());
  for (const auto& pair : pairVec) {
    MatchT match;
    match.distance = pair.first;
    match.lanelet = pair.second;
    matchVec.push_back(match);
    match.lanelet = pair.second.invert();
    matchVec.push_back(match);
  }

  // sort ascending by distance
  std::sort(matchVec.begin(), matchVec.end(),
            [](const auto& lhs, const auto& rhs) { return lhs.distance < rhs.distance; });

  return matchVec;
}

template <typename LaneletT, typename MatchT>
std::vector<MatchT> getProbabilisticMatchesImpl(const std::vector<std::pair<double, LaneletT>>& pairVec,
                                                const lanelet::matching::ObjectWithCovariance2d& obj) {
  std::vector<MatchT> matchVec;
  matchVec.reserve(2 * pairVec.size());
  for (const auto& pair : pairVec) {
    MatchT match;
    match.distance = pair.first;
    match.lanelet = pair.second;
    match.mahalanobisDistSq = lanelet::matching::utils::getMahalanobisDistSq(match.lanelet, obj);
    matchVec.push_back(match);
    match.lanelet = pair.second.invert();
    match.mahalanobisDistSq = lanelet::matching::utils::getMahalanobisDistSq(match.lanelet, obj);
    matchVec.push_back(match);
  }

  // sort ascending by mahalanobisDistSq
  std::sort(matchVec.begin(), matchVec.end(),
            [](const auto& lhs, const auto& rhs) { return lhs.mahalanobisDistSq < rhs.mahalanobisDistSq; });

  return matchVec;
}
}  // namespace

namespace lanelet {
namespace matching {

std::vector<LaneletMatch> getDeterministicMatches(LaneletMap& map, const Object2d& obj, double maxDist) {
  return toMatchVector<Lanelet, LaneletMatch>(utils::findWithin(map.laneletLayer, obj, maxDist));
}

std::vector<ConstLaneletMatch> getDeterministicMatches(const LaneletMap& map, const Object2d& obj, double maxDist) {
  return toMatchVector<ConstLanelet, ConstLaneletMatch>(utils::findWithin(map.laneletLayer, obj, maxDist));
}

std::vector<LaneletMatchProbabilistic> getProbabilisticMatches(LaneletMap& map, const ObjectWithCovariance2d& obj,
                                                               double maxDist) {
  auto pairVec = utils::findWithin(map.laneletLayer, obj, maxDist);
  return getProbabilisticMatchesImpl<Lanelet, LaneletMatchProbabilistic>(pairVec, obj);
}

std::vector<ConstLaneletMatchProbabilistic> getProbabilisticMatches(const LaneletMap& map,
                                                                    const ObjectWithCovariance2d& obj, double maxDist) {
  auto pairVec = utils::findWithin(map.laneletLayer, obj, maxDist);
  return getProbabilisticMatchesImpl<ConstLanelet, ConstLaneletMatchProbabilistic>(pairVec, obj);
}

}  // namespace matching
}  // namespace lanelet
