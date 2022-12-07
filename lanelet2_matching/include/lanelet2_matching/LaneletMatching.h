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

#pragma once

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include "Exceptions.h"
#include "Types.h"
#include "Utilities.h"

namespace lanelet {
namespace matching {

/**
 * @brief get deterministic lanelet matches of an object with a maximum distance of maxDist, sorted ascending by
 * distance
 */
std::vector<LaneletMatch> getDeterministicMatches(LaneletMap& map, const Object2d& obj, double maxDist);
std::vector<ConstLaneletMatch> getDeterministicMatches(const LaneletMap& map, const Object2d& obj, double maxDist);

/**
 * @brief get probabilistic lanelet matches of an object with a maximum deterministic euler distance of maxDist, sorted
 * ascending by Mahalanobis distance
 * @throws MatchingError if the orientationCovarianceRadians or the determinant of the position covariance is zero
 *
 * for the distance computation see D. Petrich, T. Dang, D. Kasper, G. Breuel and C. Stiller,
 * "Map-based long term motion prediction for vehicles in traffic environments,"
 * 16th International IEEE Conference on Intelligent Transportation Systems (ITSC 2013),
 * The Hague, 2013, pp. 2166-2172. doi: 10.1109/ITSC.2013.6728549
 * https://ieeexplore.ieee.org/document/6728549
 */
std::vector<LaneletMatchProbabilistic> getProbabilisticMatches(LaneletMap& map, const ObjectWithCovariance2d& obj,
                                                               double maxDist);
std::vector<ConstLaneletMatchProbabilistic> getProbabilisticMatches(const LaneletMap& map,
                                                                    const ObjectWithCovariance2d& obj, double maxDist);

/**
 * @brief Determine whether an object is within a maximum distance to any primitive of the layer
 */
template <typename LayerT>
bool isCloseTo(const LayerT& map, const Object2d& obj, double maxDist) {
  auto closePrimitives = utils::findWithin(map, obj, maxDist);
  return !closePrimitives.empty();
}

/**
 * @brief Determine whether an object is (at least partially) within any primitive of the layer
 */
template <typename LayerT>
bool isWithin(const LayerT& map, const Object2d& obj) {
  return isCloseTo(map, obj, 0.);
}

/**
 * @brief Remove non traffic rule compliant probabilistic lanelet matches
 */
template <typename MatchVectorT>
MatchVectorT removeNonRuleCompliantMatches(const MatchVectorT& matches,
                                           const lanelet::traffic_rules::TrafficRulesPtr& trafficRulesPtr) {
  MatchVectorT compliantMatches = matches;
  compliantMatches.erase(
      std::remove_if(compliantMatches.begin(), compliantMatches.end(),
                     [&trafficRulesPtr](auto& match) { return !trafficRulesPtr->canPass(match.lanelet); }),
      compliantMatches.end());
  return compliantMatches;
}

}  // namespace matching
}  // namespace lanelet
