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
#include <lanelet2_core/geometry/LaneletMap.h>
#include <lanelet2_core/geometry/Polygon.h>

#include "Exceptions.h"
#include "Types.h"

namespace lanelet {
namespace matching {
namespace utils {

/**
 * @brief Find all primitives as close as or closer than maxDist to an object
 */
template <typename LayerT>
auto findWithin(LayerT& map, const Object2d& obj, double maxDist)
    -> std::vector<std::pair<double, traits::LayerPrimitiveType<LayerT>>> {
  if (obj.absoluteHull.empty()) {
    return lanelet::geometry::findWithin2d(map, lanelet::BasicPoint2d{obj.pose.translation()}, maxDist);
  }
  return lanelet::geometry::findWithin2d(map, lanelet::BasicPolygon2d{obj.absoluteHull}, maxDist);
}

/**
 * @brief Compute squared mahalanobis distance based on pose and covariance, hull is not used
 * @throws MatchingError if the orientationCovarianceRadians or the determinant of the position covariance is zero
 *
 * see D. Petrich, T. Dang, D. Kasper, G. Breuel and C. Stiller,
 * "Map-based long term motion prediction for vehicles in traffic environments,"
 * 16th International IEEE Conference on Intelligent Transportation Systems (ITSC 2013),
 * The Hague, 2013, pp. 2166-2172. doi: 10.1109/ITSC.2013.6728549
 * https://ieeexplore.ieee.org/document/6728549
 *
 * uses approximation orientationCovariance = 1./obj.vonMisesKappa
 */
double getMahalanobisDistSq(const ConstLanelet& lanelet, const ObjectWithCovariance2d& obj);

}  // namespace utils
}  // namespace matching
}  // namespace lanelet
