/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Authors: Shuwei Qiang
 */

/*
 * Modifications:
 *  - Added class comments and tweaked some function comments
 *    - 1/16/2020
 *    - Michael McConnell
 */

#ifndef LANELET2_EXTENSION_PROJECTION_LOCAL_FRAME_PROJECTOR_H
#define LANELET2_EXTENSION_PROJECTION_LOCAL_FRAME_PROJECTOR_H

#include <proj.h>
#include <lanelet2_io/Projection.h>

#include <string>

namespace lanelet
{
namespace projection
{

  /**
   * LocalFrameProjector uses the Proj library to project maps based on provided proj strings
   * See https://proj.org/ for details on Proj library api and string definitions
   * 
   * Example Usage
   * 
   * LocalFrameProjector proj = LocalFrameProjector('EPSG:4326', '+proj=tmerc +lat_0=38.85197911150576 +lon_0=-77.24835128349988 +k=1 +x_0=0 +y_0=0 +units=m +vunits=m');
   * 
   * The above projection can now be used for converting points from a lat/lon reference in EPSG:4326 (WGS84) to a more local transverse mercator x,y frame.
   * 
   */
class LocalFrameProjector : public Projector
{

public:

  explicit LocalFrameProjector(const char* base_frame, const char* target_frame, Origin origin = Origin({ 0.0, 0.0 }));

  /**
   * [LocalFrameProjector::forward projects gps lat/lon to local map frame]
   * @param  gps [point with latitude longitude information]
   * @return     [projected point in local map coordinate]
   */
  BasicPoint3d forward(const GPSPoint& p) const override;

  /**
   * [LocalFrameProjector::reverse projects point within local map frame into gps lat/lon]
   * @param  local_point [3d point in local map frame]
   * @return             [projected point]
   */
  GPSPoint reverse(const BasicPoint3d& p) const override;

private:
  
  PJ *P;
};

}  // namespace projection
}  // namespace lanelet

#endif  // LANELET2_EXTENSION_PROJECTION_LOCAL_FRAME_PROJECTOR_H
