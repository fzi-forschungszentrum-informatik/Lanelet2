#pragma once
#include "lanelet2_core/Forward.h"

namespace lanelet {

//! A raw GPS point
class GPSPoint {
 public:
  double lat{0.};  //! lat according to WGS84
  double lon{0.};  //! lon according to WGS84
  double ele{0.};  //! elevation according to WGS84 (m)
};

}  // namespace lanelet
