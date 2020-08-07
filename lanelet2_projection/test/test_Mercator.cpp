#include <gtest/gtest.h>

#include "lanelet2_projection/Mercator.h"

TEST(Mercator, origin) {  // NOLINT
  lanelet::projection::Mercator mercatorProjection(lanelet::Origin{{45, 45, 0}});
  auto pLocal = mercatorProjection.forward({45, 45, 0});
  EXPECT_DOUBLE_EQ(pLocal.x(), 0.);
  EXPECT_DOUBLE_EQ(pLocal.y(), 0.);
  EXPECT_DOUBLE_EQ(pLocal.z(), 0.);
}

TEST(Mercator, roundTrip) {  // NOLINT
  lanelet::projection::Mercator mercatorProjection;
  lanelet::GPSPoint pGps{49.01439, 8.41722, 2};
  auto pLocal = mercatorProjection.forward(pGps);
  auto pGpsRound = mercatorProjection.reverse(pLocal);
  EXPECT_NEAR(pGps.lat, pGpsRound.lat, 1e-8);
  EXPECT_NEAR(pGps.lon, pGpsRound.lon, 1e-8);
  EXPECT_NEAR(pGps.ele, pGpsRound.ele, 1e-8);
}
