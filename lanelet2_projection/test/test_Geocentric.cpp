#include "gtest/gtest.h"
#include "lanelet2_projection/Geocentric.h"

using namespace lanelet;
using GeocentricProjector = lanelet::projection::GeocentricProjector;

class GeocentricProjectionTest : public ::testing::Test {
 public:
  void SetUp() override {
    geocentricProjector = std::make_shared<GeocentricProjector>();
  }
  GeocentricProjector::Ptr geocentricProjector;

  // Lat, Lon, Ele with respect to the WGS84 ellipsoid
  double originLat{49.01439};
  double originLon{8.41722};
  double originEle{123.0};
  lanelet::GPSPoint originGps{originLat, originLon, originEle};
  lanelet::Origin origin{originGps};

  // X, Y, Z with respect to the center of the earth
  double originX = 4146160.550580083;
  double originY = 613525.0621995202;
  double originZ = 4791701.343249619;
  lanelet::BasicPoint3d originECEF{originX, originY, originZ};
};

TEST_F(GeocentricProjectionTest, TestForward) {  // NOLINT
  BasicPoint3d ecefPoint = geocentricProjector->forward(originGps);
  ASSERT_NEAR(ecefPoint.x(), originX, 0.00001);
  ASSERT_NEAR(ecefPoint.y(), originY, 0.00001);
  ASSERT_NEAR(ecefPoint.z(), originZ, 0.00001);
}

TEST_F(GeocentricProjectionTest, TestReverse) {  // NOLINT
  lanelet::GPSPoint gpsPoint = geocentricProjector->reverse(originECEF);
  ASSERT_NEAR(gpsPoint.lat, originLat, 0.00001);
  ASSERT_NEAR(gpsPoint.lon, originLon, 0.00001);
  ASSERT_NEAR(gpsPoint.ele, originEle, 0.00001);
}
