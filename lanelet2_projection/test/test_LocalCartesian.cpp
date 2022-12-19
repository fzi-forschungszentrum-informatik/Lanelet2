#include "gtest/gtest.h"
#include "lanelet2_projection/LocalCartesian.h"

using namespace lanelet;
using LocalCartesianProjector = lanelet::projection::LocalCartesianProjector;

class LocalCartesianProjectionTest : public ::testing::Test {
 public:
  void SetUp() override {
    localCartesianProjector = std::make_shared<LocalCartesianProjector>(origin);
  }
  LocalCartesianProjector::Ptr localCartesianProjector;

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

TEST_F(LocalCartesianProjectionTest, TestForward) {  // NOLINT
  BasicPoint3d localCartesianPoint = localCartesianProjector->forward(originGps);
  ASSERT_NEAR(localCartesianPoint.x(), 0., 0.00001);
  ASSERT_NEAR(localCartesianPoint.y(), 0., 0.00001);
  ASSERT_NEAR(localCartesianPoint.z(), 0., 0.00001);
}

TEST_F(LocalCartesianProjectionTest, TestReverse) {  // NOLINT
  lanelet::GPSPoint gpsPoint = localCartesianProjector->reverse({0., 0., 0.});
  ASSERT_NEAR(gpsPoint.lat, originLat, 0.00001);
  ASSERT_NEAR(gpsPoint.lon, originLon, 0.00001);
  ASSERT_NEAR(gpsPoint.ele, originEle, 0.00001);
}
