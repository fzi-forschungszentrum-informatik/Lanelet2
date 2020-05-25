#include "gtest/gtest.h"
#include "lanelet2_projection/UTM.h"

using namespace lanelet;
using UtmProjector = lanelet::projection::UtmProjector;

class UTMProjectionTest : public ::testing::Test {
 public:
  void SetUp() override {
    utmProjector = std::make_shared<UtmProjector>(origin);
    utmProjectorNoOffset = std::make_shared<UtmProjector>(origin, false);
    utmProjectorNoOffsetThrow = std::make_shared<UtmProjector>(origin, false, true);
  }
  UtmProjector::Ptr utmProjector;
  UtmProjector::Ptr utmProjectorNoOffset;
  UtmProjector::Ptr utmProjectorNoOffsetThrow;

  double originLat = 49.01439;
  double originLon = 8.41722;
  lanelet::GPSPoint originGps{originLat, originLon, 0.};
  lanelet::Origin origin{originGps};

  double originX = 457386.38238563854;
  double originY = 5429219.051147663;
  lanelet::BasicPoint3d originMetric{originX, originY, 0.};
};

TEST_F(UTMProjectionTest, TestForward) {  // NOLINT
  BasicPoint3d metricPoint = utmProjector->forward(originGps);
  ASSERT_NEAR(metricPoint.x(), 0., 0.001);
  ASSERT_NEAR(metricPoint.y(), 0., 0.001);

  metricPoint = utmProjectorNoOffset->forward(originGps);
  ASSERT_NEAR(metricPoint.x(), originX, 0.001);
  ASSERT_NEAR(metricPoint.y(), originY, 0.001);
}

TEST_F(UTMProjectionTest, TestReverse) {  // NOLINT
  lanelet::GPSPoint gpsPoint = utmProjector->reverse({0., 0., 0.});
  ASSERT_NEAR(gpsPoint.lat, originLat, 0.00001);
  ASSERT_NEAR(gpsPoint.lon, originLon, 0.00001);

  gpsPoint = utmProjectorNoOffset->reverse(originMetric);
  ASSERT_NEAR(gpsPoint.lat, originLat, 0.00001);
  ASSERT_NEAR(gpsPoint.lon, originLon, 0.00001);
}

TEST_F(UTMProjectionTest, TestForwardOutOfHemisphere) {  // NOLINT
  // origin is in northern hemisphere
  double latTest = -originLat;                                       // southern hemisphere
  ASSERT_NO_THROW(utmProjector->forward({latTest, originLon, 0.}));  // NOLINT
  ASSERT_THROW(utmProjectorNoOffsetThrow->forward({latTest, originLon, 0.}), ForwardProjectionError);  // NOLINT
}

TEST_F(UTMProjectionTest, TestForwardOutOfZoneButInPadding) {  // NOLINT
  // origin is in UTM zone 32
  double lonTest = 5.;  // UTM zone 31 but less than 100km away from zone 32
  ASSERT_NO_THROW(utmProjector->forward({originLat, lonTest, 0.}));                                    // NOLINT
  ASSERT_THROW(utmProjectorNoOffsetThrow->forward({originLat, lonTest, 0.}), ForwardProjectionError);  // NOLINT
}

TEST_F(UTMProjectionTest, TestForwardOutOfZoneOutOfPadding) {  // NOLINT
  // origin is in UTM zone 32
  double lonTest = 0.;  // UTM zone 31 and out of padding area of zone 32
  ASSERT_THROW(utmProjector->forward({originLat, lonTest, 0.}), ForwardProjectionError);               // NOLINT
  ASSERT_THROW(utmProjectorNoOffsetThrow->forward({originLat, lonTest, 0.}), ForwardProjectionError);  // NOLINT
}

TEST_F(UTMProjectionTest, TestReverseOutOfZoneButInPadding) {  // NOLINT
  // origin is in UTM zone 32
  double x = 207547.;  // UTM zone 31 but less than 100km away from zone 32
  double y = 5436767.;
  ASSERT_NO_THROW(utmProjectorNoOffset->reverse({x, y, 0.}););                            // NOLINT
  ASSERT_THROW(utmProjectorNoOffsetThrow->reverse({x, y, 0.});, ReverseProjectionError);  // NOLINT
}

TEST_F(UTMProjectionTest, TestReverseOutOfZoneOutOfPadding) {  // NOLINT
  // origin is in UTM zone 32
  double x = 2000000.0;  // out of padding area of zone 32
  double y = 5000000.0;
  ASSERT_THROW(utmProjectorNoOffset->reverse({x, y, 0.});, ReverseProjectionError);       // NOLINT
  ASSERT_THROW(utmProjectorNoOffsetThrow->reverse({x, y, 0.});, ReverseProjectionError);  // NOLINT
}
