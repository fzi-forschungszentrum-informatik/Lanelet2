import unittest
import lanelet2  # if we fail here, there is something wrong with lanelet2 registration
from lanelet2.core import (BasicPoint3d, GPSPoint)
from lanelet2.io import Origin
from lanelet2.projection import (UtmProjector,
                                 GeocentricProjector,
                                 LocalCartesianProjector)


class MatchingApiTestCase(unittest.TestCase):
    origin_lat = 49.01439
    origin_lon = 8.41722
    origin_ele = 123.0
    origin_gps_point = GPSPoint(origin_lat, origin_lon, origin_ele)

    # X, Y, Z with respect to the center of the earth
    origin_x = 4146160.550580083
    origin_y = 613525.0621995202
    origin_z = 4791701.343249619
    origin_ecef = BasicPoint3d(origin_x, origin_y, origin_z)

    # the larges number of decimal places for assertEqual comparisons
    decimals = 8

    def test_UtmProjector(self):
        # NOTE: the projected plane is on the WGS84 ellipsoid
        origin = Origin(self.origin_lat, self.origin_lon)
        utm_projector = UtmProjector(origin)

        utm_point = utm_projector.forward(self.origin_gps_point)
        self.assertEqual(utm_point.x, 0.0)
        self.assertEqual(utm_point.y, 0.0)
        self.assertEqual(utm_point.z, self.origin_ele)

        utm_point = BasicPoint3d(0.0, 0.0, self.origin_ele)
        gps_point = utm_projector.reverse(utm_point)
        self.assertEqual(round(gps_point.lat, 5), self.origin_lat)
        self.assertEqual(round(gps_point.lon, 5), self.origin_lon)
        self.assertEqual(round(gps_point.alt, 5), self.origin_ele)

    def test_LocalCartesianProjector(self):
        # NOTE: the projected plane is tangential to the WGS84 ellipsoid
        # but it is at the given elevation above the ellipsoid
        origin = Origin(self.origin_lat, self.origin_lon, self.origin_ele)
        local_cartesian_projector = LocalCartesianProjector(origin)

        local_cartesian_point = local_cartesian_projector.forward(
            self.origin_gps_point)
        self.assertEqual(local_cartesian_point.x, 0.0)
        self.assertEqual(local_cartesian_point.y, 0.0)
        self.assertEqual(local_cartesian_point.z, 0.0)

        local_cartesian_point = BasicPoint3d(0.0, 0.0, 0.0)
        gps_point = local_cartesian_projector.reverse(local_cartesian_point)
        self.assertEqual(round(gps_point.lat, self.decimals), self.origin_lat)
        self.assertEqual(round(gps_point.lon, self.decimals), self.origin_lon)
        self.assertEqual(round(gps_point.alt, self.decimals), self.origin_ele)

    def test_GeocentricProjector(self):
        geocentric_projector = GeocentricProjector()

        ecef_point = geocentric_projector.forward(self.origin_gps_point)
        self.assertEqual(ecef_point.x, self.origin_ecef.x)
        self.assertEqual(ecef_point.y, self.origin_ecef.y)
        self.assertEqual(ecef_point.z, self.origin_ecef.z)

        gps_point = geocentric_projector.reverse(self.origin_ecef)
        self.assertEqual(round(gps_point.lat, self.decimals), self.origin_lat)
        self.assertEqual(round(gps_point.lon, self.decimals), self.origin_lon)
        self.assertEqual(round(gps_point.alt, self.decimals), self.origin_ele)


if __name__ == '__main__':
    unittest.main()
