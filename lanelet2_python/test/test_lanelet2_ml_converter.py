import unittest
import lanelet2  # if we fail here, there is something wrong with lanelet2 registration
from lanelet2.core import getId, Point3d, BasicPoint2d, LineString3d, Lanelet, LaneletMap
from lanelet2.ml_converter import MapDataInterface

def get_sample_lanelet_map():
    mymap = LaneletMap()
    x_left = 4
    x_middle = 2
    x_right = 0
    ls_left = LineString3d(1234, [Point3d(getId(), x_left, i, 0) for i in range(0, 3)], {"type": "road_border"})
    ls_middle = LineString3d(getId(), [Point3d(getId(), x_middle, i, 0) for i in range(0, 3)], {"type": "line", "subtype": "dashed"})
    ls_right = LineString3d(getId(), [Point3d(getId(), x_right, i, 0) for i in range(0, 3)], {"type": "road_border"})
    llet = Lanelet(getId(), ls_left, ls_middle)
    llet_2 = Lanelet(getId(), ls_middle, ls_right)
    mymap.add(llet)
    mymap.add(llet_2)
    return mymap

class MapDataInterfaceTestCase(unittest.TestCase):
    def test_map_data_interface(self):
        mymap = get_sample_lanelet_map()
        pos = BasicPoint2d(1, 1)
        mDataIf = MapDataInterface(mymap)
        mDataIf.setCurrPosAndExtractSubmap2d(pos, 0)
        lData = mDataIf.laneData(True)
        tfData = lData.getTensorInstanceData(True, False)
        self.assertEqual(len(tfData.compoundCenterlines), 2)
        self.assertEqual(tfData.compoundLaneDividerTypes[-1], 1)


if __name__ == '__main__':
    unittest.main()
