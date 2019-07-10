
import unittest

from sim.parse_robot_w_field import extractPathsFromSVG


class TestParseRobotWField(unittest.TestCase):

    @unittest.skip('Add into full')
    def test_parse_robot_w_field_simple_hab_lower(self):
        paths_as_pts = extractPathsFromSVG('robot_w_field_plain_simple_hab_lower.svg')
        paths_as_pts = [(round(pt[0], 2), round(pt[1], 2)) for pt in paths_as_pts['hab_lower']]
        assert_list = [(20.0, 15.0), (24.0, 14.5), (32.0, 14.5), (32.0, 15.0)]
        self.assertListEqual(paths_as_pts, assert_list)

    @unittest.skip('Add into full')
    def test_parse_robot_w_field_complex_hab_lower(self):
        paths_as_pts = extractPathsFromSVG('robot_w_field_plain.svg')
        paths_as_pts = [(round(pt[0], 2), round(pt[1], 2)) for pt in paths_as_pts['hab_lower']]
        assert_list = [(20.0, 15.0), (24.0, 14.5), (32.0, 14.5), (32.0, 15.0)]
        self.assertListEqual(paths_as_pts, assert_list)

    @unittest.skip('Add into full')
    def test_parse_robot_w_field_simple_left_wheel(self):
        paths_as_pts = extractPathsFromSVG('robot_w_field_plain_simple_left_wheel.svg')
        paths_as_pts = [(round(pt[0], 3), round(pt[1], 3)) for pt in paths_as_pts['left_wheel']]
        assert_list = [(3.0, 15.0), (2.845, 14.977), (2.706, 14.906), (2.595, 14.796), (2.524, 14.656), (2.5, 14.501), (2.524, 14.347), (2.595, 14.207), (2.706, 14.097), (2.845, 14.026), (3.0, 14.003), (3.155, 14.026), (3.294, 14.097), (3.405, 14.207), (3.476, 14.347), (3.5, 14.501), (3.476, 14.656), (3.405, 14.796), (3.294, 14.906), (3.155, 14.977)]
        self.assertListEqual(paths_as_pts, assert_list)

    # @unittest.skip('Add into full')
    def test_parse_robot_w_field_complex_left_wheel(self):
        paths_as_pts = extractPathsFromSVG('robot_w_field_plain.svg')
        paths_as_pts = [(round(pt[0], 3), round(pt[1], 3)) for pt in paths_as_pts['rear_wheel'][1]]
        assert_list = [(30.0, 150.0), (28.455, 149.756), (27.061, 149.045), (25.955, 147.939), (25.245, 146.545), (25.0, 145.0), (25.245, 143.455), (25.955, 142.061), (27.061, 140.955), (28.455, 140.245), (30.0, 140.0), (31.545, 140.245), (32.939, 140.955), (34.045, 142.061), (34.755, 143.455), (35.0, 145.0), (34.755, 146.545), (34.045, 147.939), (32.939, 149.045), (31.545, 149.756)]
        self.assertListEqual(paths_as_pts, assert_list)
