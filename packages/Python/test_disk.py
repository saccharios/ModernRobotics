import pathlib
import unittest
import course4_week2 as code


class MyTestCase(unittest.TestCase):
    def test_intersect_line_fully_inside(self):
        disk = code.Disk(code.Point(0,0),1.0)
        start = code.Point(x=0.5,y=0.5)
        end = code.Point(x=-0.5,y=0.5)
        self.assertTrue(disk.intersects(code.Line(start,end)))

    def test_intersect_start_in_disk(self):
        disk = code.Disk(code.Point(0,0),1.0)
        start = code.Point(x=0.5,y=0.5)
        end = code.Point(x=-1.5,y=0.5)
        self.assertTrue(disk.intersects(code.Line(start,end)))

    def test_intersect_end_in_disk(self):
        disk = code.Disk(code.Point(0,0),1.0)
        start = code.Point(x=1.5,y=0.5)
        end = code.Point(x=0.5,y=0.5)
        self.assertTrue(disk.intersects(code.Line(start,end)))

    def test_intersect_intersection_outside_line(self):
        disk = code.Disk(code.Point(10.0,0),1.0)
        start = code.Point(x=1.5,y=0.5)
        end = code.Point(x=-1.5,y=0.5)
        self.assertFalse(disk.intersects(code.Line(start,end)))

    def test_intersect_tangent(self):
        disk = code.Disk(code.Point(0,0),1.0)
        start = code.Point(x=-1.5,y=1.0)
        end = code.Point(x=1.5,y=1.0)
        self.assertTrue(disk.intersects(code.Line(start,end)))

    def test_intersect_not_at_all(self):
        disk = code.Disk(code.Point(0,0),1.0)
        start = code.Point(x=1.5,y=1.5)
        end = code.Point(x=-1.5,y=1.5)
        self.assertFalse(disk.intersects(code.Line(start,end)))

    def test_intersect_normal_1(self):
        disk = code.Disk(code.Point(0,0),1.0)
        start = code.Point(x=1.5,y=0.5)
        end = code.Point(x=-1.5,y=0.5)
        self.assertTrue(disk.intersects(code.Line(start,end)))

    def test_intersect_normal_2(self):
        disk = code.Disk(code.Point(0,0),1.0)
        start = code.Point(x=-0.5,y=2.0)
        end = code.Point(x=0.5,y=-2.0)
        self.assertTrue(disk.intersects(code.Line(start,end)))

    def test_intersect_normal_3(self):
        disk = code.Disk(code.Point(-0.3,-0.2),0.2)
        start = code.Point(x=-0.5,y=-0.5)
        end = code.Point(x=-0.28899437717111176,y=-0.21233798104890322)
        self.assertTrue(disk.intersects(code.Line(start,end)))

    def test_is_collision_free(self):
        obstacles = code.read_in_obstacles(pathlib.Path('/packages/Python/results_course4_week2/obstacles.csv'))
        start = code.Point(x=-0.5,y=-0.5)
        end = code.Point(x=-0.28899437717111176,y=-0.21233798104890322)
        self.assertFalse(code.is_collision_free(obstacles,code.Line(start,end)))





if __name__ == '__main__':
    unittest.main()
