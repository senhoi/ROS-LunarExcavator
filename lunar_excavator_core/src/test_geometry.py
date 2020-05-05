import unittest
from geometry import *
from math import *
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches


class TestGeometry(unittest.TestCase):
    def test_point_distance(self):
        p1 = Point([0, 2 * sqrt(3)])
        p2 = Point([2, 0])
        self.assertEqual(is_equal(p1.get_distance_with(p2), 4), True)

    def test_line_slope_and_intercept(self):
        line = Line(Point([0, 1]), Point([1, 0]))
        self.assertEqual(is_equal(line.slope, -1), True)
        self.assertEqual(is_equal(line.intercept, 1), True)

    def test_line_slope_and_intercept_none(self):
        line = Line(Point([1, 1]), Point([1, 3]))
        self.assertEqual(line.slope, None)
        self.assertEqual(line.intercept, None)

    def test_line_angle(self):
        line = Line(Point([0, 1]), Point([1, 0]))
        self.assertEqual(is_equal(np.rad2deg(line.angle), -45), True)

    def test_line_angle_none(self):
        line = Line(Point([1, 1]), Point([1, 3]))
        self.assertEqual(is_equal(np.rad2deg(line.angle), 90), True)

    def test_line_has_no_intersection(self):
        line1 = Line(Point([0, 1]), Point([1, 0]))
        line2 = Line(Point([0, 3]), Point([3, 0]))
        self.assertEqual(line1.has_intersection_with(line2), False)

    def test_line_has_intersection(self):
        line1 = Line(Point([0, 1]), Point([1, 0]))
        line2 = Line(Point([0, 3]), Point([3, 4]))
        self.assertEqual(line1.has_intersection_with(line2), True)

    def test_line_get_intersection(self):
        line1 = Line(Point([0, 2]), Point([2, 0]))
        line2 = Line(Point([-1, -1]), Point([3, 3]))
        self.assertEqual(line1.get_intersection_with(line2), Point([1, 1]))

    def test_line_get_distance(self):
        p = Point([1, 1])
        line = Line(Point([1, 0]), Point([0, 1]))
        self.assertEqual(is_equal(line.get_distance_with(p), sqrt(2) / 2), True)

    def test_segment_has_intersection_none(self):
        p1 = Point([2, 1])
        p2 = Point([1, 2])
        p3 = Point([3, 4])
        p4 = Point([4, 2])
        line1 = LineSegment(p1, p2)
        line2 = LineSegment(p3, p4)
        self.assertEqual(line1.has_intersection_with(line2), False)

    def test_segment_has_intersection(self):
        p1 = Point([2, 1])
        p2 = Point([1, 2])
        p3 = Point([3, 4])
        p4 = Point([-1, -1])
        line1 = LineSegment(p1, p2)
        line2 = LineSegment(p3, p4)
        self.assertEqual(line1.has_intersection_with(line2), True)

    def test_segment_get_intersection(self):
        p1 = Point([2, 1])
        p2 = Point([1, 2])
        p3 = Point([3, 4])
        p4 = Point([4, 2])
        line1 = LineSegment(p1, p2)
        line2 = LineSegment(p3, p4)
        self.assertEqual(line1.get_intersection_with(line2), None)

    def test_vector_simplified(self):
        v = Vector(Point([5, 5]), Point([10, 8]))
        self.assertEqual(v.simplified, Point([5, 3]))

    def test_vector_angle_with(self):
        v1 = Vector(Point([5, 5]), Point([8, 8]))
        v2 = Vector(Point([3, 3]), Point([6, 0]))
        self.assertEqual(is_equal(np.rad2deg(v1.get_angle_with(v2)), 90), True)

    def test_vector_angle(self):
        v1 = Vector(Point([5, 5]), Point([8, 8]))
        self.assertEqual(is_equal(np.rad2deg(v1.angle), 45), True)
        v2 = Vector(Point([3, 3]), Point([6, 0]))
        self.assertEqual(is_equal(np.rad2deg(v2.angle), 315), True)

    def test_circle_has_two_intersections_with_line(self):
        line = LineSegment(Point([2, 2]), Point([-2, -2]))
        circle = Circle(Point([0, 0]), sqrt(2))
        self.assertEqual(circle.has_intersection_with(line), True)

    def test_circle_has_one_intersections_with_line(self):
        line = LineSegment(Point([-5, 5]), Point([5, 5]))
        circle = Circle(Point([0, 0]), 5)
        self.assertEqual(circle.has_intersection_with(line), True)

    def test_circle_has_no_intersections_with_line(self):
        line = LineSegment(Point([1, 10]), Point([10, 2]))
        circle = Circle(Point([0, 0]), 5)
        self.assertEqual(circle.has_intersection_with(line), False)

    def test_circle_get_two_intersections_with_line(self):
        line = LineSegment(Point([2, 2]), Point([-2, -2]))
        circle = Circle(Point([0, 0]), sqrt(2))
        for res, p in zip(list(circle.get_intersection_with(line)), [Point([1, 1]), Point([-1, -1])]):
            self.assertEqual(res, p)

    def test_circle_get_one_intersections_with_line(self):
        line = LineSegment(Point([-5, 5]), Point([5, 5]))
        circle = Circle(Point([0, 0]), 5)
        self.assertEqual(circle.get_intersection_with(line), Point([0, 5]))

    def test_circle_get_no_intersections_with_line(self):
        line = LineSegment(Point([1, 10]), Point([10, 2]))
        circle = Circle(Point([0, 0]), 5)
        self.assertEqual(circle.get_intersection_with(line), None)

    def test_circle_has_no_intersection_with_segment_inner(self):
        segment = LineSegment(Point([2, 2]), Point([-2, -2]))
        circle = Circle(Point([0, 0]), 5)
        self.assertEqual(circle.has_intersection_with(segment), False)

    def test_circle_has_no_intersection_with_segment_outer(self):
        segment = LineSegment(Point([1, 10]), Point([10, 2]))
        circle = Circle(Point([0, 0]), 5)
        self.assertEqual(circle.has_intersection_with(segment), False)

    def test_circle_has_one_intersections_with_segment(self):
        segment = LineSegment(Point([-5, 5]), Point([0, 0]))
        circle = Circle(Point([0, 0]), 5)
        self.assertEqual(circle.has_intersection_with(segment), True)

    def test_circle_has_two_intersections_with_segment(self):
        segment = LineSegment(Point([-5, 5]), Point([5, -5]))
        circle = Circle(Point([0, 0]), 5)
        self.assertEqual(circle.has_intersection_with(segment), True)

    def test_circle_get_two_intersections_with_segment(self):
        segment = LineSegment(Point([2, 2]), Point([-2, -2]))
        circle = Circle(Point([0, 0]), sqrt(2))
        for res, p in zip(list(circle.get_intersection_with(segment)), [Point([1, 1]), Point([-1, -1])]):
            self.assertEqual(res, p)

    def test_circle_get_one_intersection_with_segment(self):
        segment = LineSegment(Point([-5, 5]), Point([0, 0]))
        circle = Circle(Point([0, 0]), sqrt(2))
        self.assertEqual(circle.get_intersection_with(segment), Point([-1, 1]))

    def test_circle_get_no_intersection_with_segment(self):
        segment = LineSegment(Point([-10, 10]), Point([-5, 5]))
        circle = Circle(Point([0, 0]), 5)
        circle.get_intersection_with(segment)
        self.assertEqual(circle.get_intersection_with(segment), None)

    def test_arc_creation(self):
        arc = Arc(Point([0, 0]), 5, 270, 120)
        fig, ax = plt.subplots()
        arc1 = patches.Arc(arc.p.p, arc.r, arc.r,
                           theta1=arc.deg_angle1, theta2=arc.deg_angle2)
        ax.add_patch(arc1)
        plt.xlim([-5, 5])
        plt.ylim([-5, 5])
        plt.show()

    def test_arc_has_no_intersection(self):
        segment = LineSegment(Point([-2, -2]), Point([0, -2]))
        arc = Arc(Point([0, 0]), 5, 270, 120)
        self.assertEqual(arc.has_intersection_with(segment), False)

    def test_arc_get_intersection(self):
        segment = LineSegment(Point([2, 2]), Point([-2, -2]))
        arc = Arc(Point([0, 0]), sqrt(2), 270, 120)
        print(arc.get_intersection_with(segment))
        self.assertEqual(arc.get_intersection_with(segment), Point([1, 1]))


if __name__ == '__main__':
    unittest.main()
