from typing import Union
import matplotlib.pyplot as plt


class Point(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __add__(self, other):
        return Point(self.x + other.x, self.y + other.y)

    def __mul__(self, other):
        return Point(self.x * other, self.y * other)

    def __str__(self):
        return "({}, {})".format(self.x, self.y)

    @staticmethod
    def compose_point_list(xs, ys):
        p_list = []
        for x, y in zip(xs, ys):
            p_list.append(Point(x, y))
        return p_list

    @staticmethod
    def decompose_point_list(p_list):
        xs = []
        ys = []
        for point in p_list:
            xs.append(point.x)
            ys.append(point.y)
        return xs, ys


class Vector(object):
    def __init__(self, start_point, end_point):
        self.start_point = start_point
        self.end_point = end_point


class LineSegment(object):
    def __init__(self, point1, point2):
        """

        :type point1: Point
        :type point2: Point
        """
        self.point1 = point1
        self.point2 = point2

    def __iter__(self):
        p_list = [self.point1, self.point2]
        for point in p_list:
            yield point


class Arc(object):
    def __init__(self, center, theta1, theta2, radius):
        """

        :type center: Point
        :type theta1: float
        :type theta2: float
        :type radius: float
        """
        self.center = center
        self.begin_angle = theta1
        self.end_angle = theta2
        self.radius = radius


def judge_line_segment_intersection(seg1, seg2):
    pass


def calculate_line_segment_intersection(seg1, seg2):
    pass


if __name__ == '__main__':
    p1 = Point(0, -3.0)
    p2 = Point(0.1, -3.1)
    xs, ys = Point.decompose_point_list([p1, p2])
    plt.plot(xs, ys)
    plt.xlim(-0.2, 0.2)
    plt.ylim(-4, 0)
    plt.grid()
    plt.gca().set_aspect('equal', adjustable='box')
    plt.show()
