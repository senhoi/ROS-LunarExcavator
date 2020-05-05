import numpy as np
import math as m


def is_equal(a, b, rel_tol=1e-09):
    if a is None and b is None:
        return True
    elif (a is None and b is not None) or (a is not None and b is None):
        return False
    else:
        return abs(a - b) <= rel_tol * max(abs(a), abs(b))


def in_middle(val, a, b):
    bound = sorted([a, b])
    return bound[0] <= val <= bound[1]


def in_middle_angle(val, a, b):
    if a > b:
        return in_middle(val, 0, b) or in_middle(val, a, 360)
    else:
        return in_middle(val, a, b)


def get_transform(angle, displacement):
    angle = np.deg2rad(angle)
    rot = np.array([
        [m.cos(angle), -m.sin(angle), displacement[0]],
        [m.sin(angle),  m.cos(angle), displacement[1]],
        [           0,             0,               1]
    ])
    return rot


class Point(object):
    def __init__(self, p):
        if len(p) == 2:
            self.p = np.array(p)
        else:
            raise ValueError

    @property
    def x(self):
        return self.p[0]

    @property
    def y(self):
        return self.p[1]

    def __eq__(self, other):
        return self.p[0] == other.p[0] and self.p[1] == other.p[1]

    def __str__(self):
        return "x:{}, y:{}".format(self.p[0], self.p[1])

    def get_distance_with(self, other):
        """
        Calculate distance between points.
        :type other: Point
        """
        return m.sqrt((self.x - other.x) ** 2 + (self.y - other.y) ** 2)

    def transform(self, rot):
        p = np.matmul(rot, np.array([self.x, self.y, 1]))
        return Point(p[:2])


class Line(object):
    def __init__(self, p1, p2):
        """
        :type p1: Point
        :type p2: Point
        """
        self.p1 = p1
        self.p2 = p2

    @property
    def slope(self):
        if self.p1.x == self.p2.x:
            return None
        else:
            return (self.p2.y - self.p1.y) / (self.p2.x - self.p1.x)

    @property
    def intercept(self):
        if self.p1.x == self.p2.x:
            return None
        else:
            return self.p1.y - self.slope * self.p1.x

    @property
    def angle(self):
        """
        :return: [pi/2, -pi/2)
        """
        if self.slope is None:
            return m.pi / 2  # or -pi/2
        else:
            return m.atan(self.slope)

    def has_intersection_with(self, other):
        """
        :type other: Line
        :return: bool
        """
        # print(self.p1, other.p1)
        # print(self.p2, other.p2)
        return not is_equal(self.slope, other.slope)

    def get_intersection_with(self, other):
        """
        :type other: Line
        :return: Point
        """
        if Line.has_intersection_with(self, other):
            mat_a = np.array([
                [self.p2.y - self.p1.y, self.p1.x - self.p2.x],
                [other.p2.y - other.p1.y, other.p1.x - other.p2.x]
            ])
            mat_b = np.array([
                self.p1.x * (self.p2.y - self.p1.y) - self.p1.y * (self.p2.x - self.p1.x),
                other.p1.x * (other.p2.y - other.p1.y) - other.p1.y * (other.p2.x - other.p1.x),
            ])
            try:
                return Point(np.linalg.solve(mat_a, mat_b))
            except np.linalg.LinAlgError:
                return None
        else:
            return None

    def get_distance_with(self, point):
        """
        Calculate distance to the point.
        :type point: Point
        """
        return abs(self.slope * point.x - point.y + self.intercept) / m.sqrt(self.slope ** 2 + 1)

    def transform(self, rot):
        return Line(self.p1.transform(rot), self.p2.transform(rot))


class LineSegment(Line):
    def __init__(self, p1, p2):
        Line.__init__(self, p1, p2)

    def has_intersection_with(self, other):
        """
        :type other: LineSegment
        :return: bool
        """
        line_intersection = Line.get_intersection_with(self, other)
        if line_intersection is not None:
            return in_middle(line_intersection.x, self.p1.x, self.p2.x) and \
                   in_middle(line_intersection.x, other.p1.x, other.p2.x)
        return False

    def get_intersection_with(self, other):
        """
        :type other: LineSegment
        :return: Point
        """
        line_intersection = Line.get_intersection_with(self, other)
        if line_intersection is not None:
            if in_middle(line_intersection.x, self.p1.x, self.p2.x) and \
                    in_middle(line_intersection.x, other.p1.x, other.p2.x):
                return line_intersection
        return None

    def transform(self, rot):
        return LineSegment(self.p1.transform(rot), self.p2.transform(rot))


class Vector(object):
    def __init__(self, p1, p2):
        """
        :type p1: Point
        :type p2: Point
        """
        self.p1 = p1  # from
        self.p2 = p2  # to

    @property
    def simplified(self):
        return Point([self.p2.x - self.p1.x, self.p2.y - self.p1.y])

    @property
    def angle(self):
        """
        :return: [0, 2pi)
        """
        v0 = Vector(self.p1, Point([self.p1.x + 1, self.p1.y]))
        if self.p2.y < self.p1.y:
            return 2 * m.pi - self.get_angle_with(v0)
        else:
            return self.get_angle_with(v0)

    def get_angle_with(self, other):
        """
        :type other: Vector
        """
        v1 = self.simplified
        v2 = other.simplified
        dot_product = np.dot(v1.p, v2.p)
        len_p1 = np.linalg.norm(v1.p)
        len_p2 = np.linalg.norm(v2.p)
        return m.acos(dot_product / (len_p1 * len_p2))

    def transform(self, rot):
        return Vector(self.p1.transform(rot), self.p2.transform(rot))


class Circle(object):
    def __init__(self, p, r):
        """
        :type p: Point
        :type r: float
        """
        self.p = p
        self.r = r

    def has_intersection_with(self, other):
        """
        :type other: Optional[Line, LineSegment]
        """
        if type(other) == Line:
            return other.get_distance_with(self.p) <= self.r
        elif type(other) == LineSegment:
            p1_in = self.p.get_distance_with(other.p1) < self.r
            p2_in = self.p.get_distance_with(other.p2) < self.r
            if p1_in ^ p2_in:
                return True
            else:
                if p1_in and p2_in:
                    return False
                else:
                    # can be optimized. Find a better way to check the relationship when both points of the line segment are outside.
                    return Circle.get_intersection_with(self, other) is not None
        else:
            raise TypeError

    def get_intersection_with(self, other):
        """
        :type other: Optional[Line, LineSegment]
        """
        is_line = isinstance(other, Line)
        is_segment = isinstance(other, LineSegment)
        if is_line or is_segment:
            if other.slope is not None:
                # can be solved by vector operation as well.
                a = 1 + other.slope ** 2
                b = 2 * (other.intercept - self.p.y) * other.slope - 2 * self.p.x
                c = self.p.x ** 2 + (other.intercept - self.p.y) ** 2 - self.r ** 2
                delta = b ** 2 - 4 * a * c
                if delta < 0:
                    return None
                elif is_equal(delta, 0):
                    x = -b / (2 * a)
                    p = Point([x, other.slope * x + other.intercept])
                    if is_segment:
                        if in_middle(p.x, other.p1.x, other.p2.x):
                            return p
                        else:
                            return None
                    else:
                        return p
                else:
                    x1 = (-b + m.sqrt(delta)) / (2 * a)
                    x2 = (-b - m.sqrt(delta)) / (2 * a)
                    p1, p2 = (Point([x1, other.slope * x1 + other.intercept]),
                              Point([x2, other.slope * x2 + other.intercept]))
                    if is_segment:
                        if in_middle(p1.x, other.p1.x, other.p2.x) and in_middle(p2.x, other.p1.x, other.p2.x):
                            return p1, p2
                        elif in_middle(p1.x, other.p1.x, other.p2.x):
                            return p1
                        elif in_middle(p2.x, other.p1.x, other.p2.x):
                            return p2
                        else:
                            return None
                    else:
                        return p1, p2
            else:
                x = other.p1.x
                y1 = self.p.y + m.sqrt(self.r**2 - (x - self.p.x)**2)
                y2 = self.p.y - m.sqrt(self.r**2 - (x - self.p.x)**2)
                p1, p2 = (Point([x, y1]), Point([x, y2]))
                return p1, p2
        else:
            raise TypeError


class Arc(Circle):
    def __init__(self, p, r, angle1, angle2):
        Circle.__init__(self, p, r)
        self.angle1 = np.deg2rad(angle1)
        self.angle2 = np.deg2rad(angle2)

    @property
    def deg_angle1(self):
        return np.rad2deg(self.angle1)

    @property
    def deg_angle2(self):
        return np.rad2deg(self.angle2)

    def has_intersection_with(self, other):
        """
        :type other: Optional[Line, LineSegment]
        """
        if Circle.has_intersection_with(self, other):
            p_list = Circle.get_intersection_with(self, other)
            if type(p_list) == tuple:
                for p in p_list:
                    if in_middle_angle(Vector(self.p, p).angle, self.angle1, self.angle2):
                        return True
                else:
                    return False
            else:
                return in_middle_angle(Vector(self.p, p_list).angle, self.angle1, self.angle2)
        else:
            return False

    def get_intersection_with(self, other):
        """
        :type other: Optional[Line, LineSegment]
        """
        if Circle.has_intersection_with(self, other):
            p_list = Circle.get_intersection_with(self, other)
            res = []
            if type(p_list) == tuple:
                for p in p_list:
                    if in_middle_angle(Vector(self.p, p).angle, self.angle1, self.angle2):
                        res.append(p)
            else:
                if in_middle_angle(Vector(self.p, p_list).angle, self.angle1, self.angle2):
                    res.append(p_list)
            if not res:
                return None
            elif len(res) == 1:
                return res[0]
            elif len(res) == 2:
                return tuple(res)
            else:
                return None
        else:
            return None

