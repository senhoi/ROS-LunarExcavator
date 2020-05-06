#!/usr/bin/env python
from geometry import *
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.lines as lines
import matplotlib.patches as patches

from geometry import LineSegment

import rospy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64

from excavation_force import ExcavationForce

import os
import json


class Excavator(object):
    def __init__(self):
        config_file_path = os.path.join(os.path.dirname(__file__), 'config/param_moon.json')
        with open(config_file_path) as config_file:
            param = json.load(config_file)
            param = dict([(str(k), v) for k, v in param.items()])

        self.state = 0
        self.old_angle = 0
        self.arm_angle = 359
        self.arm_length = param["arm_length"]
        self.arm_speed = 0
        self.scooper_angle = 45
        self.scooper_length = param["tool_length"]

        self.tool_depth = 0
        self.tool_angle = 0
        self.center_height = 2.45
        self.center_height_step = 0.02
        self.translation_distance = 0

        self.excavator_force = ExcavationForce(param)

        self.scooper_segment = None  # 1

        self.origin_p = Point([0, 0])
        self.left_inf_p = Point([-10, 0])
        self.right_inf_p = Point([10, 0])

        self.left_segment0 = LineSegment(self.left_inf_p, self.origin_p)
        self.left_segment1 = LineSegment(self.left_inf_p, self.origin_p)
        self.middle_arc_old = None
        self.middle_arc_new = None
        self.right_segment1 = LineSegment(self.origin_p, self.right_inf_p)
        self.right_segment0 = LineSegment(self.origin_p, self.right_inf_p)
        self.circle = None

        self.left_segment1_temp = None
        self.middle_arc_temp_old = None
        self.middle_arc_temp_new = None
        self.right_segment1_temp = None

        self.left_p_new = self.origin_p
        self.left_p_old = self.origin_p
        self.right_p_new = self.origin_p
        self.right_p_old = self.origin_p
        self.center_p = Point([0, self.center_height])

        self.scooper_p_end = None

        self.fig = plt.figure()
        self.ax = plt.gca()
        self.ax.set_aspect('equal')
        plt.show(block=False)
        plt.draw()
        self.plot_middle_arc_temp_old = None
        self.plot_middle_arc_temp_new = None
        self.plot_left_segment1_temp = None
        self.plot_right_segment1_temp = None
        self.plot_circle = None
        self.plot_left_segment0 = None
        self.plot_right_segment0 = None
        self.plot_scooper_segment = None

        self.plot_update_flag = False
        self.plot_in_ground_flag = False

    def update_scooper_position(self):
        scooper_s = LineSegment(Point([0, 0]), Point([self.scooper_length, 0]))
        scooper_a = scooper_s.transform(get_transform(self.scooper_angle, [self.arm_length, 0]))
        self.scooper_segment = scooper_a.transform(get_transform(self.arm_angle, [0, self.center_height]))
        self.scooper_p_end = self.scooper_segment.p2

    def update_scooper_intersection(self):
        if self.state == 0:
            self.tool_depth = 0
            self.tool_angle = 0
            if not self.plot_update_flag:
                self.left_segment1_temp = self.left_segment1
                self.middle_arc_temp_old = self.middle_arc_old
                self.middle_arc_temp_new = None
                self.right_segment1_temp = self.right_segment1
            else:
                self.left_segment1_temp = None
                self.middle_arc_temp_old = None
                self.middle_arc_temp_new = self.middle_arc_new
                self.right_segment1_temp = None
        elif self.state == 1:  # only intersect with left_segment1
            p = self.left_segment1.get_intersection_with(self.scooper_segment)
            self.tool_depth = p.get_distance_with(self.scooper_p_end)
            self.tool_angle = abs(self.scooper_segment.angle)
            self.left_segment1_temp = LineSegment(p, self.left_p_old)
            self.middle_arc_temp_old = self.middle_arc_old
            v = Vector(self.middle_arc_new.p, p)
            self.middle_arc_temp_new = Arc(self.middle_arc_new.p, self.middle_arc_new.r,
                                           self.middle_arc_new.deg_angle1, np.rad2deg(v.angle))
            self.right_segment1_temp = self.right_segment1
        elif self.state == 2:  # only intersect with middle_arc_old
            p = self.middle_arc_old.get_intersection_with(self.scooper_segment)
            self.tool_depth = p.get_distance_with(self.scooper_p_end)
            self.tool_angle = 90 - self.scooper_angle
            self.left_segment1_temp = None
            v1 = Vector(self.middle_arc_old.p, p)
            self.middle_arc_temp_old = Arc(self.middle_arc_old.p, self.middle_arc_old.r, np.rad2deg(v1.angle),
                                           self.middle_arc_old.deg_angle2)
            v2 = Vector(self.middle_arc_new.p, self.scooper_p_end)
            self.middle_arc_temp_new = Arc(self.middle_arc_new.p, self.middle_arc_new.r, self.middle_arc_new.deg_angle1,
                                           np.rad2deg(v2.angle))
            self.right_segment1_temp = self.right_segment1
        elif self.state == 3:  # intersect with right_segment1
            if self.middle_arc_old is not None and self.middle_arc_old.has_intersection_with(self.scooper_segment):
                p1 = self.middle_arc_old.get_intersection_with(self.scooper_segment)
                p2 = self.right_segment1.get_intersection_with(self.scooper_segment)
                self.tool_depth = p1.get_distance_with(p2)
                self.tool_angle = 90 - self.scooper_angle
                self.left_segment1_temp = None
                v1 = Vector(self.middle_arc_old.p, p1)
                self.middle_arc_temp_old = Arc(self.middle_arc_old.p, self.middle_arc_old.r, np.rad2deg(v1.angle),
                                               self.middle_arc_old.deg_angle2)
                self.middle_arc_temp_new = self.middle_arc_new
                self.right_segment1_temp = LineSegment(self.right_p_old, p2)
            else:
                p = self.right_segment1.get_intersection_with(self.scooper_segment)
                self.tool_depth = p.get_distance_with(self.scooper_p_end)
                self.tool_angle = abs(self.scooper_segment.angle)
                self.left_segment1_temp = None
                self.middle_arc_temp_old = None
                v2 = Vector(self.center_p, self.scooper_p_end)
                self.middle_arc_temp_new = Arc(self.middle_arc_new.p, self.middle_arc_new.r,
                                               self.middle_arc_new.deg_angle1, np.rad2deg(v2.angle))
                self.right_segment1_temp = LineSegment(p, self.right_p_new)
        else:
            self.tool_depth = 0

    def update_ground_line(self):
        self.circle = Circle(self.center_p, self.scooper_p_end.get_distance_with(self.center_p))
        self.left_p_old = self.left_p_new
        self.right_p_old = self.right_p_new
        self.left_p_new = self.circle.get_intersection_with(self.left_segment0)
        self.right_p_new = self.circle.get_intersection_with(self.right_segment0)
        if self.left_p_new and self.right_p_new:
            self.left_segment0 = LineSegment(self.left_inf_p, self.left_p_new)
            self.right_segment0 = LineSegment(self.right_inf_p, self.right_p_new)
            self.left_segment1 = LineSegment(self.left_p_new, self.left_p_old)
            self.right_segment1 = LineSegment(self.right_p_new, self.right_p_old)
            self.middle_arc_old = self.middle_arc_new
            v1 = Vector(self.center_p, self.left_p_new)
            v2 = Vector(self.center_p, self.right_p_new)
            self.middle_arc_new = Arc(self.circle.p, self.circle.r, np.rad2deg(v1.angle), np.rad2deg(v2.angle))
        else:
            self.plot_in_ground_flag = True
            self.left_segment1 = LineSegment(self.left_segment0.p2, Point([-self.circle.r, self.center_height]))
            self.right_segment1 = LineSegment(self.right_segment0.p2, Point([self.circle.r, self.center_height]))
            self.middle_arc_old = self.middle_arc_new
            self.middle_arc_new = Arc(self.circle.p, self.circle.r, 180, 360)

    def render(self):
        if self.middle_arc_temp_old is not None:
            # if self.plot_middle_arc_temp_old is not None:
            self.plot_middle_arc_temp_old = \
                patches.Arc(self.middle_arc_temp_old.p.p, 2 * self.middle_arc_temp_old.r,
                            2 * self.middle_arc_temp_old.r,
                            theta1=self.middle_arc_temp_old.deg_angle1, theta2=self.middle_arc_temp_old.deg_angle2,
                            color='r')
            self.ax.add_patch(self.plot_middle_arc_temp_old)
        if self.middle_arc_temp_new is not None:
            # if self.plot_middle_arc_temp_new is not None:
            #     self.ax.remove(self.plot_middle_arc_temp_new)
            self.plot_middle_arc_temp_new = \
                patches.Arc(self.middle_arc_temp_new.p.p, 2 * self.middle_arc_temp_new.r,
                            2 * self.middle_arc_temp_new.r,
                            theta1=self.middle_arc_temp_new.deg_angle1, theta2=self.middle_arc_temp_new.deg_angle2,
                            color='k')
            self.ax.add_patch(self.plot_middle_arc_temp_new)
        if self.left_segment1_temp is not None:
            # if self.plot_middle_arc_temp_new is not None:
            #     self.ax.remove(self.plot_left_segment1_temp)
            self.plot_left_segment1_temp = \
                lines.Line2D([self.left_segment1_temp.p1.x, self.left_segment1_temp.p2.x],
                             [self.left_segment1_temp.p1.y, self.left_segment1_temp.p2.y], color='r')
            self.ax.add_artist(self.plot_left_segment1_temp)
        if self.right_segment1_temp is not None:
            # if self.plot_middle_arc_temp_new is not None:
            #     self.ax.remove(self.plot_right_segment1_temp)
            self.plot_right_segment1_temp = \
                lines.Line2D([self.right_segment1_temp.p1.x, self.right_segment1_temp.p2.x],
                             [self.right_segment1_temp.p1.y, self.right_segment1_temp.p2.y], color='r')
            self.ax.add_artist(self.plot_right_segment1_temp)
        # if self.plot_circle is not None:
        #     self.ax.remove(self.plot_circle)
        self.plot_circle = \
            patches.Circle(self.circle.p.p, self.circle.r, color='g', fill=False, linestyle='--')
        self.ax.add_artist(self.plot_circle)
        # if self.plot_left_segment0 is not None:
        #     self.ax.remove(self.plot_left_segment0)
        self.plot_left_segment0 = \
            lines.Line2D([self.left_segment0.p1.x, self.left_segment0.p2.x],
                         [self.left_segment0.p1.y, self.left_segment0.p2.y])
        self.ax.add_artist(self.plot_left_segment0)
        # if self.plot_right_segment0 is not None:
        #     self.ax.remove(self.plot_right_segment0)
        self.plot_right_segment0 = \
            lines.Line2D([self.right_segment0.p1.x, self.right_segment0.p2.x],
                         [self.right_segment0.p1.y, self.right_segment0.p2.y])
        self.ax.add_artist(self.plot_right_segment0)
        # if self.plot_scooper_segment is not None:
        #     self.ax.remove(self.plot_scooper_segment)
        self.plot_scooper_segment = \
            lines.Line2D([self.scooper_segment.p2.x, self.scooper_segment.p1.x, self.center_p.x],
                         [self.scooper_segment.p2.y, self.scooper_segment.p1.y, self.center_p.y])
        self.ax.add_artist(self.plot_scooper_segment)
        if self.plot_in_ground_flag:
            self.ax.add_artist(lines.Line2D([self.left_segment1.p2.x, self.left_segment1.p1.x],
                                            [self.left_segment1.p2.y, self.left_segment1.p1.y]))
            self.ax.add_artist(lines.Line2D([self.right_segment1.p2.x, self.right_segment1.p1.x],
                                            [self.right_segment1.p2.y, self.right_segment1.p1.y]))
        plt.text(0, 0, str(self.state))
        plt.pause(0.001)
        plt.cla()
        plt.xlim([-4, 4])
        plt.ylim([-6, 6])

    def run(self, t):
        # self.old_angle = self.arm_angle
        # old_state = self.state
        # self.arm_angle = (t * 180) % 360
        print self.old_angle, self.arm_angle
        if self.old_angle > 330 and self.arm_angle < 30:
            self.center_height -= self.center_height_step
            self.center_p = Point([0, self.center_height])
            self.translation_distance += self.center_height_step
        self.update_scooper_position()
        if self.middle_arc_old is None:
            if self.left_segment1.has_intersection_with(self.scooper_segment):
                self.state = 1
            elif self.right_segment1.has_intersection_with(self.scooper_segment):
                self.state = 3
            else:
                self.state = 0
        else:
            if self.left_segment1.has_intersection_with(self.scooper_segment):
                self.state = 1
            elif self.middle_arc_old.has_intersection_with(self.scooper_segment):
                if self.right_segment1.has_intersection_with(self.scooper_segment):
                    self.state = 3
                else:
                    self.state = 2
            else:
                if self.right_segment1.has_intersection_with(self.scooper_segment):
                    self.state = 3
                else:
                    self.state = 0
        if self.state == 1:
            self.plot_update_flag = True
        # if old_state == 2 and self.state == 0:
        #     rospy.logerr("Height step is too large!")
        if self.old_angle > 330 and self.arm_angle < 30:
            self.plot_update_flag = False
            self.update_ground_line()
        self.update_scooper_intersection()
        self.excavator_force.tool_depth = self.tool_depth
        self.excavator_force.tool_speed = self.arm_length * self.arm_speed
        self.excavator_force.rank_angle = np.deg2rad(self.scooper_angle)
        self.render()

    def set_states_parameter(self, scooper_angle, arm_angle, arm_speed):
        # Note that an offset value might be required to add to scooper angle
        self.scooper_angle = scooper_angle  # deg
        self.old_angle = self.arm_angle
        self.arm_angle = arm_angle  # deg
        self.arm_speed = arm_speed  # rad/s

    def get_translation_distance(self):
        return self.translation_distance


if __name__ == '__main__':
    node = rospy.init_node("core")
    excavator = Excavator()

    # def listener_cb(msg):
    #     excavator.set_states_parameter(msg.x, msg.y, msg.z)
    #
    # def talker_pub(exc_force_x, exc_force_y, height_command):
    #     msg = Vector3()
    #     msg.x = exc_force_x
    #     msg.y = exc_force_y
    #     msg.z = height_command
    #     talker_excavator.publish(msg)
    #
    # # publish excavation force(x,y) and height_command
    # talker_excavator = rospy.Publisher("excavation_forces", Vector3, queue_size=5)
    # # receive arm_angle, scooper_angle, tool_speed
    # listener_excavator = rospy.Subscriber("excavation_states", Vector3, listener_cb)
    start_t = rospy.get_time()
    while not rospy.is_shutdown():
        t = rospy.get_time() - start_t
        excavator.run(t)
        f, f_x, f_y = excavator.excavator_force.SwickPerumpralModel()
        print f, f_x, f_y
        # talker_pub(f_x, f_y, excavator.center_height)

