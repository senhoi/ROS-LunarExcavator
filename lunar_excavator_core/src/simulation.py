#!/usr/bin/env python

import rospy
from math import cos
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from excavator import Excavator
import numpy as np


class Joint(object):
    def __init__(self, name):
        self.name = name
        self.position = 0
        self.velocity = 0
        self.effort = 0

    def update(self, position, velocity, effort):
        self.position = position
        self.velocity = velocity
        self.effort = effort


auto_release_flag = False
rotation_velocity = 0
scooper_angle = 1.5
slope_angle = 0
translation_distance = 0

joints_group = {'excavation_slope_control_joint': Joint('excavation_slope_control_joint'),
                'excavation_1st_translation_joint': Joint('excavation_1st_translation_joint'),
                'excavation_2nd_translation_joint': Joint('excavation_2nd_translation_joint'),
                'excavation_arm_translation_joint': Joint('excavation_arm_translation_joint'),
                'excavation_arm_control_joint': Joint('excavation_arm_control_joint'),
                'scooper_control_joint': Joint('scooper_control_joint')}


def cmd_vel_callback(msg):
    global slope_angle
    global translation_distance
    translation_distance = msg.linear.x
    slope_angle = msg.angular.z / 180.0 * 3.1416


def cmd_pose_callback(msg):
    global rotation_velocity
    rotation_velocity = msg.x


def joint_callback(msg):
    global joints_group
    for name, position, velocity, effort in zip(msg.name, msg.position, msg.velocity, msg.effort):
        joints_group[name].update(position, velocity, effort)


def cmd_event_callback(msg):
    global scooper_angle
    if msg.data == 'EVENT_0':  # pressed space
        auto_release_flag = not auto_release_flag
    elif msg.data == 'EVENT_1':  # pressed /
        scooper_angle = 1.5
        print "scooper_angle:{}".format(scooper_angle)
    elif msg.data == 'EVENT_2':  # pressed *
        scooper_angle = 0
        print "scooper_angle:{}".format(scooper_angle)
    elif msg.data == 'EVENT_3':  # pressed -
        # maybe do some 'wait for service' here
        reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        reset_simulation()
        scooper_angle = 1.5
        print "reset simulation"


listener_joint = rospy.Subscriber("joint_states", JointState, joint_callback)
talker_slope_control_joint = rospy.Publisher("excavation_slope_controller/command", Float64, queue_size=5)
talker_1st_translation_joint = rospy.Publisher("excavation_1st_translation_controller/command", Float64, queue_size=5)
talker_2nd_translation_joint = rospy.Publisher("excavation_2nd_translation_controller/command", Float64, queue_size=5)
talker_arm_translation_joint = rospy.Publisher("excavation_arm_translation_controller/command", Float64, queue_size=5)
talker_excavation_joint = rospy.Publisher("excavation_arm_controller/command", Float64, queue_size=5)
talker_scooper_joint = rospy.Publisher("scooper_controller/command", Float64, queue_size=5)

listener_keyboard_vel = rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback)
listener_keyboard_pose = rospy.Subscriber("/cmd_pose", Vector3, cmd_pose_callback)
listener_keyboard_event = rospy.Subscriber("/cmd_event", String, cmd_event_callback)

node = rospy.init_node("excavator_sim")
rate = rospy.Rate(100)

excavator = Excavator()


def calc_arm_translation_joint_angle(excavation_joint_angle):
    excavation_joint_angle = excavation_joint_angle % (2 * 3.1416)
    if 3.1416 / 2 < excavation_joint_angle < 3 * 3.1416 / 2:
        return 0.06 * cos(2 * excavation_joint_angle) + 0.06
    else:
        return 0


old_angle = 0
new_angle = 0
sim_start_flag = False
start_t = rospy.get_time()
while not rospy.is_shutdown():
    try:
        global translation_distance
        # comment below line to ban auto height adjustment
        translation_distance = excavator.get_translation_distance()/2
        talker_1st_translation_joint.publish(Float64(translation_distance))
        talker_2nd_translation_joint.publish(Float64(-translation_distance))
        talker_slope_control_joint.publish(Float64(slope_angle))
        talker_excavation_joint.publish(Float64(rotation_velocity))
        talker_scooper_joint.publish(Float64(scooper_angle))

        excavation_joint_angle = joints_group['excavation_arm_control_joint'].position
        talker_arm_translation_joint.publish(Float64(calc_arm_translation_joint_angle(scooper_angle)))
    except:
        pass
    if auto_release_flag:
        pass
    try:
        t = rospy.get_time() - start_t
        old_angle = new_angle
        new_angle = (np.rad2deg(joints_group['excavation_arm_control_joint'].position) + 270) % 360
        excavator.set_states_parameter(np.rad2deg(joints_group['scooper_control_joint'].position),
                                       new_angle,
                                       joints_group['excavation_arm_control_joint'].velocity)
        if old_angle > 330 and new_angle < 30:
            sim_start_flag = True
        if sim_start_flag:
            excavator.run(t)
            f, f_x, f_y = excavator.excavator_force.SwickPerumpralModel()
            print f, f_x, f_y
        rate.sleep()
    except rospy.ROSTimeMovedBackwardsException, e:
        rospy.logwarn("ROSTimeMovedBackwardsException during sleep(). Continue anyway...")
