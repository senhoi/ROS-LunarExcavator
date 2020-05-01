#!/usr/bin/env python

import select
import sys
import termios
import tty

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from std_msgs.msg import String


class TeleopInfo(object):
    def __init__(self):
        # moving
        self.vel = [0, 0, 0, 0]
        self.linear_acc = 0.01
        self.angular_acc = 0.1
        self.linear_vel_limit = (0, 2)  # Unit: m/s
        self.angular_vel_limit = (-10.0, 3.0)  # Unit: deg/s

        # pose
        self.rpy = [180, 0, 0]
        self.pose_vel = 5
        self.roll_limit = (-45.0, 720)  # Unit: deg
        self.pitch_limit = (-45.0, 45.0)  # Unit: deg
        self.yaw_limit = (-45.0, 45.0)  # Unit: deg

        # event
        self.event = "NO_EVENT"

    @staticmethod
    def limiting(val, limit):
        if val < limit[0]:
            return limit[0]
        elif val > limit[1]:
            return limit[1]
        else:
            return val

    def vel_limiting(self):
        self.vel[0] = self.limiting(self.vel[0], self.linear_vel_limit)
        self.vel[1] = self.limiting(self.vel[1], self.linear_vel_limit)
        self.vel[2] = self.limiting(self.vel[2], self.linear_vel_limit)
        self.vel[3] = self.limiting(self.vel[3], self.angular_vel_limit)

    def pose_limiting(self):
        self.rpy[0] = self.limiting(self.rpy[0], self.roll_limit)
        self.rpy[1] = self.limiting(self.rpy[1], self.pitch_limit)
        self.rpy[2] = self.limiting(self.rpy[2], self.yaw_limit)

    def vel_reset(self):
        self.vel = [0, 0, 0, 0]

    def pose_reset(self):
        self.rpy = [0, 0, 0]

    def event_reset(self):
        self.event = "NO_EVENT"

    def velocity_msg(self):
        return """
        x:\t{}\ty:\t{}\tz:\t{}
        step:\t{}
        
        omega:\t{}
        step:\t{}
        """.format(self.vel[0], self.vel[1], self.vel[2],
                   self.linear_acc,
                   self.vel[3],
                   self.angular_acc)

    def pose_msg(self):
        return """
        r:\t{}\tp:\t{}\ty:\t{}
        step:\t{}
        """.format(self.rpy[0], self.rpy[1], self.rpy[2],
                   self.pose_vel)

    def limit_msg(self):
        return """
        move:\t {} ~ {} m/s
        turn:\t {} ~ {} deg/s
        
        roll:\t {} ~ {} deg
        pitch:\t {} ~ {} deg
        yaw:\t {} ~ {} deg
        """.format(self.linear_vel_limit[0], self.linear_vel_limit[1],
                   self.angular_vel_limit[0], self.angular_vel_limit[1],
                   self.roll_limit[0], self.roll_limit[1],
                   self.pitch_limit[0], self.pitch_limit[1],
                   self.yaw_limit[0], self.yaw_limit[1])

    def event_msg(self):
        return """
        event: {}
        """.format(self.event)


class Teleop(object):
    movingBindings = {
        '8': (1, 0, 0, 0),
        '7': (1, 0, 0, 1),
        '9': (1, 0, 0, -1),
        '5': (0, 0, 0, 0),
        '4': (0, 0, 0, 1),
        '6': (0, 0, 0, -1),
        '2': (-1, 0, 0, 0),
        '1': (-1, 0, 0, 1),
        '3': (-1, 0, 0, -1),
    }

    strafingBindings = {
        '8': (1, 0, 0, 0),
        '7': (1, 1, 0, 0),
        '9': (1, -1, 0, 0),
        '5': (0, 0, 0, 0),
        '4': (0, 1, 0, 0),
        '6': (0, -1, 0, 0),
        '2': (-1, 0, 0, 0),
        '1': (-1, 1, 0, 0),
        '3': (-1, -1, 0, 0),
    }

    poseBindings = {
        'k': (1, 0, 0),
        'h': (-1, 0, 0),
        'u': (0, 1, 0),
        'm': (0, -1, 0),
        'i': (0, 0, 1),
        'n': (0, 0, -1),
    }

    stepBindings = {
        'q': (1.1, 1.0, 1.0),
        'z': (0.9, 1.0, 1.0),
        'w': (1.0, 1.1, 1.0),
        'x': (1.0, 0.9, 1.0),
        'e': (1.0, 1.0, 1.1),
        'c': (1.0, 1.0, 0.9),
    }

    eventBindings = {
        ' ': 'EVENT_0',
        '/': 'EVENT_1',
        '*': 'EVENT_2',
        '-': 'EVENT_3',
    }

    def __init__(self):
        self.settings = termios.tcgetattr(sys.stdin)
        self.mode = 'moving'
        self.info = TeleopInfo()
        self.talker_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.talker_pose = rospy.Publisher('cmd_pose', Vector3, queue_size=1)
        self.talker_event = rospy.Publisher('cmd_event', String, queue_size=1)
        rospy.init_node('lunar_excavator_input')
        print self.instruction_msg()

    def _get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        ch = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return ch

    def instruction_msg(self):
        return """
        Keyboard teleoperator for JunGo
        + : switch moving mode         ? : get instruction
        . : show the limit             ctrl + c : exit.
        ---------------------------
        Topic : cmd_vel     Msg : geometry_msgs.msg/Twist
        Moving around:
           7    8    9
           4    5    6
           1    2    3
        
        0 : stop
        q : linear step up     w : angular step up
        z : linear step down   x : angular step down
        ---------------------------
        Topic : cmd_pose     Msg : geometry_msgs.msg/Pose
        u : pitch up    h : roll down  i : yaw up
        m : pitch down  k : roll up    n : yaw down
        
        j : reset
        e : pose step up
        c : pose step down
        ---------------------------
        Topic : cmd_event     Msg : std_msgs/String
        [space] : event 0
        / : event 1     * : event 2    - : event 3
        """

    def limit_msg(self):
        return self.info.limit_msg()

    def handle_keys(self, ch):
        if ch in Teleop.movingBindings or ch in Teleop.strafingBindings or ch == '0':
            if ch == '0':
                self.info.vel_reset()
            else:
                if self.mode == 'moving':
                    bindings = Teleop.movingBindings
                else:
                    bindings = Teleop.strafingBindings
                self.info.vel[0] += self.info.linear_acc * bindings[ch][0]
                self.info.vel[1] += self.info.linear_acc * bindings[ch][1]
                self.info.vel[2] += self.info.linear_acc * bindings[ch][2]
                self.info.vel[3] += self.info.angular_acc * bindings[ch][3]
                self.info.vel_limiting()
            twist = Twist()
            twist.linear.x = self.info.vel[0]
            twist.linear.y = self.info.vel[1]
            twist.linear.z = self.info.vel[2]
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = self.info.vel[3]
            self.talker_vel.publish(twist)
            print self.info.velocity_msg()
        elif ch in Teleop.poseBindings or ch == 'j':
            if ch == 'j':
                self.info.pose_reset()
            else:
                bindings = Teleop.poseBindings
                self.info.rpy[0] += self.info.pose_vel * bindings[ch][0]
                self.info.rpy[1] += self.info.pose_vel * bindings[ch][1]
                self.info.rpy[2] += self.info.pose_vel * bindings[ch][2]
                self.info.pose_limiting()
            vector = Vector3()
            vector.x = self.info.rpy[0]/180.0*3.141592654
            vector.y = self.info.rpy[1]/180.0*3.141592654
            vector.z = self.info.rpy[2]/180.0*3.141592654
            self.talker_pose.publish(vector)
            print self.info.pose_msg()
        elif ch in Teleop.stepBindings:
            bindings = Teleop.stepBindings
            self.info.linear_acc *= bindings[ch][0]
            self.info.angular_acc *= bindings[ch][1]
            self.info.pose_vel *= bindings[ch][2]
            print self.info.velocity_msg()
        elif ch in Teleop.eventBindings:
            bindings = Teleop.eventBindings
            self.info.event = bindings[ch]
            string = String()
            string.data = self.info.event
            self.talker_event.publish(string)
            print self.info.event_msg()
            self.info.event_reset()
        else:
            pass

    def run(self):
        try:
            while 1:
                ch = self._get_key()
                if ch == '\x03':
                    break
                # elif ch == '\x1b':
                #     if self._get_key() == '[':
                #         arrow_ch = self._get_key()
                #         if arrow_ch == 'A':
                #             print "arrow_up"
                #         elif arrow_ch == 'B':
                #             print "arrow_down"
                #         elif arrow_ch == 'C':
                #             print "arrow_right"
                #         elif arrow_ch == 'D':
                #             print "arrow_left"
                elif ch == '?':
                    print self.instruction_msg()
                elif ch == '.':
                    print self.limit_msg()
                elif ch == '+':
                    if self.mode == 'strafing':
                        self.mode = 'moving'
                    elif self.mode == 'moving':
                        self.mode = 'strafing'
                    else:
                        raise ValueError
                    print "mode: {}".format(self.mode)
                else:
                    self.handle_keys(ch)

        except Exception as e:
            print e

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


if __name__ == "__main__":
    tele = Teleop()
    tele.run()
