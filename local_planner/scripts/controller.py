#!/usr/bin/env python3

import numpy as np
import rospy
import tf
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


class Controller():
    def __init__(self):
        self.N = 10
        self.rate = rospy.Rate(50)

        self.local_plan_sub = rospy.Subscriber('/local_plan', Float32MultiArray, self.local_planner_cb)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.curr_state_pub = rospy.Publisher('/curr_state', Float32MultiArray, queue_size=10)

        self.__timer_localization = rospy.Timer(rospy.Duration(0.01), self.get_current_state)
        self.listener = tf.TransformListener()

        self.linear_speed = self.angular_speed = 0.0
        self.local_plan = np.zeros([self.N, 2])

        self.control_loop()

    def quart_to_rpy(self, x, y, z, w):
        r = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
        p = math.asin(2*(w*y-z*x))
        y = math.atan2(2*(w*z+x*y), 1-2*(z*z+y*y))
        return r, p, y

    def get_current_state(self, event):
        try:
            (trans, rot) = self.listener.lookupTransform('world', 'base_link', rospy.Time(0))
            _, _, yaw = self.quart_to_rpy(rot[0], rot[1], rot[2], rot[3])
            curr_state = Float32MultiArray()
            curr_state.data = [trans[0], trans[1], yaw]
            self.curr_state_pub.publish(curr_state)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

    def pub_vel(self):
        control_cmd = Twist()
        control_cmd.linear.x = self.linear_speed
        control_cmd.angular.z = self.angular_speed
        rospy.loginfo("Linear Speed: %.1f, Angular Speed: %.1f" % (self.linear_speed, self.angular_speed))
        self.vel_pub.publish(control_cmd)

    def control_loop(self):
        while not rospy.is_shutdown():
            self.linear_speed = self.local_plan[0, 0]
            self.angular_speed = self.local_plan[0, 1]
            self.pub_vel()
            self.rate.sleep()

    def local_planner_cb(self, msg):
        for i in range(self.N):
            self.local_plan[i, 0] = msg.data[0+2*i]
            self.local_plan[i, 1] = msg.data[1+2*i]


if __name__ == '__main__':
    rospy.init_node('control')
    controller = Controller()
