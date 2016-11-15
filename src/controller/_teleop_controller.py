import numpy as np
import rospy

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class TeleopController:

    def __init__(self, robot_name):
        self._twist = Twist()
        self._enabled = False

        self._vel_axis = rospy.get_param('~vel_axis')
        self._omega_axis = rospy.get_param('~omega_axis')
        self._enable_button = rospy.get_param('~enable_button')

        max_rpm = rospy.get_param(robot_name + '/motors/max_rpm')
        r = rospy.get_param(robot_name + '/geometry/r')
        d = rospy.get_param(robot_name + '/geometry/d')
        self._max_vel = (np.pi * max_rpm * r) / 30
        self._max_omega = ((np.pi * max_rpm * r) / (15 * d)) * .50

        self._twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self._joy_sub = rospy.Subscriber('joy', Joy, self._joy_handler)

    def _joy_handler(self, msg):
        if msg.buttons[self._enable_button]:
            self._enabled = not self._enabled
        self._twist.linear.x = msg.axes[self._vel_axis] * self._max_vel
        self._twist.angular.z = msg.axes[self._omega_axis] * self._max_omega

    def update(self):
        if self._enabled:
            self._twist_pub.publish(self._twist)

    def shutdown(self):
        self._joy_sub = None

        self._twist.linear.x = 0
        self._twist.angular.z = 0
        self._twist_pub.publish(self._twist)
        self._twist_pub = None
