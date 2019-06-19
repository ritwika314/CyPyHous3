#!/usr/bin/env python2.7

import rospy
import time


from math import atan2
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist, Pose,PoseStamped


class MotionAutomaton():

    def __init__(self):
         self.__reached = False
         self.__position = Pose()

         self.__pub = rospy.Publisher('Waypoint_bot', Pose, queue_size=1)
         self.__sub_reached = rospy.Subscriber('/Reached', String, self._getReached, queue_size=1)
         self.__sub_odom = rospy.Subscriber('/odom', Odometry, self._getOdom, queue_size=1) #Subscribeed to odom instead of vicon

         time.sleep(1)

    def pub(self):
        """
        getter method for publisher
        :return:
        """
        return self.__pub

    def _getOdom(self, data):  # -> NoReturn:
        """
        This is a callback function that updates the internal position and heading,
        :param data: posestamped message.
        :return:
        """
        self.position = data.pose

    def goTo(self, dest):  # -> NoReturn:

        """
        noramally PoseStamped would be used, but pose will be used for simplicity here
        """

        pose = Pose()
        pose.pose = dest
        self.reached = False

        self.pub.publish(pose)

    def run(self):
        """
        calls the spin function to check for new vicon data
        :return:
        """
        rospy.spin()
