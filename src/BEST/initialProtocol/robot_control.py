#!/usr/bin/env python2.7

import rospy

from math import atan2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist, Pose
from tf.transformations import euler_from_quaternion



class robotControl():
    def __init__(self):
        self.pos_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.OdomCallback)
        self.__sub_waypoint = rospy.Subscriber('best_Waypoint', Pose, self._waypoint, queue_size=1)


        self.agent_x = 0
        self.agent_y = 0
        self.agent_z = 0
        self.agent_theta = 0


    def OdomCallback(self, msg):

        self.agent_x = msg.pose.pose.position.x
        self.agent_y = msg.pose.pose.position.y
        (roll, pitch, yaw) = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.agent_theta = yaw



    def _waypoint(self, data):

        waypoint = Pose()
        speed = Twist()
        rate = rospy.Rate(4)

        waypoint_x = data.position.x
        waypoint_y = data.position.y
        waypoint_z = data.position.z

        while not rospy.is_shutdown():
            rospy.loginfo('goto {} {}'.format(x,y))
            dist_to_target_x = waypoint_x - self.agent_x
            dist_to_target_y = waypoint_y - self.agent_y
            dist_to_target_z = waypoint_z - self.agent_z
            angle_to_target = atan2(dist_to_target_y, dist_to_target_x)


            if (abs(dist_to_target_x) <= 0.08 and abs(dist_to_target_y) <= 0.08):
                speed.linear.x = 0.0
                speed.angular.z = 0.0
                self.pos_pub.publish(speed)
                #rospy.loginfo('Home')
                return 0;
            elif abs(angle_to_target - self.agent_theta ) > 0.3:
                speed.linear.x = 0.0
                speed.angular.z = 0.3
                #rospy.loginfo('Turning: {}'.format(angle_to_target))
            else:
                #rospy.loginfo('Moving: {}, {}'.format(dist_to_target_x, dist_to_target_y))
                speed.linear.x = 0.5
                speed.angular.z = 0.0

            self.pos_pub.publish(speed)
            rate.sleep()

        return 1
