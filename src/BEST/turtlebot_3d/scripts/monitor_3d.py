#!/usr/bin/env python2.7

import rospy

from math import atan2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
from tf.transformations import euler_from_quaternion




class monitorAndReport(object):
    def __init__(self):
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.OdomCallback)
        self.pos_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)

        self.agent_x =0
        self.agent_y = 0
        self.agent_theta = 0
        self.safety_pt_x = 0
        self.safety_pt_y = 0

        self.boundary_x_min = 0.0;
        self.boundary_x_max = 3.0;
        self.boundary_y_min = 0.0;
        self.boundary_y_max = 3.0;

    def OdomCallback(self, msg):

        self.agent_x = msg.pose.pose.position.x
        self.agent_y = msg.pose.pose.position.y
        (roll, pitch, yaw) = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.agent_theta = yaw

        rospy.loginfo('x: {}, y: {}, theta: {}'.format(self.agent_x, self.agent_y, self.agent_theta))

        if(self.agent_x < self.boundary_x_min || self.agent_x > self.boundary_x_max || self.agent_y < self.boundary_y_min || self.agent_y > self.boundary_x_max):
            rospy.loginfo('boundary crossed at: x: {}, y: {}, theta: {}'.format(self.agent_x, self.agent_y, self.agent_theta))
            safety_pt()





    def safety_pt(self):

        speed = Twist()
        rate = rospy.Rate(5)

        while not rospy.is_shutdown():
            dist_to_target_x = self.safety_pt_x - self.agent_x
            dist_to_target_y = self.safety_pt_y - self.agent_y
            angle_to_target = atan2(dist_to_target_y, dist_to_target_x)

            if abs(angle_to_target - self.agent_theta ) > 0.3:
                speed.linear.x = 0.0
                speed.angular.z = 0.3
            else:
                speed.linear.x = 0.5
                speed.angular.z = 0.0

            self.pos_pub.publish(speed)
            rate.sleep()






def main():
    rospy.init_node('monitor_3d')
    monitorAndReportObject = monitorAndReport()
    rospy.spin()

if __name__ == '__main__':
    main()
