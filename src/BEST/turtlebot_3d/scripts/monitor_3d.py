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
        #self.safety_pt()
        self.agent_x =0
        self.agent_y = 0
        self.agent_theta = 0
        self.safety_pt_x = 0
        self.safety_pt_y = 0

        self.boundary_x_min = -1
        self.boundary_x_max = 1
        self.boundary_y_min = -1
        self.boundary_y_max = 1
        self.BEST = 0

    def OdomCallback(self, msg):

        self.agent_x = msg.pose.pose.position.x
        self.agent_y = msg.pose.pose.position.y
        (roll, pitch, yaw) = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.agent_theta = yaw

        #rospy.loginfo(': {}, y: {}, theta: {}'.format(self.agent_x, self.agent_y, self.agent_theta))

        if(self.agent_x < self.boundary_x_min or self.agent_x > self.boundary_x_max or self.agent_y < self.boundary_y_min or self.agent_y > self.boundary_x_max):
            rospy.loginfo('boundary crossed at: x: {}, y: {}, theta: {}'.format(self.agent_x, self.agent_y, self.agent_theta))
            self.BEST = 1;



    def goto_safety(self, x, y):
        speed = Twist()
        rate = rospy.Rate(4)

        speed.linear.x = 0.0
        speed.angular.z = 0.0
        self.pos_pub.publish(speed)


        while self.BEST:
            rospy.loginfo('go to safety_pt')

            dist_to_target_x = x - self.agent_x
            dist_to_target_y = y - self.agent_y
            angle_to_target = atan2(dist_to_target_y, dist_to_target_x)


            if (abs(dist_to_target_x) <= 0.08 and abs(dist_to_target_y) <= 0.08):
                speed.linear.x = 0.0
                speed.angular.z = 0.0
                #rospy.loginfo('Home')
                self.BEST = 0;
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
        return


    def goto(self, x, y):
        speed = Twist()
        rate = rospy.Rate(4)

        while not self.BEST:
            rospy.loginfo('goto {} {}'.format(x,y))
            dist_to_target_x = x - self.agent_x
            dist_to_target_y = y - self.agent_y
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


def main():
    rospy.init_node('monitor_3d')
    monitorAndReportObject = monitorAndReport()
    if(monitorAndReportObject.goto(2,2)):
        monitorAndReportObject.goto_safety(0,0)


    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except:
        rospy.loginfo("User terminated with Crtl-C!")
