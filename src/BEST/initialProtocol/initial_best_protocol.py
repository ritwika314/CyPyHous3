#!/usr/bin/env python3

import rospy

from math import atan2
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist, Pose, PoseStamped



class BestProtocol():

    def __init__(self):
        rospy.init_node('protocol_node')
        self.__sub_waypoint = rospy.Subscriber('Waypoint_bot', Pose, self._evalWaypoint, queue_size=1)
        self.__pub = rospy.Publisher('best_Waypoint', Pose, queue_size=1)

        self.safety_pt_x = 0
        self.safety_pt_y = 0
        self.safety_pt_z = 0

        #Geofence boundary conditions
        self.boundary_x_min = -1
        self.boundary_x_max = 1
        self.boundary_y_min = -1
        self.boundary_y_max = 1
        rospy.loginfo("protcol initialized")


    def pub(self):
        """
        getter method for publisher
        :return:
        """
        return self.__pub

    def _evalWaypoint(self, data):

        rospy.loginfo('Evaluating waypoint')
        corrected_Waypoint = Pose()

        waypoint_x = data.position.x
        waypoint_y = data.position.y
        waypoint_z = data.position.z

        if(waypoint_x < self.boundary_x_min or waypoint_x > self.boundary_x_max or waypoint_y < self.boundary_y_min or waypoint_y > self.boundary_y_max):
            rospy.loginfo("Waypoint out of bounds, sending home")
            corrected_Waypoint.position.x = self.safety_pt_x
            corrected_Waypoint.position.y = self.safety_pt_y
            corrected_Waypoint.position.z = self.safety_pt_z
        else:
            corrected_Waypoint.position.x = waypoint_x
            corrected_Waypoint.position.y = waypoint_y
            corrected_Waypoint.position.z = waypoint_z

        self.pub().publish(corrected_Waypoint)

def main():
    #rospy.init_node('protocol_node')
    BestProtocolObject = BestProtocol()
    rospy.spin()



if __name__ == '__main__':
    try:
        main()
    except:
        rospy.loginfo("BEST")
