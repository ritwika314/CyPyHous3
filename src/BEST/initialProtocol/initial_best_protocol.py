#!/usr/bin/env python3

"""
    Description: An initital BEST protocol which only involves checking preset boundary locations
    for a vehicle
"""


import rospy

from math import atan2
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist, Pose, PoseStamped



class BestProtocol():

    def __init__(self):
        rospy.init_node('protocol_node')

        #Topics Subscribed to:
        self.__sub_waypoint = rospy.Subscriber('Waypoint_bot', Pose, self._evalWaypoint, queue_size=1)
        self.__sub_odom = rospy.Subscriber('/drone1/ground_truth/state', Odometry, self._OdomCallback, queue_size=1)

        #Topics Published to:
        self.__pub = rospy.Publisher('best_Waypoint', Pose, queue_size=1)

        #static safety point defintions
        self.safety_pt_x = 0
        self.safety_pt_y = 0
        self.safety_pt_z = 0

        #Geofence boundary conditions
        self.boundary_x_min = -1
        self.boundary_x_max = 1
        self.boundary_y_min = -1
        self.boundary_y_max = 1
        self.boundary_z_min = -1
        self.boundary_z_max = 1

        rospy.loginfo("protcol initialized")


    def pub(self):
        """
        getter method for publisher
        :return:
        """
        return self.__pub

    def _OdomCallback(self, data):
        """
        Description: call back method for odometry topic. Modifies waypoint if
                     if necessary

        Inputs: data -> contains information on current vehicle position

        Ouputs: publishes to best_Waypoint topic if necessary

        """
        agent_x = data.pose.pose.position.x
        agent_y = data.pose.pose.position.y
        agent_z = data.pose.pose.position.z

        #geofence condition check
        if(agent_x < self.boundary_x_min or agent_x > self.boundary_x_max or agent_y < self.boundary_y_min or agent_y > self.boundary_y_max or agent_z < self.boundary_z_min or agent_z > self.boundary_z_max):
            rospy.loginfo("monitoring system triggered")
            corrected_Waypoint = Pose()
            corrected_Waypoint.position.x = self.safety_pt_x
            corrected_Waypoint.position.y = self.safety_pt_y
            corrected_Waypoint.position.z = self.safety_pt_z
            self.pub().publish(corrected_Waypoint)



    def _evalWaypoint(self, data):
        """
        Description: call back method for waypoint bot topic. Modifies waypoint if
                     if necessary

        Inputs: data -> contains information on desired waypoint

        Ouputs: publishes to best_Waypoint topic whenever called

        """

        rospy.loginfo('Evaluating waypoint')
        corrected_Waypoint = Pose()

        waypoint_x = data.position.x
        waypoint_y = data.position.y
        waypoint_z = data.position.z

        #geofence condition  check
        if(waypoint_x < self.boundary_x_min or waypoint_x > self.boundary_x_max or waypoint_y < self.boundary_y_min or waypoint_y > self.boundary_y_max or waypoint_z < self.boundary_z_min or waypoint_z > self.boundary_z_max):
            rospy.loginfo("Waypoint out of bounds, sending home")
            corrected_Waypoint.position.x = self.safety_pt_x
            corrected_Waypoint.position.y = self.safety_pt_y
            corrected_Waypoint.position.z = self.safety_pt_z
        else:
            rospy.loginfo('safe waypoint')
            corrected_Waypoint.position.x = waypoint_x
            corrected_Waypoint.position.y = waypoint_y
            corrected_Waypoint.position.z = waypoint_z

        # corrected_Waypoint.position.x = waypoint_x
        # corrected_Waypoint.position.y = waypoint_y
        # corrected_Waypoint.position.z = waypoint_z

        self.pub().publish(corrected_Waypoint)

def main():
    #rospy.init_node('protocol_node')
    BestProtocolObject = BestProtocol()
    rospy.spin()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("BEST protocol terminated")
