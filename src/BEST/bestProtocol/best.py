#!/usr/bin/env python3

'''
    FILE DESCRIPTION: B.E.S.T protocol for CyPhyHouse Drones

    NOTES: Use the config.yml file to set BEST conditions

    CREATOR: Hebron Taylor EMAIL: hdt2@illinois.edu
'''

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

import yaml



class BESTProtocol():
    def __init__(self):
        '''
            DESCRIPTION:
                Constructor that initalizes BEST protocol
            INPUTS: NONE
            OUTPUTS: NONE
        '''

        rospy.init_node('protocol_node')

        self.vehicle_id = "/drone"


        #Subscribed from MotionAutomata
        self.__wayptSub = rospy.Subscriber("Waypoint_tobest", PoseStamped, self.__evalWaypt, queue_size=10)

        self.__wayptSub = rospy.Subscriber("Manual_terminate", String, self.__manualTerminate, queue_size=10)

        #Monitoring System
        #self.__odomSub = rospy.Subscriber(vehicle_id+"/ground_truth/state", Odometry, self.__monitor, queue_size=10)

        #Subsribed from fakegps.cpp (for drone)
        self.__reachedSub = rospy.Subscriber("/drone/reached", String, self.__reachedWaypt, queue_size=10)


        #Publishes to fakegps.cpp (for drones)
        self.__bestPub = rospy.Publisher("/drone/waypoint", PoseStamped, queue_size=10)

        with open("config.yml", "r") as ymlfile:
            cfg = yaml.safe_load(ymlfile)

        #static safety point defintions (taken from config file)
        self.__safetyPtX = cfg.get(self.vehicle_id).get("safetyPoint")[0]
        self.__safetyPtY = cfg.get(self.vehicle_id).get("safetyPoint")[1]
        self.__safetyPtZ = cfg.get(self.vehicle_id).get("safetyPoint")[2]
        #rospy.loginfo("SAFE PT:[%f %f, %f]",self.__safetyPtX, self.__safetyPtY, self.__safetyPtZ)

        #Geofence boundary conditions (taken from config file)
        self.__minBoundX = cfg.get(self.vehicle_id).get("xBound")[0]
        self.__maxBoundX = cfg.get(self.vehicle_id).get("xBound")[1]
        self.__minBoundY = cfg.get(self.vehicle_id).get("yBound")[0]
        self.__maxBoundY = cfg.get(self.vehicle_id).get("yBound")[1]
        self.__minBoundZ = cfg.get(self.vehicle_id).get("zBound")[0]
        self.__maxBoundZ = cfg.get(self.vehicle_id).get("zBound")[1]

        self.__bestTriggered = 0
        self.__landed = 0

        rospy.loginfo("Drone BEST Protocol inititalized")


    def __reachedWaypt(self,data):
        '''
            DESCRIPTION:
                Monitors whether a vehicle has reached a waypoint and tells drone to land at its current position
                if BEST was triggered
                (frame_id of 2 corrsponds to a land command)

            INPUTS: NONE

            OUTPUTS:
                land: PoseStamped type message which is published on the "Waypoint" topic

        '''
        rospy.loginfo("Reached waypoint called")
        if self.__bestTriggered and self.__landed == 0:
            rospy.loginfo("Landing..")
            land = PoseStamped()
            land.header.frame_id = "2"
            land.pose.position.x = self.__safetyPtX
            land.pose.position.y = self.__safetyPtY
            land.pose.position.z = self.__safetyPtZ
            self.__bestPub.publish(land)
            self.__landed = 1



    def __manualTerminate(self,data):
        rospy.loginfo("Manual terminate trigged, landing immediately")
        land = PoseStamped()
        land.header.frame_id = "2"
        land.pose.position.x = self.__safetyPtX
        land.pose.position.y = self.__safetyPtY
        land.pose.position.z = self.__safetyPtZ
        self.__bestPub.publish(land)
        rospy.loginfo("Land command sended to drone controller")
        self.__landed = 1

    def __monitor(self, data):
        '''
            DESCRIPTION:
                Monitors drone location and verfies it is within geofenced location
                (ex. a gust of wind moves the drones out of bounds)

            INPUTS:
                data: PoseStamped type message which contains the drones position data

            OUTPUTS:
                BESTwaypt: PoseStamped type message which is published on the "Waypoint" topic
        '''

        currLocationX = data.pose.position.x
        currLocationY = data.pose.position.y
        currLocationZ = data.pose.position.z
        currFrameID = data.header.frame_id

        #compare current position to geofence values (if out of bounds then land)
        if(currLocationX < self.__minBoundX or currLocationX > self.__maxBoundX or currLocationY < self.__minBoundY or currLocationY > self.__maxBoundY or currLocationZ < self.__minBoundZ or currLocationZ > self.__maxBoundZ):
            rospy.loginfo("Monitoring system triggered")
            BESTwaypt = PoseStamped()
            BESTwaypt.pose.position.x = self.__safetyPtX
            BESTwaypt.pose.position.y = self.__safetyPtY
            BESTwaypt.pose.position.z = self.__safetyPtZ
            BESTwaypt.header.frame_id = currFrameID
            self.__bestTriggered = 1
            self.__bestPub.publish(BESTwaypt)


    def __evalWaypt(self, data):
        '''
            DESCRIPTION:
                Premptively verify if a given waypoint will violate the geofence conditions

            INPUTS:
                data: PoseStamped type message which contains the drones position data

            OUTPUTS:
                BESTwaypt: PoseStamped type message which is published on the "Waypoint" topic
        '''


        BESTwaypt = PoseStamped()

        wayptX = data.pose.position.x
        wayptY = data.pose.position.y
        wayptZ = data.pose.position.z
        frame_id = data.header.frame_id

        #compare current position to geofence values (if out of bounds then land)
        if(wayptX < self.__minBoundX or wayptX > self.__maxBoundX or wayptY < self.__minBoundY or wayptY > self.__maxBoundY or wayptZ < self.__minBoundZ or wayptZ > self.__maxBoundZ):
            rospy.loginfo(self.vehicle_id+": Waypoint out of bounds, sending home")
            BESTwaypt.pose.position.x = self.__safetyPtX
            BESTwaypt.pose.position.y = self.__safetyPtY
            if self.__safetyPtZ <= 0:
                BESTwaypt.pose.position.z = .5
            else:
                BESTwaypt.pose.position.z = self.__safetyPtZ
            BESTwaypt.header.frame_id = frame_id
            self.__bestTriggered = 1

        else:
            rospy.loginfo(self.vehicle_id+': Safe waypoint [%f %f %f]', wayptX, wayptY, wayptZ)
            BESTwaypt.pose.position.x = wayptX
            BESTwaypt.pose.position.y = wayptY
            BESTwaypt.pose.position.z = wayptZ
            BESTwaypt.header.frame_id = frame_id
            self.__bestTriggered = 0




        self.__bestPub.publish(BESTwaypt)



def main():
    protocol = BESTProtocol()
    rospy.spin()


if __name__ == '__main__':
    main()
