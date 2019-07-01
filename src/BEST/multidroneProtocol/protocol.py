#!/usr/bin/env python3


import rospy
import yaml

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose

class BESTProtocol():
    def __init__(self, number):


        self.vehicle_id = "/drone" + str(number)

        rospy.init_node('protocol_node')

        self.__wayptSub = rospy.Subscriber(self.vehicle_id+"/waypoint_bot", Pose, self.__evalWaypt, queue_size=10)
        #self.__odomSub = rospy.Subscriber(vehicle_id+"/ground_truth/state", Odometry, self.__monitor, queue_size=10)


        self.__bestPub = rospy.Publisher(self.vehicle_id+"/best_waypoint", Pose, queue_size=10)

        with open("config.yml", "r") as ymlfile:
            cfg = yaml.safe_load(ymlfile)



        #static safety point defintions
        self.__safetyPtX = cfg.get(self.vehicle_id).get("safetyPoint")[0]
        self.__safetyPtY = cfg.get(self.vehicle_id).get("safetyPoint")[1]
        self.__safetyPtZ = cfg.get(self.vehicle_id).get("safetyPoint")[2]
        #print(self.__safetyPtX, self.__safetyPtY, self.__safetyPtZ)

        #Geofence boundary conditions
        self.__minBoundX = cfg.get(self.vehicle_id).get("xBound")[0]
        self.__maxBoundX = cfg.get(self.vehicle_id).get("xBound")[1]
        self.__minBoundY = cfg.get(self.vehicle_id).get("yBound")[0]
        self.__maxBoundY = cfg.get(self.vehicle_id).get("yBound")[1]
        self.__minBoundZ = cfg.get(self.vehicle_id).get("zBound")[0]
        self.__maxBoundZ = cfg.get(self.vehicle_id).get("zBound")[1]

        rospy.loginfo("Drone [%d] Protocol inititalized", number)




    def __monitor(self, data):

        currLocationX = data.pose.pose.position.x
        currLocationY = data.pose.pose.position.y
        currLocationZ = data.pose.pose.position.z
        #print(self.minBoundX, self.maxBoundX, self.minBoundY, self.maxBoundY, self.minBoundZ, self.maxBoundZ)

        if(currLocationX < self.__minBoundX or currLocationX > self.__maxBoundX or currLocationY < self.__minBoundY or currLocationY > self.__maxBoundY or currLocationZ < self.__minBoundZ or currLocationZ > self.__maxBoundZ):
            rospy.loginfo("Monitoring system triggered")
            BESTwaypt = Pose()
            BESTwaypt.position.x = self.__safetyPtX
            BESTwaypt.position.y = self.__safetyPtY
            BESTwaypt.position.z = self.__safetyPtZ
            self.__bestPub.publish(BESTwaypt)


    def __evalWaypt(self, data):


        BESTwaypt = Pose()

        wayptX = data.position.x
        wayptY = data.position.y
        wayptZ = data.position.z

        if(wayptX < self.__minBoundX or wayptX > self.__maxBoundX or wayptY < self.__minBoundY or wayptY > self.__maxBoundY or wayptZ < self.__minBoundZ or wayptZ > self.__maxBoundZ):
            rospy.loginfo(self.vehicle_id+": Waypoint out of bounds, sending home")
            BESTwaypt.position.x = self.__safetyPtX
            BESTwaypt.position.y = self.__safetyPtY
            BESTwaypt.position.z = self.__safetyPtZ
        else:
            rospy.loginfo(self.vehicle_id+': Safe waypoint [%f %f %f]', wayptX, wayptY, wayptZ)
            BESTwaypt.position.x = wayptX
            BESTwaypt.position.y = wayptY
            BESTwaypt.position.z = wayptZ

        self.__bestPub.publish(BESTwaypt)



def main():


    proto = BESTProtocol(1)
    roto = BESTProtocol(2)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()

    except rospy.ROSInterruptException:
        rospy.loginfo("User pressed  Ctrl-C, quit!")
