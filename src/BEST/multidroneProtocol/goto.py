#!/usr/bin/env python3

'''
    Use ros messages to drive drones
    Involves physics
'''

import rospy
import queue
from protocol import BESTProtocol
import multiprocessing

from geometry_msgs.msg import Point, Twist, Pose
from math import sqrt
import sys
from hector_uav_msgs.srv import EnableMotors
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class Drone:
    def __init__(self, number):
        # Drone's position and orientation inforamtion
        self._x = 0.0
        self._y = 0.0
        self._z = 0.0
        self._theta = 0.0
        self.goal = Point()




        # Set up subscriber and publisher
        my_number = "/drone" + str(number)
        self.sub = rospy.Subscriber(my_number + "/ground_truth/state", Odometry, self.newPos)
        self.pub = rospy.Publisher(my_number + "/cmd_vel", Twist, queue_size=10)


        # Enable motors using ROS service
        rospy.wait_for_service(my_number + '/enable_motors')
        try:
            # Set 'enable_motors' to be True
            enable = rospy.ServiceProxy(my_number + '/enable_motors', EnableMotors)
            resp = enable(True)
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed with %s", e)

    def newPos(self, msg):
        '''
        Callback function to get drone's current location
        :param msg: Odometry type msg that contains position and orientation of a drone
        :return: Nothing
        '''
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y
        self._z = msg.pose.pose.position.z

        quat = msg.pose.pose.orientation
        (_, _, self._theta) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

class setup:
    def __init__(self, num, goals):
        self.numberOfDrones = num

        drones = [GoTo(i+1, goals[i]) for i in range(num)]
        

        #drones[1].goto()

        # What to do if shut down
        rospy.on_shutdown(self.shutdown)



    def shutdown(self):
        '''
        Stop all drones when rospy shuts down
        :return: Nothing
        '''
        rospy.loginfo("Stop Drones")
        # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop Drones

        #for i in range(self.numberOfDrones):
        #    self.drones[i].pub.publish(Twist())
        # sleep just makes sure Drones receives the stop command prior to shutting down the script

        rospy.sleep(1)





class GoTo:
    '''
    This is the class that handles GOTO functionality of drones
    '''
    def __init__(self, num, goals):
        '''
        Constructor
        :param num: number of drones
        :param goals: the goals that drones are flying to
        '''

        self.drone = Drone(num)
        #BESTProtocol(num)

        vehicle_id = "/drone" + str(num)
        print(vehicle_id)

        self.waypointQueue = queue.Queue()

        self.sub = rospy.Subscriber(vehicle_id+"/best_waypoint", Pose, self.next_waypoint, queue_size=10)
        self.pub = rospy.Publisher(vehicle_id+"/waypoint_bot", Pose, queue_size=10)

        rospy.sleep(1)      #wait sometime so publisher and subsriber can connect

        for goal in goals:
            print(goal)
            dest = Pose()
            dest.position.x = goal[0]
            dest.position.y = goal[1]
            dest.position.z = goal[2]
            self.pub.publish(dest)

        print("drone", num, "ready")







    def next_waypoint(self, data):
        rospy.loginfo("(%f,%f,%f) logged in queue", data.position.x, data.position.y, data.position.z)
        self.waypointQueue.put(data)

    def goto(self):
        '''
        The actual goto method that drives the drones towards goal points
        :param goals: the list of goal points
        :return: 1 - if complete
        '''
        rospy.loginfo("Ready to move. To stop Drone , press CTRL + C")
        r = rospy.Rate(10)  # Setup ROS spin rate
        move_cmd = Twist()  # Twist messages

        reached = 0

        next_position = self.waypointQueue.get()
        goalX = next_position.position.x
        goalY = next_position.position.y
        goalZ = next_position.position.z

        while not rospy.is_shutdown():

            if reached and not self.waypointQueue.empty():
                next_position = self.waypointQueue.get()
                goalX = next_position.position.x
                goalY = next_position.position.y
                goalZ = next_position.position.z
                reached = 0



            diff_x = goalX - self.drone._x
            diff_y = goalY - self.drone._y
            diff_z = goalZ - self.drone._z


            if abs(diff_x) < 0.1 and abs(diff_y) < 0.1 and abs(diff_z) < 0.1:
                rospy.loginfo("reached")
                move_cmd.linear.x = 0.0
                move_cmd.linear.y = 0.0
                move_cmd.linear.z = 0.0
                reached = 1
            else:
                if abs(diff_x) > 0.1:
                    if diff_x > 0: move_cmd.linear.x = 0.1
                    else: move_cmd.linear.x = -0.1
                else:
                    move_cmd.linear.x = 0.0

                if abs(diff_y) > 0.1:
                    if diff_y > 0: move_cmd.linear.y = 0.1
                    else: move_cmd.linear.y = -0.1
                else:
                    move_cmd.linear.y = 0.0

                if abs(diff_z) > 0.1:
                    #rospy.loginfo(diff_z)
                    if diff_z > 0: move_cmd.linear.z = 0.1
                    else: move_cmd.linear.z = -0.1
                else:
                    move_cmd.linear.z = 0.0

            rospy.loginfo("Location [%f %f %f]", diff_x, diff_y, diff_z)
            rospy.loginfo("Speed [%f %f %f]", move_cmd.linear.x, move_cmd.linear.y, move_cmd.linear.z)
            self.drone.pub.publish(move_cmd)



            r.sleep()



if __name__ == '__main__':
    try:
        rospy.init_node('goto')
        # sys.path.append('..')
        # from util import parse_goal_pose
        # num = int(sys.argv[1])
        # goals = parse_goal_pose(num, sys.argv[2:], 'drone')
        #
        # num = 1
        # goal = Pose()
        # goal.position.x = 0
        # goal.position.y = 0
        # goal.position.z = 1
        # goals = [[0,0,1], [0,2,1]]
        #
        # navigator = GoTo(num, goals)
        # navigator.pub.publish(goal)
        #
        #
        # if navigator.success:
        #     rospy.loginfo("Yep, we made it!")
        # else:
        #     rospy.loginfo("Something is wrong")

        num = 2
        goals = [[[0,0,1], [1,0,.5], [5,5,5]]]
        goals = [ [[0,0,1]], [[0,2,1]] ]

        setup(num, goals)
        rospy.spin()

        rospy.sleep(1)
    except rospy.ROSInterruptException:
        rospy.loginfo("User pressed  Ctrl-C, quit!")
