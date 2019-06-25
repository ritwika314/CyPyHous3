#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist, Pose
from math import atan2, sqrt
from drone import Drone


class GoTo():
    def __init__(self):
        rospy.init_node('Drone_Test', anonymous=False)
        rospy.loginfo("Drone initialized")
        self.numberOfDrones = 1
        self.drones = []
        self.complete = []
        for i in range(self.numberOfDrones):
            self.drones.append(Drone(i+1))
            self.complete.append(0)

        self.__sub_waypoint = rospy.Subscriber('best_Waypoint', Pose, self.goto, queue_size=1)

        # What to do if shut down
        rospy.on_shutdown(self.shutdown)


    def goto(self, goals):
        rospy.loginfo("Ready to move. To stop Drone , press CTRL + C")
        rospy.loginfo("Drone1 goes to (%f, %f, %f)",goals.position.x, goals.position.y, goals.position.z)
        #rospy.loginfo("Drone2 goes to (%f, %f)", pos2['x'], pos2['y'])
        r = rospy.Rate(10)
        move_cmd = Twist()

        # Set up goal
        for i in range(self.numberOfDrones):
            self.drones[i].goal.x = goals.position.x
            self.drones[i].goal.y = goals.position.y
            self.drones[i].goal.z = goals.position.z
            self.complete[i] = 0
        #rospy.loginfo("Drone1 goes to (%f, %f, %f)",goals.position.x, goals.position.y, goals.position.z)

        while not rospy.is_shutdown():

            if sum(self.complete) == self.numberOfDrones:
                return 1

            for i in range(self.numberOfDrones):
                diff_x = self.drones[i].goal.x - self.drones[i]._x
                diff_y = self.drones[i].goal.y - self.drones[i]._y
                diff_z = self.drones[i].goal.z - self.drones[i]._z



                if abs(diff_x) < 0.1 and abs(diff_y) < 0.1 and abs(diff_z) < 0.2:
                    rospy.loginfo("reached")
                    move_cmd.linear.x = 0.0
                    move_cmd.linear.y = 0.0
                    move_cmd.linear.z = 0.0
                    self.complete[i] = 1
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

                    if abs(diff_z) > 0.2:
                        rospy.loginfo(diff_z)
                        if diff_z > 0: move_cmd.linear.z = 0.1
                        else: move_cmd.linear.z = -0.1
                    else:
                        move_cmd.linear.z = 0.0

                self.drones[i].pub.publish(move_cmd)
                rospy.loginfo("Location %d is at (%f, %f, %f)", i, self.drones[i]._x, self.drones[i]._y, self.drones[i]._z)
                rospy.loginfo("speed %d is at (%f, %f, %f)", i, move_cmd.linear.x, move_cmd.linear.y, move_cmd.linear.z)


            r.sleep()


    def shutdown(self):
        rospy.loginfo("Stop Drones")
	    # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        for i in range(self.numberOfDrones):
            self.drones[i].pub.publish(Twist())
	    # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        navigator = GoTo()

        while not rospy.is_shutdown():
            pass

        rospy.sleep(1)
    except rospy.ROSInterruptException:
        rospy.loginfo("User pressed  Ctrl-C, quit!")
