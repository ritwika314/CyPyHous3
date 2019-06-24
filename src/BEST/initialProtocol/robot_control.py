#!/usr/bin/env python3

import rospy

from math import atan2, sqrt
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist, Pose
#from tf.transformations import euler_from_quaternion

from hector_uav_msgs.srv import EnableMotors

from std_msgs.msg import String




class robotControl():
    def __init__(self):
        rospy.init_node("control_node")
        self.pos_pub = rospy.Publisher('/drone1/cmd_vel', Twist, queue_size = 1)
        self.reached_pub = rospy.Publisher('reached', String, queue_size = 1)

        #self.odom_sub = rospy.Subscriber('/odom', Odometry, self.OdomCallback)
        self.odom_sub = rospy.Subscriber('/drone1/ground_truth/state', Odometry, self.OdomCallback)
        self.__sub_waypoint = rospy.Subscriber('best_Waypoint', Pose, self._waypoint, queue_size=1)


        self.agent_x = 0
        self.agent_y = 0
        self.agent_z = 0
        self.agent_theta = 0

        # Enable motors using ROS service
        rospy.wait_for_service('/drone1/enable_motors')
        try:
            # Set 'enable_motors' to be True
            enable = rospy.ServiceProxy('/drone1/enable_motors', EnableMotors)
            resp = enable(True)
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed with %s", e)
        rospy.loginfo("drone initialized")


    def OdomCallback(self, msg):

        self.agent_x = msg.pose.pose.position.x
        self.agent_y = msg.pose.pose.position.y
        self.agent_z = msg.pose.pose.position.z
        #(roll, pitch, yaw) = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        #self.agent_theta = yaw



    def _waypoint(self, data):

        waypoint = Pose()
        speed = Twist()
        rate = rospy.Rate(4)

        speed.linear.x = 0.0
        speed.linear.y = 0.0
        speed.linear.z = 0.0

        waypoint_x = data.position.x
        waypoint_y = data.position.y
        waypoint_z = data.position.z

        reached = 0

        # waypoint_x = 0
        # waypoint_y = 0
        # waypoint_z = 1
        rospy.loginfo('goto {} {} {}'.format(waypoint_x,waypoint_y,waypoint_z))

        while not rospy.is_shutdown():

            dist_to_target_x = waypoint_x - self.agent_x
            dist_to_target_y = waypoint_y - self.agent_y
            dist_to_target_z = waypoint_z - self.agent_z
            angle_to_target = atan2(dist_to_target_y, dist_to_target_x)

            #rospy.loginfo('dist {} {} {}'.format(dist_to_target_x, dist_to_target_y, dist_to_target_z))

            #rospy.loginfo(sqrt(dist_to_target_x*dist_to_target_x + dist_to_target_y*dist_to_target_y + dist_to_target_z*dist_to_target_z))
            if(abs(dist_to_target_x) < 0.1 and abs(dist_to_target_y) < 0.1 and abs(dist_to_target_z) < 0.2):
                speed.linear.x = 0.0
                speed.linear.y = 0.0
                speed.linear.z = 0.0
                rospy.loginfo("Location reached {} {} {}".format(waypoint_x, waypoint_y, waypoint_z))
                break;

            else:
                reached = 0

                rospy.loginfo('pos {} {} {}'.format(self.agent_x, self.agent_y, self.agent_z))


                if(abs(dist_to_target_x) > 0.1):
                    if dist_to_target_x > 0:
                        speed.linear.x = 0.1
                    else:
                        speed.linear.x = -0.1
                else:
                    speed.linear.x = 0.0

                if(abs(dist_to_target_y) > 0.1):
                    if dist_to_target_y > 0:
                        speed.linear.y = 0.1
                    else:
                        speed.linear.y = -0.1
                else:
                    speed.linear.y = 0.0

                if(abs(dist_to_target_z) > 0.1):
                    if dist_to_target_z > 0:
                        speed.linear.z = 0.1
                    else:
                        speed.linear.z = -0.1
                else:
                    speed.linear.z = 0.0



            self.pos_pub.publish(speed)
            rate.sleep()

        return 1

def main():
    #rospy.init_node("control_node")
    robotControlObject = robotControl()

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except:
        rospy.loginfo("control")
