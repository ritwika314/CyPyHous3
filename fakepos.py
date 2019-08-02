import rospy
import time
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from src.motion.pos import pos3d



pub = rospy.Publisher('vrpn_client_node/drone2/pose', PoseStamped, queue_size=10)
pub2 = rospy.Publisher('vrpn_client_node/drone3/pose', PoseStamped, queue_size=10)
rospy.init_node('fake_drone_info', anonymous=False)
rate = rospy.Rate(10) # 10hz
while not rospy.is_shutdown():
           rospy.loginfo("telling it to do stuff")
           pose = PoseStamped()
           pose.header.stamp = rospy.Time.now()
           pose.header.frame_id = "0"

           pos = pos3d(0.0, 0.0, 0.0)
           pose.pose = pos.to_pose()

           pub.publish(pose)
           pub2.publish(pose)
           rate.sleep()
