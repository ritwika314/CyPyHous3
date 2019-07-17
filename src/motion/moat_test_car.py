import numpy as np

from src.CyPyHous3.src.motion.motionautomaton import MotionAutomaton
from src.CyPyHous3.src.motion.pos import Pos


class MoatTestCar(MotionAutomaton):

    def __init__(self, config):
        super(MoatTestCar, self).__init__(config)

    def _getPositioning(self, data) -> Pos:
        self.position = Pos(np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z]))

    def _getReached(self, data) -> bool:
        a = str(data).upper()
        if 'TRUE' in a:
            self.reached = True

    def goTo(self, dest: Pos, wp_type: int = None) -> None:
        print("going to point", dest)
        if wp_type is not None:
            frame_id = str(wp_type)
        else:
            if self.waypoint_count == 0:
                frame_id = '0'
            elif dest.z <= 0:
                frame_id = '2'
            else:
                frame_id = '1'

        import rospy
        from geometry_msgs.msg import PoseStamped

        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = frame_id
        pose.pose = dest.to_pose()

        self.reached = False

        self.waypoint_count += 1
        self.pub.publish(pose)

    def follow_path(self, path: list) -> None:
        wp_list = []
        for point in path[1:]:
            point = point.topoint()
            wp = Pos()
            wp.x = point[0]
            wp.y = point[1]
            wp.z = point[2]
            wp_list.append(wp)
        for wp in wp_list[:-1]:
            self.goTo(wp, 0)
        self.goTo(wp_list[-1], 1)

    def run(self):
        import rospy
        rospy.spin()
