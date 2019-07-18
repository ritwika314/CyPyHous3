import threading
import time
from abc import ABC, abstractmethod
from typing import Union

from src.harness.configs import MoatConfig
from src.motion.pos import Pos


class MotionAutomaton(threading.Thread, ABC):

    def __init__(self, config: MoatConfig):
        # TODO: work on all the configs, print initialization messages
        # print("initializing motion automaton")
        threading.Thread.__init__(self)
        self.__waypoint_count = 0
        self.__position = Pos()
        self.__reached = False
        self.__path = []
        self.__planner = config.planner
        self.__bot_type = config.bot_type

        try:
            import rospy
            rospy.init_node(config.rospy_node, anonymous=True)
            self.__pub = rospy.Publisher(config.waypoint_topic, config.pub_msg_type, queue_size=config.queue_size)
            self.__sub_reached = rospy.Subscriber(*config.reached_params, self._getReached,
                                                  queue_size=config.queue_size)
            self.__sub_positioning = rospy.Subscriber(*config.positioning_params, self._getPositioning,
                                                      queue_size=config.queue_size)

        except ImportError:
            self.__pub = None
            self.__sub_reached = None
            self.__sub_positioning = None
            print("maybe issue with ros installation")

        time.sleep(1)

    @property
    def path(self) -> Union[Pos, list]:
        """
        getter method for path
        :return:
        """
        if self.__path is not []:
            return self.__path
        else:
            return self.position

    @path.setter
    def path(self, path):
        """
        setter method for path
        :param path:
        :return:
        """
        self.__path = path

    @property
    def pub(self):
        """
        getter method for publisher
        :return:
        """
        return self.__pub

    @property
    def position(self) -> Pos:
        """
        getter method for position"
        :return:
        """
        return self.__position

    @position.setter
    def position(self, pos: Pos) -> None:
        """
        setter method for position
        :param pos: position
        :return:
        """
        self.__position = pos

    @property
    def waypoint_count(self) -> int:
        """
        current method of figuring out whether the current point is a takeoff point
        :return:
        """
        return self.__waypoint_count

    @waypoint_count.setter
    def waypoint_count(self, wpc: int) -> None:
        """
        setter method for an internal function
        :return:
        """
        self.__waypoint_count = wpc

    @property
    def reached(self) -> bool:
        """
        getter method for reached"
        :return:
        """
        return self.__reached

    @reached.setter
    def reached(self, r: bool) -> None:
        """
        setter method for reached
        :param r: bool
        :return:
        """
        self.__reached = r

    @property
    def bot_type(self):
        """
        getter method for bot type
        :return:
        """
        return self.__bot_type

    @bot_type.setter
    def bot_type(self, bot_type):
        """
        setter method for bot type
        :param bot_type:
        :return:
        """
        self.__bot_type = bot_type

    @property
    def planner(self):
        """
        getter method for planner
        :return:
        """
        return self.__planner

    @planner.setter
    def planner(self, p):
        """
        setter method for planner
        :param p:
        :return:
        """
        self.__planner = p

    @abstractmethod
    def _getPositioning(self, data) -> Pos:
        """
        This is a callback function that updates the internal position and heading,
        :param data: position message.
        :return:
        """
        pass

    @abstractmethod
    def _getReached(self, data) -> bool:
        """
        callback function that updates the reached flag
        :param data:
        :return:
        """
        pass

    @abstractmethod
    def goTo(self, dest: Pos, wp_type: int = None) -> None:
        """
        goto position. should publish correct ros message
        :param dest:
        :param wp_type:
        :return:
        """
        pass

    @abstractmethod
    def follow_path(self, path: list) -> None:
        """
        Follow defined path
        :param path:
        :return:
        """
        pass

    @abstractmethod
    def run(self):

        """
        abstract method for running the motion automaton thread
        calls the spin function to check for new vicon data
        :return:
        """
        pass
