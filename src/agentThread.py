import pickle
import signal
import socket
from abc import ABC,abstractmethod
from threading import Thread, Event
from base_mutex import BaseMutex
from mutex_handler import BaseMutexHandler

from comm_handler import CommHandler
from comm_handler import CommTimeoutError
from gvh import Gvh
from message import Message


class config(object):
    def __init__(self, pid, bots, rip,rport,plist):
        self.pid = pid
        self.bots = bots
        self.rip = rip
        self.rport = rport
        self.plist = plist

class AgentThread(ABC, Thread):
    """
    abstract object for each agent thread/application. This contains
    the methods and fields each agent application must implement.

    __gvh : global variable holder.
    __comm_handler : communication handler
    __stop_event : stop thread safely.
    """

    def __init__(self, c:config) -> None:
        """
        init method for for agent application thread object.
        :param agent_gvh: agent global variable holder object
        :param agent_comm_handler: agent communication handler thread object
        """
        super(AgentThread, self).__init__()
        agent_gvh = Gvh(c.pid, c.bots)
        agent_gvh.port_list = c.plist
        agent_gvh.is_leader = c.pid == 0

        mutex_handler = BaseMutexHandler(agent_gvh.is_leader, c.pid)
        agent_gvh.mutex_handler = mutex_handler
        agent_comm_handler = CommHandler(c.rip, c.rport)
        agent_comm_handler.agent_gvh = agent_gvh

        self.__agent_gvh = agent_gvh
        self.__agent_comm_handler = agent_comm_handler
        self.__stop_event = Event()
        self.__mutex_handler = mutex_handler
        self.requestedlock = False
        self.req_num = 0
        self.baselock = None
        self.__locals = {}

        # create a signal handler to handle ctrl + c
        signal.signal(signal.SIGINT, self.signal_handler)

    def num_agents(self):
        return self.agent_gvh.participants

    def create_ar_var(self, name, type, initial_value=None):
        self.agent_gvh.create_ar_var(name, type, initial_value)
        pass

    def create_aw_var(self, name, type, initial_value=None):
        self.agent_gvh.create_aw_var(name, type, initial_value)
        pass

    @property
    def locals(self):
        return self.__locals

    @locals.setter
    def locals(self, locals):
        self.__locals = locals

    @property
    def agent_gvh(self) -> Gvh:
        """
        getter method for agent gvh
        :return: gvh of the current agent thread object
        """
        return self.__agent_gvh

    @agent_gvh.setter
    def agent_gvh(self, agent_gvh: Gvh) -> None:
        """
        setter method for gvh for the current agent thread object
        :param agent_gvh: new Gvh object to be set
        :return: None
        """
        self.__agent_gvh = agent_gvh

    @property
    def agent_comm_handler(self) -> CommHandler:
        """
        getter method for agent comm handler
        :return: the communication handler of the agent thread object
        """
        return self.__agent_comm_handler

    @agent_comm_handler.setter
    def agent_comm_handler(self, agent_comm_handler) -> None:
        """
        setter method for agent comm handler
        :param agent_comm_handler: comm handler object to be set
        :return: None
        """
        self.__agent_comm_handler = agent_comm_handler

    def stop(self) -> None:
        """
         a flag to set to to safely exit the thread
        :return: None
        """
        if self.agent_gvh.moat is not None:
            if self.agent_gvh.moat.bot_type is 0:
                self.agent_gvh.moat.land()
        if self.agent_gvh.dsm is not None:
            for dsmvar in self.agent_gvh.dsm:
                print(dsmvar, "for", self.agent_gvh.pid)
        if self.agent_gvh.mutex_handler is not None:
            if not self.agent_gvh.mutex_handler.stopped():
                self.agent_gvh.mutex_handler.stop()
        if self.agent_comm_handler is not None:
            if not self.agent_comm_handler.stopped():
                self.agent_comm_handler.stop()
        self.__stop_event.set()
        print("stopped", self.pid())


    @abstractmethod
    def initialize_vars(self):
        pass

    def stopped(self) -> bool:
        """
        set the stop flag
        :return: true if stop event is set, false otherwise
        """
        return self.__stop_event.is_set()

    def pid(self) -> int:
        """
        get the pid of the current agent.
        uses the agent gvh to retrieve it
        :return: integer pid of the current agent
        """
        return self.agent_gvh.pid

    def participants(self) -> int:
        """
        method to return the number of participants in the system
        :return: integer number of participants
        """
        return self.agent_gvh.participants

    def receiver_port(self) -> int:
        """
        gets the port the application is receiving messages on
        :return: integer receiver port
        """
        return self.agent_comm_handler.r_port

    def receiver_ip(self) -> str:
        """
        method to get the ip of receiver
        :return: string ip of receiver
        """
        return self.agent_comm_handler.ip

    def signal_handler(self, sig, frame):
        """
        method for handling ctrl + C safely to stop agent thread.
        :param sig:
        :param frame:
        :return:
        """
        self.stop()



    def initialize_lock(self):
        self.baselock = BaseMutex(1, self.agent_gvh.port_list)
        self.agent_gvh.mutex_handler.add_mutex(self.baselock)
        self.baselock.agent_comm_handler = self.agent_comm_handler

    @abstractmethod
    def loop_body(self):
        pass

    def write_to_shared(self, var_name, index, value):
        if index is not None:
            self.agent_gvh.put(var_name,value,index)
        else:
            self.agent_gvh.put(var_name,value)

    def read_from_shared(self, var_name, index):
        if index is not None:
            return self.agent_gvh.get(var_name,index)
        else:
            return self.agent_gvh.get(var_name)
        pass


    def run(self) -> None:
        """
        needs to be implemented for any agenThread
        :return:
        """

        self.initialize_lock()
        self.initialize_vars()
        import time
        while not (self.stopped()):
            time.sleep(0.4)
            self.agent_gvh.flush_msgs()
            self.agent_comm_handler.handle_msgs()
            time.sleep(0.4)

            try:
                self.loop_body()

            except CommTimeoutError:
                print("timed out on communication")
                self.stop()

	
    def lock(self):
        if not self.requestedlock:
            self.baselock.request_mutex(self.req_num)
            self.requestedlock = True
            self.req_num += 1
            return False
        else:
            if not self.agent_gvh.mutex_handler.has_mutex(self.baselock.mutex_id):
                return False
        return True

    def unlock(self):
        time.sleep(0.4)
        self.baselock.release_mutex()
        self.requestedlock = False


def send(msg: Message, ip: str, port: int) -> None:
    """
    :param msg: message to be sent
    :param ip: ip to be sent to
    :param port: port to be sent to
    :return:
    """
    client_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    client_sock.sendto(pickle.dumps(msg), (ip, port))
