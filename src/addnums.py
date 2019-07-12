import time

import basic_synchronizer
from agentThread import AgentThread
from base_mutex import BaseMutex
from comm_handler import CommHandler, CommTimeoutError
from gvh import Gvh
from mutex_handler import BaseMutexHandler


class AgentCreation(AgentThread):
    """
    test class to test that agent thread objects are created
    and fields are accessed safely and correctly .
    """

    def __init__(self, pid, participants, receiver_ip, r_port):
        """
        parameters to instantiate the gvh and communication handler.
        :param pid:
        :param participants:
        :param r_port:
        """
        #config object.
        #agent_gvh = Gvh(config.pid, config.participants)
        #agent_gvh.port_list = config.port_list
        agent_gvh = Gvh(pid, participants)
        agent_gvh.port_list = [2000,2001,2002,2005,2004]
        if pid == 0:
            agent_gvh.is_leader = True

        mutex_handler = BaseMutexHandler(agent_gvh.is_leader, pid)
        agent_gvh.mutex_handler = mutex_handler
        agent_comm_handler = CommHandler(receiver_ip, r_port)
        agent_comm_handler.agent_gvh = agent_gvh
        super(AgentCreation, self).__init__(agent_gvh,agent_comm_handler,mutex_handler)
        self.start()

    def initialize_vars(self):
        self.locals = {}
        self.create_aw_var('sum', int, 0)
        self.create_aw_var('numadded', int, 0)
        self.locals['added'] = False
        self.locals['finalsum'] = 0

    def loop_body(self):
        if not self.locals['added']:
            # lock()
            if not self.requestedlock:
                self.baselock.request_mutex(self.req_num)
                self.requestedlock = True
                self.req_num += 1
                return
            else:
                if not self.agent_gvh.mutex_handler.has_mutex(self.baselock.mutex_id):
                    return

            self.write_to_shared('sum', None, self.read_from_shared('sum', None) + self.pid() * 2)
            self.write_to_shared('numadded', None, self.read_from_shared('numadded', None) + 1)
            self.locals['added'] = True

            time.sleep(0.4)
            self.baselock.release_mutex()
            self.requestedlock = False
            return

        if self.agent_gvh.get('numadded') >= self.agent_gvh.participants:
            self.locals['finalsum'] = self.agent_gvh.get('sum')
            print('final sum for', self.pid(), 'is', self.locals['finalsum'])
            return



b,c,d = AgentCreation(2, 3, "", 2001),AgentCreation(4, 3, "", 2002),AgentCreation(0, 3, "", 2000)


