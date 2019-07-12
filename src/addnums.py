import time
from agentThread import AgentThread


class DefaultName(AgentThread):

    def __init__(self, config):
        super(DefaultName, self).__init__(config)
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

            #self.unlock()
            time.sleep(0.4)
            self.baselock.release_mutex()
            self.requestedlock = False
            return

        if self.read_from_shared('numadded',None) >= self.num_agents():
            self.locals['finalsum'] = self.read_from_shared('sum',None)
            print('final sum for', self.pid(), 'is', self.locals['finalsum'])
            return


