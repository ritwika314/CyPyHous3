import time

import numpy as np

from src.harness.agentThread import AgentThread
from src.harness.comm_handler import CommHandler
from src.harness.gvh import Gvh
from src.motion.moat_test_car import MoatTestCar
from src.motion.motionautomaton import MoatConfig, default_car_moat_config
from src.motion.pos import Pos


class BasicFollowApp(AgentThread):

    def __init__(self, pid: int, num_bots: int, moat_config: MoatConfig):
        agent_gvh = Gvh(pid, num_bots)
        agent_gvh.moat = MoatTestCar(moat_config)
        super(BasicFollowApp, self).__init__(agent_gvh, CommHandler())
        self.start()

    def run(self):

        tries = 1

        dest1 = Pos(np.array([2., 1., 0.]))
        dest2 = Pos(np.array([-2., 1., 0.]))
        dest3 = Pos(np.array([2., -1., 0.]))

        while not self.stopped():

            if tries == 1:
                self.agent_gvh.moat.goTo(dest1)
                time.sleep(5)
                tries = 2
                continue
            if tries == 2:
                self.agent_gvh.moat.goTo(dest2)
                tries = 3
                time.sleep(5)
                continue
            if tries == 3:
                self.agent_gvh.moat.goTo(dest3)
                tries = 4
                time.sleep(5)
                self.stop()
                continue


m = default_car_moat_config('hotdec_car')
app = BasicFollowApp(1, 1, m)
