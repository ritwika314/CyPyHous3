import sys

from src.config.config_funcs import get_configs
from src.config.configs import AgentConfig, default_car_moat_config
from src.functionality.base_mutex_handler import BaseMutexHandler

bots = 2
mhargs = [[True, 0], [False, 1], [False, 2], [False, 3]]
plist = []
r_port = 2000
r_ip = ""
pid = [0, 1, 2, 3]

m0 = default_car_moat_config('hotdec_car')
a1, m1 = get_configs(config_filename=sys.argv[1])
print(m0,"\n\n",m1)
# TaskApp(a1, m1)
