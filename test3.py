from src.apps.addnums import AddNums
from src.config.configs import AgentConfig
from src.functionality.base_mutex_handler import BaseMutexHandler

bots = 3


a3 = AgentConfig(2, bots, "", rport=2003, iplist=['192.168.1.4', '192.168.1.27', '192.168.1.14'],plist=[2001,2002,2003], mh=BaseMutexHandler, is_leader=False, mhargs=[False,2])

app3 = AddNums(a3)
