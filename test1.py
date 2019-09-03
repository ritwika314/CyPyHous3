from src.apps.addnums import AddNums
from src.config.configs import AgentConfig
from src.functionality.base_mutex_handler import BaseMutexHandler

bots = 3

a1 = AgentConfig(0, bots, "", rport=2001, iplist=['192.168.1.4', '192.168.1.27', '192.168.1.14'],plist=[2001,2002,2003], mh=BaseMutexHandler, is_leader=False, mhargs=[False,0])

app1 = AddNums(a1)

