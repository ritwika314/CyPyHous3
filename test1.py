from src.apps.addnums import AddNums
from src.config.configs import AgentConfig
from src.functionality.base_mutex_handler import BaseMutexHandler

bots = 3

a1 = AgentConfig(0, bots, "", rport=2001, iplist=['<broadcast>'],plist=[2001,2002,2003], mh=BaseMutexHandler, is_leader=False, mhargs=[False,0])

app1 = AddNums(a1)

