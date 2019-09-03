from src.apps.addnums import AddNums
from src.config.configs import AgentConfig
from src.functionality.base_mutex_handler import BaseMutexHandler

bots = 3


a2 = AgentConfig(1, bots, "", rport=2002, iplist=['<broadcast>'],plist=[2001,2002,2003], mh=BaseMutexHandler, is_leader=True, mhargs=[True,1])

app2 = AddNums(a2)
