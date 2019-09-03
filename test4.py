from src.apps.addnums import AddNums
from src.apps.taskwithpaths import TaskApp
from src.config.configs import AgentConfig
from src.functionality.base_mutex_handler import BaseMutexHandler

bots = 3
p = [2001,2002,2003,2004,2005]
#p = []
#iplist = ['192.168.1.4', '192.168.1.27','192.168.1.14' ]
iplist = ["<broadcast>"]
p0,p1,p2,p3,p4 = 2001,2002,2003,2004,2005
#p0,p1,p2,p3,p4 = 2000,2000,2000,2000,2000
a0 = AgentConfig(0, bots, "", rport=p0, plist= p, mh=BaseMutexHandler, is_leader=False, mhargs=[False,0])
a1 = AgentConfig(1, bots, "", rport=p1, plist= p, mh=BaseMutexHandler, is_leader=False, mhargs=[False,1])
a2 = AgentConfig(2, bots, "", rport=p2, plist= p, mh=BaseMutexHandler, is_leader=True, mhargs=[True,2])
a3 = AgentConfig(3, bots, "", rport=p3, plist= p, mh=BaseMutexHandler, is_leader=False, mhargs=[False,3])
a4 = AgentConfig(4, bots, "", rport=p4, plist=p, mh=BaseMutexHandler, is_leader=False, mhargs=[False,4])

app3 = AddNums(a3)
#app3 = TaskApp(a3)
