from addnums import DefaultName
from agentThread import config

plist = [2000,2001,2002,2003,2004]
bots = 3
c1 = config(2,bots,"",2001,plist)
c2 = config(3,bots,"",2002,plist)
c3 = config(0,bots,"",2003,plist)

b,c,d = DefaultName(c1),DefaultName(c2),DefaultName(c3)

