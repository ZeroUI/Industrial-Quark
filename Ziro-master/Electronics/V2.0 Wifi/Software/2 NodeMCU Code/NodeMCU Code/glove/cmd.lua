cmdID={}
cmdResource={}
cmdDataStream={}

-- CMDs for Slave 0-Motor Connected
local ID="00"
cmdID[ID]=ID
cmdResource[ID] ={"LR","LG","LB","MA"}
cmdDataStream[ID]={"G" ,0   , 0  ,"accZ"}

-- CMDs for Slave 1
local ID="01"
cmdID[ID]=ID
cmdResource[ID]    ={"LR","LG","LB"}
cmdDataStream[ID]={"G" ,0   ,"A"}
