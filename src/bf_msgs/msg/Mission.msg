# message type
uint8 COMMAND = 100
uint8 STATUS = 101

# status
uint8 FAILURE   = 0
uint8 SUCCESS = 1
uint8 RUNNING  = 2
uint8 IDLE  = 3

string robot_id

# command
string mission_tree
string[] plugins

# status
string mission_id
uint8 status


