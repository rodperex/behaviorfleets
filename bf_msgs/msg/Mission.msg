# message type
uint8 COMMAND = 100
uint8 REQUEST = 101
uint8 STATUS = 102
uint8 HALT = 103
uint8 REJECT = 104

# status codes
uint8 FAILURE = 0
uint8 SUCCESS = 1
uint8 RUNNING = 2
uint8 IDLE = 3

# common
uint8 msg_type
string mission_id
string robot_id

# command
string mission_tree
string[] plugins

# status
uint8 status


