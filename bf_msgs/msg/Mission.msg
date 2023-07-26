# message type
uint8 OFFER = 100
uint8 COMMAND = 101
uint8 REQUEST = 102
uint8 STATUS = 103
uint8 HALT = 104
uint8 REJECT = 105

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
string cost_f # identifier of the function cost for the agents to apply

# request
float64 cost

# status
uint8 status


