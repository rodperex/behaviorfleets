# message type
uint8 REQUEST = 1
uint8 GRANT = 2
uint8 ACK = 3
uint8 DENY = 4
uint8 UPDATE = 5
uint8 PUBLISH = 6

# std_msgs/Header header
# float64 double_content
# int32 int_content
# string string_content

uint8 type
string robot_id
string[] keys
string[] key_types
string[] values

