# Joint command submitted to the control gateway.
#
# ``header.stamp`` belongs to the source publisher clock domain.  Gateway
# receive fields are populated only on the forwarded audit topic.
std_msgs/Header header

uint8 SOURCE_UNKNOWN=0
uint8 SOURCE_VLA=1
uint8 SOURCE_TELEOP=2

string session_id
uint64 command_id
uint64 control_generation
uint8 source

string publisher_host
string publisher_instance_id
string clock_domain

string[] name
float64[] position

# Populated by the gateway for accepted/forwarded commands.
uint64 gateway_sequence_id
int64 gateway_receive_unix_ms
uint64 gateway_receive_monotonic_ns
