# Main-joint command submitted to the Control Gateway.
#
# ``header.stamp`` belongs to the source publisher clock domain. Gateway
# receive fields are populated only on the forwarded audit topic.
std_msgs/Header header

uint8 SOURCE_UNKNOWN=0
uint8 SOURCE_VLA=1
uint8 SOURCE_TELEOP=2
uint8 SOURCE_SYSTEM=3

string session_id
uint64 command_id
uint64 control_generation
uint8 source

# ``publisher_instance_id`` must be unique for each publisher process lifetime.
# A restarted process generates a new ID and may restart ``command_id`` from 0.
string publisher_host
string publisher_instance_id
string clock_domain

string[] name
float64[] position

# Populated by the Gateway for accepted/forwarded commands.
# Acceptance/forwarding does not prove that the hardware executed the command.
uint64 gateway_sequence_id
int64 gateway_receive_unix_ms
uint64 gateway_receive_monotonic_ns
