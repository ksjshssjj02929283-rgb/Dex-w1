# EEJointControl submitted to the Control Gateway.
uint8 SOURCE_UNKNOWN=0
uint8 SOURCE_VLA=1
uint8 SOURCE_TELEOP=2
uint8 SOURCE_SYSTEM=3

uint8 source
string session_id
uint64 command_id

# Unique for one publisher process lifetime. A restarted publisher may reset
# command_id to zero by using a new instance ID.
string publisher_instance_id

end_effector_interfaces/EEJointControl command
