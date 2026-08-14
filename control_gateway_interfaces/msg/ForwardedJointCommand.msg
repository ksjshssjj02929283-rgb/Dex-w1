# Main-joint command accepted and forwarded by the Control Gateway.
control_gateway_interfaces/JointCommand command

uint8 TARGET_SIMULATION=1
uint8 TARGET_REAL=2

# Execution target latched by StartJointCommandSession.
uint8 target

# Gateway-local acceptance order and receive times.
# Acceptance/forwarding does not prove that the hardware executed the command.
uint64 gateway_sequence_id
int64 gateway_receive_unix_ms
uint64 gateway_receive_monotonic_ns
