# One atomic physics sample and the command active during that physics step.
sensor_msgs/JointState state
# Original command stamp identifies issuance, applied_at is Isaac simulation time.
sensor_msgs/JointState applied_command
builtin_interfaces/Time applied_at
# False before the first complete command has been applied.
bool command_valid
