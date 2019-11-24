std_msgs/Header header
string[] name
# Joint Space
float64[] command
float64[] command_dot
float64[] state
float64[] state_dot

#Task Space
float64[] desired_pose
float64[] actual_pose

# Error
float64[] error
float64[] error_dot
float64[] error_taskspace # x, y, z, roll, pitch, yaw

# Effort
float64[] effort_command
float64[] effort_feedforward
float64[] effort_feedback
