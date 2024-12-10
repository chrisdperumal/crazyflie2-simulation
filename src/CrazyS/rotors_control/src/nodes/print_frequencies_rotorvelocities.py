import matplotlib.pyplot as plt
import re

# File path
file_path = '/home/user/catkin_ws/src/CrazyS/rotors_control/src/nodes/att_contr.txt'

# Lists to hold the ROS time and Gazebo time values
ros_times = []
gazebo_times = []

# Regex pattern to match ROS time and Gazebo time
pattern = r"ROS Time: ([\d\.]+), Gazebo Time: ([\d\.]+)"

# Read the file
with open(file_path, 'r') as file:
    for line in file:
        match = re.search(pattern, line)
        if match:
            ros_time = float(match.group(1))
            gazebo_time = float(match.group(2))
            ros_times.append(ros_time)
            gazebo_times.append(gazebo_time)

# Plotting the data
plt.figure(figsize=(10, 6))
plt.plot(ros_times, gazebo_times, color='b')

# Adding labels and title
plt.xlabel('ROS Time (seconds)')
plt.ylabel('Gazebo Time (seconds)')
plt.title('Gazebo Time vs ROS Time')

# Display the plot
plt.grid(True)
plt.show()
