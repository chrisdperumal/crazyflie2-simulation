import matplotlib.pyplot as plt
import re
from datetime import datetime

# File path
file_path = '/home/user/catkin_ws/src/CrazyS/rotors_control/src/nodes/att_contr.txt'

# Lists to hold the real system time, ROS time, Gazebo time, and the difference
real_times = []
ros_times = []
gazebo_times = []
time_differences = []

# Regex pattern to match system time, ROS time, and Gazebo time
pattern = r"AttitudeControllerNode published rotorVelocities at (.*) \(System Time\), ROS Time: ([\d\.]+), Gazebo Time: ([\d\.]+)"

# Read the file
with open(file_path, 'r') as file:
    last_real_time = None
    last_recorded_time = None  # Keep track of the last recorded time (for 0.1s interval check)
    for line in file:
        match = re.search(pattern, line)
        if match:
            # Extract real system time, ROS time, and Gazebo time from the line
            real_time_str = match.group(1)
            ros_time = float(match.group(2))
            gazebo_time = float(match.group(3))

            # Convert real time string to datetime object
            real_time = datetime.strptime(real_time_str, "%a %b %d %H:%M:%S %Y")
            
            # If this is the first line, initialize the last_real_time
            if last_real_time is None:
                last_real_time = real_time

            # Calculate the time difference in seconds from the first recorded time
            time_diff = (real_time - last_real_time).total_seconds()

            # Start recording data only after the first 5 seconds of real time have passed
            if time_diff >= 5.0:
                # Record data only if 0.1 seconds have passed
                if last_recorded_time is None or time_diff - last_recorded_time >= 0.01:
                    # Store the real time, ROS time, Gazebo time, and the difference
                    real_times.append(real_time)
                    ros_times.append(ros_time)
                    gazebo_times.append(gazebo_time)
                    time_differences.append(ros_time - gazebo_time)  # Calculate the difference

                    # Update the last recorded time to the current time_diff
                    last_recorded_time = time_diff

# Plotting the difference between ROS time and Gazebo time
plt.figure(figsize=(10, 6))
plt.plot(real_times, time_differences, linestyle='-', color='r')

# Adding labels and title
plt.xlabel('Real System Time')
plt.ylabel('Difference (ROS Time - Gazebo Time) (seconds)')
plt.title('Difference between ROS Time and Gazebo Time over Real Time')

# Format the x-axis to display the date and time correctly
plt.gcf().autofmt_xdate()

# Display the plot
plt.grid(True)
plt.show()
