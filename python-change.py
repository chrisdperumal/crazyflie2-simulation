#!/usr/bin/env python

import rosbag
import matplotlib.pyplot as plt

# Path to your rosbag file
bag_file = "2024-11-07-16-55-00.bag"  # Updated filename
topic_name = "/crazyflie2/motor_speed/0"

# Track previous motor speed value to detect changes
previous_motor_speed = None
time_stamps = []
motor_speeds = []

# Open the rosbag
with rosbag.Bag(bag_file, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=[topic_name]):
        # Check if motor speed has changed
        if previous_motor_speed != msg.data:
            # Record the timestamp and motor speed change
            time_in_sec = t.to_sec()
            time_stamps.append(time_in_sec)
            motor_speeds.append(msg.data)
            previous_motor_speed = msg.data

# Plot the changes in motor speed over time
plt.figure(figsize=(12, 6))
plt.plot(time_stamps, motor_speeds, marker='o', color='b', label="Motor Speed Changes")
plt.xlabel("Time (seconds)")
plt.ylabel("Motor Speed")
plt.title("Motor Speed Changes Over Time")
plt.legend()
plt.grid(True)
plt.show()
