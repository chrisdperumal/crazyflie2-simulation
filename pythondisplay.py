#!/usr/bin/env python

import rosbag
import matplotlib.pyplot as plt
from collections import defaultdict

# Path to your rosbag file
bag_file = "2024-11-07-17-13-42.bag" 
topic_names = ["/crazyflie2/ground_truth/odometry", "/crazyflie2/odometry", "/crazyflie2/motor_speed/0"]

# Dictionary to store message counts per topic per second
message_counts = {topic: defaultdict(int) for topic in topic_names}

# Open the rosbag
with rosbag.Bag(bag_file, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=topic_names):
        # Extract the timestamp in seconds
        time_in_sec = int(t.to_sec())
        # Count messages per second for each topic
        message_counts[topic][time_in_sec] += 1

# Prepare data for plotting
plt.figure(figsize=(12, 6))
for topic in topic_names:
    time_stamps = sorted(message_counts[topic].keys())
    message_frequencies = [message_counts[topic][t] for t in time_stamps]
    
    # Plot frequency data for each topic
    plt.plot(time_stamps, message_frequencies, label=f"{topic} Messages per Second")

# Set plot labels and title
plt.xlabel("Time (seconds)")
plt.ylabel("Message Frequency (messages/sec)")
plt.title("Message Frequency for Topics Over Time")
plt.legend()
plt.grid()
plt.show()
