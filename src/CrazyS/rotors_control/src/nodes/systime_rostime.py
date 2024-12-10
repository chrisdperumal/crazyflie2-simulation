import re
from datetime import datetime
import matplotlib.pyplot as plt

def parse_file(file_path):
    system_times = []
    ros_times = []
    
    # Open and read the file
    with open(file_path, 'r') as file:
        lines = file.readlines()
        
        for i, line in enumerate(lines):
            # Match system time and ROS time
            if "System Time" in line:
                # Extract system time
                system_time_match = re.search(r"at (.+) \(System Time\)", line)
                ros_time_match = re.search(r"ROS Time: ([\d.]+)", line)
                
                if system_time_match and ros_time_match:
                    system_time_str = system_time_match.group(1)
                    ros_time = float(ros_time_match.group(1))
                    
                    # Parse system time into datetime object
                    system_time = datetime.strptime(system_time_str, "%a %b %d %H:%M:%S %Y")
                    
                    system_times.append(system_time)
                    ros_times.append(ros_time)
    
    return system_times, ros_times

def plot_graph(system_times, ros_times):
    # Convert datetime objects to numerical values for plotting
    plt.figure(figsize=(10, 6))
    plt.plot(system_times, ros_times, label="ROS Time vs System Time")
    
    plt.xlabel("System Time")
    plt.ylabel("ROS Time")
    plt.title("ROS Time over System Time")
    plt.grid()
    plt.legend()
    plt.xticks(rotation=45)
    plt.tight_layout()
    plt.show()

# File path
file_path = "/home/user/catkin_ws/src/CrazyS/rotors_control/src/nodes/att_contr.txt"

# Parse and plot
system_times, ros_times = parse_file(file_path)
plot_graph(system_times, ros_times)
