#include <Eigen/Core>
#include <chrono>
#include <cmath>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <thread>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

Eigen::Vector3d current_position;

void odometryCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  current_position.x() = msg->pose.pose.position.x;
  current_position.y() = msg->pose.pose.position.y;
  current_position.z() = msg->pose.pose.position.z;
  // ROS_INFO("Odometry position received: [x: %f, y: %f, z: %f]",
  //          current_position.x(), current_position.y(), current_position.z());
}

std::vector<Eigen::Vector3d> computeWaypoints(double t_start, double t_end,
                                              double dt) {
  std::vector<Eigen::Vector3d> waypoints;
  for (double t = t_start; t <= t_end; t += dt) {
    double x = 2 * std::sin(0.5 * t); // x(t) = 2 * sin(0.5 * t)
    double y = std::sin(t);           // y(t) = sin(t)
    double z = 1.0;                   // z(t) = 1
    waypoints.emplace_back(x, y,
                           z); // Add the point (x, y, z) to the waypoints list
  }
  return waypoints;
}

void publishTrajectory(const ros::Publisher &publisher,
                       const Eigen::Vector3d &position, double yaw) {
  trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
  trajectory_msg.header.stamp = ros::Time::now();
  trajectory_msg.header.frame_id = "world"; // or "odom"
  trajectory_msg.header.stamp = ros::Time::now();
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(position, yaw,
                                                      &trajectory_msg);
  publisher.publish(trajectory_msg);
  // ROS_INFO("Publishing waypoint: [%f, %f, %f].", position.x(), position.y(),
  // position.z());
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "figure_eight_example");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  ros::Publisher trajectory_pub =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
          mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
  ROS_INFO("Started figure-eight example.");
  ROS_INFO("Hi Chris!");

  std_srvs::Empty srv;
  bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
  unsigned int i = 0;

  while (i <= 10 && !unpaused) {
    ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    ++i;
  }

  if (!unpaused) {
    ROS_FATAL("Could not wake up Gazebo.");
    return -1;
  } else {
    ROS_INFO("Unpaused the Gazebo simulation.");
  }

  // Subscribe to odometry node, so that we can see exact position of node:
  ros::Subscriber odometry_sub =
      nh.subscribe<nav_msgs::Odometry>("odometry", 10, odometryCallback);

  // Wait for 5 seconds to let the Gazebo GUI show up.
  ROS_INFO("WAIT 3 seconds for GUI to show up");
  ros::Duration(3.0).sleep();

  ros::Duration(5.0).sleep();

  // Compute waypoints using the provided trajectory
  double t_start = 0.0;
  double t_end = 4 * M_PI; // Duration of the trajectory in seconds
  double dt = 0.2;         // Time step (adjust for smoothness)

  std::vector<Eigen::Vector3d> waypoints = computeWaypoints(t_start, t_end, dt);

  double yaw = 0.0; // Desired yaw

  for (const auto &waypoint : waypoints) {
    publishTrajectory(trajectory_pub, waypoint, yaw);
    ros::Duration(5.0).sleep();
  }

  //   ROS_INFO("Odometry position received: [x: %f, y: %f, z: %f]",
  //            current_position.x(), current_position.y(),
  //            current_position.z());

  ros::spinOnce();
  ROS_INFO("SHUTDOWN ROS! ");
  ros::shutdown();

  return 0;
}