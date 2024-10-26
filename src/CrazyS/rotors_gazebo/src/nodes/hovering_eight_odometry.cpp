#include "ros/duration.h"
#include <Eigen/Core>
#include <chrono>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <thread>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

Eigen::Vector3d current_position;
double max_allowed_deviation = 1.0; // Set a threshold for maximum deviation

void odometryCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  current_position.x() = msg->pose.pose.position.x;
  current_position.y() = msg->pose.pose.position.y;
  current_position.z() = msg->pose.pose.position.z;
  //   ROS_INFO("Odometry position: [x: %f, y: %f, z: %f]",
  //   current_position.x(),
  //    current_position.y(), current_position.z());
}

void publishTrajectory(const ros::Publisher &publisher,
                       const Eigen::Vector3d &position, double yaw,
                       const ros::Duration time_from_start) {

  trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
  trajectory_msg.header.stamp = ros::Time::now(); // Set the header timestamp
  trajectory_msg.header.frame_id = "world";       // Set the reference frame

  // Create a new trajectory point
  trajectory_msgs::MultiDOFJointTrajectoryPoint point;

  // Set the position and yaw for the trajectory point
  geometry_msgs::Transform transform;
  transform.translation.x = position.x();
  transform.translation.y = position.y();
  transform.translation.z = position.z();
  transform.rotation.x = 0.0;
  transform.rotation.y = 0.0;
  transform.rotation.z = 0.0;
  transform.rotation.w = 1.0; // No rotation (yaw fixed at 0)

  // Add the transform to the trajectory point
  point.transforms.push_back(transform);

  // Set velocities and accelerations (optional, zero for simplicity)
  geometry_msgs::Twist velocity, acceleration;
  velocity.linear.x = 0.0;
  velocity.linear.y = 0.0;
  velocity.linear.z = 0.0;
  velocity.angular.x = 0.0;
  velocity.angular.y = 0.0;
  velocity.angular.z = 0.0;

  point.velocities.push_back(velocity); // Add the velocity (optional)
  point.accelerations.push_back(
      acceleration); // Add the acceleration (optional)

  // Set the time from start for this point
  point.time_from_start = time_from_start;

  // Add the point to the trajectory message
  trajectory_msg.points.push_back(point);

  // Publish the trajectory message
  publisher.publish(trajectory_msg);

  // Log the information
  ROS_INFO("Published Waypoint: [%f, %f, %f] to be reached in [%f] seconds.",
           position.x(), position.y(), position.z(), time_from_start.toSec());
}

bool isOutOfControl(const Eigen::Vector3d &target_position,
                    double max_deviation) {
  double distance =
      (current_position - target_position).norm(); // Euclidean distance
  return distance > max_deviation;
}

bool isCloseEnough(const Eigen::Vector3d &target_position, double threshold) {
  double distance =
      std::sqrt(std::pow(current_position.x() - target_position.x(), 2) +
                std::pow(current_position.y() - target_position.y(), 2) +
                std::pow(current_position.z() - target_position.z(), 2));
  return distance < threshold;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "trajectory_follower");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  ros::Publisher trajectory_pub =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
          mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);

  ros::Subscriber odometry_sub =
      nh.subscribe<nav_msgs::Odometry>("odometry", 10, odometryCallback);

  ROS_INFO("Started trajectory follower.");

  bool use_sim_time = true;
  if (ros::param::get("/use_sim_time", use_sim_time)) {
    ROS_INFO("Using simulated time.");
  } else {
    ROS_WARN("Not using simulated time, check your environment.");
  }

  std_srvs::Empty srv;
  bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
  unsigned int i = 0;

  while (i <= 15 && !unpaused) {
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

  ros::Duration(3.0).sleep();

  // define waypoints where to fly to:
  std::vector<Eigen::Vector3d> waypoints = {
      {0.0, 0.0, 1.0}, {0.0, 1.0, 1.0},
      // {2.0, 0.0, 0.0}
  };

  double threshold =
      0.5; // Distance threshold to consider the drone has reached the waypoint
  double desired_yaw = 0.0;

  for (int i = 0; i < waypoints.size(); ++i) {
    Eigen::Vector3d desired_position = waypoints[i];

    // Increment the time_from_start to give enough time for the drone to reach
    // the next waypoint
    ros::Duration time_from_start(
        5.0 * (i + 1)); // Allow 5 seconds to reach each waypoint

    // Publish the trajectory
    publishTrajectory(trajectory_pub, desired_position, desired_yaw,
                      time_from_start);

    // Wait for the drone to reach the waypoint
    while (ros::ok() && !isCloseEnough(desired_position, threshold)) {
      ros::spinOnce();
      ROS_INFO("Current position: [%f, %f, %f]", current_position.x(),
               current_position.y(), current_position.z());
      ROS_INFO("Distance to target: %f",
               (current_position - desired_position).norm());
      ros::Duration(0.1)
          .sleep(); // Sleep briefly to avoid overwhelming the loop
    }

    // Hover for 5 seconds after reaching the waypoint
    ROS_INFO("WAYPOINT %d REACHED! Hovering for 5 seconds...", i + 1);
    ros::Time start_hover = ros::Time::now();
    ros::Duration hover_duration(5.0);
    while (ros::Time::now() - start_hover < hover_duration) {
      ros::spinOnce();
      ros::Duration(0.1).sleep();
    }
  }

  // Now descend to the ground and land
  ROS_INFO("All waypoints reached. Starting landing sequence.");

  ros::Duration landing_time(10);

  Eigen::Vector3d landing_position = current_position; // Current x, y
  landing_position.z() = 0.0;                          // Set z to ground level

  publishTrajectory(trajectory_pub, landing_position, desired_yaw,
                    landing_time);

  // Wait for the drone to reach the ground
  while (ros::ok() && !isCloseEnough(landing_position, threshold)) {
    ros::spinOnce();
    ROS_INFO("Landing... Current position: [x: %f, y: %f, z: %f]",
             current_position.x(), current_position.y(), current_position.z());
    ros::Duration(0.5).sleep();
  }

  ROS_INFO("Landed. Shutting down...");
  ros::shutdown();

  return 0;
}
