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

  // Compute waypoints using the provided trajectory
  double t_start = 0.0;
  double t_end = 4 * M_PI; // Duration of the trajectory in seconds
  double dt = 0.02;        // Time step (adjust for smoothness)

  std::vector<Eigen::Vector3d> waypoints = computeWaypoints(t_start, t_end, dt);

  double threshold =
      0.2; // Distance threshold to consider the drone has reached the waypoint
  double desired_yaw = 0.0;
  ros::Duration time_from_start(5.0);

  ROS_INFO("Total waypoints: %ld", waypoints.size());

  for (int i = 0; i <= waypoints.size(); i++) {
    Eigen::Vector3d desired_position = waypoints[i];

    // Overwrite defaults if set as node parameters.
    nh_private.param("x", desired_position.x(), desired_position.x());
    nh_private.param("y", desired_position.y(), desired_position.y());
    nh_private.param("z", desired_position.z(), desired_position.z());
    nh_private.param("yaw", desired_yaw, desired_yaw);

    publishTrajectory(trajectory_pub, desired_position, desired_yaw,
                      time_from_start);

    ros::Time last_info_time = ros::Time::now();

    while (ros::ok() && !isCloseEnough(desired_position, threshold)) {
      ros::spinOnce();
      ROS_INFO("Current pos: [x: %f, y: %f, z: %f], Target pos: [x: %f, y: %f, "
               "z: %f], Distance: %f",
               current_position.x(), current_position.y(), current_position.z(),
               desired_position.x(), desired_position.y(), desired_position.z(),
               (current_position - desired_position).norm());

      ros::Duration(0.1).sleep();
    }

    ROS_INFO("WAYPOINT %d REACHED!", i + 1);
  }

  // Now descend to the ground and land
  ROS_INFO("All waypoints reached. Starting landing sequence.");

  Eigen::Vector3d landing_position = current_position; // Current x, y
  landing_position.z() = 0.0;                          // Set z to ground level

  publishTrajectory(trajectory_pub, landing_position, desired_yaw,
                    time_from_start);

  // Wait for the drone to reach the ground
  while (ros::ok() && !isCloseEnough(landing_position, threshold)) {
    ros::spinOnce();
    // ROS_INFO("Landing... Current position: [x: %f, y: %f, z: %f]",
    //          current_position.x(), current_position.y(),
    //          current_position.z());
    ros::Duration(0.5).sleep();
  }

  ROS_INFO("Landed. Shutting down...");
  ros::shutdown();
  return 0;
}
