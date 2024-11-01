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

void odometryCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  current_position.x() = msg->pose.pose.position.x;
  current_position.y() = msg->pose.pose.position.y;
  current_position.z() = msg->pose.pose.position.z;
  ROS_INFO_THROTTLE(3, "Odometry current_state: [x: %f, y: %f, z: %f]",
                    current_position.x(), current_position.y(),
                    current_position.z());
}

void publishTrajectory(const ros::Publisher &publisher,
                       const Eigen::Vector3d &position, double yaw,
                       double time_from_start) {

  trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
  trajectory_msg.header.stamp = ros::Time::now();

  // Create a trajectory point
  mav_msgs::EigenTrajectoryPoint trajectory_point;
  trajectory_point.position_W = position;
  trajectory_point.setFromYaw(yaw);
  trajectory_point.time_from_start_ns =
      static_cast<int64_t>(time_from_start * 1e9);

  // Convert to ROS message
  mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory_point,
                                                &trajectory_msg);

  publisher.publish(trajectory_msg);
  ROS_INFO(
      "Publishing waypoint: [%f, %f, %f] with time_from_start: %f seconds.",
      position.x(), position.y(), position.z(), time_from_start);
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
      {0.0, 0.0, 1.0}, {0.0, 1.0, 1.0}, {0.0, 0.0, 0.0}};

  double threshold = 0.1;
  double desired_yaw = 0.0;
  double time_to_reach_waypoint = 7.0; // seconds

  for (int i = 0; i < waypoints.size(); i++) {
    Eigen::Vector3d desired_position = waypoints[i];

    // Overwrite defaults if set as node parameters.
    nh_private.param("x", desired_position.x(), desired_position.x());
    nh_private.param("y", desired_position.y(), desired_position.y());
    nh_private.param("z", desired_position.z(), desired_position.z());
    nh_private.param("yaw", desired_yaw, desired_yaw);

    publishTrajectory(trajectory_pub, desired_position, desired_yaw,
                      time_to_reach_waypoint);

    ros::Rate rate(2); // 2 Hz
    while (ros::ok() && !isCloseEnough(desired_position, threshold)) {
      ros::spinOnce();
      rate.sleep(); // Sleep for 500 millisesconds
    }

    ROS_INFO("WAYPOINT %d REACHED! Hovering for 5 seconds...", i + 1);
    ros::Time start_hover = ros::Time::now();
    ros::Duration hover_duration(5.0);
    while (ros::Time::now() - start_hover < hover_duration) {
      ros::spinOnce();
      rate.sleep();
    }
  }

  ROS_INFO("SHUTDOWN ROS! ");
  ros::shutdown();

  return 0;
}
