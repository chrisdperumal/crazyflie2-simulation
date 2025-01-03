#ifndef ROTORS_CONTROL_ATTITUDE_CONTROLLER_NODE_H
#define ROTORS_CONTROL_ATTITUDE_CONTROLLER_NODE_H

#include <Eigen/Eigen>
#include <boost/bind.hpp>
#include <stdio.h>

#include <fstream> // Include this for file operations
#include <geometry_msgs/PoseStamped.h>
#include <mav_msgs/Actuators.h>
#include <mav_msgs/AttitudeThrust.h>
#include <mav_msgs/DroneState.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <nav_msgs/Odometry.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include "rotors_control/attitude_controller.h"
#include "rotors_control/common.h"

#include <rosgraph_msgs/Clock.h>

namespace rotors_control {

class AttitudeControllerNode {
private:
  std::ofstream clock_file_;

public:
  ros::NodeHandle nh_;
  ros::Time gazebo_time_;
  AttitudeControllerNode();
  ~AttitudeControllerNode();
  void clockCallback(const rosgraph_msgs::Clock::ConstPtr &msg);

  void InitializeParams();
  void MultiDofJointTrajectoryCallback(
      const trajectory_msgs::MultiDOFJointTrajectoryConstPtr
          &trajectory_reference_msg);
  void OdometryCallback(const nav_msgs::OdometryConstPtr &msg);
  void UpdateController();
  void geometryCallback(const mav_msgs::RateThrustConstPtr &rate_thrust);
  void writeGazeboClock();
  ros::Subscriber cmd_multi_dof_joint_trajectory_sub_;
  ros::Subscriber odometry_sub_;
  ros::Subscriber geometry_sub_;
  ros::Subscriber clock_sub_;
  ros::Publisher motor_velocity_reference_pub_;
  bool waypointHasBeenPublished_ = false;

  mav_msgs::EigenTrajectoryPointDeque commands_;
  std::deque<ros::Duration> command_waiting_times_;
  void writeToFile(double thrust, double theta_command, double phi_command);

  AttitudeController attitude_controller_;

  mav_msgs::RateThrust last_rate_thrust_;
};

} // namespace rotors_control

#endif // ROTORS_CONTROL_ATTITUDE_CONTROLLER_NODE_H