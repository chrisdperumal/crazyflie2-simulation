
#include "lqr_feedforward_controller_node.h"

#include <chrono>
#include <mav_msgs/default_topics.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <time.h>

#include "rotors_control/parameters_ros.h"
#include "rotors_control/stabilizer_types.h"
// #include "rotors_control/crazyflie_complementary_filter.h"

#define GRAVITATIONAL_FORCE                                                    \
  9.81 * 0.025; // 0.025 is the mass of the drone in Kg

#define DENORMALIED_YAW_CONSTANT 8.06428 * 10e-5 * 2618 * 2618;

namespace rotors_control {

LQRControllerNode::LQRControllerNode() {

  ROS_INFO("Started LQR feedforward controller");

  ros::NodeHandle nh;
  ros::NodeHandle pnh_node("~");

  InitializeParams();

  // Subscribe to command trajectory! Subscribe, so it will be able to get
  // commands on where to go.
  cmd_multi_dof_joint_trajectory_sub_ =
      nh.subscribe(mav_msgs::default_topics::COMMAND_TRAJECTORY, 1,
                   &LQRControllerNode::MultiDofJointTrajectoryCallback, this);

  odometry_sub_ = nh.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
                               &LQRControllerNode::OdometryCallback, this);

  // To publish the propellers angular velocities
  motor_velocity_reference_pub_ = nh.advertise<mav_msgs::Actuators>(
      mav_msgs::default_topics::COMMAND_ACTUATORS, 1);

  // Subscribe to RateThrust message
  geometry_sub_ = nh.subscribe<mav_msgs::RateThrust>(
      mav_msgs::default_topics::COMMAND_RATE_THRUST, 1,
      &LQRControllerNode::geometryCallback, this);
}

void LQRControllerNode::MultiDofJointTrajectoryCallback(
    const trajectory_msgs::MultiDOFJointTrajectoryConstPtr &msg) {
  // publishes the desired point only!
  // Clear all pending commands.
  // command_timer_.stop();

  commands_.clear();
  command_waiting_times_.clear();

  const size_t n_commands = msg->points.size();

  if (n_commands < 1) {
    ROS_WARN_STREAM(
        "Got MultiDOFJointTrajectory message, but message has no points.");
    return;
  }

  mav_msgs::EigenTrajectoryPoint eigen_reference;
  mav_msgs::eigenTrajectoryPointFromMsg(msg->points.front(), &eigen_reference);
  commands_.push_front(eigen_reference);

  ROS_INFO("Drone desired position [x_d: %f, y_d: %f, z_d: %f]",
           eigen_reference.position_W[0], eigen_reference.position_W[1],
           eigen_reference.position_W[2]);

  // ROS_INFO_STREAM("Eigen Reference: Position [" <<
  // eigen_reference.transpose() << "], ");

  // We can trigger the first command immediately.
  lqr_feedforward_controller_.SetTrajectoryPoint(eigen_reference);
  commands_.pop_front();

  // trajectory_msg
  if (n_commands >= 1) {
    waypointHasBeenPublished_ = true;
    ROS_INFO_ONCE(
        "AttitudeController got first MultiDOFJointTrajectory message.");
  }
}

void LQRControllerNode::InitializeParams() {
  ros::NodeHandle pnh("~");

  ROS_INFO("IN InitalizePARAMS");

  // Parameters reading from rosparam.
  GetRosParameter(
      pnh, "xy_gain_kp/x",
      lqr_feedforward_controller_.controller_parameters_.xy_gain_kp_.x(),
      &lqr_feedforward_controller_.controller_parameters_.xy_gain_kp_.x());
  GetRosParameter(
      pnh, "xy_gain_kp/y",
      lqr_feedforward_controller_.controller_parameters_.xy_gain_kp_.y(),
      &lqr_feedforward_controller_.controller_parameters_.xy_gain_kp_.y());
  GetRosParameter(
      pnh, "xy_gain_ki/x",
      lqr_feedforward_controller_.controller_parameters_.xy_gain_ki_.x(),
      &lqr_feedforward_controller_.controller_parameters_.xy_gain_ki_.x());
  GetRosParameter(
      pnh, "xy_gain_ki/y",
      lqr_feedforward_controller_.controller_parameters_.xy_gain_ki_.y(),
      &lqr_feedforward_controller_.controller_parameters_.xy_gain_ki_.y());

  GetRosParameter(
      pnh, "attitude_gain_kp/phi",
      lqr_feedforward_controller_.controller_parameters_.attitude_gain_kp_.x(),
      &lqr_feedforward_controller_.controller_parameters_.attitude_gain_kp_
           .x());
  GetRosParameter(
      pnh, "attitude_gain_kp/phi",
      lqr_feedforward_controller_.controller_parameters_.attitude_gain_kp_.y(),
      &lqr_feedforward_controller_.controller_parameters_.attitude_gain_kp_
           .y());
  GetRosParameter(
      pnh, "attitude_gain_ki/theta",
      lqr_feedforward_controller_.controller_parameters_.attitude_gain_ki_.x(),
      &lqr_feedforward_controller_.controller_parameters_.attitude_gain_ki_
           .x());
  GetRosParameter(
      pnh, "attitude_gain_ki/theta",
      lqr_feedforward_controller_.controller_parameters_.attitude_gain_ki_.y(),
      &lqr_feedforward_controller_.controller_parameters_.attitude_gain_ki_
           .y());

  GetRosParameter(
      pnh, "rate_gain_kp/p",
      lqr_feedforward_controller_.controller_parameters_.rate_gain_kp_.x(),
      &lqr_feedforward_controller_.controller_parameters_.rate_gain_kp_.x());
  GetRosParameter(
      pnh, "rate_gain_kp/q",
      lqr_feedforward_controller_.controller_parameters_.rate_gain_kp_.y(),
      &lqr_feedforward_controller_.controller_parameters_.rate_gain_kp_.y());
  GetRosParameter(
      pnh, "rate_gain_kp/r",
      lqr_feedforward_controller_.controller_parameters_.rate_gain_kp_.z(),
      &lqr_feedforward_controller_.controller_parameters_.rate_gain_kp_.z());
  GetRosParameter(
      pnh, "rate_gain_ki/p",
      lqr_feedforward_controller_.controller_parameters_.rate_gain_ki_.x(),
      &lqr_feedforward_controller_.controller_parameters_.rate_gain_ki_.x());
  GetRosParameter(
      pnh, "rate_gain_ki/q",
      lqr_feedforward_controller_.controller_parameters_.rate_gain_ki_.y(),
      &lqr_feedforward_controller_.controller_parameters_.rate_gain_ki_.y());
  GetRosParameter(
      pnh, "rate_gain_ki/r",
      lqr_feedforward_controller_.controller_parameters_.rate_gain_ki_.z(),
      &lqr_feedforward_controller_.controller_parameters_.rate_gain_ki_.z());

  GetRosParameter(
      pnh, "yaw_gain_kp/yaw",
      lqr_feedforward_controller_.controller_parameters_.yaw_gain_kp_,
      &lqr_feedforward_controller_.controller_parameters_.yaw_gain_kp_);
  GetRosParameter(
      pnh, "yaw_gain_ki/yaw",
      lqr_feedforward_controller_.controller_parameters_.yaw_gain_ki_,
      &lqr_feedforward_controller_.controller_parameters_.yaw_gain_ki_);

  GetRosParameter(
      pnh, "hovering_gain_kp/z",
      lqr_feedforward_controller_.controller_parameters_.hovering_gain_kp_,
      &lqr_feedforward_controller_.controller_parameters_.hovering_gain_kp_);
  GetRosParameter(
      pnh, "hovering_gain_ki/z",
      lqr_feedforward_controller_.controller_parameters_.hovering_gain_ki_,
      &lqr_feedforward_controller_.controller_parameters_.hovering_gain_ki_);
  GetRosParameter(
      pnh, "hovering_gain_kd/z",
      lqr_feedforward_controller_.controller_parameters_.hovering_gain_kd_,
      &lqr_feedforward_controller_.controller_parameters_.hovering_gain_kd_);

  lqr_feedforward_controller_.SetControllerGains();

  ROS_INFO_ONCE(
      "[Position Controller] Set controller gains and vehicle parameters");

  // Reading the parameters come from the launch file

  std::string user;

  if (pnh.getParam("user_account", user)) {
    ROS_INFO("Got param 'user_account': %s", user.c_str());
    lqr_feedforward_controller_.user_ = user;
  }
}

LQRControllerNode::~LQRControllerNode() {}

void LQRControllerNode::geometryCallback(
    const mav_msgs::RateThrustConstPtr &rate_thrust) {
  // ROS_INFO("ATTITUDECONTROLLERMESSAGE");
  last_rate_thrust_ = *rate_thrust;
}

void LQRControllerNode::OdometryCallback(
    const nav_msgs::OdometryConstPtr &odometry_msg) {
  EigenOdometry odometry;
  eigenOdometryFromMsg(odometry_msg, &odometry);

  lqr_feedforward_controller_.SetCurrentStateFromOdometry(odometry);
}

void LQRControllerNode::UpdateController() {

  if (waypointHasBeenPublished_) {
    EigenOdometry odometry;

    Eigen::Vector4d ref_rotor_velocities;

    // Compute control signals directly
    double delta_phi, delta_theta, delta_psi;
    double thrust;
    double p_command, q_command, r_command;

    double theta_command, phi_command;
    lqr_feedforward_controller_.control_t_.thrust = last_rate_thrust_.thrust.x;
    theta_command = last_rate_thrust_.thrust.y;
    phi_command = last_rate_thrust_.thrust.z;

    // lqr_feedforward_controller_.HoveringController(
    //     &lqr_feedforward_controller_.control_t_.thrust);
    // lqr_feedforward_controller_.XYController(&theta_command, &phi_command);

    // // Compute Attitude Controller
    // lqr_feedforward_controller_.LQRFeedforwardControllerFunction(
    //     &p_command, &q_command, theta_command, phi_command);

    // // Compute Yaw Position Controller
    // lqr_feedforward_controller_.YawPositionController(&r_command);

    // // Compute Rate Controller
    // lqr_feedforward_controller_.RateController(
    //     &delta_phi, &delta_theta, &delta_psi, p_command, q_command,
    //     r_command);

    // Update the values with the K Matrix
    Eigen::Vector4d control =
        lqr_feedforward_controller_.UpdateControllerWithLQR();

    // lqr_feedforward_controller_.control_t_.thrust = control(0);
    thrust = control(0);
    delta_phi = control(1);
    delta_theta = control(2);
    delta_psi = control(3);

    lqr_feedforward_controller_.control_t_.thrust = thrust;

    // ROS_INFO_STREAM_THROTTLE(1, "Computed thrust: " << control(0));
    ROS_INFO_STREAM_THROTTLE(1, "Control Signal: ["
                                    << thrust << ", " << delta_phi << ", "
                                    << delta_theta << ", " << delta_psi << "]");

    // Send the new values to the control mixer
    double PWM_1, PWM_2, PWM_3, PWM_4;
    lqr_feedforward_controller_.ControlMixer(thrust, delta_phi, delta_theta,
                                             delta_psi, &PWM_1, &PWM_2, &PWM_3,
                                             &PWM_4);

    // Calculate Rotor Velocities

    lqr_feedforward_controller_.CalculateRotorVelocities(
        &ref_rotor_velocities, PWM_1, PWM_2, PWM_3, PWM_4);

    // Prepare the actuator message
    mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);
    actuator_msg->angular_velocities.clear();
    for (int i = 0; i < ref_rotor_velocities.size(); i++)
      actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);

    actuator_msg->header.stamp = ros::Time::now();
    motor_velocity_reference_pub_.publish(actuator_msg);
  }
}

} // namespace rotors_control

int main(int argc, char **argv) {
  ros::init(argc, argv, "lqr_feedforward_controller_node");
  rotors_control::LQRControllerNode lqr_controller_node;

  ros::Rate rate(100); // 100 Hz

  while (ros::ok()) {
    lqr_controller_node.UpdateController();
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
