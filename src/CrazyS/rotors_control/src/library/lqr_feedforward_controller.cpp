#include "rotors_control/lqr_feedforward_controller.h"
#include "rotors_control/Matrix3x3.h"
#include "rotors_control/Quaternion.h"
#include "rotors_control/sensfusion6.h"
#include "rotors_control/stabilizer_types.h"
#include "rotors_control/transform_datatypes.h"

#include <chrono>
#include <cmath>
#include <math.h>
#include <ros/ros.h>
#include <time.h>

#include <fstream>
#include <iostream>
#include <iterator>
#include <sstream>
#include <string>

#include <inttypes.h>

#include <nav_msgs/Odometry.h>
#include <ros/console.h>

#define M_PI 3.14159265358979323846      // Pi [rad]
#define OMEGA_OFFSET 6874                // Omega Offset [PWM]
#define ANGULAR_MOTOR_COEFFICIENT 0.2685 // Angular Motor Coefficient
#define MOTORS_INTERCEPT 426.24          // Motors Intercept [rad/s]
#define MAX_PROPELLERS_ANGULAR_VELOCITY                                        \
  2618                               // Max Propellers Angular Velocity [rad/s]
#define MAX_R_DESIDERED 3.4907       // Max R Desired Value [rad/s]
#define MAX_THETA_COMMAND 0.7236     // Max Theta Command [rad]
#define MAX_PHI_COMMAND 0.7236       // Max Phi Command [rad]
#define MAX_POS_DELTA_OMEGA 1289     // Max Positive Delta Omega [PWM]
#define MAX_NEG_DELTA_OMEGA -1718    // Max Negative Delta Omega [PWM]
#define SAMPLING_TIME 0.001          // Sampling Time [s]
#define SAMPLING_TIME_HOVERING 0.001 // Sampling Time Hovering [s]
#define MAX_ROT_VELOCITY 3052

#define GRAVITATIONAL_FORCE                                                    \
  (9.81 * 0.025)                   // Gravitational force (N) with 0.025 kg mass
#define MOTOR_CONSTANT 1.28192e-08 // Motor Thrust Constant [kg m/s^2]

// Constants for Moment of Inertia Calculations
const double M_Q = 0.025;           // Total Mass (kg) of the quadrotor
const double L = 0.046;             // Arm Length (m)
const double GAMMA = 0.00000214923; // Rotor Drag Coefficient
const double M_S = 0.410;           // Mass (kg) of the central sphere
const double R = 0.0503513;         // Radius (m) of the central sphere
const double M_PROP = 0.00311;      // Mass (kg) of the propeller
const double M_M = 0.036 + M_PROP;  // Mass (kg) of the motor + propeller

// Define the inverse control matrix M_inv
const Eigen::Matrix4d M_inv =
    (Eigen::Matrix4d() << 0.25, -1.0 / (4 * L), 1.0 / (4 * L),
     1.0 / (4 * GAMMA), 0.25, 1.0 / (4 * L), -1.0 / (4 * L), 1.0 / (4 * GAMMA),
     0.25, 1.0 / (4 * L), 1.0 / (4 * L), -1.0 / (4 * GAMMA), 0.25,
     -1.0 / (4 * L), -1.0 / (4 * L), -1.0 / (4 * GAMMA))
        .finished();

namespace rotors_control {
Eigen::Matrix<double, 4, 12> K_;

LQRFeedforwardController::LQRFeedforwardController()
    : controller_active_(false), state_estimator_active_(false),
      dataStoring_active_(false), dataStoringTime_(0), phi_command_ki_(0),
      theta_command_ki_(0), p_command_ki_(0), q_command_ki_(0),
      r_command_ki_(0), delta_psi_ki_(0), delta_omega_ki_(0),
      hovering_gain_kd_(0) {

  // The control variables are initialized to zero
  control_t_.roll = 0;
  control_t_.pitch = 0;
  control_t_.yawRate = 0;
  control_t_.thrust = 0;

  state_.angularAcc.x = 0; // Angular Acceleration x
  state_.angularAcc.y = 0; // Angular Acceleration y
  state_.angularAcc.z = 0; // Angular Acceleration z

  state_.attitude.roll = 0;  // Roll
  state_.attitude.pitch = 0; // Pitch
  state_.attitude.yaw = 0;   // Yaw

  state_.position.x = 0; // Position.x
  state_.position.y = 0; // Position.y
  state_.position.z = 0; // Position.z

  state_.angularVelocity.x = 0; // Angular velocity x
  state_.angularVelocity.y = 0; // Angular velocity y
  state_.angularVelocity.z = 0; // Angular velocity z

  state_.linearVelocity.x = 0; // Linear velocity x
  state_.linearVelocity.y = 0; // Linear velocity y
  state_.linearVelocity.z = 0; // Linear velocity z

  state_.attitudeQuaternion.x = 0; // Quaternion x
  state_.attitudeQuaternion.y = 0; // Quaternion y
  state_.attitudeQuaternion.z = 0; // Quaternion z
  state_.attitudeQuaternion.w = 0; // Quaternion w

  K_ << 3.34346154e-20, 9.82008020e-20, 5.62063822e-20, 1.22657644e-19,
      -1.88597448e-02, -6.58666618e-03, 8.69525627e-21, 1.00150277e-19,
      -9.88396508e-21, -6.82907040e-20, 9.07692670e-20, 7.19431089e-20,
      1.09427230e-18, 2.02425131e-18, 4.84821958e-04, 1.04167886e-03,
      1.92338800e-16, 1.08067201e-16, 2.08020229e-04, 1.28298213e-03,
      -4.03432595e-19, -1.29781499e-18, 7.82638971e-20, -5.62365530e-18,
      -6.81956991e-04, -1.23527414e-03, -7.99166140e-19, 4.99589885e-19,
      2.14119721e-18, 3.84185385e-18, -1.56832241e-19, -2.89601305e-19,
      2.36402264e-04, 1.66355887e-03, -4.56419088e-20, 8.77916357e-19,
      2.10693953e-19, 1.07500827e-18, 2.33324996e-18, 6.66643592e-18,
      1.21402992e-16, 2.57256807e-17, 3.46153349e-19, 3.32226906e-18,
      -1.72258166e-20, -3.87826323e-20, 2.00489708e-04, 6.74024195e-04;
}

LQRFeedforwardController::~LQRFeedforwardController() {}

// Controller gains are entered into local global variables
void LQRFeedforwardController::SetControllerGains() {

  xy_gain_kp_ = Eigen::Vector2f(controller_parameters_.xy_gain_kp_.x(),
                                controller_parameters_.xy_gain_kp_.y());
  xy_gain_ki_ = Eigen::Vector2f(controller_parameters_.xy_gain_ki_.x(),
                                controller_parameters_.xy_gain_ki_.y());

  attitude_gain_kp_ =
      Eigen::Vector2f(controller_parameters_.attitude_gain_kp_.x(),
                      controller_parameters_.attitude_gain_kp_.y());
  attitude_gain_ki_ =
      Eigen::Vector2f(controller_parameters_.attitude_gain_ki_.x(),
                      controller_parameters_.attitude_gain_ki_.y());

  rate_gain_kp_ = Eigen::Vector3f(controller_parameters_.rate_gain_kp_.x(),
                                  controller_parameters_.rate_gain_kp_.y(),
                                  controller_parameters_.rate_gain_kp_.z());
  rate_gain_ki_ = Eigen::Vector3f(controller_parameters_.rate_gain_ki_.x(),
                                  controller_parameters_.rate_gain_ki_.y(),
                                  controller_parameters_.rate_gain_ki_.z());

  yaw_gain_kp_ = controller_parameters_.yaw_gain_kp_;
  yaw_gain_ki_ = controller_parameters_.yaw_gain_ki_;

  hovering_gain_kp_ = controller_parameters_.hovering_gain_kp_;
  hovering_gain_ki_ = controller_parameters_.hovering_gain_ki_;
  hovering_gain_kd_ = controller_parameters_.hovering_gain_kd_;
}

Eigen::Vector4d LQRFeedforwardController::UpdateControllerWithLQR() {
  Eigen::VectorXd current_state(12);
  Eigen::VectorXd desired_state(12);
  Eigen::VectorXd error(12);

  current_state << state_.linearVelocity.x, // \dot{p}_x
      state_.position.x,                    // p_x
      state_.linearVelocity.y,              // \dot{p}_y
      state_.position.y,                    // p_y
      state_.linearVelocity.z,              // \dot{p}_z
      state_.position.z,                    // p_z
      state_.angularVelocity.x,             // \dot{\phi}
      state_.attitude.roll,                 // \phi
      state_.angularVelocity.y,             // \dot{\theta}
      state_.attitude.pitch,                // \theta
      state_.angularVelocity.z,             // \dot{\psi}
      state_.attitude.yaw;                  // \psi

  // Convert desired quaternion to Euler angles
  double desired_roll, desired_pitch, desired_yaw;
  tf::Quaternion desired_q(command_trajectory_.orientation_W_B.x(),
                           command_trajectory_.orientation_W_B.y(),
                           command_trajectory_.orientation_W_B.z(),
                           command_trajectory_.orientation_W_B.w());
  tf::Matrix3x3(desired_q).getRPY(desired_roll, desired_pitch, desired_yaw);

  desired_state << command_trajectory_.velocity_W[0], // \dot{p}_x
      command_trajectory_.position_W[0],              // p_x
      command_trajectory_.velocity_W[1],              // \dot{p}_y
      command_trajectory_.position_W[1],              // p_y
      command_trajectory_.velocity_W[2],              // \dot{p}_z
      command_trajectory_.position_W[2],              // p_z
      command_trajectory_.angular_velocity_W[0],      // \dot{\phi}
      desired_roll,                                   // \phi
      command_trajectory_.angular_velocity_W[1],      // \dot{\theta}
      desired_pitch,                                  // \theta
      command_trajectory_.angular_velocity_W[2],      // \dot{\psi}
      desired_yaw;                                    // \psi

  error = current_state - desired_state;

  Eigen::Vector4d control_input = -K_ * error;

  Eigen::Vector4d control_input_scaled =
      control_input * 2918 * 11.5; //* (6905.1912 * 4);

  return control_input_scaled;
}

void LQRFeedforwardController::SetTrajectoryPoint(
    const mav_msgs::EigenTrajectoryPoint &command_trajectory) {
  command_trajectory_ = command_trajectory;
  controller_active_ = true;
}

void LQRFeedforwardController::HoveringController(double *omega) {

  assert(omega);

  double z_error, z_reference, dot_zeta;
  z_reference = command_trajectory_.position_W[2];

  // the state_.position.z is not updated. always 0.
  z_error = z_reference - state_.position.z;

  // Velocity along z-axis from body to inertial frame
  double roll, pitch, yaw;
  Quaternion2Euler(&roll, &pitch, &yaw);

  // Needed because both angular and linear velocities are expressed in the
  // aircraft body frame
  dot_zeta = -sin(pitch) * state_.linearVelocity.x +
             sin(roll) * cos(pitch) * state_.linearVelocity.y +
             cos(roll) * cos(pitch) * state_.linearVelocity.z;

  double delta_omega, delta_omega_kp, delta_omega_kd;
  delta_omega_kp = hovering_gain_kp_ * z_error;
  delta_omega_ki_ =
      delta_omega_ki_ + (hovering_gain_ki_ * z_error * SAMPLING_TIME_HOVERING);
  delta_omega_kd = hovering_gain_kd_ * -dot_zeta;
  delta_omega = delta_omega_kp + delta_omega_ki_ + delta_omega_kd;

  // Delta omega value is saturated considering the aircraft physical
  // constraints
  if (delta_omega > MAX_POS_DELTA_OMEGA || delta_omega < MAX_NEG_DELTA_OMEGA)
    if (delta_omega > MAX_POS_DELTA_OMEGA)
      delta_omega = MAX_POS_DELTA_OMEGA;
    else
      delta_omega = -MAX_NEG_DELTA_OMEGA;

  *omega = OMEGA_OFFSET + delta_omega;
}
void LQRFeedforwardController::LQRFeedforwardControllerFunction(
    double *p_command, double *q_command, double theta_command,
    double phi_command) {

  assert(p_command);
  assert(q_command);

  double roll, pitch, yaw;
  Quaternion2Euler(&roll, &pitch, &yaw);

  double phi_error, theta_error;
  phi_error = phi_command - roll;
  theta_error = theta_command - pitch;

  // p = angular velocity around x axis - roll rate
  double p_command_kp, q_command_kp;
  p_command_kp = attitude_gain_kp_.x() * phi_error;
  p_command_ki_ = p_command_ki_ +
                  (attitude_gain_ki_.x() * phi_error * SAMPLING_TIME_HOVERING);
  *p_command = p_command_kp + p_command_ki_;

  // q = pitch rate
  q_command_kp = attitude_gain_kp_.y() * theta_error;
  q_command_ki_ = q_command_ki_ + (attitude_gain_ki_.y() * theta_error *
                                   SAMPLING_TIME_HOVERING);
  *q_command = q_command_kp + q_command_ki_;

  if (dataStoring_active_) {
    // Saving drone attitude in a file
    std::stringstream tempPQCommands;
    tempPQCommands << *p_command << "," << *q_command << ","
                   << odometry_.timeStampSec << "," << odometry_.timeStampNsec
                   << "\n";

    listPQCommands_.push_back(tempPQCommands.str());
  }

  ROS_DEBUG("Phi_c: %f, Phi_e: %f, Theta_c: %f, Theta_e: %f", phi_command,
            phi_error, theta_command, theta_error);
}

void LQRFeedforwardController::XYController(double *theta_command,
                                            double *phi_command) {

  assert(theta_command);
  assert(phi_command);

  double v, u;
  u = state_.linearVelocity.x;
  v = state_.linearVelocity.y;

  // u = state_.position.x;
  // v = state_.position.y;

  double xe, ye;
  ErrorBodyFrame(&xe, &ye);

  //   ROS_INFO("ErrorBodyFrame: returned: xe = %f, ye = %f", xe, ye);
  double e_vx, e_vy;
  e_vx = xe - u;
  e_vy = ye - v;

  double theta_command_kp;
  theta_command_kp = xy_gain_kp_.x() * e_vx;
  theta_command_ki_ =
      theta_command_ki_ + (xy_gain_ki_.x() * e_vx * SAMPLING_TIME);
  *theta_command = theta_command_kp + theta_command_ki_;

  double phi_command_kp;
  phi_command_kp = xy_gain_kp_.y() * e_vy;
  phi_command_ki_ = phi_command_ki_ + (xy_gain_ki_.y() * e_vy * SAMPLING_TIME);
  *phi_command = phi_command_kp + phi_command_ki_;

  // Saturate theta_command
  if (*theta_command > MAX_THETA_COMMAND) {
    *theta_command = MAX_THETA_COMMAND;
  } else if (*theta_command < -MAX_THETA_COMMAND) {
    *theta_command = -MAX_THETA_COMMAND;
  }

  // Saturate phi_command
  if (*phi_command > MAX_PHI_COMMAND) {
    *phi_command = MAX_PHI_COMMAND;
  } else if (*phi_command < -MAX_PHI_COMMAND) {
    *phi_command = -MAX_PHI_COMMAND;
  }

  ROS_DEBUG("Theta_kp: %f, Theta_ki: %f", theta_command_kp, theta_command_ki_);
  ROS_DEBUG("Phi_kp: %f, Phi_ki: %f", phi_command_kp, phi_command_ki_);
  ROS_DEBUG("Phi_c: %f, Theta_c: %f", *phi_command, *theta_command);
  ROS_DEBUG("E_vx: %f, E_vy: %f", e_vx, e_vy);
  ROS_DEBUG("E_x: %f, E_y: %f", xe, ye);
}

void LQRFeedforwardController::ControlMixer(double thrust, double delta_phi,
                                            double delta_theta,
                                            double delta_psi, double *PWM_1,
                                            double *PWM_2, double *PWM_3,
                                            double *PWM_4) {
  assert(PWM_1);
  assert(PWM_2);
  assert(PWM_3);
  assert(PWM_4);

  control_t_.thrust = thrust;

  *PWM_1 = thrust - delta_theta - delta_phi - delta_psi;
  *PWM_2 = thrust + (delta_theta / 2) - (delta_phi / 2) + delta_psi;
  *PWM_3 = thrust + (delta_theta / 2) + (delta_phi / 2) - delta_psi;
  *PWM_4 = thrust - (delta_theta / 2) + (delta_phi / 2) + delta_psi;
  Eigen::Vector4d control_inputs;

  control_inputs << thrust, delta_theta, delta_phi, delta_psi;
  Eigen::Vector4d PWM = M_inv * control_inputs;

  // Is this line necessary?
  //   control_t_.thrust = thrust;
  //   *PWM_1 = PWM(0);
  //   *PWM_2 = PWM(1);
  //   *PWM_3 = PWM(2);
  //   *PWM_4 = PWM(3);

  ROS_INFO_THROTTLE(1,
                    "Omega: %f, Delta_theta: %f, Delta_phi: %f, delta_psi: %f",
                    control_t_.thrust, delta_theta, delta_phi, delta_psi);
  ROS_INFO_THROTTLE(1, "PWM1: %f, PWM2: %f, PWM3: %f, PWM4: %f", *PWM_1, *PWM_2,
                    *PWM_3, *PWM_4);
}

void LQRFeedforwardController::YawPositionController(double *r_command) {
  assert(r_command);

  double roll, pitch, yaw;
  Quaternion2Euler(&roll, &pitch, &yaw);

  double yaw_error, yaw_reference;
  yaw_reference = command_trajectory_.getYaw();
  yaw_error = yaw_reference - yaw;

  double r_command_kp;
  r_command_kp = yaw_gain_kp_ * yaw_error;
  r_command_ki_ = r_command_ki_ + (yaw_gain_ki_ * yaw_error * SAMPLING_TIME);
  *r_command = r_command_ki_ + r_command_kp;

  // R command value is saturated considering the aircraft physical constraints
  if (!(*r_command<MAX_R_DESIDERED && * r_command> - MAX_R_DESIDERED))
    if (*r_command > MAX_R_DESIDERED)
      *r_command = MAX_R_DESIDERED;
    else
      *r_command = -MAX_R_DESIDERED;
}

void LQRFeedforwardController::ErrorBodyFrame(double *xe, double *ye) const {
  // ROS_INFO("WUHU");
  assert(xe);
  assert(ye);

  // X and Y reference coordinates
  double x_r = command_trajectory_.position_W[0];
  double y_r = command_trajectory_.position_W[1];

  // Position error
  double x_error_, y_error_;
  x_error_ = x_r - state_.position.x;
  y_error_ = y_r - state_.position.y;

  // The aircraft attitude (estimated or not, it depends by the employed
  // controller)
  double yaw, roll, pitch;
  Quaternion2Euler(&roll, &pitch, &yaw);

  // Tracking error in the body frame
  *xe = x_error_ * cos(yaw) + y_error_ * sin(yaw);
  *ye = y_error_ * cos(yaw) - x_error_ * sin(yaw);
}

// Odometry values are put in the state structure. The structure contains the
// aircraft state
void LQRFeedforwardController::SetSensorData() {
  // ROS_INFO("In sensorDATA");

  // Only the position sensor is ideal, any virtual sensor or systems is
  // available to get it
  state_.position.x = odometry_.position[0];
  state_.position.y = odometry_.position[1];
  state_.position.z = odometry_.position[2];

  state_.linearVelocity.x = odometry_.velocity[0];
  state_.linearVelocity.y = odometry_.velocity[1];
  state_.linearVelocity.z = odometry_.velocity[2];

  state_.attitudeQuaternion.x = odometry_.orientation.x();
  state_.attitudeQuaternion.y = odometry_.orientation.y();
  state_.attitudeQuaternion.z = odometry_.orientation.z();
  state_.attitudeQuaternion.w = odometry_.orientation.w();

  state_.angularVelocity.x = odometry_.angular_velocity[0];
  state_.angularVelocity.y = odometry_.angular_velocity[1];
  state_.angularVelocity.z = odometry_.angular_velocity[2];
}

void LQRFeedforwardController::RateController(
    double *delta_phi, double *delta_theta, double *delta_psi, double p_command,
    double q_command, double r_command) {

  assert(delta_phi);
  assert(delta_theta);
  assert(delta_psi);

  double p, q, r;
  p = state_.angularVelocity.x;
  q = state_.angularVelocity.y;
  r = state_.angularVelocity.z;

  double p_error, q_error, r_error;
  p_error = p_command - p;
  q_error = q_command - q;
  r_error = r_command - r;

  double delta_phi_kp, delta_theta_kp, delta_psi_kp;
  delta_phi_kp = rate_gain_kp_.x() * p_error;
  *delta_phi = delta_phi_kp;

  // computing roll and pitch?
  delta_theta_kp = rate_gain_kp_.y() * q_error;
  *delta_theta = delta_theta_kp;

  delta_psi_kp = rate_gain_kp_.z() * r_error;
  delta_psi_ki_ = delta_psi_ki_ + (rate_gain_ki_.z() * r_error * SAMPLING_TIME);
  *delta_psi = delta_psi_kp + delta_psi_ki_;
}
// Such function is invoked by the position controller node when the state
// estimator is not in the loop
void LQRFeedforwardController::SetCurrentStateFromOdometry(
    const EigenOdometry &odometry) {

  odometry_ = odometry;

  // Such function is invoked when the ideal odometry sensor is employed
  SetSensorData();
}

void LQRFeedforwardController::Quaternion2Euler(double *roll, double *pitch,
                                                double *yaw) const {
  assert(roll);
  assert(pitch);
  assert(yaw);

  // The estimated quaternion values
  double x, y, z, w;
  x = state_.attitudeQuaternion.x;
  y = state_.attitudeQuaternion.y;
  z = state_.attitudeQuaternion.z;
  w = state_.attitudeQuaternion.w;

  tf::Quaternion q(x, y, z, w);
  tf::Matrix3x3 m(q);
  m.getRPY(*roll, *pitch, *yaw);

  // ROS_DEBUG("Roll: %f, Pitch: %f, Yaw: %f", *roll, *pitch, *yaw);
}

void LQRFeedforwardController::CalculateRotorVelocities(
    Eigen::Vector4d *rotor_velocities, double PWM_1, double PWM_2, double PWM_3,
    double PWM_4) {
  assert(rotor_velocities);

  // Disable the controller if no trajectory is received
  if (!controller_active_) {
    *rotor_velocities = Eigen::Vector4d::Zero(rotor_velocities->rows());
    return;
  }

  // Compute motor velocities

  //   double omega_1 = PWM_1;
  //   double omega_2 = PWM_2;
  //   double omega_3 = PWM_3;
  //   double omega_4 = PWM_4;

  double omega_1 = (PWM_1 * ANGULAR_MOTOR_COEFFICIENT) + MOTORS_INTERCEPT;
  double omega_2 = (PWM_2 * ANGULAR_MOTOR_COEFFICIENT) + MOTORS_INTERCEPT;
  double omega_3 = (PWM_3 * ANGULAR_MOTOR_COEFFICIENT) + MOTORS_INTERCEPT;
  double omega_4 = (PWM_4 * ANGULAR_MOTOR_COEFFICIENT) + MOTORS_INTERCEPT;

  // Saturate omega values considering physical constraints
  if (!(omega_1 < MAX_PROPELLERS_ANGULAR_VELOCITY && omega_1 > 0))
    omega_1 = (omega_1 > MAX_PROPELLERS_ANGULAR_VELOCITY)
                  ? MAX_PROPELLERS_ANGULAR_VELOCITY
                  : 0;

  if (!(omega_2 < MAX_PROPELLERS_ANGULAR_VELOCITY && omega_2 > 0))
    omega_2 = (omega_2 > MAX_PROPELLERS_ANGULAR_VELOCITY)
                  ? MAX_PROPELLERS_ANGULAR_VELOCITY
                  : 0;

  if (!(omega_3 < MAX_PROPELLERS_ANGULAR_VELOCITY && omega_3 > 0))
    omega_3 = (omega_3 > MAX_PROPELLERS_ANGULAR_VELOCITY)
                  ? MAX_PROPELLERS_ANGULAR_VELOCITY
                  : 0;

  if (!(omega_4 < MAX_PROPELLERS_ANGULAR_VELOCITY && omega_4 > 0))
    omega_4 = (omega_4 > MAX_PROPELLERS_ANGULAR_VELOCITY)
                  ? MAX_PROPELLERS_ANGULAR_VELOCITY
                  : 0;

  ROS_INFO_THROTTLE(1, "Omega_1: %f Omega_2: %f Omega_3: %f Omega_4: %f",
                    omega_1, omega_2, omega_3, omega_4);
  *rotor_velocities = Eigen::Vector4d(omega_1, omega_2, omega_3, omega_4);
}

} // namespace rotors_control
