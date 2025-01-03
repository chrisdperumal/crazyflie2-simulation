
#include "attitude_controller_node.h"

#include <ros/ros.h>
#include <mav_msgs/default_topics.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <time.h>
#include <chrono>

#include "rotors_control/parameters_ros.h"
#include "rotors_control/stabilizer_types.h"
//#include "rotors_control/crazyflie_complementary_filter.h"

namespace rotors_control {

LQRControllerNode::LQRControllerNode() {

    ROS_INFO("Started attitude controller");

    ros::NodeHandle nh;
    ros::NodeHandle pnh_node("~");

    InitializeParams();

        // Subscribe to command trajectory! Subscribe, so it will be able to get commands on where to go.
    cmd_multi_dof_joint_trajectory_sub_ = nh.subscribe(mav_msgs::default_topics::COMMAND_TRAJECTORY, 1,
      &LQRControllerNode::MultiDofJointTrajectoryCallback, this);

    odometry_sub_ = nh.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
      &LQRControllerNode::OdometryCallback, this);

          // To publish the propellers angular velocities
    motor_velocity_reference_pub_ = nh.advertise<mav_msgs::Actuators>(mav_msgs::default_topics::COMMAND_ACTUATORS, 1);

    // Subscribe to RateThrust message
    geometry_sub_ = nh.subscribe<mav_msgs::RateThrust>(mav_msgs::default_topics::COMMAND_RATE_THRUST, 1, 
                    &LQRControllerNode::geometryCallback, this);

}



void LQRControllerNode::MultiDofJointTrajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg) {
  // publishes the desired point only! 
  // Clear all pending commands.
   //command_timer_.stop();

   commands_.clear();
   command_waiting_times_.clear();

  const size_t n_commands = msg->points.size();

  if(n_commands < 1){
    ROS_WARN_STREAM("Got MultiDOFJointTrajectory message, but message has no points.");
    return;
  }

  mav_msgs::EigenTrajectoryPoint eigen_reference;
  mav_msgs::eigenTrajectoryPointFromMsg(msg->points.front(), &eigen_reference);
  commands_.push_front(eigen_reference);

  // ROS_INFO("Drone desired position [x_d: %f, y_d: %f, z_d: %f]", eigen_reference.position_W[0],
  //       eigen_reference.position_W[1], eigen_reference.position_W[2]);


  // We can trigger the first command immediately.
  attitude_controller_.SetTrajectoryPoint(eigen_reference);
  commands_.pop_front();


  //trajectory_msg
  if (n_commands >= 1) {
    //waypointHasBeenPublished_ = true;
    ROS_INFO_ONCE("AttitudeController got first MultiDOFJointTrajectory message.");
  }
}

void LQRControllerNode::InitializeParams() {
  ros::NodeHandle pnh("~");

  ROS_INFO("IN InitalizePARAMS");

  // Parameters reading from rosparam.
  GetRosParameter(pnh, "xy_gain_kp/x",
                  attitude_controller_.controller_parameters_.xy_gain_kp_.x(),
                  &attitude_controller_.controller_parameters_.xy_gain_kp_.x());
  GetRosParameter(pnh, "xy_gain_kp/y",
                  attitude_controller_.controller_parameters_.xy_gain_kp_.y(),
                  &attitude_controller_.controller_parameters_.xy_gain_kp_.y());
  GetRosParameter(pnh, "xy_gain_ki/x",
                  attitude_controller_.controller_parameters_.xy_gain_ki_.x(),
                  &attitude_controller_.controller_parameters_.xy_gain_ki_.x());
  GetRosParameter(pnh, "xy_gain_ki/y",
                  attitude_controller_.controller_parameters_.xy_gain_ki_.y(),
                  &attitude_controller_.controller_parameters_.xy_gain_ki_.y());

  GetRosParameter(pnh, "attitude_gain_kp/phi",
                  attitude_controller_.controller_parameters_.attitude_gain_kp_.x(),
                  &attitude_controller_.controller_parameters_.attitude_gain_kp_.x());
  GetRosParameter(pnh, "attitude_gain_kp/phi",
                  attitude_controller_.controller_parameters_.attitude_gain_kp_.y(),
                  &attitude_controller_.controller_parameters_.attitude_gain_kp_.y());
  GetRosParameter(pnh, "attitude_gain_ki/theta",
                  attitude_controller_.controller_parameters_.attitude_gain_ki_.x(),
                  &attitude_controller_.controller_parameters_.attitude_gain_ki_.x());
  GetRosParameter(pnh, "attitude_gain_ki/theta",
                  attitude_controller_.controller_parameters_.attitude_gain_ki_.y(),
                  &attitude_controller_.controller_parameters_.attitude_gain_ki_.y());

  GetRosParameter(pnh, "rate_gain_kp/p",
                  attitude_controller_.controller_parameters_.rate_gain_kp_.x(),
                  &attitude_controller_.controller_parameters_.rate_gain_kp_.x());
  GetRosParameter(pnh, "rate_gain_kp/q",
                  attitude_controller_.controller_parameters_.rate_gain_kp_.y(),
                  &attitude_controller_.controller_parameters_.rate_gain_kp_.y());
  GetRosParameter(pnh, "rate_gain_kp/r",
                  attitude_controller_.controller_parameters_.rate_gain_kp_.z(),
                  &attitude_controller_.controller_parameters_.rate_gain_kp_.z());
  GetRosParameter(pnh, "rate_gain_ki/p",
                  attitude_controller_.controller_parameters_.rate_gain_ki_.x(),
                  &attitude_controller_.controller_parameters_.rate_gain_ki_.x());
  GetRosParameter(pnh, "rate_gain_ki/q",
                  attitude_controller_.controller_parameters_.rate_gain_ki_.y(),
                  &attitude_controller_.controller_parameters_.rate_gain_ki_.y());
  GetRosParameter(pnh, "rate_gain_ki/r",
                  attitude_controller_.controller_parameters_.rate_gain_ki_.z(),
                  &attitude_controller_.controller_parameters_.rate_gain_ki_.z());

  GetRosParameter(pnh, "yaw_gain_kp/yaw",
                  attitude_controller_.controller_parameters_.yaw_gain_kp_,
                  &attitude_controller_.controller_parameters_.yaw_gain_kp_);
  GetRosParameter(pnh, "yaw_gain_ki/yaw",
                  attitude_controller_.controller_parameters_.yaw_gain_ki_,
                  &attitude_controller_.controller_parameters_.yaw_gain_ki_);

  GetRosParameter(pnh, "hovering_gain_kp/z",
                  attitude_controller_.controller_parameters_.hovering_gain_kp_,
                  &attitude_controller_.controller_parameters_.hovering_gain_kp_);
  GetRosParameter(pnh, "hovering_gain_ki/z",
                  attitude_controller_.controller_parameters_.hovering_gain_ki_,
                  &attitude_controller_.controller_parameters_.hovering_gain_ki_);
  GetRosParameter(pnh, "hovering_gain_kd/z",
                  attitude_controller_.controller_parameters_.hovering_gain_kd_,
                  &attitude_controller_.controller_parameters_.hovering_gain_kd_);

  attitude_controller_.SetControllerGains();

  ROS_INFO_ONCE("[Position Controller] Set controller gains and vehicle parameters");


  //Reading the parameters come from the launch file

  std::string user;

  if (pnh.getParam("user_account", user)){
    ROS_INFO("Got param 'user_account': %s", user.c_str());
    attitude_controller_.user_ = user;
  }
  
}


LQRControllerNode::~LQRControllerNode() {}

void LQRControllerNode::geometryCallback(const mav_msgs::RateThrustConstPtr& rate_thrust) {
   //ROS_INFO("ATTITUDECONTROLLERMESSAGE");
   last_rate_thrust_ = *rate_thrust;
}


void LQRControllerNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {

  // This functions allows us to put the odometry message into the odometry variable--> _position,
  //_orientation,_velocit_body,_angular_velocity
  EigenOdometry odometry;
  eigenOdometryFromMsg(odometry_msg, &odometry);
  ROS_INFO("ODOMETRYCALLBACK ATTITUDE CONTROLLER ");
  attitude_controller_.SetCurrentStateFromOdometry(odometry);
  
}

void LQRControllerNode::UpdateController() {

  EigenOdometry odometry;


  Eigen::Vector4d ref_rotor_velocities;
  
  // Compute control signals directly
  double delta_phi, delta_theta, delta_psi;
  double p_command, q_command, r_command;
  double theta_command, phi_command;


  // These two run in position_controller_node on different frequency. 
  // // Compute Hovering Controller
  attitude_controller_.HoveringController(&attitude_controller_.control_t_.thrust);
  // // Compute XY Controller


  attitude_controller_.XYController(&theta_command, &phi_command);
  ROS_INFO("thetha x: %f", theta_command);
  ROS_INFO("phi : %f", phi_command);
  
  // read in message that is published by position_controller_node
 


  //attitude_controller_.control_t_.thrust = last_rate_thrust_.thrust.x ;         
  // theta_command = last_rate_thrust_.thrust.y ;
  // phi_command =  last_rate_thrust_.thrust.z;

//  ROS_INFO("-----------------Received NEW--------------------------");
//   ROS_INFO("Thrust x: %f", attitude_controller_.control_t_.thrust);
//   ROS_INFO("Thrust y: %f", theta_command);
//   ROS_INFO("Thrust z: %f", phi_command);
//   ROS_INFO("-------------------------------------------");

  // Compute Attitude Controller
  attitude_controller_.AttitudeControllerFunction(&p_command, &q_command, theta_command, phi_command);

  // Compute Yaw Position Controller
  attitude_controller_.YawPositionController(&r_command);

  // Compute Rate Controller
  attitude_controller_.RateController(&delta_phi, &delta_theta, &delta_psi, p_command, q_command, r_command);

  // Compute Control Mixer
  double PWM_1, PWM_2, PWM_3, PWM_4;
  attitude_controller_.ControlMixer(attitude_controller_.control_t_.thrust, delta_phi, delta_theta, delta_psi, &PWM_1, &PWM_2, &PWM_3, &PWM_4);

  // Calculate Rotor Velocities
  attitude_controller_.CalculateRotorVelocities(&ref_rotor_velocities, PWM_1, PWM_2, PWM_3, PWM_4);



  mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);
  actuator_msg->angular_velocities.clear();
  for (int i = 0; i < ref_rotor_velocities.size(); i++)
    actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);

  actuator_msg->header.stamp = ros::Time::now();
  motor_velocity_reference_pub_.publish(actuator_msg);
  
}

} // namespace rotors_control

int main(int argc, char** argv) {
    ros::init(argc, argv, "attitude_controller_node");

    rotors_control::LQRControllerNode attitude_controller_node;

    ros::Rate rate(100); // 10 Hz

    while (ros::ok()) {
        //attitude_controller_node.UpdateController();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
