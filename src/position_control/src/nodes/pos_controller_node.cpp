#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <stdio.h>
#include <mav_msgs/default_topics.h>
#include <geometry_msgs/PoseStamped.h>
#include <mav_msgs/Actuators.h>
#include <mav_msgs/AttitudeThrust.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include "parameters.h"
#include "parameters_ros.h"
#include "common.h"
#include <traj_gen/min_snap_traj.h>
#include <target_detection/target_dect_msg.h>
#include <position_control/des_acc_ang.h>
#include <pd_pos_controller.h>
#include <dynamic_reconfigure/server.h>
#include <position_control/controllerPOSCFGConfig.h>
/*
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <boost/bind.hpp>
#include <mav_msgs/default_topics.h>
#include <nav_msgs/Odometry.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_msgs/conversions.h>
#include <geometry_msgs/Vector3.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>*/




namespace pos_control{
PD_position_controller pos_controller;

bool takeoff_flag = 0;

void controller_dyn_callback(position_control::controllerPOSCFGConfig &config, uint32_t level) {
  if (level & position_control::controllerPOSCFG_ENABLE_CTRL){
      if (config.new_controller_gains){
        Eigen::Vector3d ang_gain_input;
        Eigen::Vector3d pos_gain_input;
        Eigen::Vector3d velocity_gain_input;

        ang_gain_input.x() = config.ktheta;
        ang_gain_input.y() = config.kphi;
        ang_gain_input.z() = config.kpsi;

        pos_gain_input.x() = config.kx;
        pos_gain_input.y() = config.ky;
        pos_gain_input.z() = config.kz;

        velocity_gain_input.x() = config.kvx;
        velocity_gain_input.y() = config.kvy;
        velocity_gain_input.z() = config.kvz;

        pos_controller.SetParameters(ang_gain_input,pos_gain_input,velocity_gain_input);
        ROS_INFO("New controller gains");
        config.new_controller_gains   = false;
      }
  }
}

void TrajectoryCallback(const target_detection::target_dect_msg::ConstPtr &traj) {
  ROS_INFO_ONCE("PD_controller got first trajectory message.");
  pos_controller.command_trajectory_.position_W = mav_msgs::vector3FromMsg(traj->position_ref);
  pos_controller.command_trajectory_.velocity_W = mav_msgs::vector3FromMsg(traj->velocity_ref);
  pos_controller.command_trajectory_.acceleration_W = mav_msgs::vector3FromMsg(traj->accel_ref);
  pos_controller.command_trajectory_.orientation_W_B =  mav_msgs::quaternionFromYaw(traj->yaw_ref[0]);
  takeoff_flag = traj->launch_flag;
}

void OdometryCallback(const nav_msgs::Odometry::ConstPtr &odom) {

/*  ros::Time current_time = ros::Time::now();
  ros::Duration Td = current_time - gPrev_it;
  gPrev_it = current_time;
  Eigen::VectorXd prev_ang_velocity(3);
  prev_ang_velocity[0] = gOdometry.angular_velocity_B.x();
  prev_ang_velocity[1] = gOdometry.angular_velocity_B.y();
  prev_ang_velocity[2] = gOdometry.angular_velocity_B.z(); */
  EigenOdometry gOdometry;
  ROS_INFO_ONCE("PD_controller got first odometry message.");
  eigenOdometryFromMsg(odom, &gOdometry);
  pos_controller.SetOdometry(gOdometry);




}

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "position_controller_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ROS_INFO("position_controller_node main started");
  ros::Publisher commanded_thrust_pub;
  commanded_thrust_pub = nh.advertise<position_control::des_acc_ang>("/accel_ang_cmd", 1);


  //ros::Publisher normalized_motor_velocity_reference_pub_;
  //normalized_motor_velocity_reference_pub_ = nh.advertise<mavros_msgs::Thrust>("/setpoint_attitude/thrust", 1);
  //ros::Publisher angular_rate_reference_pub_;
  //angular_rate_reference_pub_ = nh.advertise<geometry_msgs::TwistStamped>("/setpoint_attitude/cmd_vel", 1);

  dynamic_reconfigure::Server<position_control::controllerPOSCFGConfig> server;
  dynamic_reconfigure::Server<position_control::controllerPOSCFGConfig>::CallbackType f;
  f = boost::bind(&pos_control::controller_dyn_callback, _1, _2);
  server.setCallback(f);


  ros::Subscriber odometry_sub_;
  //mav_msgs::default_topics::ODOMETRY
  odometry_sub_ = nh.subscribe("odometry", 1, pos_control::OdometryCallback);
  ros::Subscriber traj_sub_;
  traj_sub_ = nh.subscribe("/full_control_state", 1,
      pos_control::TrajectoryCallback);
  // int run_freq = 1000;
  // pnh.getParam("run_frequency",run_freq);
  // ROS_INFO("Param: run frequency = %d",run_freq);
  //ros::Rate r(run_freq);

  int run_freq = 200;
  ros::Rate r(run_freq);

  float normalized_thrust_out;

//  gPrev_it = ros::Time::now();

  while(ros::ok()) {

        position_control::des_acc_angPtr commanded_thrust_message(new position_control::des_acc_ang);
    // Publish data: UAV state in World frame
      if (pos_control::takeoff_flag){
        pos_control::pos_controller.CalculateThrust();

        commanded_thrust_message->accel_out.x  = pos_control::pos_controller.accel_out[0];
        commanded_thrust_message->accel_out.y  = pos_control::pos_controller.accel_out[1];
        commanded_thrust_message->accel_out.z  = pos_control::pos_controller.accel_out[2];
        commanded_thrust_message->ang_vel_out.x  = pos_control::pos_controller.att_out[0];
        commanded_thrust_message->ang_vel_out.y  = pos_control::pos_controller.att_out[1];
        commanded_thrust_message->ang_vel_out.z  = pos_control::pos_controller.att_out[2];

    } else {
        commanded_thrust_message->accel_out.x  = 0;
        commanded_thrust_message->accel_out.y  = 0;
        commanded_thrust_message->accel_out.z  = 0.1;
        commanded_thrust_message->ang_vel_out.x  = 0;
        commanded_thrust_message->ang_vel_out.y  = 0;
        commanded_thrust_message->ang_vel_out.z  = 0;
    }
      commanded_thrust_pub.publish(commanded_thrust_message);

      //mavros_msgs::ThrustPtr thrust_msg(new mavros_msgs::Thrust);
      //geometry_msgs::TwistStampedPtr ang_vel_msg(new geometry_msgs::TwistStamped);


      //thrust_msg->header.stamp = ros::Time::now();
      //ang_vel_msg->header.stamp = ros::Time::now();

      //ang_vel_msg->twist.angular.x = pos_control::pos_controller.att_out[0];
      //ang_vel_msg->twist.angular.y = pos_control::pos_controller.att_out[1];
      //ang_vel_msg->twist.angular.z = pos_control::pos_controller.att_out[2];

      //GetNormalizedThrust(&normalized_thrust_out);
      //thrust_msg->thrust = normalized_thrust_out;


    ros::spinOnce();
    r.sleep();
  } // end while

  return 0;
}
