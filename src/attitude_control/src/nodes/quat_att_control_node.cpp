#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <stdio.h>
#include <mav_msgs/default_topics.h>
#include <geometry_msgs/PoseStamped.h>
#include <mav_msgs/Actuators.h>
#include <mav_msgs/AttitudeThrust.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <nav_msgs/Odometry.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include "parameters.h"
#include "parameters_ros.h"
#include "common.h"
#include <traj_gen/min_snap_traj.h>
#include <position_control/des_acc_ang.h>
#include <attitude_control/uav_state.h>
#include <quat_att_control.h>
#include <dynamic_reconfigure/server.h>
#include <attitude_control/controllerDynAttConfig.h>

namespace att_control{
attitude_controller att_control;
int got_odom = 0;
int got_thrust_moment = 0;
bool gPublish = false;
Eigen::Vector3d pos_ref_pub;
Eigen::Vector3d vel_ref_pub;
Eigen::Vector3d acc_ref_pub;
bool launch_flag_pub;
double yaw_ref_pub;
bool emergency_mode = false;
/*void TrajectoryCallback(const position_control::des_acc_ang::ConstPtr &traj) {
  ROS_INFO_ONCE("Attitude controller got first trajectory message.");
  att_control.command_trajectory_.position_W = mav_msgs::vector3FromMsg(traj->position_ref);
  att_control.command_trajectory_.velocity_W = mav_msgs::vector3FromMsg(traj->velocity_ref);
  att_control.command_trajectory_.orientation_W_B =  mav_msgs::quaternionFromYaw(traj->yaw_ref[0]);
}*/

void controller_dyn_callback(attitude_control::controllerDynAttConfig &config, uint32_t level) {
  if (level & attitude_control::controllerDynAtt_ENABLE_CTRL){
      if (config.new_parameters){

        Eigen::Vector3d moment_gain_input;

        moment_gain_input.x() = config.Mx;
        moment_gain_input.y() = config.My;
        moment_gain_input.z() = config.Mz;

        att_control.SetParameters(config.kt,config.kd,moment_gain_input);

        ROS_INFO("New parameters");
        config.new_parameters   = false;
      }
  }
}




void TrajectoryCallback(const traj_gen::min_snap_traj::ConstPtr &traj) {
  ROS_INFO_ONCE("PD_controller got first trajectory message.");
  pos_ref_pub = mav_msgs::vector3FromMsg(traj->position_ref);
  vel_ref_pub = mav_msgs::vector3FromMsg(traj->velocity_ref);
  acc_ref_pub = mav_msgs::vector3FromMsg(traj->accel_ref);
  yaw_ref_pub = traj->yaw_ref[0];
  launch_flag_pub = traj->launch_flag;
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
  ROS_INFO_ONCE("Attidude controller got first odometry message.");
  eigenOdometryFromMsg(odom, &gOdometry);
  att_control.SetOdometry(gOdometry);
  if(!got_odom){
    got_odom =1;
  }
  if ((gOdometry.position.x() > 10000)||(gOdometry.position.x() < -10000)
    ||(gOdometry.position.y() > 10000)||(gOdometry.position.y() < -10000)
    ||(gOdometry.position.z() > 10000)) {
      emergency_mode = true;
    }
}

void ThrustCallback(const position_control::des_acc_ang::ConstPtr &thrust_pd) {
  Eigen::Vector3d thrust_pd_eig;
  Eigen::Vector3d ang_acc;
  thrust_pd_eig = mav_msgs::vector3FromMsg(thrust_pd->accel_out);
  ang_acc = mav_msgs::vector3FromMsg(thrust_pd->ang_vel_out);
  ROS_INFO_ONCE("Attidude controller got data from position controller.");
  att_control.CalculateMomentThrust(thrust_pd_eig,ang_acc);
  if (!got_thrust_moment){
    got_thrust_moment =1;
  }
  }

  void timmerCallback(const ros::TimerEvent&)
  {
    if (!gPublish){
      gPublish = true;
    }
  }

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "attitude_control_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ROS_INFO("attitude_control_node main started");

  ros::Timer timer = nh.createTimer(ros::Duration(0.01),att_control::timmerCallback);

  //mav_msgs::default_topics::COMMAND_ACTUATORS
  ros::Publisher motor_velocity_reference_pub_;
  motor_velocity_reference_pub_ = nh.advertise<mav_msgs::Actuators>(mav_msgs::default_topics::COMMAND_ACTUATORS, 1);
  ros::Publisher uav_state_pub_;
  uav_state_pub_ = nh.advertise<attitude_control::uav_state>("/uav_state", 1);

  dynamic_reconfigure::Server<attitude_control::controllerDynAttConfig> server;
  dynamic_reconfigure::Server<attitude_control::controllerDynAttConfig>::CallbackType f;
  f = boost::bind(&att_control::controller_dyn_callback, _1, _2);
  server.setCallback(f);


  ros::Subscriber commanded_thrust_sub;
  commanded_thrust_sub = nh.subscribe("/accel_ang_cmd", 1,att_control::ThrustCallback);
  ros::Subscriber odometry_sub_;
  odometry_sub_ = nh.subscribe(mav_msgs::default_topics::ODOMETRY, 1, att_control::OdometryCallback);
  ros::Subscriber traj_sub_;
  traj_sub_ = nh.subscribe("/ref_trajectory", 1,
      att_control::TrajectoryCallback);
  /*traj_sub_ = nh.subscribe("/ref_trajectory", 1,
      attitude_control::TrajectoryCallback);*/
  // int run_freq = 1000;
  // pnh.getParam("run_frequency",run_freq);
  // ROS_INFO("Param: run frequency = %d",run_freq);
  //ros::Rate r(run_freq);

  int run_freq = 1000;
  ros::Rate r(run_freq);
  Eigen::VectorXd ref_rotor_velocities(6);
  Eigen::VectorXd ref_rotor_velocities_normalized(6);
  Eigen::Vector3d moment_;
  double thrust_logging;
  att_control::EigenOdometry gOdometry;
  Eigen::Matrix3d R_W_B;
  Eigen::Vector3d velocity_W;
  double gPsi;
  double phi;
  double theta;


  while(ros::ok()) {
      if (att_control::got_odom && att_control::got_thrust_moment){
        if (att_control::emergency_mode){
          ref_rotor_velocities[0] = 0;
          ref_rotor_velocities[1] = 0;
          ref_rotor_velocities[2] = 0;
          ref_rotor_velocities[3] = 0;
          ref_rotor_velocities[4] = 0;
          ref_rotor_velocities[5] = 0;
        } else {
          att_control::att_control.SetMotorSpeed(ref_rotor_velocities);
        }


        mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

        actuator_msg->angular_velocities.clear();
        actuator_msg->normalized.clear();
      for (int i = 0; i < 6; i++){
        actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
        ref_rotor_velocities_normalized[i] = ((ref_rotor_velocities[i]*9.549296596425384)-1250.0)/43.75; // Firefly
        if (ref_rotor_velocities_normalized[i]<0.0){
          ref_rotor_velocities_normalized[i] = 0.0;
        } else if (ref_rotor_velocities_normalized[i]>200.0){
          ref_rotor_velocities_normalized[i] = 200.0;
        }
        actuator_msg->normalized.push_back(ref_rotor_velocities_normalized[i]);

      }







      actuator_msg->header.stamp = ros::Time::now();

      motor_velocity_reference_pub_.publish(actuator_msg);

    }

    if (att_control::gPublish){
      attitude_control::uav_statePtr  uav_state_msg(new attitude_control::uav_state);
      att_control::att_control.GetThrustMoment(moment_,&thrust_logging);

      att_control::att_control.GetOdometry(gOdometry);

      R_W_B = gOdometry.orientation.toRotationMatrix();
      velocity_W =  R_W_B * gOdometry.velocity;

      gPsi = atan2(R_W_B(1,0),R_W_B(0,0)); // For "+" configuration
      phi  = atan2(R_W_B(2,1),R_W_B(2,2));
      theta = asin(-R_W_B(2,0));

      uav_state_msg->position_W.x  = gOdometry.position.x();
      uav_state_msg->position_W.y  = gOdometry.position.y();
      uav_state_msg->position_W.z  = gOdometry.position.z();
      uav_state_msg->velocity_W.x  = velocity_W.x();
      uav_state_msg->velocity_W.y  = velocity_W.y();
      uav_state_msg->velocity_W.z  = velocity_W.z();
      uav_state_msg->euler_angle.x = phi;
      uav_state_msg->euler_angle.y = theta;
      uav_state_msg->euler_angle.z = gPsi;
      uav_state_msg->rotation_speed_B.x  = gOdometry.angular_velocity.x();
      uav_state_msg->rotation_speed_B.y  = gOdometry.angular_velocity.y();
      uav_state_msg->rotation_speed_B.z  = gOdometry.angular_velocity.z();
      uav_state_msg->commanded_thrust = thrust_logging;
      uav_state_msg->moment.x = moment_.x();
      uav_state_msg->moment.y = moment_.y();
      uav_state_msg->moment.z = moment_.z();

      uav_state_msg->position_ref.x = att_control::pos_ref_pub.x();
      uav_state_msg->position_ref.y = att_control::pos_ref_pub.y();
      uav_state_msg->position_ref.z = att_control::pos_ref_pub.z();
      uav_state_msg->velocity_ref.x = att_control::vel_ref_pub.x();
      uav_state_msg->velocity_ref.y = att_control::vel_ref_pub.y();
      uav_state_msg->velocity_ref.z = att_control::vel_ref_pub.z();
      uav_state_msg->accel_ref.x = att_control::acc_ref_pub.x();
      uav_state_msg->accel_ref.y = att_control::acc_ref_pub.y();
      uav_state_msg->accel_ref.z = att_control::acc_ref_pub.z();
      uav_state_msg->yaw_ref = att_control::yaw_ref_pub;
      uav_state_msg->launch_flag = att_control::launch_flag_pub;
      uav_state_msg->speed = velocity_W.norm();



      uav_state_msg->header.stamp  =  ros::Time::now();
      uav_state_pub_.publish(uav_state_msg);
      att_control::gPublish = false;
    }

    ros::spinOnce();
    r.sleep();
  } // end while

  return 0;
}
