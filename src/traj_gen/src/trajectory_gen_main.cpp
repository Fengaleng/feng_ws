
#include <ros/ros.h>

#include <mav_msgs/default_topics.h>
#include <nav_msgs/Odometry.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_msgs/conversions.h>

#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include "parameters.h"
#include "parameters_ros.h"
#include "common.h"

#include <traj_gen/min_snap_traj.h>

#include <dynamic_reconfigure/server.h>
#include <traj_gen/controllerDynConfig.h>

Eigen::VectorXd gY0(4);        // initial position (equilibrium)
Eigen::VectorXd gRef(4);
double gPsi;
bool gInit_flag = false;
//gInit_flag = false;
bool gLanding_flag = false;
bool gOdom_flag = false;
int gTest_mode = 0;
//gLanding_flag = false;
mav_msgs::EigenOdometry gOdometry;
Eigen::Matrix3d R_W_B;

void controller_dyn_callback(traj_gen::controllerDynConfig &config, uint32_t level) {
  /*if (config.RESET) {
    // TO DO : reset parameters, gains
    config.RESET = false;
  }*/
  if (level & traj_gen::controllerDyn_ENABLE_CTRL){
      if (config.enable_take_off && !gInit_flag){     // only once
        gY0[0]    = gOdometry.position_W.x();
        gY0[1]    = gOdometry.position_W.y();
        gY0[2]    = gOdometry.position_W.z();
        gY0[3]    = gPsi;
      //  if (config.test_mode == pelican::controllerDyn_TEST_MANUAL){
        gRef[0]   = gY0[0];
        gRef[1]   = gY0[1];
        gRef[2]   = gY0[2];
        gRef[3]   = gY0[3];
      //  }

        /*gGain[0]  = config.kxy;      // x
        gGain[1]  = config.kz;     // vx
        gGain[2]  = config.kxydot;     // integral x
        gGain[3]  = config.kzdot;
        gGain[4]  = config.kthetaphi;
        gGain[5]  = config.kpsi;
        gGain[6]  = config.kpq;
        gGain[7]  = config.kr;*/
        gTest_mode = config.test_mode;
        gInit_flag = true;
        ROS_INFO("Take-off Request: %s ",config.enable_take_off?"True":"False");

        config.enable_take_off = false;

      }
      else if (config.enable_landing && !gLanding_flag){   // only once
        gRef[0]  = gOdometry.position_W.x();
        gRef[1]  = gOdometry.position_W.y();
        //gRef[2]  = 0.0; //Modification to be done here!
        gRef[2]  = gOdometry.position_W.z(); //New try
        R_W_B = gOdometry.orientation_W_B.toRotationMatrix();
        gRef[3]  = atan2(R_W_B(1,0),R_W_B(0,0));
        //gTest_mode = traj_gen::controllerDyn_TEST_MANUAL;
        gLanding_flag = true;
        gInit_flag = false;
        ROS_INFO("Landing Request: %s at x_ref = %f, y_ref = %f,psi_ref = %f(deg)",config.enable_landing?"True":"False",gRef[0],gRef[1],gRef[3]*180.0/3.14159);
        config.enable_landing  = false;
      }
      /*else if (gTest_mode == pelican::controllerDyn_TEST_MANUAL){
        if (config.send_waypoint){
          gRef[0]   = config.ref_x;
          gRef[1]   = config.ref_y;
          gRef[2]   = config.ref_z;
          gRef[3]   = config.ref_yaw_deg*3.14159/180.0;
          ROS_INFO("Waypoint Request: x_ref = %f, y_ref = %f, z_ref = %f, psi_ref = %f(deg)",gRef[0],gRef[1],gRef[2],gRef[3]);
          config.send_waypoint   = false;
        }
      }*/
  }
}

double horner_poly(const Eigen::VectorXd& coefficients,double time) {
  int n = 9;
  double eval = coefficients[n];
  for (int j = n; j > 0;j=j-1) {
    eval = time*eval + coefficients[j-1];
  }

  return eval;
}

double horner_poly_dot(const Eigen::VectorXd& coefficients,double time) {
  int n = 9;
  double eval = n*coefficients[n];
  for (int j = n; j > 1;j=j-1) {
    eval = time*eval + (j-1)*coefficients[j-1];
  }
  return eval;
}

double horner_poly_dot_dot(const Eigen::VectorXd& coefficients,double time) {
  int n = 9;
  double eval = n*(n-1)*coefficients[n];
  for (int j = n; j > 2;j=j-1) {
    eval = time*eval + (j-1)*(j-2)*coefficients[j-1];
  }
  return eval;
}


void OdometryCallback(const nav_msgs::Odometry::ConstPtr &odom) {
ROS_INFO_ONCE("traj_gen got first odometry message.");
/*  ros::Time current_time = ros::Time::now();
  ros::Duration Td = current_time - gPrev_it;
  gPrev_it = current_time;

  Eigen::VectorXd prev_ang_velocity(3);
  prev_ang_velocity[0] = gOdometry.angular_velocity_B.x();
  prev_ang_velocity[1] = gOdometry.angular_velocity_B.y();
  prev_ang_velocity[2] = gOdometry.angular_velocity_B.z(); */

  mav_msgs::eigenOdometryFromMsg(*odom, &gOdometry);   // new measurement
  if (!gOdom_flag){
    gOdom_flag = true;
  }
}



int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_gen_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ROS_INFO("trajectory_gen_node main started");
  int degree = 9;
  Eigen::VectorXd coeffs(degree);

  ros::Subscriber odometry_sub_;
  //mav_msgs::default_topics::ODOMETRY
  odometry_sub_ = nh.subscribe("odometry", 1, OdometryCallback);


  ros::Publisher uav_state_pub_;
  uav_state_pub_ = nh.advertise<traj_gen::min_snap_traj>("/ref_trajectory", 1);


  dynamic_reconfigure::Server<traj_gen::controllerDynConfig> server;
  dynamic_reconfigure::Server<traj_gen::controllerDynConfig>::CallbackType f;
  f = boost::bind(&controller_dyn_callback, _1, _2);
  server.setCallback(f);


  gRef.x()=0;
  gRef.y()=0;
  gRef.z()=0;
  gRef.w()=0;

  // int run_freq = 1000;
  // pnh.getParam("run_frequency",run_freq);
  // ROS_INFO("Param: run frequency = %d",run_freq);
  //ros::Rate r(run_freq);

  int run_freq = 200;
  double step = 1.0/double(run_freq);
  ros::Rate r(run_freq);



  Eigen::VectorXd coeffs_x(70);
  Eigen::VectorXd coeffs_y(70);
  Eigen::VectorXd coeffs_z(70);
  double num_coeffs = 10;
  double num_segments = 7;
  double yaw_x_vector = 0;
  double yaw_y_vector = 0;
  Eigen::VectorXd time_segments_out(7);
  Eigen::VectorXd pose_targets_x(7);
  Eigen::VectorXd pose_targets_y(7);
  pose_targets_x(0) = 10;
  pose_targets_x(1) = 0;
  pose_targets_x(2) = 0;
  pose_targets_x(3) = -10;
  pose_targets_x(4) = -10;
  pose_targets_x(5) = 10;
  pose_targets_x(6) = 10;
  pose_targets_y(0) = 10;
  pose_targets_y(1) = 20;
  pose_targets_y(2) = 20;
  pose_targets_y(3) = 10;
  pose_targets_y(4) = 10;
  pose_targets_y(5) = 0;
  pose_targets_y(6) = 0;
  time_segments_out(0) = 5.172278e+00;
  time_segments_out(1) = 1.320987e-01;
  time_segments_out(2) = 2.429993e+00;
  time_segments_out(3) = 1.200205e-01;
  time_segments_out(4) = 2.428751e+00;
  time_segments_out(5) = 1.327614e-01;
  time_segments_out(6) = 5.166647e+00;
  coeffs_x(0) = 0;
  coeffs_x(1) = 0;
  coeffs_x(2) = 0;
  coeffs_x(3) = 0;
  coeffs_x(4) = 0;
  coeffs_x(5) = 1.064642e-01;
  coeffs_x(6) = -5.650119e-02;
  coeffs_x(7) = 1.196049e-02;
  coeffs_x(8) = -1.193124e-03;
  coeffs_x(9) = 4.694554e-05;
  coeffs_x(10) = 1.000000e+01;
  coeffs_x(11) = 2.323126e-01;
  coeffs_x(12) = -1.745676e+00;
  coeffs_x(13) = -1.096439e-01;
  coeffs_x(14) = 1.272738e-01;
  coeffs_x(15) = -9.200401e-01;
  coeffs_x(16) = 9.417862e+00;
  coeffs_x(17) = -5.296106e+01;
  coeffs_x(18) = 1.550190e+02;
  coeffs_x(19) = -1.849076e+02;
  coeffs_x(20) = 1.000000e+01;
  coeffs_x(21) = -2.338385e-01;
  coeffs_x(22) = -1.780339e+00;
  coeffs_x(23) = -6.653342e-02;
  coeffs_x(24) = 7.896456e-02;
  coeffs_x(25) = -4.836957e-03;
  coeffs_x(26) = 8.440157e-04;
  coeffs_x(27) = -2.678054e-04;
  coeffs_x(28) = 2.818946e-05;
  coeffs_x(29) = -2.358144e-06;
  coeffs_x(30) = 3.750000e-01;
  coeffs_x(31) = -6.245824e+00;
  coeffs_x(32) = -7.679472e-02;
  coeffs_x(33) = 4.237458e-01;
  coeffs_x(34) = 4.086674e-03;
  coeffs_x(35) = -1.235276e-02;
  coeffs_x(36) = -1.116120e-02;
  coeffs_x(37) = 6.230960e-02;
  coeffs_x(38) = -2.231394e-01;
  coeffs_x(39) = 4.118424e-01;
  coeffs_x(40) = -3.750000e-01;
  coeffs_x(41) = -6.245931e+00;
  coeffs_x(42) = 7.590398e-02;
  coeffs_x(43) = 4.237872e-01;
  coeffs_x(44) = -3.916258e-03;
  coeffs_x(45) = -1.344534e-02;
  coeffs_x(46) = 1.880648e-03;
  coeffs_x(47) = -2.177205e-04;
  coeffs_x(48) = 2.257053e-05;
  coeffs_x(49) = -2.277266e-06;
  coeffs_x(50) = -1.000000e+01;
  coeffs_x(51) = -2.353472e-01;
  coeffs_x(52) = 1.782873e+00;
  coeffs_x(53) = -6.610434e-02;
  coeffs_x(54) = -7.923550e-02;
  coeffs_x(55) = 3.697176e-02;
  coeffs_x(56) = 5.045694e-02;
  coeffs_x(57) = -9.727764e+00;
  coeffs_x(58) = 8.481561e+01;
  coeffs_x(59) = -2.182604e+02;
  coeffs_x(60) = -1.000000e+01;
  coeffs_x(61) = 2.338102e-01;
  coeffs_x(62) = 1.748132e+00;
  coeffs_x(63) = -1.095696e-01;
  coeffs_x(64) = -1.278704e-01;
  coeffs_x(65) = 6.095884e-02;
  coeffs_x(66) = -2.866322e-02;
  coeffs_x(67) = 7.867095e-03;
  coeffs_x(68) = -1.001308e-03;
  coeffs_x(69) = 4.742574e-05;
  coeffs_y(0) = 0;
  coeffs_y(1) = 0;
  coeffs_y(2) = 0;
  coeffs_y(3) = 0;
  coeffs_y(4) = 0;
  coeffs_y(5) = 6.319742e-03;
  coeffs_y(6) = 6.077865e-04;
  coeffs_y(7) = -5.306050e-04;
  coeffs_y(8) = 6.652439e-05;
  coeffs_y(9) = -2.617517e-06;
  coeffs_y(10) = 9.625000e+00;
  coeffs_y(11) = 5.636810e+00;
  coeffs_y(12) = 3.540271e-01;
  coeffs_y(13) = -3.379352e-01;
  coeffs_y(14) = -5.047311e-02;
  coeffs_y(15) = 6.120676e-02;
  coeffs_y(16) = -5.227194e-01;
  coeffs_y(17) = 2.951431e+00;
  coeffs_y(18) = -8.643460e+00;
  coeffs_y(19) = 1.031004e+01;
  coeffs_y(20) = 1.037500e+01;
  coeffs_y(21) = 5.712224e+00;
  coeffs_y(22) = 2.153111e-01;
  coeffs_y(23) = -3.614359e-01;
  coeffs_y(24) = -4.073230e-02;
  coeffs_y(25) = 1.151437e-02;
  coeffs_y(26) = 9.335640e-04;
  coeffs_y(27) = -1.737336e-04;
  coeffs_y(28) = -1.574656e-06;
  coeffs_y(29) = 1.317760e-07;
  coeffs_y(30) = 2.000000e+01;
  coeffs_y(31) = 2.448331e-01;
  coeffs_y(32) = -2.038050e+00;
  coeffs_y(33) = -2.674424e-02;
  coeffs_y(34) = 9.216869e-02;
  coeffs_y(35) = 2.824411e-03;
  coeffs_y(36) = -1.462195e-03;
  coeffs_y(37) = -5.320546e-03;
  coeffs_y(38) = 8.095640e-03;
  coeffs_y(39) = -3.346346e-03;
  coeffs_y(40) = 2.000000e+01;
  coeffs_y(41) = -2.448981e-01;
  coeffs_y(42) = -2.039671e+00;
  coeffs_y(43) = 1.783239e-02;
  coeffs_y(44) = 9.333282e-02;
  coeffs_y(45) = 8.157991e-04;
  coeffs_y(46) = -3.604376e-03;
  coeffs_y(47) = 3.383167e-04;
  coeffs_y(48) = 6.408422e-07;
  coeffs_y(49) = -6.459378e-08;
  coeffs_y(50) = 1.037500e+01;
  coeffs_y(51) = -5.686178e+00;
  coeffs_y(52) = 2.321339e-01;
  coeffs_y(53) = 3.538447e-01;
  coeffs_y(54) = -4.516657e-02;
  coeffs_y(55) = -8.386918e-03;
  coeffs_y(56) = 3.610880e-03;
  coeffs_y(57) = -2.745359e-01;
  coeffs_y(58) = 2.393267e+00;
  coeffs_y(59) = -6.159140e+00;
  coeffs_y(60) = 9.625000e+00;
  coeffs_y(61) = -5.606268e+00;
  coeffs_y(62) = 3.680767e-01;
  coeffs_y(63) = 3.282596e-01;
  coeffs_y(64) = -5.222635e-02;
  coeffs_y(65) = -5.996012e-03;
  coeffs_y(66) = 1.296821e-03;
  coeffs_y(67) = 9.872397e-05;
  coeffs_y(68) = -2.825954e-05;
  coeffs_y(69) = 1.338479e-06;
  coeffs_z(0) = 0;
  coeffs_z(1) = 0;
  coeffs_z(2) = 0;
  coeffs_z(3) = 0;
  coeffs_z(4) = 0;
  coeffs_z(5) = -2.494248e-23;
  coeffs_z(6) = 1.863704e-23;
  coeffs_z(7) = -5.244561e-24;
  coeffs_z(8) = 6.600910e-25;
  coeffs_z(9) = -3.141863e-26;
  coeffs_z(10) = -2.059365e-15;
  coeffs_z(11) = 3.122795e-16;
  coeffs_z(12) = -2.295149e-17;
  coeffs_z(13) = -3.683005e-18;
  coeffs_z(14) = -1.509110e-22;
  coeffs_z(15) = 1.421926e-19;
  coeffs_z(16) = -3.166226e-18;
  coeffs_z(17) = 2.945406e-17;
  coeffs_z(18) = -1.281848e-16;
  coeffs_z(19) = 2.153357e-16;
  coeffs_z(20) = -1.015445e-25;
  coeffs_z(21) = -3.663061e-23;
  coeffs_z(22) = -2.028435e-22;
  coeffs_z(23) = -1.193095e-22;
  coeffs_z(24) = 4.304378e-22;
  coeffs_z(25) = -5.146933e-22;
  coeffs_z(26) = 3.721318e-22;
  coeffs_z(27) = -1.509602e-22;
  coeffs_z(28) = 3.135366e-23;
  coeffs_z(29) = -2.615635e-24;
  coeffs_z(30) = -5.760996e-15;
  coeffs_z(31) = 1.623163e-18;
  coeffs_z(32) = -1.795265e-17;
  coeffs_z(33) = 1.687164e-18;
  coeffs_z(34) = -7.373510e-23;
  coeffs_z(35) = -1.559804e-21;
  coeffs_z(36) = 5.401925e-20;
  coeffs_z(37) = -6.011333e-19;
  coeffs_z(38) = 2.994302e-18;
  coeffs_z(39) = -5.667720e-18;
  coeffs_z(40) = 7.602833e-24;
  coeffs_z(41) = 4.698765e-22;
  coeffs_z(42) = -3.061519e-23;
  coeffs_z(43) = -2.280913e-22;
  coeffs_z(44) = -5.296421e-24;
  coeffs_z(45) = 1.039275e-22;
  coeffs_z(46) = -6.432157e-23;
  coeffs_z(47) = 2.404959e-23;
  coeffs_z(48) = -5.527576e-24;
  coeffs_z(49) = 5.492159e-25;
  coeffs_z(50) = -4.183335e-15;
  coeffs_z(51) = -3.046846e-16;
  coeffs_z(52) = -1.142120e-17;
  coeffs_z(53) = -1.041855e-18;
  coeffs_z(54) = 9.390309e-24;
  coeffs_z(55) = -2.546547e-20;
  coeffs_z(56) = 5.558669e-19;
  coeffs_z(57) = -5.114014e-18;
  coeffs_z(58) = 2.206197e-17;
  coeffs_z(59) = -3.676684e-17;
  coeffs_z(60) = -2.692728e-24;
  coeffs_z(61) = -2.582644e-23;
  coeffs_z(62) = 1.875396e-22;
  coeffs_z(63) = -1.360469e-23;
  coeffs_z(64) = -7.074965e-23;
  coeffs_z(65) = 3.114043e-23;
  coeffs_z(66) = -4.440617e-24;
  coeffs_z(67) = -3.566801e-26;
  coeffs_z(68) = 6.144173e-26;
  coeffs_z(69) = -3.984157e-27;








//  gPrev_it = ros::Time::now();
  double time_traj = -step;
  double time_poly = -step;
  double time_poly2 = 0;
  double time_poly3;
  double time_landing = -step;
  double rate = 1;
  double dephase = 0;
  double amp =1;
  double first =1;
  double condition = 0;
  // int test_case = 0; // step_z,step_psi,step_x,step_y,circle,figure 8,trajectory





  while(ros::ok()) {

      traj_gen::min_snap_trajPtr uav_state_msg(new traj_gen::min_snap_traj);

      //gInit_flag

      //gLanding_flag


        uav_state_msg->position_ref.x  = gRef[0];
        uav_state_msg->position_ref.y  = gRef[1];
        uav_state_msg->position_ref.z  = gRef[2];
        uav_state_msg->velocity_ref.x  = 0;
        uav_state_msg->velocity_ref.y  = 0;
        uav_state_msg->velocity_ref.z  = 0;
        uav_state_msg->accel_ref.x  = 0;
        uav_state_msg->accel_ref.y  = 0;
        uav_state_msg->accel_ref.z  = 0;
        uav_state_msg->yaw_ref[0]  = gRef[3];
        uav_state_msg->yaw_ref[1]  = 0;
        uav_state_msg->yaw_ref[2]  = 0;
        uav_state_msg->launch_flag = ((gOdom_flag&&gInit_flag)||(gOdom_flag&&gLanding_flag));


        if (gLanding_flag){
          time_landing = time_landing+step;
          if (time_landing > 3 && time_landing <= 6){
            // 3 second landing
            uav_state_msg->position_ref.z = (gY0[2]-gRef[2])/(3)*(time_landing-3)+gRef[2];
          }
          else if (time_landing > 6 && time_landing < 7.5){
            uav_state_msg->position_ref.z = gY0[2];
          } else if (time_landing > 7.5){
            uav_state_msg->position_ref.z = gY0[2];
            gLanding_flag = false;
            gInit_flag = false;
            gRef[2] = gY0[2];
          }

        }
        else if (gInit_flag){
          time_traj = time_traj+step;
          switch (gTest_mode) {
            case 0 :
              uav_state_msg->position_ref.x  = gRef[0];
              uav_state_msg->position_ref.y  = gRef[1];
              uav_state_msg->position_ref.z  = 0.75;
              uav_state_msg->velocity_ref.x  = 0;
              uav_state_msg->velocity_ref.y  = 0;
              uav_state_msg->velocity_ref.z  = 0;
              uav_state_msg->accel_ref.x  = 0;
              uav_state_msg->accel_ref.y  = 0;
              uav_state_msg->accel_ref.z  = 0;
              uav_state_msg->yaw_ref[0]  = gRef[3];
              uav_state_msg->yaw_ref[1]  = 0;
              uav_state_msg->yaw_ref[2]  = 0;
              break;
            case 1 :
              if (time_traj < 3){
              // Publish data: UAV state in World frame
                rate = 1;
                uav_state_msg->position_ref.x  = 0;
                uav_state_msg->position_ref.y  = 0;
                uav_state_msg->position_ref.z  = 0.75;
                uav_state_msg->velocity_ref.x  = 0;
                uav_state_msg->velocity_ref.y  = 0;
                uav_state_msg->velocity_ref.z  = 0;
                uav_state_msg->accel_ref.x  = 0;
                uav_state_msg->accel_ref.y  = 0;
                uav_state_msg->accel_ref.z  = 0;
                uav_state_msg->yaw_ref[0]  = 0;
                uav_state_msg->yaw_ref[1]  = 0;
                uav_state_msg->yaw_ref[2]  = 0;
              } else if (time_traj < 6) {
                uav_state_msg->position_ref.x  = 0;
                uav_state_msg->position_ref.y  = 0;
                uav_state_msg->position_ref.z  = 0.75;
                uav_state_msg->velocity_ref.x  = 0;
                uav_state_msg->velocity_ref.y  = 0;
                uav_state_msg->velocity_ref.z  = 0;
                uav_state_msg->accel_ref.x  = 0;
                uav_state_msg->accel_ref.y  = 0;
                uav_state_msg->accel_ref.z  = 0;
                uav_state_msg->yaw_ref[0]  = 0.78539816339;
                uav_state_msg->yaw_ref[1]  = 0;
                uav_state_msg->yaw_ref[2]  = 0;
              } else {
                uav_state_msg->position_ref.x  = 0;
                uav_state_msg->position_ref.y  = 0;
                uav_state_msg->position_ref.z  = gRef[2];
                uav_state_msg->velocity_ref.x  = 0;
                uav_state_msg->velocity_ref.y  = 0;
                uav_state_msg->velocity_ref.z  = 0;
                uav_state_msg->accel_ref.x  = 0;
                uav_state_msg->accel_ref.y  = 0;
                uav_state_msg->accel_ref.z  = 0;
                uav_state_msg->yaw_ref[0]  = 0.78539816339;
                uav_state_msg->yaw_ref[1]  = 0;
                uav_state_msg->yaw_ref[2]  = 0;
              }
              break;
            case 2 :
              if (time_traj < 3){
              // Publish data: UAV state in World frame
                rate = 1;
                uav_state_msg->position_ref.x  = 0;
                uav_state_msg->position_ref.y  = 0;
                uav_state_msg->position_ref.z  = 0.75;
                uav_state_msg->velocity_ref.x  = 0;
                uav_state_msg->velocity_ref.y  = 0;
                uav_state_msg->velocity_ref.z  = 0;
                uav_state_msg->accel_ref.x  = 0;
                uav_state_msg->accel_ref.y  = 0;
                uav_state_msg->accel_ref.z  = 0;
                uav_state_msg->yaw_ref[0]  = 0;
                uav_state_msg->yaw_ref[1]  = 0;
                uav_state_msg->yaw_ref[2]  = 0;
              } else if (time_traj < 6) {
                uav_state_msg->position_ref.x  = 1.0;
                uav_state_msg->position_ref.y  = 0;
                uav_state_msg->position_ref.z  = 0.75;
                uav_state_msg->velocity_ref.x  = 0;
                uav_state_msg->velocity_ref.y  = 0;
                uav_state_msg->velocity_ref.z  = 0;
                uav_state_msg->accel_ref.x  = 0;
                uav_state_msg->accel_ref.y  = 0;
                uav_state_msg->accel_ref.z  = 0;
                uav_state_msg->yaw_ref[0]  = 0;
                uav_state_msg->yaw_ref[1]  = 0;
                uav_state_msg->yaw_ref[2]  = 0;
              }
              else if (time_traj < 9){
                uav_state_msg->position_ref.x  = 1.0;
                uav_state_msg->position_ref.y  = 0;
                uav_state_msg->position_ref.z  = 0.75;
                uav_state_msg->velocity_ref.x  = 0;
                uav_state_msg->velocity_ref.y  = 0;
                uav_state_msg->velocity_ref.z  = 0;
                uav_state_msg->accel_ref.x  = 0;
                uav_state_msg->accel_ref.y  = 0;
                uav_state_msg->accel_ref.z  = 0;
                uav_state_msg->yaw_ref[0]  = 0;
                uav_state_msg->yaw_ref[1]  = 0;
                uav_state_msg->yaw_ref[2]  = 0;
              } else {
                uav_state_msg->position_ref.x  = 1.0;
                uav_state_msg->position_ref.y  = 0;
                uav_state_msg->position_ref.z  = gRef[2];
                uav_state_msg->velocity_ref.x  = 0;
                uav_state_msg->velocity_ref.y  = 0;
                uav_state_msg->velocity_ref.z  = 0;
                uav_state_msg->accel_ref.x  = 0;
                uav_state_msg->accel_ref.y  = 0;
                uav_state_msg->accel_ref.z  = 0;
                uav_state_msg->yaw_ref[0]  = 0;
                uav_state_msg->yaw_ref[1]  = 0;
                uav_state_msg->yaw_ref[2]  = 0;
              }
              break;
            case 3 :
              if (time_traj < 3){
              // Publish data: UAV state in World frame
                rate = 1;
                uav_state_msg->position_ref.x  = 0;
                uav_state_msg->position_ref.y  = 0;
                uav_state_msg->position_ref.z  = 0.75;
                uav_state_msg->velocity_ref.x  = 0;
                uav_state_msg->velocity_ref.y  = 0;
                uav_state_msg->velocity_ref.z  = 0;
                uav_state_msg->accel_ref.x  = 0;
                uav_state_msg->accel_ref.y  = 0;
                uav_state_msg->accel_ref.z  = 0;
                uav_state_msg->yaw_ref[0]  = 0;
                uav_state_msg->yaw_ref[1]  = 0;
                uav_state_msg->yaw_ref[2]  = 0;
              } else if (time_traj < 6) {
                uav_state_msg->position_ref.x  = 0;
                uav_state_msg->position_ref.y  = 1.0;
                uav_state_msg->position_ref.z  = 0.75;
                uav_state_msg->velocity_ref.x  = 0;
                uav_state_msg->velocity_ref.y  = 0;
                uav_state_msg->velocity_ref.z  = 0;
                uav_state_msg->accel_ref.x  = 0;
                uav_state_msg->accel_ref.y  = 0;
                uav_state_msg->accel_ref.z  = 0;
                uav_state_msg->yaw_ref[0]  = 0;
                uav_state_msg->yaw_ref[1]  = 0;
                uav_state_msg->yaw_ref[2]  = 0;
              }
              else if (time_traj < 9){
                uav_state_msg->position_ref.x  = 0;
                uav_state_msg->position_ref.y  = 1.0;
                uav_state_msg->position_ref.z  = 0.75;
                uav_state_msg->velocity_ref.x  = 0;
                uav_state_msg->velocity_ref.y  = 0;
                uav_state_msg->velocity_ref.z  = 0;
                uav_state_msg->accel_ref.x  = 0;
                uav_state_msg->accel_ref.y  = 0;
                uav_state_msg->accel_ref.z  = 0;
                uav_state_msg->yaw_ref[0]  = 0;
                uav_state_msg->yaw_ref[1]  = 0;
                uav_state_msg->yaw_ref[2]  = 0;
              } else {
                uav_state_msg->position_ref.x  = 0;
                uav_state_msg->position_ref.y  = 1.0;
                uav_state_msg->position_ref.z  = gRef[2];
                uav_state_msg->velocity_ref.x  = 0;
                uav_state_msg->velocity_ref.y  = 0;
                uav_state_msg->velocity_ref.z  = 0;
                uav_state_msg->accel_ref.x  = 0;
                uav_state_msg->accel_ref.y  = 0;
                uav_state_msg->accel_ref.z  = 0;
                uav_state_msg->yaw_ref[0]  = 0.0;
                uav_state_msg->yaw_ref[1]  = 0;
                uav_state_msg->yaw_ref[2]  = 0;
              }
              break;
            case 4 :
              if (time_traj < 3){
              // Publish data: UAV state in World frame
                rate = 1;
                uav_state_msg->position_ref.x  = 0;
                uav_state_msg->position_ref.y  = 0;
                uav_state_msg->position_ref.z  = 0.75;
                uav_state_msg->velocity_ref.x  = 0;
                uav_state_msg->velocity_ref.y  = 0;
                uav_state_msg->velocity_ref.z  = 0;
                uav_state_msg->accel_ref.x  = 0;
                uav_state_msg->accel_ref.y  = 0;
                uav_state_msg->accel_ref.z  = 0;
                uav_state_msg->yaw_ref[0]  = 0;
                uav_state_msg->yaw_ref[1]  = 0;
                uav_state_msg->yaw_ref[2]  = 0;
              } else if (time_traj < 6) {
                uav_state_msg->position_ref.x  = 0;
                uav_state_msg->position_ref.y  = 0;
                uav_state_msg->position_ref.z  = 0.75;
                uav_state_msg->velocity_ref.x  = 0;
                uav_state_msg->velocity_ref.y  = 0;
                uav_state_msg->velocity_ref.z  = 0;
                uav_state_msg->accel_ref.x  = 0;
                uav_state_msg->accel_ref.y  = 0;
                uav_state_msg->accel_ref.z  = 0;
                uav_state_msg->yaw_ref[0]  = 0;
                uav_state_msg->yaw_ref[1]  = 0;
                uav_state_msg->yaw_ref[2]  = 0;
              }
              else if (time_traj < 9){
                uav_state_msg->position_ref.x  = 1.0;
                uav_state_msg->position_ref.y  = 0;
                uav_state_msg->position_ref.z  = 0.75;
                uav_state_msg->velocity_ref.x  = 0;
                uav_state_msg->velocity_ref.y  = 0;
                uav_state_msg->velocity_ref.z  = 0;
                uav_state_msg->accel_ref.x  = 0;
                uav_state_msg->accel_ref.y  = 0;
                uav_state_msg->accel_ref.z  = 0;
                uav_state_msg->yaw_ref[0]  = 0;
                uav_state_msg->yaw_ref[1]  = 0;
                uav_state_msg->yaw_ref[2]  = 0;
              } else if (time_traj < 31.5) {
                time_poly = time_poly+step;
                uav_state_msg->position_ref.x  = cos(0.75*time_poly);
                uav_state_msg->position_ref.y  = sin(0.75*time_poly);
                uav_state_msg->position_ref.z  = 0.75;
                uav_state_msg->velocity_ref.x  = -0.75*sin(0.75*time_poly);
                uav_state_msg->velocity_ref.y  = 0.75*cos(0.75*time_poly);
                uav_state_msg->velocity_ref.z  = 0;
                uav_state_msg->accel_ref.x  = -0.75*0.75*cos(0.75*time_poly);
                uav_state_msg->accel_ref.y  = -0.75*0.75*sin(0.75*time_poly);
                uav_state_msg->accel_ref.z  = 0;
                uav_state_msg->yaw_ref[0]  = 0;
                /*if (uav_state_msg->velocity_ref.x == 0 && uav_state_msg->velocity_ref.y==0){
                  uav_state_msg->yaw_ref[0]  = 0.0;
                } else {
                  uav_state_msg->yaw_ref[0]  = atan2(uav_state_msg->velocity_ref.y,uav_state_msg->velocity_ref.x);
                }*/
                gRef[3] = uav_state_msg->yaw_ref[0];
              } else {
                uav_state_msg->position_ref.x  = 0;
                uav_state_msg->position_ref.y  = 0;
                uav_state_msg->position_ref.z  = 0.75;
                uav_state_msg->velocity_ref.x  = 0;
                uav_state_msg->velocity_ref.y  = 0;
                uav_state_msg->velocity_ref.z  = 0;
                uav_state_msg->accel_ref.x  = 0;
                uav_state_msg->accel_ref.y  = 0;
                uav_state_msg->accel_ref.z  = 0;
                uav_state_msg->yaw_ref[0]  = gRef[3];
                uav_state_msg->yaw_ref[1]  = 0;
                uav_state_msg->yaw_ref[2]  = 0;
              }
              break;
            case 5 :
              if (time_traj < 3){
              // Publish data: UAV state in World frame
                rate = 1;
                uav_state_msg->position_ref.x  = 0;
                uav_state_msg->position_ref.y  = 0;
                uav_state_msg->position_ref.z  = 0.75;
                uav_state_msg->velocity_ref.x  = 0;
                uav_state_msg->velocity_ref.y  = 0;
                uav_state_msg->velocity_ref.z  = 0;
                uav_state_msg->accel_ref.x  = 0;
                uav_state_msg->accel_ref.y  = 0;
                uav_state_msg->accel_ref.z  = 0;
                uav_state_msg->yaw_ref[0]  = 0;
                uav_state_msg->yaw_ref[1]  = 0;
                uav_state_msg->yaw_ref[2]  = 0;
              } else if (time_traj < 6) {
                uav_state_msg->position_ref.x  = 0;
                uav_state_msg->position_ref.y  = 0;
                uav_state_msg->position_ref.z  = 0.75;
                uav_state_msg->velocity_ref.x  = 0;
                uav_state_msg->velocity_ref.y  = 0;
                uav_state_msg->velocity_ref.z  = 0;
                uav_state_msg->accel_ref.x  = 0;
                uav_state_msg->accel_ref.y  = 0;
                uav_state_msg->accel_ref.z  = 0;
                uav_state_msg->yaw_ref[0]  = 0*1.5707;
                uav_state_msg->yaw_ref[1]  = 0;
                uav_state_msg->yaw_ref[2]  = 0;
              } else if (time_traj < 31.5) {
                time_poly = time_poly+step;
                uav_state_msg->position_ref.x  = sin(time_poly)*cos(time_poly);
                uav_state_msg->position_ref.y  = sin(time_poly);
                uav_state_msg->position_ref.z  = 0.75;
                uav_state_msg->velocity_ref.x  = pow(cos(time_poly),2)-pow(sin(time_poly),2);
                uav_state_msg->velocity_ref.y  = cos(time_poly);
                uav_state_msg->velocity_ref.z  = 0;
                uav_state_msg->accel_ref.x  = -4*sin(time_poly)*cos(time_poly);
                uav_state_msg->accel_ref.y  = -sin(time_poly);
                uav_state_msg->accel_ref.z  = 0;
                uav_state_msg->yaw_ref[0]  = 0.0;
                /*if (uav_state_msg->velocity_ref.x == 0 && uav_state_msg->velocity_ref.y==0){
                  uav_state_msg->yaw_ref[0]  = 0.0;
                } else {
                  uav_state_msg->yaw_ref[0]  = atan2(uav_state_msg->velocity_ref.y,uav_state_msg->velocity_ref.x);
                }*/
                gRef[3] = uav_state_msg->yaw_ref[0];
              } else {
                uav_state_msg->position_ref.x  = 0;
                uav_state_msg->position_ref.y  = 0;
                uav_state_msg->position_ref.z  = 0.75;
                uav_state_msg->velocity_ref.x  = 0;
                uav_state_msg->velocity_ref.y  = 0;
                uav_state_msg->velocity_ref.z  = 0;
                uav_state_msg->accel_ref.x  = 0;
                uav_state_msg->accel_ref.y  = 0;
                uav_state_msg->accel_ref.z  = 0;
                uav_state_msg->yaw_ref[0]  = gRef[3];
                uav_state_msg->yaw_ref[1]  = 0;
                uav_state_msg->yaw_ref[2]  = 0;
              }
              break;
            case 6 :
              if (time_traj < 3){
              // Publish data: UAV state in World frame
                rate = 1;
                uav_state_msg->position_ref.x  = 0;
                uav_state_msg->position_ref.y  = 0;
                uav_state_msg->position_ref.z  = 1.7;
                uav_state_msg->velocity_ref.x  = 0;
                uav_state_msg->velocity_ref.y  = 0;
                uav_state_msg->velocity_ref.z  = 0;
                uav_state_msg->accel_ref.x  = 0;
                uav_state_msg->accel_ref.y  = 0;
                uav_state_msg->accel_ref.z  = 0;
                uav_state_msg->yaw_ref[0]  = 0;
                uav_state_msg->yaw_ref[1]  = 0;
                uav_state_msg->yaw_ref[2]  = 0;
              } else if (time_traj < 3+time_segments_out.sum()) {
                  time_poly = time_poly+step;
                  for (int i = 0;i<num_segments;i++){
                    condition = condition + time_segments_out(i);
                    if(time_poly < condition){
                      time_poly2 = time_poly-condition+time_segments_out(i);

                      //coeffs_x.segment(i*num_coeffs,(i+1)*num_coeffs-1));
                      uav_state_msg->position_ref.x  = horner_poly(coeffs_x.segment(i*num_coeffs,num_coeffs),time_poly2);
                      uav_state_msg->position_ref.y  = horner_poly(coeffs_y.segment(i*num_coeffs,num_coeffs),time_poly2);
                      uav_state_msg->position_ref.z  = horner_poly(coeffs_z.segment(i*num_coeffs,num_coeffs),time_poly2)+1.7;
                      uav_state_msg->velocity_ref.x  = horner_poly_dot(coeffs_x.segment(i*num_coeffs,num_coeffs),time_poly2);
                      uav_state_msg->velocity_ref.y  = horner_poly_dot(coeffs_y.segment(i*num_coeffs,num_coeffs),time_poly2);
                      uav_state_msg->velocity_ref.z  = horner_poly_dot(coeffs_z.segment(i*num_coeffs,num_coeffs),time_poly2);
                      uav_state_msg->accel_ref.x  = horner_poly_dot_dot(coeffs_x.segment(i*num_coeffs,num_coeffs),time_poly2);
                      uav_state_msg->accel_ref.y  = horner_poly_dot_dot(coeffs_y.segment(i*num_coeffs,num_coeffs),time_poly2);
                      uav_state_msg->accel_ref.z  = horner_poly_dot_dot(coeffs_z.segment(i*num_coeffs,num_coeffs),time_poly2);

                      yaw_x_vector = pose_targets_x(i)-gOdometry.position_W.x();
                      yaw_y_vector = pose_targets_y(i)-gOdometry.position_W.y();

                      if (yaw_x_vector == 0 && yaw_y_vector ==0){
                        uav_state_msg->yaw_ref[0]  = 0;
                      } else {
                        uav_state_msg->yaw_ref[0]  = atan2(yaw_y_vector,yaw_x_vector);
                      }

                      condition = 0;
                      break;
                    }
                  }

                  gRef[3] = uav_state_msg->yaw_ref[0];
              } else {
                uav_state_msg->position_ref.x  = 0;
                uav_state_msg->position_ref.y  = 0;
                uav_state_msg->position_ref.z  = 0.75;
                uav_state_msg->velocity_ref.x  = 0;
                uav_state_msg->velocity_ref.y  = 0;
                uav_state_msg->velocity_ref.z  = 0;
                uav_state_msg->accel_ref.x  = 0;
                uav_state_msg->accel_ref.y  = 0;
                uav_state_msg->accel_ref.z  = 0;
                uav_state_msg->yaw_ref[0]  = gRef[3];
                uav_state_msg->yaw_ref[1]  = 0;
                uav_state_msg->yaw_ref[2]  = 0;
              }
              break;
        }
      }
        uav_state_msg->header.stamp  =  ros::Time::now();
        uav_state_pub_.publish(uav_state_msg);


      ros::spinOnce();
      r.sleep();

  } // end while

  return 0;
}
