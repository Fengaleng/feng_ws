
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
bool gInit_flag;
bool gLanding_flag;

void controller_dyn_callback(traj_gen::controllerDynConfig &config, uint32_t level) {
  if (config.RESET) {
    // TO DO : reset parameters, gains
    config.RESET = false;
  }
  else if (level & pelican::controllerDyn_ENABLE_CTRL){
      if (config.enable_take_off && !gInit_flag){     // only once
        //gY0[0]    = gOdometry.position_W.x();
        //gY0[1]    = gOdometry.position_W.y();
        gY0[2]    = gOdometry.position_W.z();
        /*gY0[3]    = gPsi;
      //  if (config.test_mode == pelican::controllerDyn_TEST_MANUAL){
        gRef[0]   = gY0[0];
        gRef[1]   = gY0[1];
        gRef[2]   = config.ref_z;
        gRef[3]   = gY0[3];
      //  }

        gGain[0]  = config.kxy;      // x
        gGain[1]  = config.kz;     // vx
        gGain[2]  = config.kxydot;     // integral x
        gGain[3]  = config.kzdot;
        gGain[4]  = config.kthetaphi;
        gGain[5]  = config.kpsi;
        gGain[6]  = config.kpq;
        gGain[7]  = config.kr;*/

        gInit_flag = true;
        ROS_INFO("Take-off Request: %s with Test_mode = %d",config.enable_take_off?"True":"False",gTest_mode);

        config.enable_take_off = false;

      }
      else if (config.enable_landing && !gLanding_flag){   // only once
        gRef[0]  = gOdometry.position_W.x();
        gRef[1]  = gOdometry.position_W.y();
        //gRef[2]  = 0.0; //Modification to be done here!
        gRef[2]  = gY0[2]; //New try
        gRef[3]  = gPsi;
        gTest_mode = pelican::controllerDyn_TEST_MANUAL;
        gLanding_flag = true;
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

int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_gen_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ROS_INFO("trajectory_gen_node main started");
  int degree = 9;
  Eigen::VectorXd coeffs(degree);
  ros::Publisher uav_state_pub_;
  uav_state_pub_ = nh.advertise<traj_gen::min_snap_traj>("/ref_trajectory", 1);


  dynamic_reconfigure::Server<traj_gen::controllerDynConfig> server;
  dynamic_reconfigure::Server<traj_gen::controllerDynConfig>::CallbackType f;
  f = boost::bind(&controller_dyn_callback, _1, _2);
  server.setCallback(f);



  // int run_freq = 1000;
  // pnh.getParam("run_frequency",run_freq);
  // ROS_INFO("Param: run frequency = %d",run_freq);
  //ros::Rate r(run_freq);

  int run_freq = 200;
  double step = 1.0/double(run_freq);
  ros::Rate r(run_freq);

  Eigen::VectorXd coeffs_x(110);
  Eigen::VectorXd coeffs_y(110);
  Eigen::VectorXd coeffs_z(110);

// Assignement
double num_coeffs = 10;
double num_segments = 11;
Eigen::VectorXd  time_segments_out(11);
time_segments_out(0) = 2.724384e+00;
time_segments_out(1) = 1.868734e+00;
time_segments_out(2) = 2.752092e+00;
time_segments_out(3) = 1.870548e+00;
time_segments_out(4) = 2.734889e+00;
time_segments_out(5) = 1.991038e+00;
time_segments_out(6) = 2.724546e+00;
time_segments_out(7) = 2.005243e+00;
time_segments_out(8) = 2.728739e+00;
time_segments_out(9) = 1.924660e+00;
time_segments_out(10) = 2.726661e+00;
coeffs_x(0) = 0;
coeffs_x(1) = 0;
coeffs_x(2) = 0;
coeffs_x(3) = 0;
coeffs_x(4) = 0;
coeffs_x(5) = 6.227033e-01;
coeffs_x(6) = -5.805648e-01;
coeffs_x(7) = 2.233442e-01;
coeffs_x(8) = -4.135294e-02;
coeffs_x(9) = 3.054555e-03;
coeffs_x(10) = 4.625000e+00;
coeffs_x(11) = 2.883015e+00;
coeffs_x(12) = -8.452977e-01;
coeffs_x(13) = -5.830921e-01;
coeffs_x(14) = 2.099791e-01;
coeffs_x(15) = -9.946593e-02;
coeffs_x(16) = 1.178258e-01;
coeffs_x(17) = -6.722506e-02;
coeffs_x(18) = 1.813609e-02;
coeffs_x(19) = -1.941112e-03;
coeffs_x(20) = 5.375000e+00;
coeffs_x(21) = -1.949630e+00;
coeffs_x(22) = -7.595508e-01;
coeffs_x(23) = 3.991178e-01;
coeffs_x(24) = 6.172762e-03;
coeffs_x(25) = -2.122845e-02;
coeffs_x(26) = 7.548776e-04;
coeffs_x(27) = 1.191180e-03;
coeffs_x(28) = -2.685698e-04;
coeffs_x(29) = 1.963969e-05;
coeffs_x(30) = 6.250000e-01;
coeffs_x(31) = -2.853512e-01;
coeffs_x(32) = 5.681361e-01;
coeffs_x(33) = -9.207193e-02;
coeffs_x(34) = -1.893817e-02;
coeffs_x(35) = 6.334773e-03;
coeffs_x(36) = 1.242593e-04;
coeffs_x(37) = -4.337045e-04;
coeffs_x(38) = 1.226202e-04;
coeffs_x(39) = -1.311202e-05;
coeffs_x(40) = 1.375000e+00;
coeffs_x(41) = 7.135496e-01;
coeffs_x(42) = -8.016613e-03;
coeffs_x(43) = -7.162332e-02;
coeffs_x(44) = 1.473035e-02;
coeffs_x(45) = 8.629236e-04;
coeffs_x(46) = -3.912121e-04;
coeffs_x(47) = 3.133750e-05;
coeffs_x(48) = -1.868618e-06;
coeffs_x(49) = 1.373647e-07;
coeffs_x(50) = 2.625000e+00;
coeffs_x(51) = 2.286411e-01;
coeffs_x(52) = -1.917766e-03;
coeffs_x(53) = 4.418997e-02;
coeffs_x(54) = 4.053256e-04;
coeffs_x(55) = -1.824550e-03;
coeffs_x(56) = 4.475988e-05;
coeffs_x(57) = 1.146982e-05;
coeffs_x(58) = 4.392728e-07;
coeffs_x(59) = -3.929156e-08;
coeffs_x(60) = 3.375000e+00;
coeffs_x(61) = 6.297272e-01;
coeffs_x(62) = 1.463427e-01;
coeffs_x(63) = -1.097278e-02;
coeffs_x(64) = -1.159995e-02;
coeffs_x(65) = -2.239705e-04;
coeffs_x(66) = 2.324635e-04;
coeffs_x(67) = -3.329242e-06;
coeffs_x(68) = 5.787637e-07;
coeffs_x(69) = -5.295902e-08;
coeffs_x(70) = 5.375000e+00;
coeffs_x(71) = 3.862543e-01;
coeffs_x(72) = -3.191415e-01;
coeffs_x(73) = -6.336126e-02;
coeffs_x(74) = 1.010710e-02;
coeffs_x(75) = 3.445765e-03;
coeffs_x(76) = 8.538129e-05;
coeffs_x(77) = -4.356702e-05;
coeffs_x(78) = -2.745004e-05;
coeffs_x(79) = 3.419369e-06;
coeffs_x(80) = 4.625000e+00;
coeffs_x(81) = -1.077246e+00;
coeffs_x(82) = -2.215040e-01;
coeffs_x(83) = 1.142110e-01;
coeffs_x(84) = 2.041077e-02;
coeffs_x(85) = -5.669714e-03;
coeffs_x(86) = -8.545535e-04;
coeffs_x(87) = -8.888563e-05;
coeffs_x(88) = 8.539336e-05;
coeffs_x(89) = -7.768879e-06;
coeffs_x(90) = 2.375000e+00;
coeffs_x(91) = -1.256748e-01;
coeffs_x(92) = 1.522778e-01;
coeffs_x(93) = -1.508529e-01;
coeffs_x(94) = -3.228139e-02;
coeffs_x(95) = 2.470529e-02;
coeffs_x(96) = -1.651435e-02;
coeffs_x(97) = 1.340898e-02;
coeffs_x(98) = -4.804456e-03;
coeffs_x(99) = 6.237849e-04;
coeffs_x(100) = 1.625000e+00;
coeffs_x(101) = -9.904204e-01;
coeffs_x(102) = -2.782357e-01;
coeffs_x(103) = 1.586861e-01;
coeffs_x(104) = 9.474334e-02;
coeffs_x(105) = -1.177637e-01;
coeffs_x(106) = 1.007910e-01;
coeffs_x(107) = -5.161318e-02;
coeffs_x(108) = 1.256060e-02;
coeffs_x(109) = -1.143167e-03;
coeffs_y(0) = 0;
coeffs_y(1) = 0;
coeffs_y(2) = 0;
coeffs_y(3) = 0;
coeffs_y(4) = 0;
coeffs_y(5) = 1.160638e+00;
coeffs_y(6) = -1.242191e+00;
coeffs_y(7) = 5.188543e-01;
coeffs_y(8) = -9.965207e-02;
coeffs_y(9) = 7.397976e-03;
coeffs_y(10) = 3.000000e+00;
coeffs_y(11) = 1.587917e-01;
coeffs_y(12) = -6.922857e-01;
coeffs_y(13) = 7.446508e-01;
coeffs_y(14) = 3.389531e-01;
coeffs_y(15) = -1.305685e+00;
coeffs_y(16) = 1.136135e+00;
coeffs_y(17) = -4.767476e-01;
coeffs_y(18) = 9.852596e-02;
coeffs_y(19) = -7.961722e-03;
coeffs_y(20) = 3.000000e+00;
coeffs_y(21) = -9.483063e-02;
coeffs_y(22) = -2.780867e-01;
coeffs_y(23) = -2.645014e-01;
coeffs_y(24) = 6.116532e-03;
coeffs_y(25) = 1.143802e-01;
coeffs_y(26) = -3.511134e-02;
coeffs_y(27) = 5.936452e-04;
coeffs_y(28) = 7.747317e-04;
coeffs_y(29) = -5.874434e-05;
coeffs_y(30) = 1.000000e+00;
coeffs_y(31) = -1.006064e-01;
coeffs_y(32) = 3.096612e-01;
coeffs_y(33) = -2.741275e-01;
coeffs_y(34) = -3.342188e-02;
coeffs_y(35) = 1.582824e-01;
coeffs_y(36) = -5.241312e-02;
coeffs_y(37) = -1.321832e-02;
coeffs_y(38) = 8.470849e-03;
coeffs_y(39) = -1.005860e-03;
coeffs_y(40) = 1.000000e+00;
coeffs_y(41) = -9.775486e-02;
coeffs_y(42) = -2.941306e-01;
coeffs_y(43) = -2.668669e-01;
coeffs_y(44) = 2.515080e-02;
coeffs_y(45) = 1.067078e-01;
coeffs_y(46) = -3.674391e-02;
coeffs_y(47) = 2.151712e-03;
coeffs_y(48) = 4.604822e-04;
coeffs_y(49) = -3.920883e-05;
coeffs_y(50) = -1.000000e+00;
coeffs_y(51) = -1.056517e-01;
coeffs_y(52) = 3.321421e-01;
coeffs_y(53) = -2.625048e-01;
coeffs_y(54) = -5.017446e-02;
coeffs_y(55) = 1.705206e-01;
coeffs_y(56) = -9.417199e-02;
coeffs_y(57) = 2.228261e-02;
coeffs_y(58) = -1.833127e-03;
coeffs_y(59) = -5.248478e-05;
coeffs_y(60) = -1.000000e+00;
coeffs_y(61) = 1.174174e-02;
coeffs_y(62) = 4.037501e-02;
coeffs_y(63) = 1.284467e-03;
coeffs_y(64) = -2.023781e-02;
coeffs_y(65) = 6.199251e-03;
coeffs_y(66) = -9.547453e-04;
coeffs_y(67) = 4.333622e-04;
coeffs_y(68) = -1.068105e-04;
coeffs_y(69) = 7.092359e-06;
coeffs_y(70) = -1.000000e+00;
coeffs_y(71) = -4.652006e-02;
coeffs_y(72) = 4.166093e-02;
coeffs_y(73) = 3.620239e-02;
coeffs_y(74) = -1.316262e-02;
coeffs_y(75) = -1.764001e-02;
coeffs_y(76) = 1.985557e-02;
coeffs_y(77) = -2.729405e-02;
coeffs_y(78) = 1.578200e-02;
coeffs_y(79) = -2.746480e-03;
coeffs_y(80) = -1.000000e+00;
coeffs_y(81) = 1.457035e-01;
coeffs_y(82) = 5.910276e-01;
coeffs_y(83) = 6.328288e-01;
coeffs_y(84) = -5.283649e-02;
coeffs_y(85) = -2.509362e-01;
coeffs_y(86) = 7.753935e-02;
coeffs_y(87) = 1.420009e-03;
coeffs_y(88) = -2.677854e-03;
coeffs_y(89) = 2.215643e-04;
coeffs_y(90) = 3.000000e+00;
coeffs_y(91) = 1.355453e-01;
coeffs_y(92) = -5.465629e-01;
coeffs_y(93) = 6.321221e-01;
coeffs_y(94) = 2.444833e-02;
coeffs_y(95) = -5.864510e-01;
coeffs_y(96) = 4.128950e-01;
coeffs_y(97) = -6.008257e-02;
coeffs_y(98) = -2.967080e-02;
coeffs_y(99) = 7.965986e-03;
coeffs_y(100) = 3.000000e+00;
coeffs_y(101) = -1.557292e-01;
coeffs_y(102) = -6.746490e-01;
coeffs_y(103) = -7.535720e-01;
coeffs_y(104) = 3.388979e-01;
coeffs_y(105) = -2.470958e-01;
coeffs_y(106) = 5.219580e-01;
coeffs_y(107) = -3.293174e-01;
coeffs_y(108) = 8.280294e-02;
coeffs_y(109) = -7.472631e-03;
coeffs_z(0) = 0;
coeffs_z(1) = 0;
coeffs_z(2) = 0;
coeffs_z(3) = 0;
coeffs_z(4) = 0;
coeffs_z(5) = 7.378266e-01;
coeffs_z(6) = -7.845922e-01;
coeffs_z(7) = 3.261565e-01;
coeffs_z(8) = -6.245173e-02;
coeffs_z(9) = 4.629720e-03;
coeffs_z(10) = 2.000000e+00;
coeffs_z(11) = 1.331465e-01;
coeffs_z(12) = -5.095580e-01;
coeffs_z(13) = 4.565531e-01;
coeffs_z(14) = 2.517127e-01;
coeffs_z(15) = -7.046080e-01;
coeffs_z(16) = 5.346707e-01;
coeffs_z(17) = -2.054955e-01;
coeffs_z(18) = 4.048075e-02;
coeffs_z(19) = -3.228178e-03;
coeffs_z(20) = 2.000000e+00;
coeffs_z(21) = -8.093791e-02;
coeffs_z(22) = -1.755646e-01;
coeffs_z(23) = -8.185434e-02;
coeffs_z(24) = 2.608581e-02;
coeffs_z(25) = 2.415591e-02;
coeffs_z(26) = -8.620303e-03;
coeffs_z(27) = 1.933141e-04;
coeffs_z(28) = 1.775760e-04;
coeffs_z(29) = -1.341378e-05;
coeffs_z(30) = 1.000000e+00;
coeffs_z(31) = -7.985800e-02;
coeffs_z(32) = 1.753957e-01;
coeffs_z(33) = -9.029027e-02;
coeffs_z(34) = -3.359694e-02;
coeffs_z(35) = 2.904169e-02;
coeffs_z(36) = 6.526155e-03;
coeffs_z(37) = -8.994571e-03;
coeffs_z(38) = 2.582515e-03;
coeffs_z(39) = -2.605456e-04;
coeffs_z(40) = 1.000000e+00;
coeffs_z(41) = 3.035498e-02;
coeffs_z(42) = 8.299558e-02;
coeffs_z(43) = 4.901782e-02;
coeffs_z(44) = -1.848977e-02;
coeffs_z(45) = -9.791166e-03;
coeffs_z(46) = 4.235046e-03;
coeffs_z(47) = -3.227413e-04;
coeffs_z(48) = -3.107763e-05;
coeffs_z(49) = 2.390111e-06;
coeffs_z(50) = 1.500000e+00;
coeffs_z(51) = 5.779965e-02;
coeffs_z(52) = -9.619224e-02;
coeffs_z(53) = 3.282729e-02;
coeffs_z(54) = 1.607376e-02;
coeffs_z(55) = -1.066735e-02;
coeffs_z(56) = -5.544073e-03;
coeffs_z(57) = 7.166793e-03;
coeffs_z(58) = -2.361804e-03;
coeffs_z(59) = 2.603729e-04;
coeffs_z(60) = 1.500000e+00;
coeffs_z(61) = 5.406696e-02;
coeffs_z(62) = 8.533523e-02;
coeffs_z(63) = 2.890938e-02;
coeffs_z(64) = -1.155779e-02;
coeffs_z(65) = -7.503573e-03;
coeffs_z(66) = 2.401468e-03;
coeffs_z(67) = 1.408765e-04;
coeffs_z(68) = -1.056187e-04;
coeffs_z(69) = 9.591200e-06;
coeffs_z(70) = 2.000000e+00;
coeffs_z(71) = 4.752624e-02;
coeffs_z(72) = -1.065025e-01;
coeffs_z(73) = 3.059953e-02;
coeffs_z(74) = 2.737900e-02;
coeffs_z(75) = -1.636121e-02;
coeffs_z(76) = 1.661706e-02;
coeffs_z(77) = -1.147483e-02;
coeffs_z(78) = 2.396688e-03;
coeffs_z(79) = -7.043652e-05;
coeffs_z(80) = 2;
coeffs_z(81) = -1.058393e-01;
coeffs_z(82) = -3.314214e-01;
coeffs_z(83) = -2.547447e-01;
coeffs_z(84) = 5.211463e-02;
coeffs_z(85) = 8.564122e-02;
coeffs_z(86) = -3.048982e-02;
coeffs_z(87) = 1.220149e-03;
coeffs_z(88) = 5.414390e-04;
coeffs_z(89) = -4.354282e-05;
coeffs_z(90) = -7.198543e-15;
coeffs_z(91) = -1.044226e-01;
coeffs_z(92) = 3.247712e-01;
coeffs_z(93) = -2.541024e-01;
coeffs_z(94) = -4.584424e-02;
coeffs_z(95) = 1.456099e-01;
coeffs_z(96) = -6.123184e-02;
coeffs_z(97) = 2.051463e-03;
coeffs_z(98) = 4.047892e-03;
coeffs_z(99) = -7.102203e-04;
coeffs_z(100) = -4.046622e-18;
coeffs_z(101) = -3.601212e-03;
coeffs_z(102) = 1.427490e-02;
coeffs_z(103) = -4.819981e-03;
coeffs_z(104) = -1.024795e-02;
coeffs_z(105) = 8.431317e-03;
coeffs_z(106) = -1.530224e-03;
coeffs_z(107) = -5.010571e-04;
coeffs_z(108) = 2.227875e-04;
coeffs_z(109) = -2.319913e-05;




//  gPrev_it = ros::Time::now();
  double time_traj = -step;
  double time_poly = -step;
  double time_poly2 = 0;
  double time_poly3;
  double rate = 1;
  double dephase = 0;
  double amp =1;
  double first =1;
  double condition = 0;
  while(ros::ok()) {

      time_traj = time_traj+step;
      traj_gen::min_snap_trajPtr uav_state_msg(new traj_gen::min_snap_traj);
      if (time_traj < 3){
    // Publish data: UAV state in World frame
      rate = 1;
      uav_state_msg->position_ref.x  = 0;
      uav_state_msg->position_ref.y  = 0;
      uav_state_msg->position_ref.z  = 1.0;
      uav_state_msg->velocity_ref.x  = 0;
      uav_state_msg->velocity_ref.y  = 0;
      uav_state_msg->velocity_ref.z  = 0;
      uav_state_msg->accel_ref.x  = 0;
      uav_state_msg->accel_ref.y  = 0;
      uav_state_msg->accel_ref.z  = 0;
      uav_state_msg->yaw_ref[0]  = 0.0;
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
          uav_state_msg->position_ref.z  = horner_poly(coeffs_z.segment(i*num_coeffs,num_coeffs),time_poly2)+1;
          uav_state_msg->velocity_ref.x  = horner_poly_dot(coeffs_x.segment(i*num_coeffs,num_coeffs),time_poly2);
          uav_state_msg->velocity_ref.y  = horner_poly_dot(coeffs_y.segment(i*num_coeffs,num_coeffs),time_poly2);
          uav_state_msg->velocity_ref.z  = horner_poly_dot(coeffs_z.segment(i*num_coeffs,num_coeffs),time_poly2);
          uav_state_msg->accel_ref.x  = horner_poly_dot_dot(coeffs_x.segment(i*num_coeffs,num_coeffs),time_poly2);
          uav_state_msg->accel_ref.y  = horner_poly_dot_dot(coeffs_y.segment(i*num_coeffs,num_coeffs),time_poly2);
          uav_state_msg->accel_ref.z  = horner_poly_dot_dot(coeffs_z.segment(i*num_coeffs,num_coeffs),time_poly2);
          break;
        }
      }
      condition = 0;
    }else {
      uav_state_msg->position_ref.x  = 0;
      uav_state_msg->position_ref.y  = 0;
      uav_state_msg->position_ref.z  = 1.0;
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







    /*else if (time_traj < 10) {
      uav_state_msg->position_ref.x  = 1.8;
      uav_state_msg->position_ref.y  = 0;
      uav_state_msg->position_ref.z  = 1.0;
      uav_state_msg->velocity_ref.x  = 0;
      uav_state_msg->velocity_ref.y  = 0;
      uav_state_msg->velocity_ref.z  = 0;
      uav_state_msg->accel_ref.x  = 0;
      uav_state_msg->accel_ref.y  = 0;
      uav_state_msg->accel_ref.z  = 0;
      uav_state_msg->yaw_ref[0]  = 0.0;
      uav_state_msg->yaw_ref[1]  = 0;
      uav_state_msg->yaw_ref[2]  = 0;
    } else {
      time_poly = time_poly+step;

      if (time_traj < 330) {

        if (time_poly-time_poly2 > 8){
          time_poly2 = time_poly;
          dephase = (rate-rate*1.1)*time_poly2+dephase;
          rate = rate*1.1;

          if (rate > 2.22222){
            if (first){
              first = 0;
              dephase = (rate-2.22222)*time_poly2+dephase;
            }

            rate = 2.222222;
          }
        }




        amp = 1.8;
        uav_state_msg->position_ref.y  = amp*sin(time_poly*rate+dephase);
        uav_state_msg->position_ref.x  = amp*cos(time_poly*rate+dephase);
        uav_state_msg->position_ref.z  = 1.0;
        uav_state_msg->velocity_ref.y  = amp*rate*cos(time_poly*rate+dephase);
        uav_state_msg->velocity_ref.x  = -amp*rate*sin(time_poly*rate+dephase);
        uav_state_msg->velocity_ref.z  = 0;
        uav_state_msg->accel_ref.x  = -rate*amp*rate*cos(time_poly*rate+dephase);
        uav_state_msg->accel_ref.y  = -rate*amp*rate*sin(time_poly*rate+dephase);
        uav_state_msg->accel_ref.z  = 0;
        uav_state_msg->yaw_ref[0]  = 0;
        uav_state_msg->yaw_ref[1]  = 0;
        uav_state_msg->yaw_ref[2]  = 0;
      }
    } /*else if (time_poly < 5*3.1416*2) {
        rate = 1.5;
        time_poly2 = time_poly-5*3.1416*2;
        amp = 1.8;
        uav_state_msg->position_ref.y  = amp*sin(time_poly2*rate);
        uav_state_msg->position_ref.x  = amp*cos(time_poly2*rate);
        uav_state_msg->position_ref.z  = 1.0;
        uav_state_msg->velocity_ref.y  = amp*rate*cos(time_poly2*rate);
        uav_state_msg->velocity_ref.x  = -amp*rate*sin(time_poly2*rate);
        uav_state_msg->velocity_ref.z  = 0;
        uav_state_msg->accel_ref.x  = -rate*amp*rate*cos(time_poly2*rate);
        uav_state_msg->accel_ref.y  = -rate*amp*rate*sin(time_poly2*rate);
        uav_state_msg->accel_ref.z  = 0;
        uav_state_msg->yaw_ref[0]  = 0;
        uav_state_msg->yaw_ref[1]  = 0;
        uav_state_msg->yaw_ref[2]  = 0;
      } else if (time_poly < 10*2*3.1416/1.5) {
        rate = 2.2222222;
        time_poly3 = time_poly-5*3.1416*2-10*2*3.1416/1.5;
        amp = 1.8;
        uav_state_msg->position_ref.y  = amp*sin(time_poly3*rate);
        uav_state_msg->position_ref.x  = amp*cos(time_poly3*rate);
        uav_state_msg->position_ref.z  = 1.0;
        uav_state_msg->velocity_ref.y  = amp*rate*cos(time_poly3*rate);
        uav_state_msg->velocity_ref.x  = -amp*rate*sin(time_poly3*rate);
        uav_state_msg->velocity_ref.z  = 0;
        uav_state_msg->accel_ref.x  = -rate*amp*rate*cos(time_poly3*rate);
        uav_state_msg->accel_ref.y  = -rate*amp*rate*sin(time_poly3*rate);
        uav_state_msg->accel_ref.z  = 0;
        uav_state_msg->yaw_ref[0]  = 0;
        uav_state_msg->yaw_ref[1]  = 0;
        uav_state_msg->yaw_ref[2]  = 0;
      }*/
      /*
      else if (time_traj < 10) {
        uav_state_msg->position_ref.x  = 0;
        uav_state_msg->position_ref.y  = 0;
        uav_state_msg->position_ref.z  = 1.0;
        uav_state_msg->velocity_ref.x  = 0;
        uav_state_msg->velocity_ref.y  = 0;
        uav_state_msg->velocity_ref.z  = 0;
        uav_state_msg->accel_ref.x  = 0;
        uav_state_msg->accel_ref.y  = 0;
        uav_state_msg->accel_ref.z  = 0;
        uav_state_msg->yaw_ref[0]  = 0.0;
        uav_state_msg->yaw_ref[1]  = 0;
        uav_state_msg->yaw_ref[2]  = 0;
      } else if (time_traj < 15) {
        uav_state_msg->position_ref.x  = 0.0;
        uav_state_msg->position_ref.y  = 1.0;
        uav_state_msg->position_ref.z  = 1.0;
        uav_state_msg->velocity_ref.x  = 0;
        uav_state_msg->velocity_ref.y  = 0;
        uav_state_msg->velocity_ref.z  = 0;
        uav_state_msg->accel_ref.x  = 0;
        uav_state_msg->accel_ref.y  = 0;
        uav_state_msg->accel_ref.z  = 0;
        uav_state_msg->yaw_ref[0]  = 0;
        uav_state_msg->yaw_ref[1]  = 0;
        uav_state_msg->yaw_ref[2]  = 0;
      } else if (time_traj < 19) {
        uav_state_msg->position_ref.x  = 0.0;
        uav_state_msg->position_ref.y  = 1.0;
        uav_state_msg->position_ref.z  = 1.0;
        uav_state_msg->velocity_ref.x  = 0;
        uav_state_msg->velocity_ref.y  = 0;
        uav_state_msg->velocity_ref.z  = 0;
        uav_state_msg->accel_ref.x  = 0;
        uav_state_msg->accel_ref.y  = 0;
        uav_state_msg->accel_ref.z  = 0;
        uav_state_msg->yaw_ref[0]  = 1.5;
        uav_state_msg->yaw_ref[1]  = 0;
        uav_state_msg->yaw_ref[2]  = 0;
      } else {
        uav_state_msg->position_ref.x  = 0;
        uav_state_msg->position_ref.y  = 0;
        uav_state_msg->position_ref.z  = 1.0;
        uav_state_msg->velocity_ref.x  = 0;
        uav_state_msg->velocity_ref.y  = 0;
        uav_state_msg->velocity_ref.z  = 0;
        uav_state_msg->accel_ref.x  = 0;
        uav_state_msg->accel_ref.y  = 0;
        uav_state_msg->accel_ref.z  = 0;
        uav_state_msg->yaw_ref[0]  = 1.5;
        uav_state_msg->yaw_ref[1]  = 0;
        uav_state_msg->yaw_ref[2]  = 0;
      }
      */





      uav_state_msg->header.stamp  =  ros::Time::now();
      uav_state_pub_.publish(uav_state_msg);

    ros::spinOnce();
    r.sleep();
  } // end while

  return 0;
}
