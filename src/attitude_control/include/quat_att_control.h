/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef POSITION_CONTROL_ATTITUDE_CONTROLLER_H
#define POSITION_CONTROL_ATTITUDE_CONTROLLER_H

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>

#include "common.h"
#include "parameters.h"
#include "parameters_ros.h"

namespace att_control {

// Default values for the lee position controller and the Asctec Firefly.
//static const Eigen::Vector3d kDefaultAttitudeGain = Eigen::Vector3d(12, 12, 5);
// bebop
//static const Eigen::Vector3d kDefaultAngVelocityGain = Eigen::Vector3d(10,10,5.5);

static const Eigen::Vector3d kDefaultAngVelocityGain = Eigen::Vector3d(7.0,7.0,8);

//pelican
//static const Eigen::Vector3d kDefaultAngVelocityGain = Eigen::Vector3d(10,10,10);
/*static const double kDefaultArm = 14.45e-2;
static const double kDefaultkt = 0.00000196056;
static const double kDefaultkd = 2.59888836e-8;*/
static const double kDefaultsqrt_2_2 = 0.70710678118;
//static const Eigen::Vector3d kDefaultJ = Eigen::Vector3d(0.001805, 0.001764, 0.003328);
//static const double kDefaultmass_ = 0.60804;

//static const Eigen::Vector3d kDefaultJ = Eigen::Vector3d(0.007, 0.007, 0.012);
//static const double kDefaultmass_ = 0.68;
//static const double kDefaultArm = 0.17;
//static const double kDefaultkt = 8.54858e-06;
//static const double kDefaultkd = 8.06428e-05;

/*static const Eigen::Vector3d kDefaultJ = Eigen::Vector3d(0.0203923, 0.0209129, 0.0338655);
static const double kDefaultmass_ =1.757;
static const double kDefaultArm = 0.21;
static const double kDefaultkt = 1.42e-05;
static const double kDefaultkd = 6.4545e-07;*/


// Firefly
static const Eigen::Vector3d kDefaultJ = Eigen::Vector3d(0.0347563, 0.0458929, 0.0977);
// 1.467
static const double kDefaultmass_ =1.467+0.04;
static const double kDefaultArm = 0.215;
static const double kDefaultkt = 6.0e-06;
static const double kDefaultkd = 3.120231700000000e-07;

class attitude_controller {
 public:
   attitude_controller()
       : ang_velocity_gain_(kDefaultAngVelocityGain),kt(kDefaultkt),
         arm(kDefaultArm),kd(kDefaultkd),sqrt_2_2(kDefaultsqrt_2_2),J(kDefaultJ),mass(kDefaultmass_){}
        //attitude_gain_(kDefaultAttitudeGain),


  ~attitude_controller();
  void InitializeParameters();
// Eigen::Vector3d* attitude_gain_input,
  void CalculateMomentThrust(const Eigen::Vector3d& accel_out_input,const Eigen::Vector3d& desired_att);
  void SetOdometry(const EigenOdometry& odometry);
  void GetOdometry(EigenOdometry& odometry);
  void SetMotorSpeed(Eigen::VectorXd& rotor_velocities);
  void GetThrustMoment(Eigen::Vector3d& moment_, double* thrust_);
  void SetParameters(double thrust_coefficient, double drag_coefficient, Eigen::Vector3d& ang_velocity_gain_input);
  /*void SetTrajectoryPoint(
    const mav_msgs::EigenTrajectoryPoint& command_trajectory);*/
  Eigen::Vector3d accel_out;
  mav_msgs::EigenTrajectoryPoint command_trajectory_;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  //Eigen::Vector3d attitude_gain_;
  Eigen::Vector3d ang_velocity_gain_;
  Eigen::Vector3d J;
  double kt;
  double arm;
  double kd;
  double sqrt_2_2;
  double thrust;
  double mass;
  Eigen::Vector3d moment;
  Eigen::Vector4d motor_speed;
  bool initialized_params_;
  bool controller_active_;


  EigenOdometry odometry_;

  //void ComputeDesiredAngVelocity(Eigen::Vector3d* desired_att, Eigen::Vector3d* accel_out) const;
};
}

#endif // POSITION_CONTROL_PD_POSITION_CONTROLLER_H
