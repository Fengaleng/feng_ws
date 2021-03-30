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

#ifndef POSITION_CONTROL_PD_POSITION_CONTROLLER_H
#define POSITION_CONTROL_PD_POSITION_CONTROLLER_H

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>

#include "common.h"
#include "parameters.h"
#include "parameters_ros.h"

namespace pos_control {

  static const Eigen::Vector3d kDefaultAttitudeGain = Eigen::Vector3d(5.25, 5.25, 1);
  static const Eigen::Vector3d kDefaultPositionGain = Eigen::Vector3d(3.9, 4.0, 10);
  static const Eigen::Vector3d kDefaultVelocityGain = Eigen::Vector3d(2.985, 3.05, 5);

// Default values for the lee position controller and the Asctec Firefly.3.2
//Good gains pelican
/*static const Eigen::Vector3d kDefaultAttitudeGain = Eigen::Vector3d(6.75, 6.75, 2);
static const Eigen::Vector3d kDefaultPositionGain = Eigen::Vector3d(4.3, 4.3, 10);
static const Eigen::Vector3d kDefaultVelocityGain = Eigen::Vector3d(2.9, 2.9, 5);
*/

/*static const Eigen::Vector3d kDefaultAttitudeGain = Eigen::Vector3d(6.75, 6.75, 3);
static const Eigen::Vector3d kDefaultPositionGain = Eigen::Vector3d(3, 3.2, 10);
static const Eigen::Vector3d kDefaultVelocityGain = Eigen::Vector3d(2.05, 2.1, 4);*/


/*static const Eigen::Vector3d kDefaultAttitudeGain = Eigen::Vector3d(6.75, 6.75, 3);
static const Eigen::Vector3d kDefaultPositionGain = Eigen::Vector3d(3, 3, 10);
static const Eigen::Vector3d kDefaultVelocityGain = Eigen::Vector3d(2.05, 2.05, 4);*/



// we choose slower response and no overshoot to quick with overshoot for camera accuracy
/*static const Eigen::Vector3d kDefaultAttitudeGain = Eigen::Vector3d(5.75, 5.75, 1);
static const Eigen::Vector3d kDefaultPositionGain = Eigen::Vector3d(3.8, 3.8, 12);
static const Eigen::Vector3d kDefaultVelocityGain = Eigen::Vector3d(2.6, 2.6, 4);*/

// hummingbird
/*static const Eigen::Vector3d kDefaultAttitudeGain = Eigen::Vector3d(5, 5, 10);
static const Eigen::Vector3d kDefaultPositionGain = Eigen::Vector3d(4, 4, 9);
static const Eigen::Vector3d kDefaultVelocityGain = Eigen::Vector3d(2.53, 2.53, 4);*/

// bebop
/*static const Eigen::Vector3d kDefaultAttitudeGain = Eigen::Vector3d(5, 5, 10);
static const Eigen::Vector3d kDefaultPositionGain = Eigen::Vector3d(2.5, 2.5, 12);
static const Eigen::Vector3d kDefaultVelocityGain = Eigen::Vector3d(1.2, 1.2, 4);*/

static const double kDefaultgrav =  9.81;
//static const double kDefaultmass_ =  0.68;
//static const double kDefaultmass_ =  0.60804;
static const double kDefaultmass_ = 1.04;

class PD_position_controller {
 public:
   PD_position_controller()
       : attitude_gain_(kDefaultAttitudeGain),
         position_gain_(kDefaultPositionGain),
         velocity_gain_(kDefaultVelocityGain),
         gravity(kDefaultgrav),
         mass(kDefaultmass_){}



  ~PD_position_controller();
  void InitializeParameters();
  void SetParameters(Eigen::Vector3d& ang_gain_input, Eigen::Vector3d& pos_gain_input, Eigen::Vector3d& velocity_gain_input);
  void CalculateThrust();
  void SetOdometry(const EigenOdometry& odometry);
  void GetNormalizedThrust(float* normalized_thrust_out);
  /*void SetTrajectoryPoint(
    const mav_msgs::EigenTrajectoryPoint& command_trajectory);*/
  Eigen::Vector3d accel_out;
  Eigen::Vector3d att_out;
  mav_msgs::EigenTrajectoryPoint command_trajectory_;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  Eigen::Vector3d attitude_gain_;
  Eigen::Vector3d position_gain_;
  Eigen::Vector3d velocity_gain_;
  double gravity;
  double mass;
  float thrust;

  float th0 = 0.457;
  float w0 = 8600/3.1416/30;
  float scaling = w0/th0;
  float thrust_normalized = 0;
  float kt = 0.00000196056;


  bool initialized_params_;
  bool controller_active_;


  EigenOdometry odometry_;

  void ComputeDesiredAcceleration(Eigen::Vector3d* acceleration) const;
  void ComputeDesiredAngVelocity(Eigen::Vector3d* desired_att, Eigen::Vector3d* acceleration_computed) const;
};
}

#endif // POSITION_CONTROL_PD_POSITION_CONTROLLER_H
