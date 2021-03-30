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

#include "pd_pos_controller.h"

namespace pos_control {



PD_position_controller::~PD_position_controller() {}

void PD_position_controller::InitializeParameters() {
  initialized_params_ = true;
}

void PD_position_controller::SetParameters(Eigen::Vector3d& ang_gain_input, Eigen::Vector3d& pos_gain_input, Eigen::Vector3d& velocity_gain_input) {
  attitude_gain_ = ang_gain_input;
  position_gain_ = pos_gain_input;
  velocity_gain_ = velocity_gain_input;
}

void PD_position_controller::CalculateThrust() {
  Eigen::Vector3d acceleration;
  Eigen::Vector3d desired_att;
  ComputeDesiredAcceleration(&acceleration);
  ComputeDesiredAngVelocity(&desired_att, &acceleration);


  // Project thrust onto body z axis.
  thrust = (float)(mass * acceleration.dot(odometry_.orientation.toRotationMatrix().col(2)));
  // scaling = 0.457; pour 8600rpm = 8600/pi/30 rad/s; thrust/4/kt = omega^2; sqrt(res) = omega; omega/scaling = normalized thrust;
  float int_value = thrust/kt/4;
  if (int_value < 0){
    int_value = 0;
  }
  thrust_normalized = sqrt(int_value);
  if (thrust_normalized > 1400){
    thrust_normalized  = 1400;
  }
  thrust_normalized = thrust_normalized/scaling;
  accel_out = acceleration;
  att_out = desired_att;
}

void PD_position_controller::GetNormalizedThrust(float* normalized_thrust_out) {

  *normalized_thrust_out = thrust_normalized;

}


void PD_position_controller::SetOdometry(const EigenOdometry& odometry) {
  odometry_ = odometry;
  //ROS_INFO("height: %f",odometry.position.z());
}

/*void PD_position_controller::SetTrajectoryPoint(
    const mav_msgs::EigenTrajectoryPoint& command_trajectory) {
  command_trajectory_ = command_trajectory;
  controller_active_ = true;
}*/

void PD_position_controller::ComputeDesiredAcceleration(Eigen::Vector3d* acceleration) const {
  assert(acceleration);

  Eigen::Vector3d position_error;
  position_error = command_trajectory_.position_W - odometry_.position;

  // Transform velocity to world frame.
  const Eigen::Matrix3d R_W_I = odometry_.orientation.toRotationMatrix();

  Eigen::Vector3d velocity_W =  R_W_I * odometry_.velocity;
  Eigen::Vector3d velocity_error;
  velocity_error = command_trajectory_.velocity_W - velocity_W;

  Eigen::Vector3d e_3(Eigen::Vector3d::UnitZ());

  *acceleration = (position_error.cwiseProduct(position_gain_)
      + velocity_error.cwiseProduct(velocity_gain_)) / mass
      + gravity * e_3 + command_trajectory_.acceleration_W;
}

void PD_position_controller::ComputeDesiredAngVelocity(Eigen::Vector3d* desired_att, Eigen::Vector3d* acceleration) const {
  //assert(desired_att);
  //assert(accel_out);
  //ROS_INFO("accel: %f %f %f",accel_out->x(),accel_out->y(),accel_out->z());
  Eigen::Vector3d normal_att;
  Eigen::Vector3d e_zb;
  Eigen::Vector3d e_zb_des;


  double angle;
  Eigen::Quaterniond q_erp;

  e_zb_des = acceleration->normalized();
  e_zb = odometry_.orientation*Eigen::Vector3d::UnitZ();
  angle = acos(e_zb.dot(e_zb_des));
  if ((fabs(angle))<(1e-5)){
      q_erp.w() = 1;
      q_erp.x() = 0;
      q_erp.y() = 0;
      q_erp.z() = 0;
  } else {
  normal_att = e_zb.cross(e_zb_des);
  normal_att.normalize();
  normal_att = odometry_.orientation.inverse()*normal_att;
  q_erp.vec() = normal_att*sin(angle/2);
  q_erp.w() = cos(angle/2);
  }
  //ROS_INFO("e_zb_des: %f %f %f",e_zb_des.x(),e_zb_des.y(),e_zb_des.z());
  //ROS_INFO("e_zb: %f %f %f",e_zb.x(),e_zb.y(),e_zb.z());
  if (q_erp.w()>=0){
    desired_att->x() = 2*attitude_gain_(0)*q_erp.x();
    desired_att->y() = 2*attitude_gain_(1)*q_erp.y();
  } else {
    desired_att->x() = -2*attitude_gain_(0)*q_erp.x();
    desired_att->y() = -2*attitude_gain_(1)*q_erp.y();
  }
  //ROS_INFO("desired_att: %f %f ",desired_att->x(),desired_att->y());
  Eigen::Vector3d e_y_C(-sin(command_trajectory_.getYaw()), cos(command_trajectory_.getYaw()), 0.0);
  Eigen::Vector3d e_x_des;
  e_x_des = e_y_C.cross(e_zb_des);

  if (e_x_des.norm() < 1e-7) {
    desired_att->z() = 0;
  } else {
    if (e_zb_des(2)<0){
      e_x_des = -e_x_des;
    }
    Eigen::Vector3d e_y_des;
    e_x_des.normalize();
    e_y_des = e_zb_des.cross(e_x_des);
    e_y_des.normalize();


    Eigen::Matrix3d R_des;
    R_des.block<3,1>(0,0) = e_x_des;
    R_des.block<3,1>(0,1) = e_y_des;
    R_des.block<3,1>(0,2) = e_zb_des;
    Eigen::Quaterniond q_des(R_des);

    Eigen::Quaterniond q_ey;

    q_ey = (odometry_.orientation*q_erp).inverse()*q_des;
    if (q_ey.w()>=0){
      desired_att->z() = 2*attitude_gain_(2)*q_ey.z();
    } else {
      desired_att->z() = -2*attitude_gain_(2)*q_ey.z();
    }
    //ROS_INFO("desired_att: %f %f %f",desired_att->x(),desired_att->y(),desired_att->z());
    //ROS_INFO("odometry_: %f %f %f %f",odometry_.orientation.x(),odometry_.orientation.y(),odometry_.orientation.z(),odometry_.orientation.w());
    //ROS_INFO("q_des: %f %f %f %f",q_des.x(),q_des.y(),q_des.z(),q_des.w());
    //ROS_INFO("q_ey: %f %f %f %f",q_ey.x(),q_ey.y(),q_ey.z(),q_ey.w());
    //ROS_INFO("q_erp: %f %f %f %f",q_erp.x(),q_erp.y(),q_erp.z(),q_erp.w());

    /*q_ey = odometry_.orientation.inverse()*q_des;
    if (q_erp.w()>=0){
      desired_att->x() = 2*attitude_gain_(0)*q_ey.x();
      desired_att->y() = 2*attitude_gain_(1)*q_ey.y();
    } else {
      desired_att->x() = -2*attitude_gain_[0]*q_ey.x();
      desired_att->y() = -2*attitude_gain_[1]*q_ey.y();
    }
    if (q_ey.w()>=0){
      desired_att->z() = 2*attitude_gain_(2)*q_ey.z();
    } else {
      desired_att->z() = -2*attitude_gain_(2)*q_ey.z();
    }*/
    //ROS_INFO
  }
  //ROS_INFO("desired_att_2: %f %f %f",desired_att->x(),desired_att->y(),desired_att->z());
  //ROS_INFO("desired_att: %f %f %f",desired_att->x(),desired_att->y(),desired_att->z());

}

}
