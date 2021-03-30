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

#include "quat_att_control.h"
#include <Eigen/Geometry>
#include <Eigen/LU>
namespace att_control {



attitude_controller::~attitude_controller() {}

void attitude_controller::InitializeParameters() {
  initialized_params_ = true;
}

void attitude_controller::SetMotorSpeed(Eigen::VectorXd& rotor_velocities) {
  Eigen::Vector4d angular_acceleration_thrust;
  angular_acceleration_thrust.block<3, 1>(1, 0) = moment;
  angular_acceleration_thrust(0) = thrust;
  rotor_velocities.resize(6);

  Eigen::Matrix4d Alloc_mat;
  Eigen::MatrixXd Alloc_mat_inverse(6,4);
  /*Alloc_mat << kt, kt, kt, kt,
              arm*kt*sqrt_2_2, -arm*kt*sqrt_2_2, -arm*kt*sqrt_2_2, arm*kt*sqrt_2_2,
              -arm*kt*sqrt_2_2, -arm*kt*sqrt_2_2, arm*kt*sqrt_2_2, arm*kt*sqrt_2_2,
              kd, -kd, kd,-kd;*/
              Alloc_mat << kt, kt, kt, kt,
                          0, arm*kt, 0, -arm*kt,
                          -arm*kt, 0, arm*kt, 0,
                          -kd, kd, -kd,kd;

// Firefly
            Alloc_mat_inverse << 1/(6*kt),1/(6*arm*kt), -pow((3.0),1.0/2.0)/(6*arm*kt), -1/(6*kd),
                                1/(6*kt),1/(3*arm*kt), 0, 1/(6*kd),
                                1/(6*kt),1/(6*arm*kt), pow((3.0),1.0/2.0)/(6*arm*kt), -1/(6*kd),
                                1/(6*kt),-1/(6*arm*kt), pow((3.0),1.0/2.0)/(6*arm*kt), 1/(6*kd),
                                1/(6*kt),-1/(3*arm*kt), 0, -1/(6*kd),
                                1/(6*kt),-1/(6*arm*kt), -pow((3.0),1.0/2.0)/(6*arm*kt), 1/(6*kd);

    /*Alloc_mat << kt, kt, kt, kt,
                -0.09784210*kt, 0.09784210*kt, 0.09784210*kt, -0.09784210*kt,
                -0.08440513*kt, -0.08440513*kt, 0.06853580*kt, 0.06853580*kt,
                -kd, kd, -kd,kd;*/
    /*Alloc_mat << kt, kt, kt, kt,
                -0.09125213*kt, 0.09125213*kt, 0.09125213*kt, -0.09125213*kt,
                -0.09125213*kt, -0.09125213*kt, 0.09125213*kt, 0.09125213*kt,
                -kd, kd, -kd,kd;*/

  // Pelican
  //*rotor_velocities = Alloc_mat.inverse()*angular_acceleration_thrust;

  // Firefly
  rotor_velocities = Alloc_mat_inverse*angular_acceleration_thrust;

  if(rotor_velocities[0]<0.0){
    rotor_velocities[0] = 0.0;
  }
  if(rotor_velocities[1]<0.0){
    rotor_velocities[1] = 0.0;
  }
  if(rotor_velocities[2]<0.0){
    rotor_velocities[2] = 0.0;
  }
  if(rotor_velocities[3]<0.0){
    rotor_velocities[3] = 0.0;
  }
  if(rotor_velocities[4]<0.0){
    rotor_velocities[4] = 0.0;
  }
  if(rotor_velocities[5]<0.0){
    rotor_velocities[5] = 0.0;
  }


  rotor_velocities = rotor_velocities.cwiseSqrt();
  if(rotor_velocities[0]>1000.0){
    rotor_velocities[0] = 1000.0;
  }
  if(rotor_velocities[1]>1000.0){
    rotor_velocities[1] = 1000.0;
  }
  if(rotor_velocities[2]>1000.0){
    rotor_velocities[2] = 1000.0;
  }
  if(rotor_velocities[3]>1000.0){
    rotor_velocities[3] = 1000.0;
  }
  if(rotor_velocities[4]>1000.0){
    rotor_velocities[4] = 1000.0;
  }
  if(rotor_velocities[5]>1000.0){
    rotor_velocities[5] = 1000.0;
  }


  //ROS_INFO("rotor speeds:%f %f %f %f",rotor_velocities->x(),rotor_velocities->y(),rotor_velocities->z(),rotor_velocities->w());

}

void attitude_controller::SetParameters(double thrust_coefficient, double drag_coefficient, Eigen::Vector3d& ang_velocity_gain_input) {
  kt = thrust_coefficient;
  kd = drag_coefficient;
  ang_velocity_gain_ = ang_velocity_gain_input;
}

void attitude_controller::CalculateMomentThrust(const Eigen::Vector3d& accel_out_input, const Eigen::Vector3d& desired_att) {
  //Eigen::Vector3d desired_att;
  accel_out = accel_out_input;
  //ComputeDesiredAngVelocity(&accel_out,&desired_att);
  // Project thrust onto body z axis.
  //ROS_INFO("accel_out: %f %f %f",accel_out(0),accel_out(1),accel_out(2));
  //ROS_INFO("desired_att: %f %f %f",desired_att(0),desired_att(1),desired_att(2));
  //thrust = mass * accel_out.dot(odometry_.orientation.toRotationMatrix().col(2));
  //Eigen::Vector3d mass_vect;
  //mass_vect(1) = mass;
  //mass_vect(2) = mass;
  //mass_vect(3) = mass;
  thrust = mass*(accel_out.norm());
  moment = (J.cwiseProduct(ang_velocity_gain_)).cwiseProduct((desired_att-odometry_.angular_velocity))+odometry_.angular_velocity.cross(J.cwiseProduct(odometry_.angular_velocity));
  //ROS_INFO("moment: %f %f %f",moment(0),moment(1),moment(2));
  //ROS_INFO("thrust: %f",thrust);
  //Eigen::Vector3d att_error;
  //att_error = desired_att-odometry_.angular_velocity;
  //ROS_INFO("att_error: %f %f %f",att_error(0),att_error(1),att_error(2));
  //ROS_INFO("accel: %f %f %f",accel_out_input(0),accel_out_input(1),accel_out_input(2));
}

void attitude_controller::SetOdometry(const EigenOdometry& odometry) {
  odometry_ = odometry;
}

void attitude_controller::GetOdometry(EigenOdometry& odometry){
  odometry = odometry_;
}


void attitude_controller::GetThrustMoment(Eigen::Vector3d& moment_, double* thrust_){
  moment_ = moment;
  *thrust_ = thrust;

}



}
