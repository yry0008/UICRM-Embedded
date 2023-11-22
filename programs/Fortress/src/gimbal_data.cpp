/*###########################################################
 # Copyright (c) 2023. BNU-HKBU UIC RoboMaster              #
 #                                                          #
 # This program is free software: you can redistribute it   #
 # and/or modify it under the terms of the GNU General      #
 # Public License as published by the Free Software         #
 # Foundation, either version 3 of the License, or (at      #
 # your option) any later version.                          #
 #                                                          #
 # This program is distributed in the hope that it will be  #
 # useful, but WITHOUT ANY WARRANTY; without even           #
 # the implied warranty of MERCHANTABILITY or FITNESS       #
 # FOR A PARTICULAR PURPOSE.  See the GNU General           #
 # Public License for more details.                         #
 #                                                          #
 # You should have received a copy of the GNU General       #
 # Public License along with this program.  If not, see     #
 # <https://www.gnu.org/licenses/>.                         #
 ###########################################################*/

//
// Created by yangr on 2023/4/15.
//

#include "gimbal_data.h"

control::gimbal_pid_t gimbalBasicPID;

void init_gimbalBasicData() {
    float pitch_theta_max_iout = 0;
    float pitch_theta_max_out = 10;
    float pitch_omega_max_iout = 10000;
    float pitch_omega_max_out = 30000;
    float yaw_theta_max_iout = 0;
    float yaw_theta_max_out = 20;
    float yaw_omega_max_iout = 0;  // 10000
    float yaw_omega_max_out = 30000;
    float* pitch_theta_pid_param = new float[3]{15, 0, 0};
    float* pitch_omega_pid_param = new float[3]{2000, 25, 4};
    float* yaw_theta_pid_param = new float[3]{15, 0, 0};
    float* yaw_omega_pid_param = new float[3]{5000, 0, 0};
    gimbalBasicPID.pitch_theta_pid = new control::ConstrainedPID(
        pitch_theta_pid_param, pitch_theta_max_iout, pitch_theta_max_out);
    gimbalBasicPID.pitch_omega_pid = new control::ConstrainedPID(
        pitch_omega_pid_param, pitch_omega_max_iout, pitch_omega_max_out);
    gimbalBasicPID.yaw_theta_pid =
        new control::ConstrainedPID(yaw_theta_pid_param, yaw_theta_max_iout, yaw_theta_max_out);
    gimbalBasicPID.yaw_omega_pid =
        new control::ConstrainedPID(yaw_omega_pid_param, yaw_omega_max_iout, yaw_omega_max_out);
}

control::gimbal_pid_t gimbalSpinPID;
void init_gimbalSpinData() {
    float pitch_theta_max_iout = 0;
    float pitch_theta_max_out = 10;
    float pitch_omega_max_iout = 10000;
    float pitch_omega_max_out = 30000;
    float yaw_theta_max_iout = 0;
    float yaw_theta_max_out = 10;
    float yaw_omega_max_iout = 5000;  // 10000
    float yaw_omega_max_out = 30000;
    float* pitch_theta_pid_param = new float[3]{15, 0, 0};
    float* pitch_omega_pid_param = new float[3]{2000, 25, 4};
    float* yaw_theta_pid_param = new float[3]{26, 0, 0.3};
    float* yaw_omega_pid_param = new float[3]{3600, 20, 0};
    gimbalSpinPID.pitch_theta_pid = new control::ConstrainedPID(
        pitch_theta_pid_param, pitch_theta_max_iout, pitch_theta_max_out);
    gimbalSpinPID.pitch_omega_pid = new control::ConstrainedPID(
        pitch_omega_pid_param, pitch_omega_max_iout, pitch_omega_max_out);
    gimbalSpinPID.yaw_theta_pid =
        new control::ConstrainedPID(yaw_theta_pid_param, yaw_theta_max_iout, yaw_theta_max_out);
    gimbalSpinPID.yaw_omega_pid =
        new control::ConstrainedPID(yaw_omega_pid_param, yaw_omega_max_iout, yaw_omega_max_out);
}