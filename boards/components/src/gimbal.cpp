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

#include "gimbal.h"

#include "motor.h"
#include "utils.h"

namespace control {

    Gimbal::Gimbal(gimbal_t gimbal)
        : pitch_detector_(BoolEdgeDetector(false)), yaw_detector_(BoolEdgeDetector(false)) {
        // acquired from user
        pitch_motor_ = gimbal.pitch_motor;
        yaw_motor_ = gimbal.yaw_motor;
        data_ = gimbal.data;

        pitch_theta_pid_ = gimbal.pid.pitch_theta_pid;
        pitch_omega_pid_ = gimbal.pid.pitch_omega_pid;
        yaw_theta_pid_ = gimbal.pid.yaw_theta_pid;
        yaw_omega_pid_ = gimbal.pid.yaw_omega_pid;

        pitch_angle_ = data_.pitch_offset_;
        yaw_angle_ = data_.yaw_offset_;

        pitch_lower_limit_ = wrap<float>(data_.pitch_offset_ - data_.pitch_max_, 0, 2 * PI);
        pitch_upper_limit_ = wrap<float>(data_.pitch_offset_ + data_.pitch_max_, 0, 2 * PI);
        yaw_lower_limit_ = wrap<float>(data_.yaw_offset_ - data_.yaw_max_, 0, 2 * PI);
        yaw_upper_limit_ = wrap<float>(data_.yaw_offset_ + data_.yaw_max_, 0, 2 * PI);
    }

    Gimbal::~Gimbal() {
    }

    gimbal_data_t* Gimbal::GetData() {
        return &data_;
    }

    void Gimbal::SetPID(gimbal_pid_t pid) {
        pitch_theta_pid_ = pid.pitch_theta_pid;
        pitch_theta_pid_->Reset();
        pitch_omega_pid_ = pid.pitch_omega_pid;
        pitch_omega_pid_->Reset();
        yaw_theta_pid_ = pid.yaw_theta_pid;
        yaw_theta_pid_->Reset();
        yaw_omega_pid_ = pid.yaw_omega_pid;
        yaw_omega_pid_->Reset();
    }

    void Gimbal::Update() {
        float pt_diff = pitch_motor_->GetThetaDelta(pitch_angle_);
        pt_diff = wrap<float>(pt_diff, -PI, PI);

        if (abs(pt_diff) < data_.pitch_eposition) {
            pt_diff = 0;
        }
        float pt_out = pitch_theta_pid_->ComputeOutput(pt_diff);

        expect_pitch_omega_ = pt_out;

        float po_in = pitch_motor_->GetOmegaDelta(pt_out);
        float po_out = pitch_omega_pid_->ComputeConstrainedOutput(po_in);

        expect_pitch_curr_ = po_out;

        // 角度差 -> 目标角速度
        float yt_diff = yaw_motor_->GetThetaDelta(yaw_angle_);
        yt_diff = wrap<float>(yt_diff, -PI, PI);

        if (abs(yt_diff) < data_.yaw_eposition) {
            yt_diff = 0;
        }

        float yt_out = yaw_theta_pid_->ComputeOutput(yt_diff);
        expect_yaw_omega_ = yt_out;

        // 角速度差 -> 目标电流
        float yo_in = yaw_motor_->GetOmegaDelta(yt_out);
        float yo_out = yaw_omega_pid_->ComputeConstrainedOutput(yo_in);

        expect_yaw_curr_ = yo_out;

        pitch_motor_->SetOutput(po_out);
        yaw_motor_->SetOutput(yo_out);
    }

    void Gimbal::UpdateIMU(float pitch, float yaw) {
        float pt_diff = pitch_angle_ - data_.pitch_offset_ - pitch;
        float actual_pitch_angle = pitch_motor_->GetTheta();
        float new_pitch_diff = wrapping_clip<float>(
            pt_diff + actual_pitch_angle, pitch_lower_limit_, pitch_upper_limit_, 0, 2 * PI);
        new_pitch_diff = new_pitch_diff - actual_pitch_angle;
        if (pt_diff != new_pitch_diff) {
            pitch_angle_ = wrap<float>(pitch_angle_ + new_pitch_diff - pt_diff, 0, 2 * PI);
        }
        pt_diff = wrap<float>(new_pitch_diff, -PI, PI);

        if (abs(pt_diff) < data_.pitch_eposition) {
            pt_diff = 0;
        }

        float pt_out = pitch_theta_pid_->ComputeOutput(pt_diff);

        expect_pitch_omega_ = pt_out;

        float po_in = pitch_motor_->GetOmegaDelta(pt_out);
        float po_out = pitch_omega_pid_->ComputeConstrainedOutput(po_in);


        expect_pitch_curr_ = po_out;

        float yt_diff = yaw_angle_ - data_.yaw_offset_ - yaw;
        if (!data_.yaw_circle_) {
            float actual_yaw_angle = yaw_motor_->GetTheta();
            float new_yaw_diff = wrapping_clip<float>(yt_diff + actual_yaw_angle, yaw_lower_limit_,
                                                      yaw_upper_limit_, 0, 2 * PI);
            new_yaw_diff = new_yaw_diff - actual_yaw_angle;
            if (yt_diff != new_yaw_diff) {
                yaw_angle_ = wrap<float>(yaw_angle_ + new_yaw_diff - yt_diff, 0, 2 * PI);
            }
            yt_diff = wrap<float>(new_yaw_diff, -PI, PI);
        } else {
            yt_diff = wrap<float>(yt_diff, -PI, PI);
        }

        if (abs(yt_diff) < data_.yaw_eposition) {
            yt_diff = 0;
        }

        float yt_out = yaw_theta_pid_->ComputeOutput(yt_diff);

        expect_yaw_omega_ = yt_out;

        float yo_in = yaw_motor_->GetOmegaDelta(yt_out);
        float yo_out = yaw_omega_pid_->ComputeConstrainedOutput(yo_in);

        expect_yaw_curr_ = yo_out;

        pitch_motor_->SetOutput(po_out);
        yaw_motor_->SetOutput(yo_out);
    }

    void Gimbal::TargetAbs(float abs_pitch, float abs_yaw) {
        if (data_.pitch_inverted)
            abs_pitch = -abs_pitch;
        if (data_.yaw_inverted)
            abs_yaw = -abs_yaw;
        float clipped_pitch = clip<float>(abs_pitch, -data_.pitch_max_, data_.pitch_max_);
        float clipped_yaw = clip<float>(abs_yaw, -data_.yaw_max_, data_.yaw_max_);
        pitch_angle_ = wrapping_clip<float>(clipped_pitch + data_.pitch_offset_, pitch_lower_limit_,
                                            pitch_upper_limit_, 0, 2 * PI);
        yaw_angle_ = wrapping_clip<float>(clipped_yaw + data_.yaw_offset_, yaw_lower_limit_,
                                          yaw_upper_limit_, 0, 2 * PI);
    }

    void Gimbal::TargetAbsWOffset(float abs_pitch, float abs_yaw) {
        if (data_.pitch_inverted)
            abs_pitch = -abs_pitch;
        if (data_.yaw_inverted)
            abs_yaw = -abs_yaw;
        float clipped_pitch = clip<float>(abs_pitch, -data_.pitch_max_, data_.pitch_max_);
        float clipped_yaw = clip<float>(abs_yaw, -data_.yaw_max_, data_.yaw_max_);
        pitch_angle_ =
            wrapping_clip<float>(clipped_pitch, pitch_lower_limit_, pitch_upper_limit_, 0, 2 * PI);
        yaw_angle_ =
            wrapping_clip<float>(clipped_yaw, yaw_lower_limit_, yaw_upper_limit_, 0, 2 * PI);
    }

    void Gimbal::TargetRel(float rel_pitch, float rel_yaw) {
        if (data_.pitch_inverted)
            rel_pitch = -rel_pitch;
        if (data_.yaw_inverted)
            rel_yaw = -rel_yaw;
        pitch_angle_ = wrap<float>(pitch_angle_ + rel_pitch, 0, 2 * PI);
        yaw_angle_ = wrap<float>(yaw_angle_ + rel_yaw, 0, 2 * PI);
    }

    void Gimbal::UpdateOffset(float pitch_offset, float yaw_offset) {
        data_.pitch_offset_ = wrap<float>(pitch_offset + data_.pitch_offset_, 0, 2 * PI);
        data_.yaw_offset_ = wrap<float>(yaw_offset + data_.yaw_offset_, 0, 2 * PI);
    }

}  // namespace control
