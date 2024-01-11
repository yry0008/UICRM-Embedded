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

#pragma once

#include "can.h"
#include "motor.h"
#include "pid.h"
#include "utils.h"

namespace control {

    /**
     * @brief 不同云台的电机偏移角度、最大角度、最小角度
     * @note 除了最小角度由用户决定，其他都应该通过读取编码器值通过uart/gdb获取
     */
    /**
     * @brief offset, max, and proximity angles of different gimbals
     * @note except for proximity is determined by user, these should be obtained
     * by reading encoder values through uart/gdb
     */
    typedef struct {
        float pitch_offset_;     /* pitch offset angle (angle when muzzle is at vertical
                                    center) */
        float yaw_offset_;       /* yaw offset angle (angle when muzzle is at horizontal
                                    center) */
        float pitch_max_;        /* maximum pitch angle the gimbal can turn from center */
        float yaw_max_;          /* maximum yaw angle the gimbal can turn from center      */
        bool yaw_circle_ = true; /* yaw angle can circle or not */
        bool pitch_inverted = false;
        bool yaw_inverted = false;
        float pitch_eposition = 0;
        float yaw_eposition = 0;
    } gimbal_data_t;

    /**
     * @brief 云台PID参数
     */
    /**
     * @brief the pid of gimbal.
     */
    typedef struct {
        ConstrainedPID* pitch_theta_pid;
        ConstrainedPID* yaw_theta_pid;
        ConstrainedPID* pitch_omega_pid;
        ConstrainedPID* yaw_omega_pid;
    } gimbal_pid_t;

    /**
     * @brief 标准云台结构体
     */
    /**
     * @brief structure used when gimbal instance is initialized
     */
    typedef struct {
        driver::MotorCANBase* pitch_motor; /* pitch motor instance */
        driver::MotorCANBase* yaw_motor;   /* yaw motor instance   */
        gimbal_data_t data;                /* gimbal related constants */
        gimbal_pid_t pid;
    } gimbal_t;

    /**
     * @brief 云台类
     * @details 用于控制云台，标准云台类的实现是控制两个6020电机
     */
    /**
     * @brief wrapper class for gimbal
     */
    class Gimbal {
      public:
        /**
         * @brief 构造函数
         *
         * @param gimbal 用于初始化云台的结构体，参考gimbal_t
         */
        /**
         * @brief constructor for gimbal
         *
         * @param gimbal structure that used to initialize gimbal, refer to type
         * gimbal_t
         */
        Gimbal(gimbal_t gimbal);

        /**
         * @brief 析构函数
         */
        /**
         * @brief destructor for gimbal
         */
        ~Gimbal();

        /**
         * @brief 获取云台相关常量
         *
         * @return 参考gimbal_data_t
         */
        /**
         * @brief get gimbal related constants
         *
         * @return refer to gimbal_data_t
         */
        gimbal_data_t* GetData();

        /**
         * @brief 更新云台的PID参数
         * @param pid 新的PID参数
         */
        /**
         * @brief update the pid of the gimbal
         * @param pid the new PID of the gimbal
         */
        void SetPID(gimbal_pid_t pid);

        /**
         * @brief 计算当前云台的输出
         * @note 不会立即控制电机
         */
        /**
         * @brief calculate the output of the motors under current configuration
         * @note does not command the motor immediately
         */
        void Update();

        /**
         * @brief 基于当前传感器数据更新云台的输出
         * @param pitch 陀螺仪测量的pitch角度，范围为[-pi, pi]
         * @param yaw 陀螺仪测量的yaw角度，范围为[-pi, pi]
         */
        /**
         * @brief update the output of the motors based on current sensor data
         * @param pitch pitch angle measured by gyroscope, range is [-pi, pi]
         * @param yaw yaw angle measured by gyroscope, range is [-pi, pi]
         */
        void UpdateIMU(float pitch, float yaw);

        /**
         * @brief 将云台指向新的方向，是绝对于车身零点的角度
         * @param new_pitch 新的pitch角度
         * @param new_yaw 新的yaw角度
         */
        /**
         * @brief set motors to point to a new orientation
         *
         * @param new_pitch new pitch angled
         * @param new_yaw   new yaw angled
         */
        void TargetAbs(float new_pitch, float new_yaw);

        /**
         * @brief 将云台指向新的方向，是相对于当前方向的角度
         *
         * @param new_pitch 新的pitch角度
         * @param new_yaw 新的yaw角度
         */
        /**
         * @brief set motors to point to a new orientation
         *
         * @param new_pitch new pitch angled
         * @param new_yaw   new yaw angled
         */
        void TargetRel(float new_pitch, float new_yaw);

        /**
         * @brief 将云台指向新的方向，是绝对于车身零点的角度，并且忽略自身偏移量
         *
         * @param abs_pitch 新的pitch角度
         * @param abs_yaw 新的yaw角度
         */
        /**
         * @brief set motors to point to a new orientation
         *
         * @param abs_pitch new pitch max angle
         * @param abs_yaw   new yaw max angle
         */
        void TargetAbsWOffset(float abs_pitch, float abs_yaw);

        /**
         * @brief 更新云台的偏移量
         *
         * @param pitch_offset 新的pitch偏移量
         * @param yaw_offset 新的yaw偏移量
         */
        /**
         * @brief update the offset of the gimbal
         *
         * @param pitch_offset new pitch offset
         * @param yaw_offset   new yaw offset
         */
        void UpdateOffset(float pitch_offset, float yaw_offset);


        float expect_yaw_curr_ = 0;
        float expect_pitch_curr_ = 0;
        float expect_yaw_omega_ = 0;
        float expect_pitch_omega_ = 0;

      public:
        // acquired from user
        driver::MotorCANBase* pitch_motor_ = nullptr;
        driver::MotorCANBase* yaw_motor_ = nullptr;

        // pitch and yaw constants
        gimbal_data_t data_;

        // pitch and yaw pid
        ConstrainedPID* pitch_theta_pid_ = nullptr; /* pitch theta pid */
        ConstrainedPID* pitch_omega_pid_ = nullptr; /* pitch omega pid */
        ConstrainedPID* yaw_theta_pid_ = nullptr;   /* yaw theta pid   */
        ConstrainedPID* yaw_omega_pid_ = nullptr;   /* yaw omega pid   */

        // pitch and yaw angle
        float pitch_angle_; /* current gimbal pitch angle */
        float yaw_angle_;   /* current gimbal yaw angle   */

        // pitch and yaw limit
        float pitch_lower_limit_; /* pitch lower limit */
        float pitch_upper_limit_; /* pitch upper limit */
        float yaw_lower_limit_;   /* yaw lower limit   */
        float yaw_upper_limit_;   /* yaw upper limit   */

        // state detectors
        BoolEdgeDetector pitch_detector_; /* pitch pid mode toggle detector */
        BoolEdgeDetector yaw_detector_;   /* yaw pid mode toggle detector   */


    };

}  // namespace control
