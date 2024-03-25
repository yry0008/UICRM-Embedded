/*###########################################################
 # Copyright (c) 2023-2024. BNU-HKBU UIC RoboMaster         #
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

#include "main.h"

#include "MotorCanBase.h"
#include "bsp_can.h"
#include "bsp_print.h"
#include "buzzer_notes.h"
#include "buzzer_task.h"
#include "chassis.h"
#include "cmsis_os.h"
#include "supercap.h"
#include "dbus.h"
#include "protocol.h"

bsp::CAN* can1 = nullptr;
bsp::CAN* can2 = nullptr;
driver::MotorCANBase* fl_motor = nullptr;
driver::MotorCANBase* fr_motor = nullptr;
driver::MotorCANBase* bl_motor = nullptr;
driver::MotorCANBase* br_motor = nullptr;


control::Chassis* chassis = nullptr;
communication::CanBridge* can_bridge = nullptr;

remote::DBUS* dbus = nullptr;
communication::Referee* referee = nullptr;
bsp::UART* referee_uart = nullptr;

void RM_RTOS_Init() {
    HAL_Delay(100);
    print_use_uart(&huart1);
    can2 = new bsp::CAN(&hcan2, false);
    can1 = new bsp::CAN(&hcan1, true);
    fl_motor = new driver::Motor3508(can1, 0x202);
    fr_motor = new driver::Motor3508(can1, 0x201);
    bl_motor = new driver::Motor3508(can1, 0x203);
    br_motor = new driver::Motor3508(can1, 0x204);

    control::ConstrainedPID::PID_Init_t omega_pid_init = {
        .kp = 2500,
        .ki = 3,
        .kd = 0,
        .max_out = 30000,
        .max_iout = 10000,
        .deadband = 0,                          // 死区
        .A = 3 * PI,                            // 变速积分所能达到的最大值为A+B
        .B = 2 * PI,                            // 启动变速积分的死区
        .output_filtering_coefficient = 0.1,    // 输出滤波系数
        .derivative_filtering_coefficient = 0,  // 微分滤波系数
        .mode = control::ConstrainedPID::Integral_Limit |       // 积分限幅
                control::ConstrainedPID::OutputFilter |         // 输出滤波
                control::ConstrainedPID::Trapezoid_Intergral |  // 梯形积分
                control::ConstrainedPID::ChangingIntegralRate,  // 变速积分
    };

    fl_motor->ReInitPID(omega_pid_init, driver::MotorCANBase::OMEGA);
    fl_motor->SetMode(driver::MotorCANBase::OMEGA);
    fl_motor->SetTransmissionRatio(19);

    fr_motor->ReInitPID(omega_pid_init, driver::MotorCANBase::OMEGA);
    fr_motor->SetMode(driver::MotorCANBase::OMEGA);
    fr_motor->SetTransmissionRatio(19);

    bl_motor->ReInitPID(omega_pid_init, driver::MotorCANBase::OMEGA);
    bl_motor->SetMode(driver::MotorCANBase::OMEGA);
    bl_motor->SetTransmissionRatio(19);

    br_motor->ReInitPID(omega_pid_init, driver::MotorCANBase::OMEGA);
    br_motor->SetMode(driver::MotorCANBase::OMEGA);
    br_motor->SetTransmissionRatio(19);



    driver::MotorCANBase* motors[control::FourWheel::motor_num];
    motors[control::FourWheel::front_left] = fl_motor;
    motors[control::FourWheel::front_right] = fr_motor;
    motors[control::FourWheel::back_left] = bl_motor;
    motors[control::FourWheel::back_right] = br_motor;

    control::chassis_t chassis_data;
    chassis_data.motors = motors;
    chassis_data.model = control::CHASSIS_MECANUM_WHEEL;
    chassis = new control::Chassis(chassis_data);

    chassis->SetMaxMotorSpeed(2 * PI * 12);


    referee_uart = new bsp::UART(&huart6);
    referee_uart->SetupTx(300);
    referee_uart->SetupRx(300);
    referee = new communication::Referee(referee_uart);

    HAL_Delay(300);
    init_buzzer();

    dbus = new remote::DBUS(&huart3);
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);

    osDelay(500);
    Buzzer_Sing(Mario);

    float ratio = 1.0f / 660.0f * 12 * PI;
    while (true) {
        if(!dbus->IsOnline()){
                chassis->Disable();
                osDelay(10);
                continue;
        }
        switch(dbus->swr){
            case remote::MID:
                chassis->Enable();
                chassis->SetSpeed(dbus->ch0 * ratio, dbus->ch1 * ratio, dbus->ch2 * ratio);
                break;
            case remote::UP:
                chassis->Enable();
                if(referee->game_status.game_progress==4)
                    chassis->SetSpeed(0, 0, 12 * PI);
                else
                    chassis->SetSpeed(0, 0, 0);
                break;
            case remote::DOWN:
                chassis->Disable();
                break;
        }
        // chassis->SetSpeed(dbus->ch0 * ratio, dbus->ch1 * ratio, dbus->ch2 * ratio);
        chassis->SetPower(true, referee->game_robot_status.chassis_power_limit, referee->power_heat_data.chassis_power, referee->power_heat_data.chassis_power_buffer);
        chassis->Update();
        osDelay(10);
    }
}
