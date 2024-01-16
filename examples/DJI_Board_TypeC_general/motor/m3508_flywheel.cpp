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

#include "bsp_gpio.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "main.h"
#include "motor.h"
#include "dbus.h"

#define KEY_GPIO_GROUP KEY_GPIO_Port
#define KEY_GPIO_PIN KEY_Pin

// Refer to typeC datasheet for channel detail
static bsp::CAN* can1 = nullptr;
static bsp::CAN* can2 = nullptr;
static driver::Motor3508* motor1 = nullptr;
static driver::Motor3508* motor2 = nullptr;
static driver::Motor3508* motor3 = nullptr;
static driver::Motor3508* motor4 = nullptr;
static float* pid1_param = nullptr;
static float* pid2_param = nullptr;
static float* pid3_param = nullptr;
static float* pid4_param = nullptr;
static driver::FlyWheelMotor* flywheel1 = nullptr;
static driver::FlyWheelMotor* flywheel2 = nullptr;
static driver::FlyWheelMotor* flywheel3 = nullptr;
static driver::FlyWheelMotor* flywheel4 = nullptr;
static remote::DBUS* dbus = nullptr;

static driver::Motor6020* yaw_motor = nullptr;
static driver::Motor3508* pitch_motor = nullptr;
static driver::Motor3508* shoot_motor = nullptr;

static float* pitch_param = nullptr;
static float* shooter_param = nullptr;
static driver::FlyWheelMotor* pitch = nullptr;
static driver::FlyWheelMotor* shooter = nullptr;

void RM_RTOS_Init() {
    print_use_uart(&huart1);
    can1 = new bsp::CAN(&hcan1, true);
    can2 = new bsp::CAN(&hcan2, false);
    motor1 = new driver::Motor3508(can1, 0x201);
    motor2 = new driver::Motor3508(can1, 0x202);
    motor3 = new driver::Motor3508(can1, 0x203);
    motor4 = new driver::Motor3508(can1, 0x204);

    pid1_param = new float[3]{250, 20, 0.15};
    pid2_param = new float[3]{250, 20, 0.15};
    pid3_param = new float[3]{250, 20, 0.15};
    pid4_param = new float[3]{250, 20, 0.15};
    driver::flywheel_t flywheel1_data = {
        .motor = motor1,
        .max_speed = 400 * PI,
        .omega_pid_param = pid1_param,
        .is_inverted = true,
    };
    driver::flywheel_t flywheel2_data = {
        .motor = motor2,
        .max_speed = 400 * PI,
        .omega_pid_param = pid2_param,
        .is_inverted = false,
    };
    driver::flywheel_t flywheel3_data = {
        .motor = motor3,
        .max_speed = 400 * PI,
        .omega_pid_param = pid3_param,
        .is_inverted = true,
    };
    driver::flywheel_t flywheel4_data = {
        .motor = motor4,
        .max_speed = 400 * PI,
        .omega_pid_param = pid4_param,
        .is_inverted = false,
    };
    flywheel1 = new driver::FlyWheelMotor(flywheel1_data);
    flywheel2 = new driver::FlyWheelMotor(flywheel2_data);
    flywheel3 = new driver::FlyWheelMotor(flywheel3_data);
    flywheel4 = new driver::FlyWheelMotor(flywheel4_data);
    // Snail need to be run at idle throttle for some
    HAL_Delay(1000);
    dbus = new remote::DBUS(&huart3);
    yaw_motor = new driver::Motor6020(can2,0x205,false);
    pitch_motor = new driver::Motor3508(can2,0x206);
    shoot_motor = new driver::Motor3508(can2,0x207);
    pitch_param = new float[3]{250, 1, 0.15};
    shooter_param = new float[3]{250, 1, 0.15};
    driver::flywheel_t pitch_data = {
        .motor = pitch_motor,
        .max_speed = 400 * PI,
        .omega_pid_param = pitch_param,
        .is_inverted = false,
    };
    driver::flywheel_t shoot_data = {
        .motor = shoot_motor,
        .max_speed = 400 * PI,
        .omega_pid_param = shooter_param,
        .is_inverted = false,
    };
    pitch = new driver::FlyWheelMotor(pitch_data);
    shooter = new driver::FlyWheelMotor(shoot_data);
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
    driver::MotorCANBase* motors[] = {motor1, motor2,motor3,motor4};
    driver::MotorCANBase* motors2[] = {pitch_motor,yaw_motor,shoot_motor};
    float speed1 = 0,speed2=0;
    while (true) {
        set_cursor(0, 0);
        clear_screen();
        if (dbus->swl == remote::UP) {
            speed1 = 200.0f / 6 * 5 * PI;
                flywheel1->SetSpeed(speed1);
                flywheel2->SetSpeed(speed1);
            speed2 = 300.0f / 6 * 5 * PI;
                flywheel3->SetSpeed(speed2);
                flywheel4->SetSpeed(speed2);
//                motor1->SetOutput(30000);
//                motor2->SetOutput(30000);
//                motor3->SetOutput(30000);
//                motor4->SetOutput(30000);
            } else {
                speed1 = 0;
                flywheel1->SetSpeed(speed1);
                flywheel2->SetSpeed(speed1);
                flywheel3->SetSpeed(speed1);
                flywheel4->SetSpeed(speed1);
//                motor1->SetOutput(0);
//                motor2->SetOutput(0);
//                motor3->SetOutput(0);
//                motor4->SetOutput(0);

        }
        flywheel1->CalcOutput();
        flywheel2->CalcOutput();
        flywheel3->CalcOutput();
        flywheel4->CalcOutput();

        driver::MotorCANBase::TransmitOutput(motors, 4);
        yaw_motor->SetOutput(dbus->ch0*35);
        pitch->SetSpeed(-dbus->ch3/2.2f*PI);
        pitch->CalcOutput();
        shooter->SetSpeed(dbus->ch1/2.2f*PI);
        shooter->CalcOutput();
        driver::MotorCANBase::TransmitOutput(motors2, 3);
        osDelay(1);
    }
}
