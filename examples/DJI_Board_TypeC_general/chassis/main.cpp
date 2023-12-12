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

#include "main.h"

#include "bsp_print.h"
#include "chassis.h"
#include "cmsis_os.h"
#include "dbus.h"

bsp::CAN* can = nullptr;
driver::MotorCANBase* fl_motor = nullptr;
driver::MotorCANBase* fr_motor = nullptr;
driver::MotorCANBase* bl_motor = nullptr;
driver::MotorCANBase* br_motor = nullptr;

control::Chassis* chassis = nullptr;
remote::DBUS* dbus = nullptr;

void RM_RTOS_Init() {
    HAL_Delay(200);
    print_use_uart(&huart1);
    can = new bsp::CAN(&hcan2, 0x201, false);
    fl_motor = new driver::Motor3508(can, 0x202);
    fr_motor = new driver::Motor3508(can, 0x201);
    bl_motor = new driver::Motor3508(can, 0x203);
    br_motor = new driver::Motor3508(can, 0x204);

    driver::MotorCANBase* motors[control::FourWheel::motor_num];
    motors[control::FourWheel::front_left] = fl_motor;
    motors[control::FourWheel::front_right] = fr_motor;
    motors[control::FourWheel::back_left] = bl_motor;
    motors[control::FourWheel::back_right] = br_motor;

    control::chassis_t chassis_data;
    chassis_data.motors = motors;
    chassis_data.model = control::CHASSIS_MECANUM_WHEEL;
    chassis = new control::Chassis(chassis_data);

    dbus = new remote::DBUS(&huart3);
    HAL_Delay(300);
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);

    osDelay(500);  // DBUS initialization needs time

    driver::MotorCANBase* motors[] = {fl_motor, fr_motor, bl_motor, br_motor};

    while (true) {
        chassis->SetSpeed(dbus->ch0, dbus->ch1, dbus->ch2);

        // Kill switch
        if (dbus->swl == remote::UP || dbus->swl == remote::DOWN) {
            RM_ASSERT_TRUE(false, "Operation killed");
        }

        chassis->Update(false, 30, 20, 60);
        driver ::MotorCANBase::TransmitOutput(motors, 4);
        osDelay(10);
    }
}
