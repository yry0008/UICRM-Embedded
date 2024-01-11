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
#include "bsp_gpio.h"
#include "cmsis_os2.h"
#include "imu_task.h"
#include "main.h"
#include "motor.h"
#include "public_port.h"
#include "referee_task.h"
#include "remote_task.h"
#include "selftest_task.h"
#include "user_define.h"
#include "utils.h"
extern osThreadId_t shootTaskHandle;
const osThreadAttr_t shootTaskAttribute = {.name = "shootTask",
                                           .attr_bits = osThreadDetached,
                                           .cb_mem = nullptr,
                                           .cb_size = 0,
                                           .stack_mem = nullptr,
                                           .stack_size = 512 * 4,
                                           .priority = (osPriority_t)osPriorityNormal,
                                           .tz_module = 0,
                                           .reserved = 0};
extern driver::MotorCANBase* steering_motor;
void shootTask(void* arg);
void init_shoot();
void kill_shoot();