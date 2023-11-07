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
#include "bsp_can.h"

typedef enum
{
    NormalMode,
    OnlyChangeMode,
    OnlyCompensateMode
} Power_Limit_mode_t;

typedef union
{
    uint8_t value;
    struct
    {
        FunctionalState CupEnable : 1;
        Power_Limit_mode_t power_limit_mode : 2;
        FunctionalState RESERVER : 5;
    } flag;
} SupercapMessage_t;


bsp::CAN* can1 = nullptr;

void RM_RTOS_Init(void) {
    can1 = new bsp::CAN(&hcan1, 0x201, true);
}

void RM_RTOS_Default_Task(const void* argument) {
    UNUSED(argument);
    osDelay(2000);
    SupercapMessage_t msg;
    msg.flag.CupEnable=ENABLE;
    msg.flag.power_limit_mode=NormalMode;
    uint8_t data[] = {120,0,250,0,150,0,msg.value,50};
    can1->Transmit(0x02f,data,8);
    while (true) {
        osDelay(3000);
        can1->Transmit(0x02f,data,8);
    }
}
