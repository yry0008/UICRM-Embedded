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
#include "cmsis_os2.h"

#include "string.h"

#include "bsp_uart.h"
#include "bsp_gpio.h"

#include "buzzer.h"
#include "tim.h"

using Note = driver::BuzzerNote;

static bsp::GPIO* key = nullptr;
static bsp::UART* uart1 = nullptr;

void RM_RTOS_Init() {
    key= new bsp::GPIO(KEY_GPIO_Port,KEY_Pin);
    uart1 = new bsp::UART(&huart1);
    uart1->SetupTx(100);
    uart1->SetupRx(100);
}

void RM_RTOS_Default_Task(const void* arg) {
    UNUSED(arg);

    while (true) {
        if(key->Read()==0){
            const char data[] = "Song";
            uart1->Write((uint8_t*)data,strlen(data));
        }else{
            const char data[] = "Stop";
            uart1->Write((uint8_t*)data,strlen(data));
        }
        //print("key: %d\n", key.Read() == true ? 1 : 0);
        osDelay(50);
    }
}