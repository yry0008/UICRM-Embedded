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

#include "bsp_gpio.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "dbus.h"

void RM_RTOS_Init() {
    print_use_uart(&huart8);
}

void RM_RTOS_Default_Task(const void* arg) {
    UNUSED(arg);
    bsp::GPIO key(TRIG_KEY_GPIO_Port, TRIG_KEY_Pin);

    while (true) {
        set_cursor(0, 0);
        clear_screen();
        print("key: %d\n", key.Read() == true ? 1 : 0);
        osDelay(10);
    }
}