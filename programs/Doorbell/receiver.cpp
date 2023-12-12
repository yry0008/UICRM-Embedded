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
#include "bsp_os.h"

#include "buzzer.h"
#include "tim.h"

#define RX_SIGNAL (1 << 0)

const osThreadAttr_t receiveTaskAttribute = {.name = "receiveTask",
                                             .attr_bits = osThreadDetached,
                                             .cb_mem = nullptr,
                                             .cb_size = 0,
                                             .stack_mem = nullptr,
                                             .stack_size = 256 * 4,
                                             .priority = (osPriority_t)osPriorityNormal,
                                             .tz_module = 0,
                                             .reserved = 0};
static osThreadId_t receiveTaskHandle;

using Note = driver::BuzzerNote;

class ReceiverUART : public bsp::UART {
  public:
    using bsp::UART::UART;

  protected:
    /* notify application when rx data is pending read */
    void RxCompleteCallback() final {
        uint8_t* data;
        uint32_t length;
        length=this->Read<true>(&data);
        if(length==5) {
            if (strcmp((char*)data, "Song") == 0) {
                osThreadFlagsSet(receiveTaskHandle, RX_SIGNAL);
            }
        }
    }
};

static ReceiverUART* uart1=nullptr;

static driver::Buzzer* buzzer= nullptr;

driver::BuzzerNoteDelayed SevenEleven[] = {
    {Note::Do1M,250},
    {Note::Mi3M,250},
    {Note::So5M,250},
    {Note::Mi3M,250},
    {Note::Fa4M,250},
    {Note::La6M,250},
    {Note::Do1H,250},
    {Note::La6M,250},
    {Note::Re2H,500},
    {Note::So5M,500},
    {Note::Do1H,1000},
    {Note::Finish,0}
};

void singTask(void* arg) {
    UNUSED(arg);
    while (true) {
        uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, osWaitForever);
        if (flags & RX_SIGNAL) {  // unnecessary check
            buzzer->SingSong(SevenEleven,bsp::DelayMilliSecond);
        }
        osThreadFlagsClear(RX_SIGNAL);
    }
}



void RM_RTOS_Init() {
    uart1 = new ReceiverUART(&huart1);
    uart1->SetupTx(6);
    uart1->SetupRx(6);
    receiveTaskHandle = osThreadNew(singTask, nullptr,&receiveTaskAttribute);
    buzzer = new driver::Buzzer(&htim1, 1, 1000000);
}

void RM_RTOS_Default_Task(const void* arg) {
    UNUSED(arg);

    while (true) {
        osDelay(200);
    }
}