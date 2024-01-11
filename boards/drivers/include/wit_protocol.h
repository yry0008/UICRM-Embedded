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
// clang-format off
#include "main.h"
#include "arm_math.h"
// clang-format on
#include "bsp_uart.h"

namespace imu {

    typedef struct {
        uint8_t* data;
        int length;
    } wit_package_t;

    typedef void (*wit_read_callback_t)();

    enum wit_rx_code_e {
        WIT_RX_TIME = 0x50,
        WIT_RX_ACCEL = 0x51,
        WIT_RX_GYRO = 0x52,
        WIT_RX_INS = 0x53,
        WIT_RX_MAG = 0x54,
        WIT_RX_QUAT = 0x59,
        WIT_RX_PORT_STATUS = 0x55,
        WIT_RX_READ_REG = 0x5F,
    };

    class WITUART {
      public:
        WITUART(bsp::UART* uart);
        ~WITUART() = default;
        float mag_[3] = {0};
        float gyro_[3] = {0};
        float accel_[3] = {0};
        float INS_angle[3] = {0};
        float temp_ = 0;
        uint16_t version_ = 0;
        uint16_t port_status_[4] = {0};
        float quat_[4] = {0};
        // TODO: Read GPS Data

        void Update(bool fromISR = false);

        void Unlock();

        void Lock();

        void ReadReg(uint8_t reg, uint8_t func, uint16_t* data);

        void RegisterReadCallback(wit_read_callback_t callback);

        void WriteReg(uint8_t reg, uint8_t* data);

      private:
        bsp::UART* uart_;
        wit_read_callback_t read_callback_;
        uint16_t* read_reg_data_;
    };
}  // namespace imu