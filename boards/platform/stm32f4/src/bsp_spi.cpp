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
#include "bsp_spi.h"

#include "bsp_error_handler.h"

void RM_SPI_IRQHandler(SPI_HandleTypeDef* hspi) {
    bsp::SPI::CallbackWrapper(hspi);
}

namespace bsp {

    std::unordered_map<SPI_HandleTypeDef*, SPI*> SPI::ptr_map;

    /* get initialized uart_t instance given its hspi handle struct */
    SPI* SPI::FindInstance(SPI_HandleTypeDef* hspi) {
        const auto it = ptr_map.find(hspi);
        if (it == ptr_map.end()) {
            return nullptr;
        }

        return it->second;
    }
    SPI::SPI(spi_init_t init) {
        hspi_ = init.hspi;
        mode_ = init.mode;
        ptr_map[hspi_] = this;
        switch (mode_) {
            case SPI_MODE_BLOCKED:
                break;
            case SPI_MODE_INTURRUPT:
            case SPI_MODE_DMA:
                HAL_SPI_RegisterCallback(hspi_, HAL_SPI_TX_COMPLETE_CB_ID, RM_SPI_IRQHandler);
                HAL_SPI_RegisterCallback(hspi_, HAL_SPI_RX_COMPLETE_CB_ID, RM_SPI_IRQHandler);
                HAL_SPI_RegisterCallback(hspi_, HAL_SPI_TX_RX_COMPLETE_CB_ID, RM_SPI_IRQHandler);
        }
    }
    SPI::~SPI() {
        ptr_map.erase(hspi_);
    }

    void SPI::TransmitReceive(uint8_t* tx_data, uint8_t* rx_data, uint32_t length) {
        rx_size_ = length;
        rx_buffer_ = rx_data;
        switch (mode_) {
            case SPI_MODE_BLOCKED:
                HAL_SPI_TransmitReceive(hspi_, tx_data, rx_data, length, 1000);
                callback_(this);
                break;
            case SPI_MODE_INTURRUPT:
                HAL_SPI_TransmitReceive_IT(hspi_, tx_data, rx_data, length);
                break;
            case SPI_MODE_DMA:
                HAL_SPI_TransmitReceive_DMA(hspi_, tx_data, rx_data, length);
                break;
            default:
                RM_ASSERT_TRUE(false, "Invalid SPI mode");
        }
    }
    void SPI::Transmit(uint8_t* tx_data, uint32_t length) {
        switch (mode_) {
            case SPI_MODE_BLOCKED:
                HAL_SPI_Transmit(hspi_, tx_data, length, 1000);
                break;
            case SPI_MODE_INTURRUPT:
                HAL_SPI_Transmit_IT(hspi_, tx_data, length);
                break;
            case SPI_MODE_DMA:
                HAL_SPI_Transmit_DMA(hspi_, tx_data, length);
                break;
            default:
                RM_ASSERT_TRUE(false, "Invalid SPI mode");
        }
    }
    void SPI::Receive(uint8_t* rx_data, uint32_t length) {
        rx_size_ = length;
        rx_buffer_ = rx_data;
        switch (mode_) {
            case SPI_MODE_BLOCKED:
                HAL_SPI_Receive(hspi_, rx_data, length, 1000);
                break;
            case SPI_MODE_INTURRUPT:
                HAL_SPI_Receive_IT(hspi_, rx_data, length);
                break;
            case SPI_MODE_DMA:
                HAL_SPI_Receive_DMA(hspi_, rx_data, length);
                break;
            default:
                RM_ASSERT_TRUE(false, "Invalid SPI mode");
        }
    }

    bool SPI::IsBusy() {
        return HAL_SPI_GetState(hspi_) == HAL_SPI_STATE_BUSY ||
               HAL_SPI_GetState(hspi_) == HAL_SPI_STATE_BUSY_TX ||
               HAL_SPI_GetState(hspi_) == HAL_SPI_STATE_BUSY_RX ||
               HAL_SPI_GetState(hspi_) == HAL_SPI_STATE_BUSY_TX_RX;
    }

    void SPI::Abort() {
        HAL_SPI_Abort_IT(hspi_);
    }

    void SPI::RegisterCallback(spi_rx_callback_t callback) {
        callback_ = callback;
    }
    void SPI::CallbackWrapper(SPI_HandleTypeDef* hspi) {
        SPI* instance = FindInstance(hspi);
        if (instance == nullptr) {
            return;
        }
        instance->callback_(instance);
    }
    bool SPI::IsDMA() {
        return mode_ == SPI_MODE_DMA;
    }
    void SPI::SetMode(spi_mode_e mode) {
        mode_ = mode;
        switch (mode_) {
            case SPI_MODE_BLOCKED:
                HAL_SPI_UnRegisterCallback(hspi_, HAL_SPI_TX_COMPLETE_CB_ID);
                HAL_SPI_UnRegisterCallback(hspi_, HAL_SPI_RX_COMPLETE_CB_ID);
                HAL_SPI_UnRegisterCallback(hspi_, HAL_SPI_TX_RX_COMPLETE_CB_ID);
                break;
            case SPI_MODE_INTURRUPT:
            case SPI_MODE_DMA:
                HAL_SPI_RegisterCallback(hspi_, HAL_SPI_TX_COMPLETE_CB_ID, RM_SPI_IRQHandler);
                HAL_SPI_RegisterCallback(hspi_, HAL_SPI_RX_COMPLETE_CB_ID, RM_SPI_IRQHandler);
                HAL_SPI_RegisterCallback(hspi_, HAL_SPI_TX_RX_COMPLETE_CB_ID, RM_SPI_IRQHandler);
                break;
        }
    }

    SPIDevice::SPIDevice(spi_device_init_t init) : spi_(init.spi), cs_(init.cs) {
    }
    void SPIDevice::RegisterCallback(spi_device_rx_callback_t callback, void* args) {
        callback_ = callback;
        args_ = args;
    }
    void SPIDevice::PrepareTransmit() {
        cs_->Low();
    }
    void SPIDevice::FinishTransmit() {
        cs_->High();
    }
    bool SPIDevice::IsTransmitting() {
        return cs_->Read() == 0;
    }
    void SPIDevice::CallbackWrapper() {
        callback_(args_);
    }

    std::unordered_map<SPI*, SPIMaster*> SPIMaster::ptr_map;

    SPIMaster::SPIMaster(spi_master_init_t init) {
        spi_ = init.spi;
        spi_->RegisterCallback(CallbackWrapper);
        ptr_map[spi_] = this;
    }

    SPIMaster::~SPIMaster() {
    }

    spi_master_status_e SPIMaster::Transmit(SPIDevice* device, uint8_t* tx_data, uint32_t length) {
        if (spi_->IsBusy()) {
            busy_count_++;
//            if (busy_count_ > 10) {
//                spi_->Abort();
//                busy_count_ = 0;
//            }
            return SPI_MASTER_STATUS_BUSY;
        }
        busy_count_ = 0;
        if (auto_cs_) {
            device->PrepareTransmit();
        }
        spi_->Transmit(tx_data, length);
        return SPI_MASTER_STATUS_OK;
    }

    spi_master_status_e SPIMaster::Receive(SPIDevice* device, uint8_t* rx_data, uint32_t length) {
        if (spi_->IsBusy()) {
            busy_count_++;
//            if (busy_count_ > 10) {
//                spi_->Abort();
//                busy_count_ = 0;
//            }
            return SPI_MASTER_STATUS_BUSY;
        }
        busy_count_ = 0;
        if (auto_cs_) {
            device->PrepareTransmit();
        }
        spi_->Receive(rx_data, length);
        return SPI_MASTER_STATUS_OK;
    }

    spi_master_status_e SPIMaster::TransmitReceive(SPIDevice* device, uint8_t* tx_data,
                                                   uint8_t* rx_data, uint32_t length) {
        if (spi_->IsBusy()) {
            busy_count_++;
//            if (busy_count_ > 10) {
//                spi_->Abort();
//                busy_count_ = 0;
//            }
            return SPI_MASTER_STATUS_BUSY;
        }
        busy_count_ = 0;
        if (auto_cs_) {
            device->PrepareTransmit();
        }
        spi_->TransmitReceive(tx_data, rx_data, length);
        return SPI_MASTER_STATUS_OK;
    }

    SPIDevice* SPIMaster::NewDevice(GPIO* cs) {
        if (device_count_ >= SPI_MAX_DEVICE) {
            RM_ASSERT_TRUE(false, "Too many SPI devices");
        }
        spi_device_init_t init;
        init.spi = spi_;
        init.cs = cs;
        device_[device_count_++] = new SPIDevice(init);
        return device_[device_count_ - 1];
    }

    void SPIMaster::AddDevice(SPIDevice* device) {
        if (device_count_ >= SPI_MAX_DEVICE) {
            RM_ASSERT_TRUE(false, "Too many SPI devices");
        }
        device_[device_count_++] = device;
    }

    void SPIMaster::CallbackWrapper(SPI* spi) {
        SPIMaster* instance = FindInstance(spi);
        if (instance == nullptr) {
            return;
        }
        instance->CallbackWrapper();
    }
    SPIMaster* SPIMaster::FindInstance(SPI* spi) {
        const auto it = ptr_map.find(spi);
        if (it == ptr_map.end()) {
            return nullptr;
        }

        return it->second;
    }
    void SPIMaster::CallbackWrapper() {
        for (int i = 0; i < device_count_; i++) {
            if (device_[i]->IsTransmitting()) {
                if (auto_cs_) {
                    device_[i]->FinishTransmit();
                }
                device_[i]->CallbackWrapper();
            }
        }
    }
    void SPIMaster::SetMode(spi_mode_e mode) {
        spi_->SetMode(mode);
    }
    void SPIMaster::SetAutoCS(bool auto_cs) {
        auto_cs_ = auto_cs;
    }
    bool SPIMaster::IsBusy() {
        return spi_->IsBusy();
    }

}  // namespace bsp
