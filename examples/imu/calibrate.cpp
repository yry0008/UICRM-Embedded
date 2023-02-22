#include "bsp_gpio.h"
#include "bsp_imu.h"
#include "bsp_os.h"
#include "bsp_usb.h"
#include "cmsis_os.h"
#include "main.h"

#define ONBOARD_IMU_SPI hspi5
#define ONBOARD_IMU_CS_GROUP GPIOF
#define ONBOARD_IMU_CS_PIN GPIO_PIN_6

typedef struct {
  char header;
  bsp::vec3f_t acce;
  bsp::vec3f_t gyro;
  bsp::vec3f_t mag;
  char terminator;
} __attribute__((packed)) imu_data_t;

static bsp::MPU6500 *imu = nullptr;
static bsp::VirtualUSB *usb = nullptr;

static imu_data_t imu_data;

void RM_RTOS_Init(void) {
  bsp::SetHighresClockTimer(&htim2);
  imu_data.header = 's';
  imu_data.terminator = '\0';
}

void RM_RTOS_Default_Task(const void *arguments) {
  UNUSED(arguments);

  bsp::GPIO chip_select(ONBOARD_IMU_CS_GROUP, ONBOARD_IMU_CS_PIN);
  imu = new bsp::MPU6500(&ONBOARD_IMU_SPI, chip_select, MPU6500_IT_Pin);
  usb = new bsp::VirtualUSB();
  usb->SetupTx(sizeof(imu_data_t));
  osDelay(10);

  while (true) {
    imu_data.acce = imu->acce;
    imu_data.gyro = imu->gyro;
    imu_data.mag = imu->mag;
    usb->Write((uint8_t *)&imu_data, sizeof(imu_data));
    osDelay(10);
  }
}
