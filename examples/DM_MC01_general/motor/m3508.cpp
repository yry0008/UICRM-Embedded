#include "bsp_gpio.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "main.h"
#include "motor.h"

#define KEY_GPIO_GROUP KEY_GPIO_Port
#define KEY_GPIO_PIN KEY_Pin

// Refer to typeA datasheet for channel detail
static bsp::CAN* can1 = nullptr;
static control::Motor3508* motor1 = nullptr;

void RM_RTOS_Init() {
    print_use_uart(&huart4);
    can1 = new bsp::CAN(&hcan1, 0x201, true);
    motor1 = new control::Motor3508(can1, 0x201);

    // Snail need to be run at idle throttle for some
    HAL_Delay(1000);
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
    bsp::GPIO key(KEY_GPIO_GROUP, KEY_GPIO_PIN);
    bsp::GPIO power_output(Power_OUT1_EN_GPIO_Port, Power_OUT1_EN_Pin);
    osDelay(1000);
    power_output.High();
    osDelay(1000);
    control::MotorCANBase* motors[] = {motor1};
    int current = 0;
    while (true) {
        set_cursor(0, 0);
        clear_screen();
        if (key.Read() == 0) {
            osDelay(30);
            if (key.Read() == 1)
                continue;
            while (key.Read() == 0) {
                osDelay(30);
            }
            if (current == 0) {
                current = 10000;
                motor1->SetOutput(current);
            } else {
                current = 0;
                motor1->SetOutput(current);
            }
        }
        motor1->PrintData();
        control::MotorCANBase::TransmitOutput(motors, 1);
        osDelay(50);
    }
}