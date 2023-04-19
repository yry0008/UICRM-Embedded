#include "main.h"

#include "bsp_os.h"
#include "bsp_print.h"
#include "buzzer_notes.h"
#include "buzzer_task.h"
#include "chassis_task.h"
#include "cmsis_os.h"
#include "gimbal_task.h"
#include "imu_task.h"
#include "public_port.h"
#include "referee_task.h"
#include "remote_task.h"
#include "selftest_task.h"
#include "shoot_task.h"
#include "ui_task.h"
#include "user_define.h"
void RM_RTOS_Init(void) {
    bsp::SetHighresClockTimer(&htim5);
    print_use_uart(&huart1);
    init_can();
    init_batt();
    init_imu();
    init_buzzer();
    init_referee();
    init_remote();
    init_shoot();
    init_gimbal();
    init_chassis();
    init_ui();
}

void RM_RTOS_Threads_Init(void) {
    imuTaskHandle = osThreadNew(imuTask, nullptr, &imuTaskAttribute);
    buzzerTaskHandle = osThreadNew(buzzerTask, nullptr, &buzzerTaskAttribute);
    refereeTaskHandle = osThreadNew(refereeTask, nullptr, &refereeTaskAttribute);
    remoteTaskHandle = osThreadNew(remoteTask, nullptr, &remoteTaskAttribute);
    gimbalTaskHandle = osThreadNew(gimbalTask, nullptr, &gimbalTaskAttribute);
    chassisTaskHandle = osThreadNew(chassisTask, nullptr, &chassisTaskAttribute);
    shootTaskHandle = osThreadNew(shootTask, nullptr, &shootTaskAttribute);
    selftestTaskHandle = osThreadNew(selftestTask, nullptr, &selftestTaskAttribute);
    if (ENABLE_UI)
        uiTaskHandle = osThreadNew(uiTask, nullptr, &uiTaskAttribute);
}

void RM_RTOS_Default_Task(const void* arg) {
    UNUSED(arg);
    osDelay(3000);
    Buzzer_Sing(DJI);
    char s[50];
    while (true) {
        set_cursor(0, 0);
        clear_screen();
        size_t availmem = xPortGetFreeHeapSize();
        size_t minavailmem = xPortGetMinimumEverFreeHeapSize();
        size_t totalmem = configTOTAL_HEAP_SIZE;
        print("Free Heap:%d/%d Bytes\r\n", availmem, totalmem);
        print("Min Free Heap:%d Bytes\r\n", minavailmem);
        switch (remote_mode) {
            case REMOTE_MODE_PREPARE:
                strcpy(s, "PREPARE");
                break;
            case REMOTE_MODE_STOP:
                strcpy(s, "STOP");
                break;
            case REMOTE_MODE_KILL:
                strcpy(s, "KILL");
                break;
            case REMOTE_MODE_FOLLOW:
                strcpy(s, "MANUAL");
                break;
            case REMOTE_MODE_SPIN:
                strcpy(s, "SPIN");
                break;
            default:
                strcpy(s, "UNKNOWN");
                break;
        }
        print("Mode:%s\r\n", s);
        //        switch (shoot_fric_mode) {
        //            case SHOOT_FRIC_MODE_PREPARING:
        //                strcpy(s, "PREPARE");
        //                break;
        //            case SHOOT_FRIC_MODE_STOP:
        //                strcpy(s, "STOP");
        //                break;
        //            case SHOOT_FRIC_MODE_PREPARED:
        //                strcpy(s, "PREPARED");
        //                break;
        //            case SHOOT_FRIC_MODE_DISABLE:
        //                strcpy(s, "DISABLE");
        //                break;
        //        }
        //        print("Shoot Fric Mode:%s\r\n", s);
        switch (shoot_mode) {
            case SHOOT_MODE_PREPARING:
                strcpy(s, "PREPARE");
                break;
            case SHOOT_MODE_STOP:
                strcpy(s, "STOP");
                break;
            case SHOOT_MODE_PREPARED:
                strcpy(s, "PREPARED");
                break;
            case SHOOT_MODE_DISABLE:
                strcpy(s, "DISABLE");
                break;
            case SHOOT_MODE_SINGLE:
                strcpy(s, "SINGLE");
                break;
            case SHOOT_MODE_BURST:
                strcpy(s, "BURST");
                break;
        }
        print("Shoot Mode:%s\r\n", s);
        print(
            "CH0: %-4d CH1: %-4d CH2: %-4d CH3: %-4d \r\nSWL: %d SWR: %d "
            "TWL: %d "
            "@ %d "
            "ms\r\n",
            dbus->ch0, dbus->ch1, dbus->ch2, dbus->ch3, dbus->swl, dbus->swr, dbus->ch4,
            dbus->timestamp);
        print("# %.2f s, IMU %s\r\n", HAL_GetTick() / 1000.0,
              imu->DataReady() ? "\033[1;42mReady\033[0m" : "\033[1;41mNot Ready\033[0m");
        print("Temp: %.2f\r\n", imu->Temp);
        print("Heater: %.2f\r\n", imu->TempPWM);
        print("Euler Angles: %.2f, %.2f, %.2f\r\n", imu->INS_angle[0] / PI * 180,
              imu->INS_angle[1] / PI * 180, imu->INS_angle[2] / PI * 180);
        print("Is Calibrated: %s\r\n",
              imu->CaliDone() ? "\033[1;42mYes\033[0m" : "\033[1;41mNo\033[0m");
        print("Chassis Volt: %.3f\r\n", referee->power_heat_data.chassis_volt / 1000.0);
        print("Chassis Curr: %.3f\r\n", referee->power_heat_data.chassis_current / 1000.0);
        print("Chassis Power: %.3f\r\n", referee->power_heat_data.chassis_power);
        print("\r\n");
        print("Shooter Cooling Heat: %hu\r\n",
              referee->power_heat_data.shooter_id1_17mm_cooling_heat);
        print("Bullet Frequency: %hhu\r\n", referee->shoot_data.bullet_freq);
        print("Bullet Speed: %.3f\r\n", referee->shoot_data.bullet_speed);
        // print("\r\n");
        // yaw_motor->PrintData();
        // pitch_motor->PrintData();

        osDelay(75);
    }
}
