#pragma once
#include "main.h"
#include "cmsis_os2.h"
#include "protocol.h"
#include "remote_task.h"
#include "referee_task.h"
#include "user_interface.h"
#include "user_define.h"
#include "imu_task.h"
#include "gimbal_task.h"
#include <cmath>
#include <cstring>

#include <algorithm>
extern osThreadId_t uiTaskHandle;
const osThreadAttr_t uiTaskAttribute = {.name = "uiTask",
                                        .attr_bits = osThreadDetached,
                                        .cb_mem = nullptr,
                                        .cb_size = 0,
                                        .stack_mem = nullptr,
                                        .stack_size = 1024 * 4,
                                        .priority = (osPriority_t)osPriorityBelowNormal,
                                        .tz_module = 0,
                                        .reserved = 0};
void uiTask(void* arg);
void init_ui();

