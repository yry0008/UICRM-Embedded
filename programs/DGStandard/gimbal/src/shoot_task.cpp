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

#include "shoot_task.h"

static driver::MotorPWMBase* flywheel_left = nullptr;
static driver::MotorPWMBase* flywheel_right = nullptr;

driver::MotorCANBase* steering_motor = nullptr;

driver::ServoMotor* load_servo = nullptr;

bsp::GPIO* shoot_key = nullptr;

void jam_callback(driver::ServoMotor* servo, const driver::servo_jam_t data) {
    UNUSED(data);
    float servo_target = servo->GetTarget();
    if (servo_target > servo->GetTheta()) {
        float prev_target = servo->GetTarget() - 2 * PI / 8;
        servo->SetTarget(prev_target, true);
    }
}

osThreadId_t shootTaskHandle;

void shootTask(void* arg) {
    UNUSED(arg);
    // 启动等待
    osDelay(1000);
    while (remote_mode == REMOTE_MODE_KILL) {
        osDelay(SHOOT_OS_DELAY);
    }
    // 等待IMU初始化
    while (ahrs->IsCailbrated()) {
        osDelay(SHOOT_OS_DELAY);
    }

    driver::MotorCANBase* motors[] = {steering_motor};


    //    int last_state = remote::MID;
    //    int last_state_2 = remote::MID;
    //    uint8_t shoot_state = 0;
    int shoot_flywheel_offset = 0;
    //    uint8_t shoot_state_2 = 0;
    //    uint8_t last_shoot_key = 0;
    uint8_t shoot_state_key = 0;
    uint8_t shoot_state_key_storage = 0;  // 射击状态保存
    //    uint16_t shoot_time_count = 0;
    uint8_t servo_back = 0;
    //    bool can_shoot_click = false;
    RampSource ramp_1 = RampSource(0, 0, 800, 0.001);
    RampSource ramp_2 = RampSource(0, 0, 800, 0.001);

    load_servo->SetTarget(load_servo->GetTheta(), true);
    load_servo->CalcOutput();
    driver::MotorCANBase::TransmitOutput(motors, 1);

    ShootMode last_shoot_mode = SHOOT_MODE_STOP;

    while (true) {
        if (remote_mode == REMOTE_MODE_KILL) {
            // 死了
            shoot_flywheel_offset = -5000;
            ramp_1.Calc(shoot_flywheel_offset);
            ramp_2.Calc(shoot_flywheel_offset);
            //            shoot_state = 0;
            //            shoot_state_2 = 0;
            kill_shoot();
            osDelay(SHOOT_OS_DELAY);
            continue;
        }
        //                if (referee->bullet_remaining.bullet_remaining_num_17mm == 0){
        //                    //没子弹了
        //                    shoot_flywheel_offset = -200;
        //                    flywheel_left->SetOutput(ramp_1.Calc(shoot_flywheel_offset));
        //                    flywheel_right->SetOutput(ramp_2.Calc(shoot_flywheel_offset));
        //                    shoot_state = 0;
        //                    shoot_state_2 = 0;
        //                    kill_shoot();
        //                    osDelay(SHOOT_OS_DELAY);
        //                    continue;
        //                }
        //         检测开关状态，向上来回打即启动拔弹
        //        if (can_shoot_click) {
        //            if (dbus->keyboard.bit.CTRL == 1) {
        //                if (shoot_state == 0) {
        //                    shoot_state = 1;
        //                } else {
        //                    shoot_state = 0;
        //                    shoot_state_2 = 0;
        //                }
        //            }
        //            if (shoot_state != 0) {
        //                if (dbus->mouse.l == 1) {
        //                    shoot_state_2 = 2;
        //                } else if (dbus->mouse.l == 0) {
        //                    shoot_state_2 = 0;
        //                }
        //            }
        //        }
        //        if (dbus->keyboard.bit.CTRL == 0) {
        //            can_shoot_click = true;
        //        } else {
        //            can_shoot_click = false;
        //        }
        //        if (dbus->swl == remote::UP) {
        //            if (last_state == remote::MID)
        //                last_state = remote::UP;
        //        } else if (dbus->swl == remote::MID) {
        //            if (last_state == remote::UP) {
        //                last_state = remote::MID;
        //                if (shoot_state == 0) {
        //                    shoot_state = 1;
        //                } else {
        //                    shoot_state = 0;
        //                    shoot_state_2 = 0;
        //                }
        //            }
        //        }
        //        switch (shoot_state) {
        //            case 0:
        //                shoot_flywheel_offset = -200;
        //
        //                break;
        //            case 1:
        //            case 2:
        //                shoot_flywheel_offset = 200;
        //                if (servo_back == 0) {
        //                    load_servo->SetTarget(load_servo->GetTheta() - 2 * PI / 32, true);
        //                    servo_back = 1;
        //                }
        //                break;
        //        }
        //        if (shoot_state == 1 && ramp_1.Get() == ramp_1.GetMax() &&
        //            ramp_2.Get() == ramp_2.GetMax()) {
        //            shoot_state = 2;
        //        }
        switch (shoot_fric_mode) {
            case SHOOT_FRIC_MODE_PREPARING:
            case SHOOT_FRIC_MODE_PREPARED:
                shoot_flywheel_offset = 400;
                // laser->SetOutput(255);
                break;
            case SHOOT_FRIC_MODE_STOP:
                shoot_flywheel_offset = -400;
                // laser->SetOutput(0);
                break;
            case SHOOT_FRIC_SPEEDUP:
                ramp_1.SetMax(min(900.0f, ramp_1.GetMax() + 25));
                ramp_2.SetMax(min(900.0f, ramp_2.GetMax() + 25));
                shoot_fric_mode = SHOOT_FRIC_MODE_PREPARING;
                break;
            case SHOOT_FRIC_SPEEDDOWN:
                ramp_1.SetMax(max(500.0f, ramp_1.GetMax() - 25));
                ramp_2.SetMax(max(500.0f, ramp_2.GetMax() - 25));
                shoot_fric_mode = SHOOT_FRIC_MODE_PREPARING;
                break;
            default:
                shoot_flywheel_offset = -1000;
                // laser->SetOutput(0);
                break;
        }
        flywheel_left->SetOutput(ramp_1.Calc(shoot_flywheel_offset));
        flywheel_right->SetOutput(ramp_2.Calc(shoot_flywheel_offset));
        if (ramp_1.Get() == ramp_1.GetMax() && ramp_2.Get() == ramp_2.GetMax() &&
            shoot_fric_mode == SHOOT_FRIC_MODE_PREPARING) {
            // 准备就绪判断
            shoot_fric_mode = SHOOT_FRIC_MODE_PREPARED;
        }
        if (shoot_fric_mode == SHOOT_FRIC_MODE_PREPARED) {
            shoot_state_key = shoot_key->Read();
            switch (shoot_mode) {
                case SHOOT_MODE_PREPARING:
                    // 发射准备就绪检测
                    if (servo_back == 0) {
                        // 第一次发射需要适当退弹
                        load_servo->SetTarget(load_servo->GetTheta() - 2 * PI / 32, true);
                        servo_back = 1;
                    }
                    if (shoot_state_key == 1) {
                        // 检测到准备就绪，转换模式与锁定拔弹电机
                        shoot_mode = SHOOT_MODE_PREPARED;
                        if (!load_servo->Holding()) {
                            load_servo->SetTarget(load_servo->GetTheta(), true);
                        }
                    } else {
                        // 没有准备就绪，则旋转拔弹电机
                        load_servo->SetTarget(load_servo->GetTarget() + 2 * PI / 8, false);
                    }
                    break;
                case SHOOT_MODE_PREPARED:
                    // 准备就绪，未发射状态
                    // 如果检测到未上膛（刚发射一枚子弹），则回到准备模式
                    shoot_state_key_storage = 0;
                    if (shoot_state_key == 0) {
                        shoot_mode = SHOOT_MODE_PREPARING;
                        break;
                    }
                    if (!load_servo->Holding()) {
                        load_servo->SetTarget(load_servo->GetTheta(), true);
                    }
                    break;
                case SHOOT_MODE_SINGLE:
                    // 发射一枚子弹
                    if (shoot_state_key_storage == 0 && shoot_state_key == 0) {
                        shoot_state_key_storage = 1;
                    } else if (shoot_state_key_storage == 1 && shoot_state_key == 1) {
                        shoot_state_key_storage = 0;
                        shoot_mode = SHOOT_MODE_PREPARED;
                        break;
                    }
                    if (last_shoot_mode != SHOOT_MODE_SINGLE)
                        load_servo->SetTarget(load_servo->GetTheta() + 2 * PI / 8, true);
                    else
                        load_servo->SetTarget(load_servo->GetTheta() + 2 * PI / 8, false);
                    break;
                case SHOOT_MODE_BURST:
                    // 连发子弹
                    load_servo->SetTarget(load_servo->GetTarget() + 2 * PI, true);
                    shoot_state_key_storage = 0;
                    break;
                case SHOOT_MODE_STOP:
                    // 停止发射
                    break;
                default:
                    break;
            }
        }
        last_shoot_mode = shoot_mode;
        //        // 启动拔弹电机后的操作
        //        if (shoot_state == 2) {
        //            // 检测是否已装填子弹
        //
        //            // 检测是否需要发射子弹
        //
        //                if (dbus->swl == remote::DOWN) {
        //                    if (last_state_2 == remote::MID) {
        //                        last_state_2 = remote::DOWN;
        //                        if (shoot_state_2 == 0) {
        //                            shoot_state_2 = 1;
        //                        }
        //                        shoot_time_count = 0;
        //                    }
        //                    shoot_time_count++;
        //                    if (shoot_time_count > 1000 / SHOOT_OS_DELAY) {
        //                        shoot_state_2 = 2;
        //                    }
        //                } else if (dbus->swl == remote::MID) {
        //                    if (last_state_2 == remote::DOWN) {
        //                        last_state_2 = remote::MID;
        //                    }
        //                    shoot_state_2 = 0;
        //                }
        //            // 发射子弹
        //            if (shoot_state_2 == 1) {
        //                // 检测是否已经发射完毕
        //                if (last_shoot_key == 0 && shoot_state_key == 1) {
        //                    last_shoot_key = 1;
        //                } else if (last_shoot_key == 1 && shoot_state_key == 0) {
        //                    last_shoot_key = 0;
        //                    shoot_state_2 = 0;
        //                }
        //                // 如果发射未完成，则需要发射子弹
        //                if (shoot_state_2 == 1) {
        //                    load_servo->SetTarget(load_servo->GetTarget() + 2 * PI / 8, false);
        //                } else {
        //                    if (!load_servo->Holding()) {
        //                        load_servo->SetTarget(load_servo->GetTheta(), true);
        //                    }
        //                }
        //            } else if (shoot_state_2 == 2) {
        //                // 连续发射
        //                load_servo->SetTarget(load_servo->GetTarget() + 2 * PI / 8, false);
        //            } else if (shoot_state_key == 1) {
        //                // 不需要发射子弹，但是未装弹完毕，则需要装填子弹
        //                load_servo->SetTarget(load_servo->GetTarget() + 2 * PI / 8, false);
        //            } else {
        //                // 不需要发射子弹，且装弹完毕，则需要锁定拔弹电机
        //                if (!load_servo->Holding()) {
        //                    load_servo->SetTarget(load_servo->GetTheta(), true);
        //                }
        //            }
        //        }
        // 计算输出
        load_servo->CalcOutput();
        driver::MotorCANBase::TransmitOutput(motors, 1);
        osDelay(SHOOT_OS_DELAY);
    }
}

void init_shoot() {
    flywheel_left = new driver::MotorPWMBase(&htim1, 1, 1000000, 500, 1000);
    flywheel_right = new driver::MotorPWMBase(&htim1, 4, 1000000, 500, 1000);
    flywheel_left->SetOutput(0);
    flywheel_right->SetOutput(0);

    steering_motor = new driver::Motor2006(can1, 0x207);

    driver::servo_t servo_data;
    servo_data.motor = steering_motor;
    servo_data.max_speed = 2.5 * PI;
    servo_data.max_acceleration = 16 * PI;
    servo_data.transmission_ratio = M2006P36_RATIO;
    servo_data.omega_pid_param = new float[3]{6000, 80, 0.3};
    servo_data.max_iout = 4000;
    servo_data.max_out = 10000;
    servo_data.hold_pid_param = new float[3]{150, 2, 0.01};
    servo_data.hold_max_iout = 2000;
    servo_data.hold_max_out = 10000;

    load_servo = new driver::ServoMotor(servo_data);
    load_servo->SetTarget(load_servo->GetTheta(), true);
    load_servo->RegisterJamCallback(jam_callback, 0.6);

    shoot_key = new bsp::GPIO(TRIG_KEY_GPIO_Port, TRIG_KEY_Pin);
    // laser = new bsp::Laser(&htim3, 3, 1000000);
}
void kill_shoot() {
    driver::MotorCANBase* motors[] = {steering_motor};
    steering_motor->SetOutput(0);
    flywheel_left->SetOutput(0);
    flywheel_right->SetOutput(0);
    driver::MotorCANBase::TransmitOutput(motors, 1);
}