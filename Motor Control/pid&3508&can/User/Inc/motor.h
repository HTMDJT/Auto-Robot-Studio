/**
 * @file motor.h
 * @author 7415 (2789152534@qq.com)
 * @date 2024-11-25 13:45:25
 * @brief 驱动库，电机控制库，目前只针对C620电调，辅佐chassis.c
 * @version 0.1
 * @note
*/

#ifndef __CONTROL_H
#define __CONTROL_H

#include "stm32f427xx.h"
#include "stdint.h"
#include "can.h"
#include "bsp_can.h"

void send_M3508_speed_to_ctrler(uint8_t MotorId,uint8_t M3508Spd);
void set_M3508_current(CAN_HandleTypeDef *hcan, short iq1, short iq2, short iq3, short iq4);
void get_moto_angle(motor_measure_t *p, uint8_t *Data, char *mode);

#endif
