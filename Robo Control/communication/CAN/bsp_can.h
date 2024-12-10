/**
 * @file bsp_can.h
 * @author 7415 (2789152534@qq.com)
 * @date 2024-11-24 19:17:25
 * @brief 底层库，pid计算库，也含有一些基础数学函数库
 * @version 0.1
 * @note
*/


#ifndef __BSP_CAN
#define __BSP_CAN

#include "stm32f427xx.h"
#include "stdint.h"

/**
 * @brief PID相关的参量限制，避免超限
*/
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -10.0f
#define T_MAX 10.0f
#define FILTER_BUF_LEN 5

/**
 * @brief 电机测量数据结构体

 * 该结构体用于表示电机的实时测量数据。
 * 每个字段对应电机的不同测量项，适用于电机控制与监测系统中。
 * 
 * @note 电机角度采用12位表示，范围为[0, 8191]。
 */
typedef struct
{
	int16_t speed_rpm;	// 电机的转速
	int16_t real_current; // 当前电流值
	int16_t given_current;// 给定电流值
	uint8_t hall;	// 霍尔传感器状态
	uint16_t angle;		 // 电机当前的角度，范围为[0, 8191]
	uint16_t last_angle;  // 上一次电机的角度，范围为[0, 8191]
	uint16_t offset_angle; // 电机偏移角度
	int32_t round_cnt; // 电机旋转圈数
	int32_t total_angle; // 电机总角度
	uint8_t buf_idx; // 缓冲区索引
	uint16_t angle_buf[FILTER_BUF_LEN]; // 角度数据缓冲区
	uint16_t fited_angle; // 滤波后的电机角度
	uint32_t msg_cnt; // 电机消息计数，用于统计接收到的电机数据消息数量。
} motor_measure_t;

/**
 * @brief 电机控制参数数据结构体
 * @note
 * 该结构体包含电机的控制状态、PID参数以及电机的相关物理量（如位置、速度、扭矩等）。
 */
typedef struct {
    int id;           // 电机的唯一标识符
    int state;        // 电机的当前工作状态（启用、禁用、故障等）
    int p_int;        // 电机位置控制的积分项
    int v_int;        // 电机速度控制的积分项
    int t_int;        // 电机温度控制的积分项
    int kp_int;       // 电机位置控制PID的P增益（整数）
    int kd_int;       // 电机速度控制PID的D增益（整数）
    float pos;        // 电机的当前位置信息
    float vel;        // 电机的当前速度
    float toq;        // 电机的输出扭矩
    float Kp;         // 电机位置控制PID的P增益（浮动值）
    float Kd;         // 电机速度控制PID的D增益（浮动值）
    float Tmos;       // 电机MOS管的温度
    float Tcoil;      // 电机线圈的温度
} motor_ctrl_t;

void CAN_Init_and_Start();
#endif
