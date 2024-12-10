/**
 * @file motor.c
 * @author 7415 (2789152534@qq.com)
 * @date 2024-11-25 13:45:25
 * @brief 驱动库，电机控制库，目前只针对C620电调，辅佐chassis.c
 * @version 0.1
 * @note
 */

#include "stm32f4xx_hal.h"
#include "can.h"
#include <math.h>
#include <string.h> // 引入字符串处理函数库
#include "pid.h"
#include "stdint.h"
#include "bsp_can.h"


uint8_t M3508_speed_data[8] = {0};	 // 控制M3508速度
uint8_t M3508_current_data[8] = {0}; // 控制M3508电流

/**
 * @description: 发送电机控制信息
 * @param Motor_controller_ID 分控器的ID
 * @param {int16_t} M3508Spd M3508的速度
 *
 * @note 里面的忙等待和发送函数没有明确是哪一个CAN，需要结合具体情况，要看M3508挂载到哪一个CAN上
 * @note 本函数不是通过CAN发送控制速度，而是通过CAN发送给分控器，分控器通过设置电流控制
 * @return {*}无
 */
void send_M3508_speed_to_ctrler(uint8_t Motor_controller_ID, uint8_t M3508Spd)
{
	CAN_TxHeaderTypeDef CAN_TX;
	uint32_t TX_MAILBOX;
	M3508_speed_data[0] = M3508Spd >> 8; // 提取高八位存到数组中，这是因为CAN接受时候为2字节
	M3508_speed_data[1] = M3508Spd;		 // 这里自然是低八位

	CAN_TX.DLC = 0x08;					// data length code,数据的长度，单位为字节，取值为0~8
	CAN_TX.ExtId = 0x0000;				// 扩展标识符（Extended Identifier），用于扩展帧格式的消息。
	CAN_TX.StdId = Motor_controller_ID; // 标准帧ID
	CAN_TX.IDE = CAN_ID_STD;			// 标准帧
	CAN_TX.RTR = CAN_RTR_DATA;			// 数据帧
	CAN_TX.TransmitGlobalTime = DISABLE;

	while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0)
		; // 忙等待.看你想用什么can了

	HAL_CAN_AddTxMessage(&hcan1, &CAN_TX, M3508_speed_data, &TX_MAILBOX);
}

/**
 * @brief 设置M3508电机的电流值，并通过CAN总线发送电流控制指令,通过电流映射来控制速度。
 *
 * @param hcan 指向CAN_HandleTypeDef类型的指针，用于指定要使用的CAN接口。
 * @param iq1 第一个电机的电流值
 * @param iq2 第二个电机的电流值
 * @param iq3 第三个电机的电流值
 * @param iq4 第四个电机的电流值
 *
 * @note 本函数将每个电流值拆分为高低字节，并通过CAN总线发送给M3508电机[强调只针对M3508]。
 *       电流值被分配到M3508_current_data数组中，每个电流值占用2个字节（高字节在前，低字节在后）。
 * 			 如果只想设置一个电机的话，那就别的电机对应电流传参为0
 * @note 控制电流值范围-16384~ 0~ 16384,对应电调输出的转矩电流范围-20~ 0~ 20A。
 *
 * @retval 无
 */
void set_M3508_current(CAN_HandleTypeDef *hcan, short iq1, short iq2, short iq3, short iq4)
{
	CAN_TxHeaderTypeDef CAN_TX;
	uint32_t TX_MAILBOX;
	// 手册有说明，该段表示1-2号继电器得电，45-48号继电器得电。
	M3508_current_data[0] = iq1 >> 8;
	M3508_current_data[1] = iq1;

	M3508_current_data[2] = iq2 >> 8;
	M3508_current_data[3] = iq2;

	M3508_current_data[4] = iq3 >> 8;
	M3508_current_data[5] = iq3;

	M3508_current_data[6] = iq4 >> 8;
	M3508_current_data[7] = iq4;

	CAN_TX.DLC = 0x08;
	CAN_TX.ExtId = 0x0000;
	CAN_TX.StdId = 0x200;	   // 标准帧ID，这里只针对M3508，函数名有强调，不需要单独拎出来定义
	CAN_TX.IDE = CAN_ID_STD;   // 标准帧
	CAN_TX.RTR = CAN_RTR_DATA; // 数据帧
	CAN_TX.TransmitGlobalTime = DISABLE;

	HAL_CAN_AddTxMessage(hcan, &CAN_TX, M3508_current_data, &TX_MAILBOX);
}

/**
 * @brief 获取电机角度，并根据模式计算不同的角度变化。
 * 
 * 该函数根据 `mode` 参数决定如何处理电机的角度信息。如果 `mode` 为 "offset"，
 * 计算当前角度并保存为偏移角度。如果 `mode` 为 `"total"`，则根据电机的当前角度和
 * 上次记录的角度计算总的旋转角度。
 *
 * @param p 指向 motor_measure_t 结构体的指针，用于存储电机角度信息。
 * @param Data 指向包含电机角度数据的数组，通常为两个字节组成的角度数据。
 * @param mode 模式字符串，指定是计算偏移角度还是总角度。可能的值为 `"offset"` 或 `"total"`。
 */
void get_moto_angle(motor_measure_t *p, uint8_t *Data, char *mode)
{
    // 如果模式为 "offset"，表示只需计算当前角度并存储偏移角度
    if (strcmp(mode, "offset") == 0) // 使用strcmp比较字符串
    {
        // 从Data中获取两个字节，合并为一个16位的角度值
        p->angle = (uint16_t)(Data[0] << 8 | Data[1]);
        // 保存当前角度为偏移角度
        p->offset_angle = p->angle;
    }
    // 如果模式为 "total"，表示需要计算电机的总旋转角度
    else if (strcmp(mode, "total") == 0) // 使用strcmp比较字符串
    {
        int res1, res2, delta;

        // 计算正转和反转的两个角度差值
        if (p->angle < p->last_angle)
        {                                           // 当前角度小于上次角度，表示发生了反转
            res1 = p->angle + 8192 - p->last_angle; // 正转，delta=+（超过最大值时回绕）
            res2 = p->angle - p->last_angle;        // 反转，delta=-（直接差值）
        }
        else
        {                                           // 当前角度大于上次角度，表示发生了正转
            res1 = p->angle - 8192 - p->last_angle; // 反转，delta=-（回绕）
            res2 = p->angle - p->last_angle;        // 正转，delta=+（直接差值）
        }

        // 选择更小的角度差作为实际的角度变化，避免计算错误
        if (abs_value(res1) < abs_value(res2))
            delta = res1;  // 如果res1更小，选择res1
        else
            delta = res2;  // 否则选择res2

        // 累加角度变化，更新总角度
        p->total_angle += delta;
        // 更新上次记录的角度
        p->last_angle = p->angle;
    }
}

