/*
 * @Author: Wan Peng 102275086+HTMDJT@users.noreply.github.com
 * @Date: 2024-11-24 19:08:13
 * @LastEditors: Wan Peng 102275086+HTMDJT@users.noreply.github.com
 * @LastEditTime: 2024-11-26 00:23:33
 * @FilePath: \lib_for_C620&pid&can\bsp_can.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/**
 * @file bsp_can.c
 * @author 7415 (2789152534@qq.com)
 * @date 2024-11-24 19:17:25
 * @brief 底层库，CAN通讯底层库，完成CAN通讯的基础配置，辅佐motor.h库
 * @version 0.1
 * @note
*/

#include "stm32f427xx.h"
#include "can.h"
#include "bsp_can.h"
#include "pid.h"

// 用以接收某电机的信息
motor_measure_t motor_3508[8];

/*******************************CAN基础配置*******************************/
/**
 * @brief  配置接收过滤器（全部接收）
 * @param {CAN_HandleTypeDef} *_hcan使用的CAN
 * @note 对于银行的处理不太理解怎么写
 * @return 无
 */
void my_can_filter_init_recv_all(CAN_HandleTypeDef *_hcan)
{
    CAN_FilterTypeDef CAN_FilterConfigStructure;

    CAN_FilterConfigStructure.FilterBank = 0;
    CAN_FilterConfigStructure.FilterMode = CAN_FILTERMODE_IDMASK;
    CAN_FilterConfigStructure.FilterScale = CAN_FILTERSCALE_32BIT;
    CAN_FilterConfigStructure.FilterIdHigh = 0x0000;
    CAN_FilterConfigStructure.FilterIdLow = 0x0000;
    CAN_FilterConfigStructure.FilterMaskIdHigh = 0x0000;
    CAN_FilterConfigStructure.FilterMaskIdLow = 0x0000;
    CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_FilterFIFO0;
    CAN_FilterConfigStructure.SlaveStartFilterBank = 14; //can1(0-13)和can2(14-27)分别得到一半的filter
    CAN_FilterConfigStructure.FilterActivation = ENABLE;


    //can1(0-13)和can2(14-27)分别得到一半的filter 这些好像没什么用？？？
    CAN_FilterConfigStructure.FilterBank = 14;
    if (HAL_CAN_ConfigFilter(_hcan, &CAN_FilterConfigStructure) != HAL_OK)
    {
        Error_Handler();
    }
    HAL_CAN_ActivateNotification(_hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
 * @brief 初始化并开启CAN
*/
void CAN_Init_and_Start()
{ 
	// 不过滤地开启CAN1
	my_can_filter_init_recv_all(&hcan1);
  HAL_CAN_Start(&hcan1);
	
  // 不过滤地开启CAN2
  my_can_filter_init_recv_all(&hcan2);
  HAL_CAN_Start(&hcan2);
}


/*******************************CAN接收与处理配置**************************/
/*********************针对DJI_M3508点击的接收与处理配置****************/
/**
 * @brief  处理电机数据，根据消息计数器自动切换初始化和实时测量
 * @param  motor 指向电机结构体的指针
 * @param  Data  CAN 接收的数据
 * @return 无
 */

void measure_motor(motor_measure_t *motor, uint8_t *Data)
{
    const uint16_t angle_threshold = 4096; // 角度变化判断的阈值
    const uint16_t full_circle = 8192;     // 一圈的角度范围
    // 检查是否仍在初始化阶段
    if (motor->msg_cnt <= 50)
    {
        motor->angle = (uint16_t)(Data[0] << 8 | Data[1]);
        motor->offset_angle = motor->angle;
    }
    else
    {
         // 非初始化阶段，更新电机数据
        motor->last_angle = motor->angle;
        motor->angle = (uint16_t)(Data[0] << 8 | Data[1]);
        motor->real_current = (int16_t)(Data[2] << 8 | Data[3]);
        motor->speed_rpm = motor->real_current; // 这里是因为两种电调对应位不一样的信息
        motor->given_current = (int16_t)(Data[4] << 8 | Data[5]) / -5; // 电流单位转换
        motor->hall = Data[6];

        // 根据角度变化调整圈数
        if (motor->angle - motor->last_angle > angle_threshold)
            motor->round_cnt--;
        else if (motor->angle - motor->last_angle < -angle_threshold)
            motor->round_cnt++;

        motor->total_angle = motor->round_cnt * full_circle + motor->angle - motor->offset_angle;
    }
    motor->msg_cnt++; // 消息计数器自增
}

/**
 * @brief  根据电机 ID 处理对应的电机数据，目前只能处理3508，如果还需要处理别的等给了电调再说
 * @param  StdId CAN 消息的标准帧 ID，用于识别电机
 * @param  Data  接收到的 CAN 数据
 * @return 无
 */
void handle_M3508motor_data_by_id(uint32_t StdId, uint8_t *Data)
{
    motor_measure_t *motor = NULL;

    // 根据 StdId 判断电机
    switch (StdId)
    {
    // CAN1 电机映射
    case 0x201:
        motor = &motor_3508[0];
        break;
    case 0x202:
        motor = &motor_3508[1];
        break;
    case 0x203:
        motor = &motor_3508[2];
        break;
    case 0x204:
        motor = &motor_3508[3];
        break;
    // CAN2 电机映射
    case 0x205:
        motor = &motor_3508[4];
        break;
    case 0x206:
        motor = &motor_3508[5];
        break;
    case 0x207:
        motor = &motor_3508[6];
        break;
    case 0x208:
        motor = &motor_3508[7];
        break;
    // 这里可以继续留接口

    default:
        return; // 如果不是已知电机 ID，直接返回，也可以继续拓展电机ID
    }

    // 调用封装的统一处理函数
    if (motor != NULL)
    {
        measure_motor(motor, Data);
    }
}

/*CAN接收*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *_hcan)
{
    if (_hcan == &hcan1 || _hcan == &hcan2)
    {	
        CAN_RxHeaderTypeDef RxMessage;
        uint8_t Data[8];

        RxMessage.DLC = 2;
        RxMessage.StdId = 0x00;
        RxMessage.ExtId = 0x0000;
        RxMessage.IDE = CAN_ID_EXT;
        RxMessage.RTR = CAN_RTR_DATA;
        if (HAL_CAN_GetState(_hcan) != RESET)
        {
            HAL_CAN_GetRxMessage(_hcan, CAN_RX_FIFO0, &RxMessage, Data);
            handle_M3508motor_data_by_id(RxMessage.StdId, Data);
        }
    }
}
