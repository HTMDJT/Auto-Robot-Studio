/**
 * @file DL-LN.h
 * @author 万鹏 7415 (2789152534@qq.com)
 * @date 2024-11-28 21:56:15
 * @brief 应用库，无线自组网通讯模块，基于DL-LN32P
 * @version 0.1
 * @note
*/


#ifndef DL_LN_H
#define DL_LN_H

#include <stdint.h>  // 为了声明 uint8_t
#include "stm32f4xx_hal.h"  // 包含 HAL 库中的 UART 相关定义

#define SET_ADDRESS_MODE        0x01  // 设置地址模式
#define SET_NETWORK_ID_MODE     0x02  // 设置网络 ID 模式
#define SET_CHANNEL_MODE        0x03  // 设置信道模式
#define SET_BAUD_RATE_MODE      0x04  // 设置波特率模式


// 函数声明
void DL_LN_parse_link_quality(void);  // 解析链路质量信息并发送到上位机
void DL_LN_parse_module_info(void);   // 解析模块信息并发送到上位机
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);  // UART接收完成回调函数
void DL_LN_send_command(const uint8_t *command, uint8_t length);  // 发送命令到UART接口
void DL_LN_read(void);  // 读取模块信息
void DL_LN_link_quality_test(uint16_t module_1_address, uint16_t module_2_address);  // 执行链路质量测试
void DL_LN_set_module_param(uint8_t mode, uint16_t param1, uint8_t param2);
void DL_LN_restart();
void DL_LN_send_packet(uint8_t send_port, uint8_t recv_port, uint16_t target_address, const uint8_t *data)

#endif /* DL_LN_H */
