/**
 * @file pid.h
 * @author 7415 (2789152534@qq.com)
 * @date 2024-11-24 19:17:25
 * @brief 底层库，pid计算库，也含有一些基础数学函数库
 * @version 0.1
 * @note
*/

#ifndef __pid_H
#define __pid_H
#include "stm32f4xx_hal.h"


/*数学与基础计算函数相关声明*/
float calculate_2D_distance(float x1, float y1, float x2, float y2);
// void calculate_speed(float target_x, float target_y, float current_x, float current_y, 
//                      float k, int speed, int *speed_x, int *speed_y, int flag);

float abs_value(float x);
void abs_limit(float *a, float ABS_MAX);

/*PID相关声明*/
/**
 * @brief PID 控制器结构体
 * 
 * 主要内容：
 * - **PID 控制参数**：比例增益（p）、积分增益（i）、微分增益（d）用于计算控制输出。
 * - **历史数据**：包括设定值（目标值）、实际测量值、误差值的历史数组，以便计算变化。
 * - **PID 输出**：包括比例输出（pout）、积分输出（iout）、微分输出（dout），以及最终的 PID 控制输出（pos_out）。
 * - **控制量变化**：包括控制量变化（delta_u）和基于变化量的输出（delta_out），用于改进控制响应。
 * - **限制条件**：包括最大误差（max_err）、死区误差（deadband）、最大输出限制（MaxOutput）、积分限制（IntegralLimit）等，防止控制器输出异常。
 * - **函数指针**：包含初始化和重置 PID 参数的函数指针，允许动态修改 PID 控制器的行为。
 * 
 * @struct __pid_t
 */
typedef struct __pid_t
{
    float p;                 // 比例增益
    float i;                 // 积分增益
    float d;                 // 微分增益

    float set[3];            // 设定值（目标值），包含当前（NOW），上一周期（LAST），前前周期（LLAST）
    float get[3];            // 实际测量值
    float err[3];            // 误差（设定值与实际测量值的差值）

    float pout;              // 比例项的输出
    float iout;              // 积分项的输出
    float dout;              // 微分项的输出

    float pos_out;           // 最终的PID输出
    float last_pos_out;      // 上一个周期的PID输出
    float delta_u;           // 控制量的变化量
    float delta_out;         // 控制量的变化后的输出 = last_delta_out + delta_u
    float last_delta_out;    // 上一个周期的delta_out

    float max_err;           // 最大误差
    float deadband;          // 误差小于deadband时，不进行PID计算
    uint32_t pid_mode;       // PID模式（可能是不同控制模式的标志，如位置控制、速度控制等）
    uint32_t MaxOutput;      // 输出限制（最大输出）
    uint32_t IntegralLimit;  // 积分限制，防止积分饱和

    // 函数指针：用于初始化PID参数
    void (*f_param_init)(struct __pid_t *pid, // PID参数初始化
                         uint32_t pid_mode,
                         uint32_t maxOutput,
                         uint32_t integralLimit,
                         float p,
                         float i,
                         float d);

    // 函数指针：用于重置PID参数
    void (*f_pid_reset)(struct __pid_t *pid, float p, float i, float d); // 重置PID的三个参数
} pid_t;


void PID_struct_init(
    pid_t *pid,
    uint32_t mode,
    uint32_t maxout,
    uint32_t intergral_limit,
    float kp,
    float ki,
    float kd);
/**
 * @brief 用于动态调整PID三个参数
*/
void pid_dynamic_set(
    pid_t *pid, 
    float kp,  
    float ki, 
    float kd    
);

float general_pid_calc(pid_t *pid, float get, float set, char* mode);



#endif
