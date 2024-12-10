/**
 * @file pid.c
 * @author 7415 (2789152534@qq.com)
 * @date 2024-11-24 19:17:25
 * @brief 底层库，pid计算库，运动学解算库，也含有一些基础数学函数库,辅佐于motor.h库
 * @version 0.1
 * @note
*/
#include "stm32f427xx.h"
#include "pid.h"
#include <math.h>
#include "stdint.h"
#include "string.h"

/*********************基本数学与基础运算相关*********************/
/**
 * @brief 绝对值运算函数
 * @param {float}x
 * @return x的绝对值
*/
float abs_value(float x)
{
    return (x > 0) ? x : -x;
}

/**
 * @brief 在限制内进行绝对值运算函数
 * @param {float}X
 * @param {float}ABS_MAX 如果X值超过ABS_MAX，X将存为ABX_MAX
*/
void abs_limit(float *a, float ABS_MAX)
{
    *a = (*a > ABS_MAX) ? ABS_MAX : (*a < -ABS_MAX) ? -ABS_MAX : *a;
}

/**
 * @brief 计算二维平面中两个点之间的欧氏距离。
 * 
 * @param x1y1 第一个点的 xy 坐标。
 * @param x2y2 第二个点的 xy 坐标。
 * @return float 返回两个点之间的欧氏距离。
 * 
 */
float calculate_2D_distance(float x1, float y1, float x2, float y2) {
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

/**
 * @brief 根据目标点和当前点计算绝对速度分量。
 * 
 * @param target_x 目标点的 x 坐标。
 * @param target_y 目标点的 y 坐标。
 * @param current_x 当前点的 x 坐标。
 * @param current_y 当前点的 y 坐标。
 * @param k 速度分量权重系数，用于调整 x 和 y 分量的比例。
 * @param speed 总速度的标量大小。
 * @param speed_x 指向 x 方向速度分量的指针，用于存储计算结果。
 * @param speed_y 指向 y 方向速度分量的指针，用于存储计算结果。
 * @param flag 模式选择标志：
 *             - 0：调整 x 方向的速度分量，权重由参数 k 决定。
 *             - 非 0：调整 y 方向的速度分量，权重由参数 k 决定。
 * 
 * @note 该函数会根据目标点和当前点之间的相对位置，结合给定的权重系数和总速度大小，
 *       计算出 x 和 y 方向上的速度分量。
 * @note 根据实际情况取舍，目前先不列入基础运算中
 */
// void calculate_speed(float target_x, float target_y, float current_x, float current_y, 
//                      float k, int speed, int *speed_x, int *speed_y, int flag) 
// {
//     float delta_x = abs_value(target_x - current_x);
//     float delta_y = abs_value(target_y - current_y);
//     float factor = sqrt(1.0 / ((k * k * delta_x * delta_x) + (delta_y * delta_y)));

//     if (flag == 0) {
//         *speed_x = k * delta_x * speed * factor;
//         *speed_y = delta_y * speed * factor;
//     } else {
//         *speed_x = delta_x * speed * factor;
//         *speed_y = k * delta_y * speed * factor;
//     }
// }


/*********************PID库*********************/

/**
 * @brief 枚举定义，包含历史数据索引和 PID 控制模式
 * 
 * @note 
 * POSITION_PID, DELTA_PID:
 * - 用于区分 PID 控制模式：
 *   - POSITION_PID: 位置式 PID 控制器，直接计算目标位置的输出
 *   - DELTA_PID: 增量式 PID 控制器，计算目标控制量的变化量
 */
enum
{
    LLAST = 0,       ///< 前前周期的值索引
    LAST = 1,        ///< 上一周期的值索引
    NOW = 2,         ///< 当前周期的值索引
    POSITION_PID,    ///< 位置式 PID 控制器模式
    DELTA_PID,       ///< 增量式 PID 控制器模式
};

/**
 * @brief 初始化 PID 控制器的参数
 * 
 * 该函数为指定的 PID 结构体设置P,I,D以及模式和限制值
 * 
 * @param pid 指向 PID 结构体的指针
 * @param mode PID 控制器模式（如 POSITION_PID[位置式PID] 或 DELTA_PID[增量式PID]）
 * @param maxout 最大输出限制，用于防止输出值过大
 * @param intergral_limit 积分项限制，用于防止积分累计过大
 * 
 * @note 本函数仅在本文件内可使用
 */
static void pid_param_init(
    pid_t *pid,
    uint32_t mode,
    uint32_t maxout,
    uint32_t intergral_limit,
    float kp,
    float ki,
    float kd)
{

    pid->IntegralLimit = intergral_limit;
    pid->MaxOutput = maxout;
    pid->pid_mode = mode;

    pid->p = kp;
    pid->i = ki;
    pid->d = kd;
}

/**
 * @brief 初始化 PID 控制器结构体以及相关的函数指针和参数。
 *
 * 此函数执行以下任务：
 * 1. 初始化 PID 结构体中的函数指针成员，使得 PID 控制器能够调用其它函数
 * 2. 使用传入的参数初始化 PID 控制器的各项配置
 *
 * @param pid PID 控制器实例。
 */
void PID_struct_init(
    pid_t *pid,
    uint32_t mode,
    uint32_t maxout,
    uint32_t intergral_limit,
    float kp,
    float ki,
    float kd)
{
    /* 初始化函数指针 */
    pid->f_param_init = pid_param_init;  // 设置 f_param_init 指针，指向 pid_param_init 函数
    pid->f_pid_reset = pid_dynamic_set;  // 设置 f_pid_reset 指针，指向 pid_reset 函数
    // pid->f_cal_pid = pid_calc;        // 可选：设置 PID 计算函数
    // pid->f_cal_sp_pid = pid_sp_calc;  // 可选：设置设定点 PID 计算函数

    /* 初始化 PID 参数 */
    pid->f_param_init(pid, mode, maxout, intergral_limit, kp, ki, kd);  // 调用 f_param_init 初始化 PID 参数
}

/**
 * @brief 动态调整 PID 控制器的增益参数
 * 
 * 该函数用于在运行时修改 PID 控制器的比例、积分、微分增益值
 * 用于调试阶段
 * 
 * @param pid 指向 PID 控制器结构体的指针
 */

void pid_dynamic_set(
    pid_t *pid, // PID 控制器结构体指针
    float kp,  
    float ki, 
    float kd    
)
{
    // 更新比例、积分、微分增益
    pid->p = kp;  
    pid->i = ki;
    pid->d = kd;  
}

/**
 * @brief 计算位置式或增量式 PID 输出
 *
 * 该函数支持两种模式：
 * - **位置式 PID**：直接基于当前误差计算输出。
 * - **增量式 PID**：基于误差的变化计算输出增量。
 *
 * @param[in] pid 指向 PID 控制结构体的指针
 * @param[in] get 当前测量值
 * @param[in] set 目标设定值
 * @return 计算后的 PID 输出
 * @note ！！！未引出该接口！！！
 */
float pid_calc(pid_t *pid, float get, float set)
{
    // 更新当前测量值和设定值
    pid->get[NOW] = get;
    pid->set[NOW] = set;
    pid->err[NOW] = set - get; // 计算当前误差

    // 检查误差是否超出范围
    if (pid->max_err != 0 && abs_value(pid->err[NOW]) > pid->max_err)
        return 0;
    if (pid->deadband != 0 && abs_value(pid->err[NOW]) < pid->deadband)
        return 0;

    if (pid->pid_mode == POSITION_PID) // 位置式 PID 模式
    {
        // 计算 P、I、D 输出
        pid->pout = pid->p * pid->err[NOW];
        pid->iout += pid->i * pid->err[NOW];
        pid->dout = pid->d * (pid->err[NOW] - pid->err[LAST]);

        // 限制积分输出和总输出
        abs_limit(&(pid->iout), pid->IntegralLimit);
        pid->pos_out = pid->pout + pid->iout + pid->dout;
        abs_limit(&(pid->pos_out), pid->MaxOutput);

        // 保存上一次输出
        pid->last_pos_out = pid->pos_out;
    }
    else if (pid->pid_mode == DELTA_PID) // 增量式 PID 模式
    {
        // 计算增量输出
        pid->pout = pid->p * (pid->err[NOW] - pid->err[LAST]);
        pid->iout = pid->i * pid->err[NOW];
        pid->dout = pid->d * (pid->err[NOW] - 2 * pid->err[LAST] + pid->err[LLAST]);

        pid->delta_u = pid->pout + pid->iout + pid->dout;
        abs_limit(&(pid->delta_u), pid->IntegralLimit);

        // 更新增量式输出
        pid->delta_out = pid->last_delta_out + pid->delta_u;
        abs_limit(&(pid->delta_out), pid->MaxOutput);

        // 保存上一次增量输出
        pid->last_delta_out = pid->delta_out;
    }

    // 更新历史误差和输入输出值
    pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];
    pid->get[LLAST] = pid->get[LAST];
    pid->get[LAST] = pid->get[NOW];
    pid->set[LLAST] = pid->set[LAST];
    pid->set[LAST] = pid->set[NOW];

    // 这行代码根据当前 pid_mode 来选择对应的 PID 输出值，返回给调用者。
    return pid->pid_mode == POSITION_PID ? pid->pos_out : pid->delta_out;
}

/**
 * @brief 通用PID输出，总接口，融合上述三个函数
 * 
 * 该函数计算PID输出，支持位置式PID、增量式PID，普通角度和姿态角度处理。
 *
 * @param[in] pid 指向PID控制结构体的指针
 * @param[in] get 当前测量值
 * @param[in] set 目标设定值
 * @param[in] mode 角度误差范围的类型（com_angle: 普通角度处理,pos_angle：姿态角度处理,其他:跳过角度处理）
 * @return 计算后的PID输出
 * 
 * @note 通过mode参数选择模式！
 */
float general_pid_calc(pid_t *pid, float get, float set, char* mode)
{
    pid->get[NOW] = get;
    pid->set[NOW] = set;
    pid->err[NOW] = set - get; // 计算当前误差

    // 角度误差范围处理
    if (strcmp(mode, "com_angle") == 0) {
        if (pid->err[NOW] < -8191)
            pid->err[NOW] += 16383;
        if (pid->err[NOW] > 8191)
            pid->err[NOW] -= 16383;
    }
    else if (strcmp(mode, "pos_angle") == 0) {
        if (pid->err[NOW] < -180)
            pid->err[NOW] += 360;
        if (pid->err[NOW] > 180)
            pid->err[NOW] -= 360;
    }

    // 检查误差范围
    if (pid->max_err != 0 && abs_value(pid->err[NOW]) > pid->max_err)
        return 0;
    if (pid->deadband != 0 && abs_value(pid->err[NOW]) < pid->deadband)
        return 0;

    // 调用标准的 PID 计算函数
    return pid_calc(pid, pid->get[NOW], pid->set[NOW]);
}

/**
 * @brief 计算运动状态下的 PID 输出(带陀螺仪量)
 * 
 * @param pid      指向 PID 控制器结构体的指针，包含了控制器的所有参数和历史状态。
 * @param get      当前测量值，用于与目标值 (`set`) 计算误差。
 * @param set      目标值，控制器的期望输出值。
 * @param gyro     陀螺仪反馈值，代表系统的角速度或变化率，用于微分控制项（`dout`）。
 * 
 * @return float：
 * - 如果使用的是position_PID 模式，返回 `pos_out`
 * - 如果使用的是增量式 PID 模式，返回 `delta_out`（未启用）。
 * 
 * @note 
 * 1. 陀螺仪反馈值 (`gyro`) 作为微分控制项的一部分用于调整响应速度，减小系统振荡。
 * 2. 积分控制项有一个阈值判断，如果积分系数 (`i`) 较小，则禁用积分控制，避免出现积分饱和。
 * 3. 输出会受到设定的最大输出限制（`MaxOutput`）和积分限制（`IntegralLimit`）。
 * @note 要看实际情况，这个先不引出
 *  
 * @note ！！！未引出该接口！！！
 */
float pid_sp_calc(pid_t *pid, float get, float set, float gyro)
{
    // 更新当前测量值、目标值和误差
    pid->get[NOW] = get;
    pid->set[NOW] = set;
    pid->err[NOW] = set - get; // 计算偏差：目标值 - 测量值

    if (pid->pid_mode == POSITION_PID) // 如果是位置式 PID 控制
    {
        // 计算比例输出
        pid->pout = pid->p * pid->err[NOW];

        // 如果积分系数不小于 0.001，执行积分操作
        if (fabs(pid->i) >= 0.001f)
            pid->iout += pid->i * pid->err[NOW];
        else
            pid->iout = 0; // 若积分系数较小，则不进行积分

        // 计算微分输出（基于陀螺仪反馈）
        pid->dout = -pid->d * gyro / 100.0f;

        // 积分输出限幅
        abs_limit(&(pid->iout), pid->IntegralLimit);

        // 计算总输出
        pid->pos_out = pid->pout + pid->iout + pid->dout;

        // 输出限幅，防止过大
        abs_limit(&(pid->pos_out), pid->MaxOutput);

        // 保存当前输出作为上一轮输出
        pid->last_pos_out = pid->pos_out;
    }
    else if (pid->pid_mode == DELTA_PID) // 如果是增量式 PID 控制
    {
        // 增量式 PID 控制的实现（当前未启用）
        // 使用差分误差进行增量计算
        // pid->pout = pid->p * (pid->err[NOW] - pid->err[LAST]);
        // pid->iout = pid->i * pid->err[NOW];
        // pid->dout = pid->d * (pid->err[NOW] - 2*pid->err[LAST] + pid->err[LLAST]);
        // abs_limit(&(pid->iout), pid->IntegralLimit);
        // pid->delta_u = pid->pout + pid->iout + pid->dout;
        // pid->delta_out = pid->last_delta_out + pid->delta_u;
        // abs_limit(&(pid->delta_out), pid->MaxOutput);
        // pid->last_delta_out = pid->delta_out;
    }

    // 更新历史误差和测量值，供下一次计算使用
    pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];
    pid->get[LLAST] = pid->get[LAST];
    pid->get[LAST] = pid->get[NOW];
    pid->set[LLAST] = pid->set[LAST];
    pid->set[LAST] = pid->set[NOW];

    // 根据当前 PID 模式返回控制输出
    return pid->pid_mode == POSITION_PID ? pid->pos_out : pid->delta_out;
}