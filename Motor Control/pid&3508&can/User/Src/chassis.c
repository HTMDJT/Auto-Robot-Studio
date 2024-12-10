/**
 * @file bsp_can.c
 * @author 7415 (2789152534@qq.com)
 * @date 2024-11-24 19:17:25
 * @brief 应用库，包括了整体舵轮的运动解算和控制
 * @version 0.1
 * @note
*/

#include "pid.h"
#include "motor.h"
#include "bsp_can.h"
#include "chassis.h"

float TargetXAbs = 0;
float TargetYAbs = 0;
float TargetYaw = 0;
int MoveMode = 0;		     // 底盘运动模式
uint8_t YawArriveFlag = 0;   // 偏航角终止判断标志
int Crash[4] = {0, 0, 0, 0}; // 这个应该放在中断，暂且放在这里
pid_t PidCoordinateX; // X坐标控制PID
pid_t PidCoordinateY; // Y坐标控制PID
pid_t PidPosture;    // 姿态控制PID

//// 世界坐标系到局部坐标系的转换
//float vx_from_world_to_local(int vx, int vy, int vw) {
//    return cos(vw / 180 * M_PI) * vx + sin(vw / 180 * M_PI) * vy;
//}

//float vy_from_world_to_local(int vx, int vy, int vw) {
//    return sin(-vw / 180 * M_PI) * vx + cos(vw / 180 * M_PI) * vy;
//}

//float vw_from_world_to_local(int vx, int vy, int vw) {
//    return vw;
//}

//void move_to(float x, float y, int Speed, float k,float slow_rate, char EarlierDer) {
//    float MoveLength = 0; // 移动的距离
//    int SpeedX = 0;
//    int SpeedY = 0;
//    int Flag = 0;
//    float Dx = 0;
//    float Dy = 0;

//    // 判断优先方向或使用指定方向
//    if (EarlierDer == 'x') {
//        Flag = 0;  // 优先沿X方向
//    } else if (EarlierDer == 'y') {
//        Flag = 1;  // 优先沿Y方向
//    } else {
//        Flag = (abs_value(x - odometer_data.x) >= abs_value(y - odometer_data.y)) ? 0 : 1;
//    }
//    // 如果优先方向是Y轴，需要调整比例因子k
//    if (Flag == 1) {
//        k = 1 / k;
//    }

//    // 设置目标
//    TargetXAbs = x;
//    TargetYAbs = y;
//    MoveLength = calculate_2D_distance(x, y, odometer_data.x, odometer_data.y);
//    MoveMode = COORDINATE_ABS;

//    while (YawArriveFlag < 10 && ((Crash[0] == 1 || Crash[1] == 1) || CorrectFlag == 0)) 
//    {
//        Dx = abs_value(x - odometer_data.x);
//        Dy = abs_value(y - odometer_data.y);
//        // 计算速度分量
//        calculate_speed(x, y, odometer_data.x, odometer_data.y, k, Speed, &SpeedX, &SpeedY, Flag);

//        // 根据距离判断 PID 参数
//        if (fmax((calculate_2D_distance(x, y, odometer_data.x, odometer_data.y)) < MoveLength * 0.15, 300)) {
//            PID_struct_init(&PidCoordinateX, POSITION_PID, SpeedX, SpeedX, 10, 0, 0);
//            PID_struct_init(&PidCoordinateY, POSITION_PID, SpeedY, SpeedY, 10, 0, 0);
//            PID_struct_init(&PidPosture, POSITION_PID, 7000, 7000, 120, 0, 4);
//        } else {
//            PID_struct_init(&PidCoordinateX, POSITION_PID, SpeedX, SpeedX, 20, 0, 0);
//            PID_struct_init(&PidCoordinateY, POSITION_PID, SpeedY, SpeedY, 20, 0, 0);
//        }
//    }

//    MoveMode = STOP;
//    PID_struct_init(&PidPosture, POSITION_PID, 8500, 8500, 320, 0, 9);
    // 剩下的还没有写，这个跟具体的结构有关。
    // if (CorrectFlag == 1) {
    //     update_odometer_data('J', 0);
    //     update_odometer_data('Y', 250);

    //     LaserWaitFlag = 1;
    //     while (LaserWaitFlag) {
    //         HAL_Delay(1);
    //     }

    //     // 更新里程计 X 坐标
    //     if (ZoneXDirec == 1) {
    //         update_odometer_data('X', -(4300 - (Distance * 1000)));
    //     } else {
    //         update_odometer_data('X', Distance * 1000 - 940);
    //     }

    //     CorrectFlag = 0;
    // }

//    HAL_Delay(6);
//}
