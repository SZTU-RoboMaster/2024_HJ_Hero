//
// Created by Gularx on 2024/3/3.
//

#ifndef INC_2024_HERO_TRIGGER_FUNCTION_H
#define INC_2024_HERO_TRIGGER_FUNCTION_H

#include "Hero.h"
#include "user_lib.h"
#include "can_receive.h"
#include "remote.h"

#define TRIGGER_SPEED (-350)
///往前就是大了
//// 3508编码器转一圈编码值加8192  减速比1:19  编码器转19圈输出轴才转一圈  19×8192
#define DEGREE_60_TO_ENCODER 26185.f    //25941.3333f   //3508拨盘
#define DEGREE_120_TO_ENCODER 52370.f    //25941.3333f   //3508拨盘


/******************Trigger******************/

extern void Trigger_init();
extern void Trigger_relax_handle();
extern void Trigger_control();
extern void Keep_rotating();
extern void Rotation_angle();
extern void trigger_finish_judge();

////拨盘角度环PID
//#define TRIGGER_ANGLE_PID_KP 1.9f
//#define TRIGGER_ANGLE_PID_KI 0.f
//#define TRIGGER_ANGLE_PID_KD 3.5f
//#define TRIGGER_ANGLE_PID_MAX_OUT    6000
//#define TRIGGER_ANGLE_PID_MAX_IOUT   10
//
////拨盘速度环PID
//#define TRIGGER_SPEED_PID_KP 2.f//3.8f//30
//#define TRIGGER_SPEED_PID_KI 0.f
//#define TRIGGER_SPEED_PID_KD 3.5f//28.f
//#define TRIGGER_SPEED_PID_MAX_OUT    20000
//#define TRIGGER_SPEED_PID_MAX_IOUT   10

////拨盘角度环PID
//#define TRIGGER_ANGLE_PID_KP 1.9f
//#define TRIGGER_ANGLE_PID_KI 0.f
//#define TRIGGER_ANGLE_PID_KD 3.5f
//#define TRIGGER_ANGLE_PID_MAX_OUT    6000
//#define TRIGGER_ANGLE_PID_MAX_IOUT   10
//
////拨盘速度环PID
//#define TRIGGER_SPEED_PID_KP 2.f//3.8f//30
//#define TRIGGER_SPEED_PID_KI 0.f
//#define TRIGGER_SPEED_PID_KD 3.5f//28.f
//#define TRIGGER_SPEED_PID_MAX_OUT    25000
//#define TRIGGER_SPEED_PID_MAX_IOUT   10

//拨盘角度环PID
//#define TRIGGER_ANGLE_PID_KP 0.5f
//#define TRIGGER_ANGLE_PID_KI 0.f
//#define TRIGGER_ANGLE_PID_KD 0.f
//#define TRIGGER_ANGLE_PID_MAX_OUT    5000
//#define TRIGGER_ANGLE_PID_MAX_IOUT   0
//
////拨盘速度环PID
//#define TRIGGER_SPEED_PID_KP 24.f//3.8f//30
//#define TRIGGER_SPEED_PID_KI 0.2f
//#define TRIGGER_SPEED_PID_KD 14.1f//28.f
//#define TRIGGER_SPEED_PID_MAX_OUT    28000
//#define TRIGGER_SPEED_PID_MAX_IOUT   0

//拨盘角度环PID
#define TRIGGER_ANGLE_PID_KP 0.9f
#define TRIGGER_ANGLE_PID_KI 0.f
#define TRIGGER_ANGLE_PID_KD 0.f//1.5f
#define TRIGGER_ANGLE_PID_MAX_OUT    3500//3000
#define TRIGGER_ANGLE_PID_MAX_IOUT   10

//拨盘速度环PID
#define TRIGGER_SPEED_PID_KP 12//11//1.3//30
#define TRIGGER_SPEED_PID_KI 0.1f
#define TRIGGER_SPEED_PID_KD 5//1.5//28.f
#define TRIGGER_SPEED_PID_MAX_OUT    20000
#define TRIGGER_SPEED_PID_MAX_IOUT   5000


//拨盘速度环PID,一直转
#define TRIGGER_PID_KP 800.f//22.4f//30
#define TRIGGER_PID_KI 0.4f//0.4f
#define TRIGGER_PID_KD 0.001f//0.00001f//28.f
#define TRIGGER_PID_MAX_OUT    25000
#define TRIGGER_PID_MAX_IOUT   10000

#endif //INC_2024_HERO_TRIGGER_FUNCTION_H
