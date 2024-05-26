//
// Created by xhuanc on 2021/10/13.
//

#ifndef HERO_GIMBAL_H
#define HERO_GIMBAL_H

/******************include******************/
/** 结构体 **/
#include "Hero.h"
#include "protocol_hero.h"
#include "launcher.h"
/** 功能 **/
#include "user_lib.h"
#include "bsp_servo_pwm.h"
#include "bsp_laser.h"
#include "bsp_led.h"
#include "user_lib.h"
#include "filter.h"
/** 操作 **/
#include "key_board.h"
/** **/
#include "can_receive.h"
#include "Detection.h"
#include "packet.h"
#include "decode.h"
#include "Cap.h"
#include "Atti.h"

/******************define******************/
// 云台任务初始化时间
#define GIMBAL_TASK_INIT_TIME 800

//左摇杆的左右和上下
#define YAW_CHANNEL 2
#define PITCH_CHANNEL 3
#define AUTO_CHANNEL 4

//右摇杆的左右和上下
#define CHASSIS_X_CHANNEL 1
#define CHASSIS_Y_CHANNEL 0
#define CHASSIS_Z_CHANNEL 2

//鼠标输入的比例系数
#define RC_TO_YAW 0.02f
#define RC_TO_PITCH 0.01f

//云台转动速度系数
#define GIMBAL_RC_MOVE_RATIO_PIT 0.03f
#define GIMBAL_RC_MOVE_RATIO_YAW 0.05f

//对 pitch 进行动态限位
#define MAX_ABS_ANGLE 30//39
#define MIN_ABS_ANGLE (-8)//(-17.5)

//对 yaw 进行ecd值误差限制
#define MAX_ECD_ERROR 1

/******************PID******************/
// pitch轴 PID
#define GIMBAL_PITCH_ANGLE_PID_KP  50//30.0f
#define GIMBAL_PITCH_ANGLE_PID_KI  0.2f
#define GIMBAL_PITCH_ANGLE_PID_KD  500.f//600.0f
#define GIMBAL_PITCH_ANGLE_MAX_IOUT 15000.f
#define GIMBAL_PITCH_ANGLE_MAX_OUT  15000.f

#define GIMBAL_PITCH_SPEED_PID_KP   100.0f
#define GIMBAL_PITCH_SPEED_PID_KI   0.f
#define GIMBAL_PITCH_SPEED_PID_KD   250.0f
#define GIMBAL_PITCH_SPEED_MAX_IOUT 3000.f
#define GIMBAL_PITCH_SPEED_MAX_OUT  25000.f

// 自瞄pitch轴 PID
#define GIMBAL_PITCH_AUTO_ANGLE_PID_KP     25.0f
#define GIMBAL_PITCH_AUTO_ANGLE_PID_KI     0.08f
#define GIMBAL_PITCH_AUTO_ANGLE_PID_KD     950.0f
#define GIMBAL_PITCH_AUTO_ANGLE_MAX_IOUT   30.f
#define GIMBAL_PITCH_AUTO_ANGLE_MAX_OUT    15000.f

#define GIMBAL_PITCH_AUTO_SPEED_PID_KP     50.0f
#define GIMBAL_PITCH_AUTO_SPEED_PID_KI     1.5f
#define GIMBAL_PITCH_AUTO_SPEED_PID_KD     10.0f
#define GIMBAL_PITCH_AUTO_SPEED_MAX_IOUT   1000.f
#define GIMBAL_PITCH_AUTO_SPEED_MAX_OUT    25000.f

// yaw轴角度环PID
#define GIMBAL_YAW_ANGLE_PID_KP     30.f//15.f//100.0f//20
#define GIMBAL_YAW_ANGLE_PID_KI     0.f//0.01
#define GIMBAL_YAW_ANGLE_PID_KD     750.0f//150 800
#define GIMBAL_YAW_ANGLE_MAX_IOUT   80.f//80
#define GIMBAL_YAW_ANGLE_MAX_OUT    3000.f//500

// yaw轴速度环PID
#define GIMBAL_YAW_SPEED_PID_KP     100.0f//200
#define GIMBAL_YAW_SPEED_PID_KI     0.f//20
#define GIMBAL_YAW_SPEED_PID_KD     5.0f//100
#define GIMBAL_YAW_SPEED_MAX_IOUT   3000.f
#define GIMBAL_YAW_SPEED_MAX_OUT    20000.f//15000.f

// 自瞄yaw轴角度环PID
#define GIMBAL_YAW_AUTO_ANGLE_PID_KP     7.f//7
#define GIMBAL_YAW_AUTO_ANGLE_PID_KI     0.f//0
#define GIMBAL_YAW_AUTO_ANGLE_PID_KD     200.0f//20
#define GIMBAL_YAW_AUTO_ANGLE_MAX_IOUT   1000.f//1000
#define GIMBAL_YAW_AUTO_ANGLE_MAX_OUT    8.f//8

// 自瞄yaw轴速度环PID
#define GIMBAL_YAW_AUTO_SPEED_PID_KP     8.f//8
#define GIMBAL_YAW_AUTO_SPEED_PID_KI     0.f//0
#define GIMBAL_YAW_AUTO_SPEED_PID_KD     5.f//5
#define GIMBAL_YAW_AUTO_SPEED_MAX_IOUT   2000.f//2000
#define GIMBAL_YAW_AUTO_SPEED_MAX_OUT    3.f//3


/******************函数声明******************/
/******************上下板传输******************/
extern void Send_Chassis_Speed(int16_t ch1, int16_t ch0, int16_t ch2,uint8_t s_l,int32_t Mouse_l);
extern void Send_Keyboard(int32_t W, int32_t A, int32_t S, int32_t D, int16_t trigger_mode);
extern void Send_Yaw_Angle(fp32 relative_angle, uint8_t fire_lock, int32_t V);
extern void Send_Mode(int16_t mode1, int16_t mode2, int16_t mode3, int16_t mode4);
extern void Send_command(int8_t command);
//extern void Send_command(int8_t command, int32_t Mouse_r);
/******************云台功能实现******************/
extern void gimbal_relax_handle();
extern void gimbal_active_handle();
extern void gimbal_auto_handle();
extern void gimbal_ctrl_loop_cal();
extern void gimbal_auto_ctrl_loop_cal();
/******************云台功能实现******************/
extern void gimbal_device_offline_handle();
extern void gimbal_power_stop();
extern void gimbal_can_send_back_mapping();
extern void gimbal_uiInfo_packet();

#endif //HERO_GIMBAL_H
