//
// Created by xhuanc on 2021/11/2.
//

#ifndef DEMO1_LAUNCHER_H
#define DEMO1_LAUNCHER_H


/******************include******************/
#include "Hero.h"
#include "protocol_hero.h"
#include "can_receive.h"
#include <stdbool.h>
#include "key_board.h"
#include "cmsis_os.h"
#include "user_lib.h"

/**** 功能 ****/
#include "bsp_laser.h"
#include "bsp_led.h"

#include "filter.h"
#include "Detection.h"


/******************define******************/




//连发准备计时完成
#define CONTINUES_SHOOT_TIMING_COMPLETE() HAL_GetTick()-continue_shoot_time>1500
#define CONTINUES_BLOCKED_JUDGE() (HAL_GetTick()-blocked_start_time>900)
#define TRIGGER_REVERSE_TIME_JUDGE() (HAL_GetTick()-reverse_start_time<900)
#define FIRE_SPEED 3400  //TODO: 摩擦轮转速

//// 2006编码器转一圈编码值加8192  减速比1:36  编码器转36圈输出轴才转一圈  36×8192
#define DEGREE_93_TO_ENCODER 98304.0000f    //2006单发 120：98304    95:77824    90:73728  100:81920

//#define FIRE_ON() KeyBoard.Q.click_flag==1
//#define FIRE_OFF() KeyBoard.Q.click_flag==0


//左边摩擦轮转速PID
#define SHOOT_FIRE_L_PID_KP 30 //80
#define SHOOT_FIRE_L_PID_KI 0.f
#define SHOOT_FIRE_L_PID_KD 5.f//50.f
#define SHOOT_FIRE_L_PID_MAX_OUT    26000
#define SHOOT_FIRE_L_PID_MAX_IOUT   0

//右边摩擦轮转速PID
#define SHOOT_FIRE_R_PID_KP 30//80
#define SHOOT_FIRE_R_PID_KI 0.f
#define SHOOT_FIRE_R_PID_KD 5.f//50.f
#define SHOOT_FIRE_R_PID_MAX_OUT    26000
#define SHOOT_FIRE_R_PID_MAX_IOUT   0

//单发限位角度环PID
#define SINGLE_SHOOT_ANGLE_PID_KP 2.5f
#define SINGLE_SHOOT_ANGLE_PID_KI 0.03f
#define SINGLE_SHOOT_ANGLE_PID_KD 4.5f
#define SINGLE_SHOOT_ANGLE_PID_MAX_OUT 8000
#define SINGLE_SHOOT_ANGLE_PID_MAX_IOUT 500

//单发限位速度环PID
#define SINGLE_SHOOT_SPEED_PID_KP  3.9f
#define SINGLE_SHOOT_SPEED_PID_KI  0.01f
#define SINGLE_SHOOT_SPEED_PID_KD  3.8f
#define SINGLE_SHOOT_SPEED_PID_MAX_OUT 10000
#define SINGLE_SHOOT_SPEED_PID_MAX_IOUT 500



/**
  * @brief          发射机构初始化
  * @param[in]      none
  * 初始时发射机构模式默认关闭, 获取发射机构电机数据结构体
  * 初始化左右摩擦轮，主动单发的角度环和速度环 PID
  * 初始时发射机构为失能, 电流给定为 0
  * 初始化主动单发的 total_ecd、offset_ecd 和 ecd
  * @retval         返回空
  */
extern void launcher_init();



/**
  * @brief          发射模式设置
  * @param[in]      none
  * 左拨杆向上开关摩擦力，向下开启主动单发发弹
  * @retval         返回空
  */
extern void launcher_mode_set();



/**
  * @brief          发射机构的逻辑实现
  * @param[in]      none
  * 通过云台的状态实现发射逻辑
  * @retval         返回空
  */
extern void launcher_control();
extern void active_single_shoot_active();
extern void block_led();



/**
  * @brief          发射机构失能
  * @param[in]      none
  * 左右摩擦轮失能，单发失能
  * @retval         返回空
  */
extern void launcher_relax_handle();

#endif //DEMO1_LAUNCHER_H