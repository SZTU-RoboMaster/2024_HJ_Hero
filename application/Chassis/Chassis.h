//
// Created by xhuanc on 2021/10/10.
//

#ifndef DEMO1_CHASSIS_H
#define DEMO1_CHASSIS_H

/******************include******************/
/** 结构体 **/
#include "Hero.h"
#include "protocol_hero.h"
#include "Trigger_function.h"
/** 功能 **/
#include "filter.h"
#include "bsp_led.h"
#include "bsp_adc.h"
#include "bsp_laser.h"
#include "user_lib.h"
/** 操作 **/
#include "key_board.h"

#include "can_receive.h"
#include "Atti.h"
#include "Detection.h"
#include "arm_math.h"
#include "cmsis_os.h"
#include "packet.h"
#include "ramp.h"
#include "Cap.h"

/******************define******************/
#define resume(handle)      if(osThreadGetState(handle)==osThreadSuspended) \
                            {                                               \
                            osThreadResume(handle);                         \
                            }                                               \

/******************Chassis******************/
//任务开始空闲一段时间
#define CHASSIS_TASK_INIT_TIME 357
#define CHASSIS_PERIOD 5//10 // 单位为ms 底盘任务运行周期

#define CHASSIS_Y_CHANNEL 0
#define CHASSIS_X_CHANNEL 1
#define CHASSIS_Z_CHANNEL 2

// 键盘W A S D控制底盘最大速度增量
#define MAX_CHASSIS_VX_SPEED 3.5f
#define MAX_CHASSIS_VY_SPEED 3.5f
#define MAX_CHASSIS_VW_SPEED 3.3f

// 遥控器控制底盘移动
#define RC_TO_VX (MAX_CHASSIS_VX_SPEED/660)
#define RC_TO_VY (MAX_CHASSIS_VY_SPEED/660)
#define RC_TO_VW (MAX_CHASSIS_VW_SPEED/660)    //MAX_CHASSIS_VR_SPEED / RC_MAX_VALUE

//m3508转化成底盘速度(m/s)的比例，要根据轮子大小算 //TODO:舵轮之后需要修改
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f // 0.0003271942801448065f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

#define CHASSIS_SWING_SPEED 10.5f
#define CHASSIS_ARMOR_NOT_FACING_ENEMY_SPIN_SPEED 0.5 //装甲板没有面向敌人的速度
#define CHASSIS_ARMOR_FACING_ENEMY_SPIN_SPEED  3.0 //装甲板面向敌人的速度
//底盘运动过程最大前进速度 mm/s
#define NORMAL_MAX_CHASSIS_SPEED_X 2300.0f

//底盘运动过程最大平移速度  mm/s
#define NORMAL_MAX_CHASSIS_SPEED_Y 1500.0f

#define CHASSIS_WZ_SET_SCALE 0.1f

//功率控制
#define CHASSIS_POWER_LIMIT_NO_REF 40
#define CHASSIS_CURRENT_LIMIT_TOTAL 50000
#define CHASSIS_CURRENT_LIMIT_40W 30000
#define CHASSIS_POWER_BUFF 60
//#define CHASSIS_POWER_LIMIT_DEBUG

//功率预测模型参数，详情见华中科技大学 狼牙战队 功率控制
#define CHASSIS_POWER_K0  0.0020f   //转矩系数    0.0021
//该参数用于调整底盘功率限制模型中与速度相关的系数。增加CHASSIS_POWER_K0的值会使限制系数的调整更加敏感，从而影响底盘电机的功率限制。
#define CHASSIS_POWER_R0  0.0001f  //电机电阻      0.0820
//该参数用于调整底盘功率限制模型中与转速平方相关的系数。增加CHASSIS_POWER_R0的值会使限制系数对电机转速的影响增强，从而影响底盘电机的功率限制。
#define CHASSIS_POWER_P0  11.3380f   //底盘静息功率    8.4443加滤波后的拟合数据
//该参数用于调整底盘功率限制模型中的常数项。增加CHASSIS_POWER_P0的值会使底盘的基础功率限制增加。
#define CHASSIS_CURRENT_CONVERT (20/16384.0f)  //电机反馈电流毫安转安
//该参数用于将电机控制器输出的电流转换为实际功率的单位换算系数。它将电流转换为功率的比例，影响最终的功率限制值。

#define CONTINUES_HURT_JUDGE() (HAL_GetTick()-start_hurt_time>1000)

//底盘机械信息 m
#define WHEELBASE 0.47f     //H  轴距
#define WHEELTRACK 0.412f   //W  轮距
#define GIMBAL_OFFSET 0
#define PERIMETER 0.478f  ///轮子周长
#define M3508_DECELE_RATIO (1.0f/19.0f) // M3508 减速比
#define M3508_MAX_RPM 8000
#define PERIMETER_STEEL 314
#define W_Cdistance my_sqrt(my_pow(WHEELBASE) + my_pow(WHEELTRACK))


/******************底盘 PID ******************/

//底盘旋转跟随PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP    0.4f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI    0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD    0.5f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 3.f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0

//底盘独立PID
#define CHASSIS_3508_PID_KP     7.0f
#define CHASSIS_3508_PID_KI     0.0f
#define CHASSIS_3508_PID_KD     0.0f
#define CHASSIS_3508_PID_MAX_OUT 3000.0f
#define CHASSIS_3508_PID_MAX_IOUT 0

/******************函数声明******************/
/******************下板传上板******************/
extern void Send_referee(fp32 power);
extern void Send_id(uint8_t robot_id);
/******************底盘功能******************/
extern void chassis_relax_handle();
extern void chassis_follow_gimbal_handle();
extern void chassis_spin_handle();
extern void chassis_independent_handle();
/******************底盘运动******************/
extern void chassis_movement();
extern void chassis_meknum_wheel_cal();
extern void chassis_wheel_loop_cal();
/******************/
extern void chassis_power_limit();
void chassis_device_offline_handle();

#endif //DEMO1_CHASSIS_H