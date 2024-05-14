//
// Created by xhuanc on 2021/9/27.
//

#ifndef AVG_INFANTRY_CAN_RECEIVE_H
#define AVG_INFANTRY_CAN_RECEIVE_H
#include "struct_typedef.h"
#include "protocol_hero.h"
#include "Hero.h"
#include "PID.h"
#include "Detection.h"
#include "cmsis_os.h"
/******************** define *******************/

#define HALF_ECD_RANGE  4096
#define ECD_RANGE       8192
#define MOTOR_ECD_TO_RAD 0.000766990394f //      2*  PI  /8192
#define MOTOR_ECD_TO_ANGLE 0.0439453125  //        360  /8192

//用于将弧度转换为角度的常数。
#define MOTOR_RAD_TO_ANGLE 57.29577951308238f // 360*2PI
#define MOTOR_ANGLE_TO_RAD  0.0174532925199433f


/******************** struct *******************/

//CAN_ID 该枚举不区分CAN_1还是CAN_2
typedef enum {
    //电机控制 发送ID
    CAN_MOTOR_0x200_ID = 0x200,
    CAN_MOTOR_0x1FF_ID = 0x1FF,
    CAN_MOTOR_0x2FF_ID = 0x2FF,

    //0X200对应的电机ID：201，202，203，204
    //0x1FF对应的电机ID：205，206，207，208

    CAN_CHASSIS_3508_MOTOR_1=0x201,     //1     前右
    CAN_CHASSIS_3508_MOTOR_2=0x202,     //1     前左
    CAN_CHASSIS_3508_MOTOR_3=0x203,     //1     后左
    CAN_CHASSIS_3508_MOTOR_4=0x204,     //1     后右
    CAN_LAUNCHER_3508_TRIGGER=0X207,    //1     拨盘


    CAN_LAUNCHER_3508_FIRE_ON=0x204,    //2     上摩擦轮
    CAN_GIMBAL_6020_YAW=0x205,          //2     yaw轴
    CAN_GIMBAL_6020_PITCH=0x206,        //2     pitch轴
    CAN_LAUNCHER_3508_FIRE_R=0X207,     //2     左摩擦轮
    CAN_LAUNCHER_3508_FIRE_L=0X208,     //2     右摩擦轮
} can_msg_id_e;

typedef enum {
    CAN_1,
    CAN_2,
}CAN_TYPE;//////////////////


union ctrl_ch{
    uint8_t data[2];
    int16_t value;
};

union angle{
    uint8_t data[4];
    fp32 value;
};

union mode{
    uint8_t data[2];
    int16_t mode;
};

union referee{
    uint8_t data[4];
    fp32 referee;
};

union com{
    uint8_t data[2];
    int8_t value;
};

/******************** extern *******************/

extern motor_measure_t motor_3508_measure[8];//前4个为底盘电机 后2个为摩擦轮电机 6是拨盘，7是上摩擦轮
extern motor_measure_t motor_yaw_measure;
extern motor_measure_t motor_pitch_measure;

extern void CAN_cmd_motor(CAN_TYPE can_type,can_msg_id_e CMD_ID,int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
extern void CAN_cmd_chassis_rudder(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
extern fp32 motor_ecd_to_rad_change(uint16_t ecd, uint16_t offset_ecd);
extern fp32 motor_ecd_to_angle_change(uint16_t ecd,uint16_t offset_ecd);
extern void CAN_cmd_communication(CAN_TYPE can_type,can_msg_id_e CMD_ID,fp32 vx,fp32 vy);
extern void CAN_cap(CAN_TYPE can_type,can_msg_id_e CMD_ID,uint16_t power_limit);
extern void CAN_cmd_cap2(cap2_info_t*cap);
extern void CAN_Chass_init();
extern void CAN_Gimbal_init();

#endif //AVG_INFANTRY_CAN_RECEIVE_H
