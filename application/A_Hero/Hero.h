//
// Created by Gularx on 2024/2/26.
//
#include "PID.h"
#include "ramp.h"

#ifndef INC_2024_HERO_HERO_H
#define INC_2024_HERO_HERO_H
//#define GIMBAL

/**** 电机数组配合枚举使用 ****/
typedef enum {
    RF=0,
    LF=1,
    LB=2,
    RB=3,
    FIRE_L=4,
    FIRE_R=5,
    TRIGGER=6,
    FIRE_ON=7
}motor_3508;

/**** 模式选择 ****/
typedef enum {
    GIMBAL_RELAX,   //云台失能
    GIMBAL_ACTIVE,  //得到遥控器对云台电机的控制,自瞄判定
    GIMBAL_AUTO,    //云台自瞄模式
}gimbal_mode_e;

typedef enum {
    CHASSIS_RELAX,          //底盘失能 左下右下
    CHASSIS_ONLY,           //底盘独立 左不下 E.click_flag==1
    CHASSIS_FOLLOW_GIMBAL,  //底盘跟随云台 右中
    CHASSIS_SPIN,           //小陀螺
    CHASSIS_SPIN_1,           //小陀螺
    CHASSIS_BLOCK,          //底盘刹车
    CHASSIS_INDEPENDENT_CONTROL
} chassis_mode_e;

typedef enum{
    Fire_OFF=0,             //发射机构关闭
    Fire_ON=1,              //发射机构开启
}fire_mode_e;

typedef enum{
    SHOOT_CLOSE=0,          //发射关闭
    SHOOT_SINGLE,           //发射准备
    SHOOT_ING,              //发射
}single_shoot_cmd;

typedef enum{
    MOTIONLESS,             //没有
    KEEP_ROTATING,          //一直转
    ROTATION_ANGLE,         //转角度
}trigger_cmd;

typedef enum {
    trigger_finished = 0,   //实现上一次期望了
    trigger_unfinished = 1, //堵转了
    trigger_finishing = 2,  //卡弹处理完成
    trigger_block = 3       //卡弹标志
} locked_rotor;

typedef struct {
    trigger_cmd state;          //拨盘运动方式
    locked_rotor trigger_flag;  //堵转检测
}trigger_t;

typedef enum {
    CAP_OFF,                //电容关闭
    CAP_ON,                 //电容开启
}cap_cmd_e;

/**** 整车数据 ****/
typedef struct {
    fp32 power_buff;
    fp32 limit_k;
    fp32 total_current;
    fp32 total_current_limit;

    fp32 warning_power;
    fp32 warning_buff;

    fp32 K[4];
    fp32 M[4];
    fp32 k_c;
    fp32 predict_send_power;
    fp32 power_set;//最大功率限制
}chassis_power_limit_t;

//电机的数据
typedef struct {
    uint16_t ecd;           //电机编码器计数值
    int16_t speed_rpm;      //电机转速（每分钟转数，RPM）
    int16_t given_current;  //电机给定电流
    uint8_t temperate;      //电机温度
    int16_t last_ecd;       //上一次的电机编码器计数值

    int32_t round_cnt;      //电机旋转的总圈数
    int32_t total_ecd;      //电机旋转的总编码器数值
    uint16_t offset_ecd;    //电机的校准编码值
} motor_measure_t;

typedef struct {
    motor_measure_t *motor_measure;

    pid_type_def angle_p;//角度环pid
    pid_type_def speed_p;//速度环pid
    pid_type_def speed_p_1;//速度环pid
    fp32 speed;           //转速期望值
    int16_t give_current;//给定的电流值
}motor_2006_t;

typedef struct {
    const motor_measure_t *motor_measure;

    fp32 speed;     //速度值
    fp32 rpm_set;   //设定的转速值
    pid_type_def angle_p;  //角度环pid
    pid_type_def speed_p;  // 速度环 PID 控制参数

    int16_t give_current;   //给定的电流值
}motor_3508_t;

typedef struct {
    motor_measure_t *motor_measure;

    pid_type_def angle_p;  //角度环 PID 控制参数
    pid_type_def speed_p;  //速度环 PID 控制参数
    pid_type_def auto_angle_p;  //角度环自瞄 PID 控制参数
    pid_type_def auto_speed_p;  //速度环自瞄 PID 控制参数

    fp32 max_relative_angle; //°    相对角度
    fp32 min_relative_angle; //°
    fp32 relative_angle_get; //°    获取
    fp32 relative_angle_set; //°    设定
    fp32 absolute_angle_get;//      rad
    fp32 absolute_angle_get_down;
    fp32 relative_up_down_get;  // 云台底盘相对角度
    fp32 relative_up_down_set;
    fp32 absolute_angle_set;
    fp32 gyro_set;  //转速设置
    int16_t give_current; //最终电流值
}motor_6020_t;

typedef struct {
    chassis_mode_e mode;
    chassis_mode_e last_mode;
    motor_3508_t motor_chassis[4];
    motor_6020_t motor_steer[4];

    pid_type_def chassis_vw_pid;

    ramp_function_source_t chassis_vx_ramp;
    ramp_function_source_t chassis_vy_ramp;
    ramp_function_source_t chassis_vw_ramp;
    ramp_function_source_t chassis_3508_ramp[4];

    fp32 vx;
    fp32 vy;
    fp32 vw;
    fp32 vx_set;
    fp32 vy_set;
    fp32 vw_set;
    fp32 vx_pc;
    fp32 vy_pc;
    fp32 vw_pc;

    chassis_power_limit_t chassis_power_limit;
} chassis_t;

typedef struct {
    motor_6020_t yaw;
    motor_6020_t pitch;

    gimbal_mode_e mode;
    gimbal_mode_e last_mode;

    //AHRS_Eulr_t*Eulr;   //姿态角
    fp32 relative_gyro_yaw;
    fp32 absolute_gyro_yaw;
    fp32 relative_gyro_pitch;
    fp32 absolute_gyro_pitch;
    bool_t yaw_is_back;
    bool_t pitch_is_back;

    int32_t yaw_imu_offset_angle;
    float horizon_angle;
}gimbal_t;


typedef struct {
    single_shoot_cmd single_shoot_cmd;     //发射机构
    single_shoot_cmd single_shoot_last_cmd;//发射机构

    fire_mode_e fire_mode;      //摩擦轮状态
    fire_mode_e fire_last_mode;//摩擦轮上一次状态

    motor_2006_t fire_l;
    motor_2006_t fire_r;
    motor_2006_t fire_on;
    motor_2006_t trigger;     //拨盘电机
    motor_2006_t images;      //图传电机
}launcher_t;


typedef struct {
    float input_value;
    float input_current;
    float cap_value;
    float target_power;

    float max_cap_voltage;
    float percentage;
    float min_percentage;
    cap_cmd_e ctrl_cmd;

    float chassis_power;

    float bat_voltage;
    float min_voltage;
}cap_info_t;

typedef struct {
    uint8_t send_data[6];
    uint8_t receive_data[6];

    cap_cmd_e mode;
    float bat_voltage;

    union {
        uint16_t charge_current;
    };

    int16_t remain_power;
    uint8_t charge_status;
    uint8_t rec_cap_cmd;
    uint16_t cap_voltage;
    uint16_t chassis_current;
}__packed cap2_info_t;

#endif //INC_2024_HERO_HERO_H
