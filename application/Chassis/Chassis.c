/******************define******************/

/*轮子电机id：          前                         舵：              前                      解算坐标：            x(前)
            ****              ****                     ****               ****                           |
           *  2 *            * 1  *                   * LF *             * RF *                          |
            ****              ****                     ****               ****                           |
                                                                                                         |
           左                   右                                                          --------------z-----------y(右)
                                                                                                         |
            ****              ****                     ****               ****                           |
           *  3 *            * 4  *                   * LB *             * RB *                          |
            ****              ****                     ****               ****                           |

*/


/******************include******************/
#include "Chassis.h"
#include "tim.h"
/******************变量 ******************/
extern chassis_t chassis;       // 底盘
extern gimbal_t gimbal;         // 云台
extern launcher_t launcher;     // 发射机构
extern cap_info_t cap_info;     // 电容
extern key_board_t KeyBoard;    // 键盘
extern RC_ctrl_t rc_ctrl;       // 遥控器

first_kalman_filter_t chassis_filter[4];
first_order_filter_type_t vx_slow={.frame_period=1, .num=8};
first_order_filter_type_t vy_slow={.frame_period=1, .num=15};
first_order_filter_type_t vw_slow={.frame_period=4, .num=40};

__attribute__((unused)) fp32 volatile_judge;

float chassis_vx, chassis_vy, chassis_vw;
double test_y=0, test_w=0, test_x=0;
uint32_t time;
uint32_t test_om=0;
uint32_t time_last_time;

_Noreturn void Chassis_task(void const *pvParameters);
static void chassis_init(chassis_t *chassis);
static float chassis_speed_change();
static void chassis_pc_ctrl();
static void chassis_ctrl_info_get();
static void chassis_control();

/******************函数实现******************/
void Chassis_task(void const *pvParameters) {
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    chassis_init(&chassis); //底盘初始化
    Trigger_init();         //拨盘初始化
    //TODO:判断底盘电机是否都在线

    htim5.Instance->CCR1 = 1000;
    while (1){
        chassis_ctrl_info_get();        //遥控器获取底盘方向矢量
        volatile_judge = get_battery_voltage();//电压测试

        chassis_control();              //底盘控制
        Trigger_control();              //拨盘控制

        CAN_cmd_motor(CAN_1,
                      CAN_MOTOR_0x1FF_ID,
                      0,                                //205
                      0,                                //206
                      2000,//launcher.trigger.give_current,    //207
                      0);                               //208

        //  底盘相关模块 对底盘进行离线处理
        //  chassis_device_offline_handle();
        //  chassis_power_stop();

        Send_referee(Referee.PowerHeatData.chassis_power);
        vTaskDelay(1);
        Send_id(Referee.GameRobotStat.robot_id);

        // 底盘不是失能状态时进行麦轮运动
        if(chassis.mode != CHASSIS_RELAX) {
            chassis_meknum_wheel_cal();     //麦轮解算
            chassis_power_limit();          //功率限制
            chassis_wheel_loop_cal();       //驱电机闭环

#define CAN_CMD_CHASSIS
#ifdef CAN_CMD_CHASSIS
            CAN_cmd_motor(CAN_1,
                          CAN_MOTOR_0x200_ID,
                          chassis.motor_chassis[RF].give_current,   //201
                          chassis.motor_chassis[LF].give_current,   //202
                          chassis.motor_chassis[LB].give_current,   //203
                          chassis.motor_chassis[RB].give_current    //204
                          );
#else //CAN_CMD_CHASSIS_0
            CAN_cmd_motor(CAN_1,
                          CAN_MOTOR_0x200_ID,
                          0,//chassis.motor_chassis[RF].give_current,   //201
                          0,//chassis.motor_chassis[LF].give_current,   //202
                          0,//chassis.motor_chassis[LB].give_current,   //203
                          0
                          );//chassis.motor_chassis[RB].give_current
#endif
            //计算底盘在方向上的线速度
            chassis_vx = ((float)(chassis.motor_chassis[1].motor_measure->speed_rpm
                                  + chassis.motor_chassis[2].motor_measure->speed_rpm
                                  - chassis.motor_chassis[0].motor_measure->speed_rpm
                                  - chassis.motor_chassis[3].motor_measure->speed_rpm
            )*(M3508_MOTOR_RPM_TO_VECTOR)) / 4;
            chassis_vy = ((float)(chassis.motor_chassis[2].motor_measure->speed_rpm
                                  - chassis.motor_chassis[1].motor_measure->speed_rpm
                                  - chassis.motor_chassis[0].motor_measure->speed_rpm
                                  + chassis.motor_chassis[3].motor_measure->speed_rpm
            )*(M3508_MOTOR_RPM_TO_VECTOR)) / 4;
            chassis_vw = ((float)(-chassis.motor_chassis[2].motor_measure->speed_rpm
                                  - chassis.motor_chassis[1].motor_measure->speed_rpm
                                  - chassis.motor_chassis[0].motor_measure->speed_rpm
                                  - chassis.motor_chassis[3].motor_measure->speed_rpm
            )*(M3508_MOTOR_RPM_TO_VECTOR)) / 4;

            time=HAL_GetTick();             //获取当前系统时间
            test_om=time-time_last_time;    //计算时间间隔

            //计算在各个方向上的位移
            test_x=test_x + ((double)test_om * chassis_vx*0.001);
            test_y=test_y + ((double)test_om * chassis_vy*0.001);
            test_w=test_w + ((double)test_om * chassis_vw*0.001);
            time_last_time = time;
        }
        vTaskDelay(CHASSIS_PERIOD);
    }
}


/**
  * @brief          底盘初始化
  * @param[in]      pvParameters
  * 获取底盘四个轮子、trigger 电机数据
  * 初始化底盘四个轮子、底盘跟随云台模式、和 trigger 电机的角度环和速度环 PID
  * 将底盘和 trigger 电机的电流给定为 0，即失能
  * 设置功率限制时的缓冲能量和功率的警告线
  * 初始化主动单发的 total_ecd、offset_ecd 和 ecd
  * @retval         none
  */
static void chassis_init(chassis_t *chassis) {
    if (chassis == NULL) return;

    //底盘四个驱动电机速度环初始化和电机数据结构体获取
    for (int i = 0; i < 4; i++) {
        chassis->motor_chassis[i].motor_measure = motor_3508_measure+i;
        pid_init(&chassis->motor_chassis[i].speed_p,
                 CHASSIS_3508_PID_MAX_OUT, CHASSIS_3508_PID_MAX_IOUT,
                 CHASSIS_3508_PID_KP, CHASSIS_3508_PID_KI, CHASSIS_3508_PID_KD);
    }
    //底盘跟随云台初始化
    pid_init(&chassis->chassis_vw_pid,
             CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT,
             CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD);

    //初始时底盘模式为失能，电流给定为0
    chassis->motor_chassis[RF].give_current = 0;
    chassis->motor_chassis[LF].give_current = 0;
    chassis->motor_chassis[LB].give_current = 0;
    chassis->motor_chassis[RB].give_current = 0;

    //chassis_power_limit;功率限制时的缓冲能量和功率的警告线
    chassis->chassis_power_limit.warning_buff = 35.f;
    chassis->chassis_power_limit.warning_power = 40.f;

    ramp_init(&chassis->chassis_vx_ramp, 0.0001f, MAX_CHASSIS_VX_SPEED, -MAX_CHASSIS_VX_SPEED);
    ramp_init(&chassis->chassis_vy_ramp, 0.0001f, MAX_CHASSIS_VY_SPEED, -MAX_CHASSIS_VY_SPEED);
    ramp_init(&chassis->chassis_vw_ramp, 0.0001f, MAX_CHASSIS_VW_SPEED, -MAX_CHASSIS_VW_SPEED);
    ramp_init(&chassis->chassis_3508_ramp[LF], 0.0001f, M3508_MAX_RPM, -M3508_MAX_RPM);
    ramp_init(&chassis->chassis_3508_ramp[RF], 0.0001f, M3508_MAX_RPM, -M3508_MAX_RPM);
    ramp_init(&chassis->chassis_3508_ramp[RB], 0.0001f, M3508_MAX_RPM, -M3508_MAX_RPM);
    ramp_init(&chassis->chassis_3508_ramp[LB], 0.0001f, M3508_MAX_RPM, -M3508_MAX_RPM);

    //设定底盘最大功率，比赛根据裁判系统反馈动态设定
    chassis->chassis_power_limit.power_set = 50;
}


/******************/
/**
  * @brief          电容开启控制底盘加速度
  * @param[in]      none
  * 通过裁判系统的功率限制调整底盘加速度
  * @retval         返回空
  */
static float chassis_speed_change() {
    float speed_change = 0;
    switch (Referee.GameRobotStat.chassis_power_limit) {//最大限制功率
        case 55: {
            if(abs(chassis.vx) > 2) {
                speed_change=(float)0;
                chassis.vx = (float)2;
            }
            else speed_change=(float)0.01;
            if(abs(chassis.vy) > 2) {
                speed_change=(float)0;
                chassis.vy = (float)2;
            }
        } break;

        case 60: {
            if(abs(chassis.vx) > 2.1) {
                speed_change=(float)0;
                chassis.vx = (float)2.1;
            }
            else speed_change=(float)0.012;
            if(abs(chassis.vy) > 2.1) {
                speed_change=(float)0;
                chassis.vy = (float)2.1;
            }
        } break;

        case 65: {
            if(abs(chassis.vx) > 2.2) {
                speed_change=(float)0;
                chassis.vx = (float)2.2;
            }
            else speed_change=(float)0.012;
            if(abs(chassis.vy) > 2.2) {
                speed_change=(float)0;
                chassis.vy = (float)2.2;
            }
        } break;

        case 70: {
            if(abs(chassis.vx) > 2.3) {
                speed_change=(float)0;
                chassis.vx = (float)3;
            }
            else speed_change=(float)0.013;
            if(abs(chassis.vy) > 2.3) {
                speed_change=(float)0;
                chassis.vy = (float)2.3;
            }
        } break;

        case 75: {
            if(abs(chassis.vx) > 2.4) {
                speed_change=(float)0;
                chassis.vx = (float)2.4;
            }
            else speed_change=(float)0.013;
            if(abs(chassis.vy) > 2.4) {
                speed_change=(float)0;
                chassis.vy = (float)2.4;
            }
        } break;

        case 80: {
            if(abs(chassis.vx) > 2.5) {
                speed_change=(float)0;
                chassis.vx = (float)2.5;
            }
            else speed_change=(float)0.014;
            if(abs(chassis.vy) > 2.5) {
                speed_change=(float)0;
                chassis.vy = (float)2.5;
            }
        } break;

        case 85: {
            if(abs(chassis.vx) > 2.6) {
                speed_change=(float)0;
                chassis.vx = (float)2.6;
            }
            else speed_change=(float)0.014;
            if(abs(chassis.vy) > 2.6) {
                speed_change=(float)0;
                chassis.vy = (float)2.6;
            }
        } break;

        case 90: {
            if(abs(chassis.vx) > 2.7) {
                speed_change=(float)0;
                chassis.vx = (float)2.7;
            }
            else speed_change=(float)0.015;
            if(abs(chassis.vy) > 2.8) {
                speed_change=(float)0;
                chassis.vy = (float)2.8;
            }
        } break;

        case 100: {
            if(abs(chassis.vx) > 2.8) {
                speed_change=(float)0;
                chassis.vx = 2.8;
            }
            else speed_change=(float)0.016;
            if(abs(chassis.vy) > 2.8) {
                speed_change=(float)0;
                chassis.vy = 2.8;
            }
        } break;

        case 105: {
            if(abs(chassis.vx) > 2.9) {
                speed_change=(float)0;
                chassis.vx = (float)2.9;
            }
            else speed_change=(float)0.016;
            if(abs(chassis.vy) > 2.9) {
                speed_change=(float)0;
                chassis.vy = (float)2.9;
            }
        } break;

        case 110: {
            if(abs(chassis.vx) > 3) {
                speed_change=(float)0;
                chassis.vx = (float)3;
            }
            else speed_change=(float)0.016;
            if(abs(chassis.vy) > 3) {
                speed_change=(float)0;
                chassis.vy = (float)3;
            }
        } break;

        case 120: {
            if(abs(chassis.vx) > 3) {
                speed_change=(float)0;
                chassis.vx = 3;
            }
            else speed_change=(float)0.02;
            if(abs(chassis.vy) > 3) {
                speed_change=(float)0;
                chassis.vy = 3;
            }
        } break;

        default:{
            if(abs(chassis.vx) > 2) {
                speed_change=(float)0;
                chassis.vx = 2;
            }
            else speed_change=(float)0.02;
            if(abs(chassis.vy) > 2) {
                speed_change=(float)0;
                chassis.vy = 2;
            }
        }break;
    }
    return speed_change;
}


/**
  * @brief          键盘和遥控器控制
  * @param[in]      none
  * WSAD底盘移动，QE左右小陀螺
  * @retval         返回空
  */
static void chassis_pc_ctrl(){
    //获取加速度
    float speed_change=chassis_speed_change();
        //键盘控制下的底盘以斜坡式变化
        if(KeyBoard.W.status==KEY_PRESS) {
            chassis.vx_pc+=speed_change;//速度增量
        }
        else if(KeyBoard.S.status==KEY_PRESS) {
            chassis.vx_pc-=speed_change;
        }
        else{
            chassis.vx_pc=0;
        }
        if(KeyBoard.A.status==KEY_PRESS) {
            chassis.vy_pc-=speed_change;//=-MAX_CHASSIS_VY_SPEED;
        }
        else if(KeyBoard.D.status==KEY_PRESS) {
            chassis.vy_pc+=speed_change;
        }
        else{
            chassis.vy_pc=0;
        }
}


/**
  * @brief          初始化键盘和遥控器控制底盘
  * @param[in]      none
  * 初始化键盘信息，通过键盘信息和遥控器信息控制底盘移动
  * @retval         返回空
  */
static void chassis_ctrl_info_get() {

    chassis_pc_ctrl();//根据键盘信息 获取vx_pc,vy_pc

    chassis.vx_set=(float)(rc_ctrl.rc.ch[CHASSIS_X_CHANNEL])*RC_TO_VX
           +chassis.vx_pc;
    chassis.vy_set=(float) (rc_ctrl.rc.ch[CHASSIS_Y_CHANNEL])*RC_TO_VY
           +chassis.vy_pc;
    chassis.vw_set=(float)(rc_ctrl.rc.ch[CHASSIS_Z_CHANNEL])*RC_TO_VW
           +chassis.vw_pc;

    chassis.vx=chassis.vx_set;
    chassis.vy=chassis.vy_set;
    chassis.vw=chassis.vw_set;

    first_order_filter_cali(&vx_slow,chassis.vx_set);
    first_order_filter_cali(&vy_slow,chassis.vy_set);//缓慢上升不要太快
    first_order_filter_cali(&vw_slow,chassis.vw_set);//缓慢上升不要太快

    chassis.vx=vx_slow.out;
    chassis.vy=vy_slow.out;
    chassis.vw=vw_slow.out;
}


/**
  * @brief          底盘模式选择，yaw轴模式选择
  * @param[in]      pvParameters
  * 底盘五种模式：底盘失能，跟随，独立，小陀螺，刹车
  * 云台一种模式：trigger失能
  * @retval         none
  */
static void chassis_control() {
    switch (chassis.mode) {
        case CHASSIS_RELAX: //底盘失能
            chassis_relax_handle();
            break;
        case CHASSIS_ONLY:  //底盘独立(以初始化数据控制)
            break;
        case CHASSIS_FOLLOW_GIMBAL: //跟随环 覆盖底盘 vw;
            chassis_follow_gimbal_handle();
            break;
        case CHASSIS_SPIN:  //小陀螺 vw设置常量(150)
            chassis_spin_handle();
            break;
        case CHASSIS_INDEPENDENT_CONTROL:
            chassis_independent_handle();
            break;
        case CHASSIS_BLOCK: ///底盘刹车，从键盘传指令刹车指令
            chassis.vx=0;
            chassis.vy=0;
            chassis.vw=0;
            break;
    }

    switch (gimbal.mode) {
        case GIMBAL_RELAX://云台失能
            Trigger_relax_handle();
            break;
        default:
            break;
    }
}