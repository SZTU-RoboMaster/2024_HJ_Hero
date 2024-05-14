#include "Trigger_function.h"
#include "key_board.h"

extern gimbal_t gimbal;
extern launcher_t launcher;
extern RC_ctrl_t rc_ctrl;       //遥控器
extern key_board_t KeyBoard;    //键盘
extern trigger_t trigger;
static first_order_filter_type_t filter_trigger_rpm_in;
extern robot_ctrl_info_t robot_ctrl;
extern uint8_t fire_lock;
// 拨盘初始化
void Trigger_init(){
    // 获取 trigger 电机数据
    launcher.trigger.motor_measure=&motor_3508_measure[TRIGGER];

    pid_init(&launcher.trigger.angle_p,
             TRIGGER_ANGLE_PID_MAX_OUT, TRIGGER_ANGLE_PID_MAX_IOUT,
             TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD
    );

    pid_init(&launcher.trigger.speed_p,
             TRIGGER_SPEED_PID_MAX_OUT, TRIGGER_SPEED_PID_MAX_IOUT,
             TRIGGER_SPEED_PID_KP, TRIGGER_SPEED_PID_KI, TRIGGER_SPEED_PID_KD
    );
    pid_init(&launcher.trigger.speed_p_1,
             TRIGGER_PID_MAX_OUT, TRIGGER_PID_MAX_IOUT,
             TRIGGER_PID_KP, TRIGGER_PID_KI, TRIGGER_PID_KD
    );

    launcher.trigger.give_current = 0;
    launcher.trigger.motor_measure->total_ecd=launcher.trigger.motor_measure->offset_ecd=launcher.trigger.motor_measure->ecd;

    //最开始的编码值作为拨轮电机的校准值
    first_order_filter_init(&filter_trigger_rpm_in,1,1);
}

void Trigger_control() {

    switch (trigger.state) {
        case KEEP_ROTATING:
            Keep_rotating();
            break;
        case ROTATION_ANGLE:
            Rotation_angle();
            break;
        default:
            break;
    }
}

static uint8_t rc_last_sw_L;
// 拨盘控制(一直转)
void Keep_rotating() {
    if(gimbal.mode == GIMBAL_RELAX) {
        Trigger_relax_handle();
    }
    else if(gimbal.mode != GIMBAL_RELAX) {
        if(launcher.fire_mode == Fire_ON) {
            if(switch_is_mid(rc_ctrl.rc.s[RC_s_L])){
                if(launcher.trigger.give_current > 25000) {
                    launcher.trigger.speed = 0;
                }
                else {
                    launcher.trigger.speed = TRIGGER_SPEED;
                }
            }
            else {
                launcher.trigger.speed = 0;
            }
        }
        else {
            launcher.trigger.speed = 0;
        }
        launcher.trigger.give_current = (int16_t)pid_calc(&launcher.trigger.speed_p_1,
                                                          launcher.trigger.motor_measure->speed_rpm,
                                                          launcher.trigger.speed);
    }
}



#include "tim.h"
// 拨盘控制(转角度)

static int32_t total_ecd_ref_tri=0;
static uint32_t trigger_time=0;
static uint32_t unfinshed_time=0;
static uint8_t trigger_block_flag=0;
static int32_t key_last_mouse_l=0;
void Rotation_angle() {
    if(gimbal.mode==GIMBAL_RELAX) {
        Trigger_relax_handle();
    }
    else if(gimbal.mode != GIMBAL_RELAX) {
        if (launcher.fire_mode == Fire_ON) {
            if (launcher.single_shoot_cmd == SHOOT_ING && (!switch_is_down(rc_last_sw_L) && switch_is_down(rc_ctrl.rc.s[RC_s_L]))) { //单发状态
                trigger_time=HAL_GetTick(); //这时候开始计时，开始转的时候计时
                total_ecd_ref_tri = launcher.trigger.motor_measure->total_ecd - DEGREE_60_TO_ENCODER;
                trigger_block_flag = 0;
            }
            else if(launcher.single_shoot_cmd == SHOOT_ING && (key_last_mouse_l != KEY_PRESS && KeyBoard.Mouse_l.status == KEY_PRESS)){
                trigger_time=HAL_GetTick(); //这时候开始计时，开始转的时候计时
                total_ecd_ref_tri = launcher.trigger.motor_measure->total_ecd - DEGREE_60_TO_ENCODER;
                trigger_block_flag = 0;
            }
            else if(launcher.single_shoot_cmd == SHOOT_ING && fire_lock > 0){
                trigger_time=HAL_GetTick(); //这时候开始计时，开始转的时候计时
                total_ecd_ref_tri = launcher.trigger.motor_measure->total_ecd - DEGREE_60_TO_ENCODER;
                trigger_block_flag = 0;
            }
            else if(launcher.single_shoot_cmd == SHOOT_ING && trigger.trigger_flag == trigger_block) {    //已经堵转了，把期望设置成上一次的期望，需要再拨一次杆才能回去之前的角度
                trigger_time=HAL_GetTick(); //这时候开始计时，开始转的时候计时(反转也需要重新计时)
                total_ecd_ref_tri = total_ecd_ref_tri + DEGREE_60_TO_ENCODER;
                trigger_block_flag = 1;
            }
            launcher.trigger.speed = pid_calc(&launcher.trigger.angle_p,
                                              launcher.trigger.motor_measure->total_ecd,
                                              total_ecd_ref_tri);
            launcher.trigger.give_current =
                    (int16_t)pid_calc(&launcher.trigger.speed_p,
                                      launcher.trigger.motor_measure->speed_rpm,
                                      launcher.trigger.speed);
        }
        else {
            launcher.trigger.give_current=0;
            total_ecd_ref_tri=launcher.trigger.motor_measure->total_ecd;
        }
    }
    trigger_finish_judge(); //拨盘有无转到理想角度  判断有无卡
    rc_last_sw_L=rc_ctrl.rc.s[RC_s_L];
    key_last_mouse_l = KeyBoard.Mouse_l.status;
}


void trigger_finish_judge() {
    if(ABS(total_ecd_ref_tri - launcher.trigger.motor_measure->total_ecd) > 2000 && (trigger_block_flag == 0)) { //误差过大认为上一次发射还未完成
        // 目前问题:有多次堵转检测出现的情况,特别是最后一格有弹丸,导致堵转检测被触发,ecd值多次回退
        // 需要对堵转反转进行特殊处理

        // 待封装
        // 目前解决方法:对发弹和堵转设置两个状态trigger_block_flag 0:发弹 1:堵转
        // 在上次已经执行了堵转之后,状态会更新为1,就不会再进行堵转检测
        trigger.trigger_flag = trigger_unfinished;
    }
    else {
        trigger.trigger_flag = trigger_finished;
    }
    if(trigger.trigger_flag == trigger_unfinished && (HAL_GetTick() - trigger_time) > 500) {  //期望角度没转到并且卡了0.5s就认为是卡弹了
        trigger.trigger_flag = trigger_block;
    }
}



// 拨盘失能
void Trigger_relax_handle(){
    launcher.trigger.give_current=0;
    total_ecd_ref_tri = (int32_t )launcher.trigger.motor_measure->total_ecd;
}




