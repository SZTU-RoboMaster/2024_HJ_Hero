//
// Created by Gularx on 2024/2/23.
//

#include "Gimbal.h"

extern chassis_t chassis;       // 底盘
extern gimbal_t gimbal;         //云台
extern launcher_t launcher;     //发射机构
extern key_board_t KeyBoard;    //键盘
extern RC_ctrl_t rc_ctrl;       //遥控器
extern robot_ctrl_info_t robot_ctrl;

extern first_order_filter_type_t filter_pitch_gyro_in;
extern first_order_filter_type_t filter_yaw_gyro_in;
extern first_order_filter_type_t mouse_in_y;
extern first_order_filter_type_t mouse_in_x;
extern first_order_filter_type_t auto_pitch;
extern first_order_filter_type_t auto_yaw[2];
extern fp32 gyro_pitch;
extern fp32 gyro_yaw;

/**
  * @brief          云台失能模式(四个电机)
  * @param[in]      none
  * pitch 轴，左右摩擦轮，主动单发的给定电流设置为 0
  * @retval         返回空
  */
void gimbal_relax_handle(){
    gimbal.yaw.give_current=0;
    gimbal.pitch.give_current = 0;
    launcher.fire_r.give_current = 0;
    launcher.fire_l.give_current = 0;
    launcher.single_shoot.give_current = 0;
}

//一键掉头
static void gimbal_turn_back_judge(){
    if(KeyBoard.R.click_flag == 1){
        KeyBoard.R.click_flag = 0;
        gimbal.yaw.absolute_angle_set+=180;
    }
}


/**
  * @brief          使能模式, 得到遥控器对云台电机的控制
  * @param[in]      none
  * 通过遥控器左摇杆或鼠标控制 pitch 轴和 yaw 轴，对角度值进行限幅
  * @retval         返回空
  */
void gimbal_active_handle() {

    if (KeyBoard.CTRL.status == KEY_PRESS) {

        if (KeyBoard.B.status == KEY_CLICK) {
            // MOUSE_X_RADIO 是鼠标输入的比例系数
            gimbal.pitch.absolute_angle_set = (fp32) (gimbal.pitch.absolute_angle_set + (fp32) (0.3f));
        }
        else if (KeyBoard.C.status == KEY_CLICK) {
            gimbal.pitch.absolute_angle_set = (fp32) (gimbal.pitch.absolute_angle_set - (fp32) (0.3f));
        }
        else {
            gimbal.pitch.absolute_angle_set = gimbal.pitch.absolute_angle_set;
            gimbal.yaw.absolute_angle_set = gimbal.yaw.absolute_angle_set;
        }
    }
    //鼠标输入滤波
    else {
        //对输入滤波器进行校准，将鼠标的x坐标作为输入
        first_order_filter_cali(&mouse_in_x, rc_ctrl.mouse.x);//输入滤波
        //在yaw期望值上,按遥控器或者鼠标进行增减
        gimbal.yaw.absolute_angle_set -=
                (float) rc_ctrl.rc.ch[YAW_CHANNEL] * RC_TO_YAW * GIMBAL_RC_MOVE_RATIO_YAW
                + (float) mouse_in_x.out * MOUSE_X_RADIO;    // rc_ctrl.mouse.x

        //一键掉头判断
        gimbal_turn_back_judge();

        //对输入滤波器进行校准，将鼠标的y坐标作为输入
        first_order_filter_cali(&mouse_in_y, rc_ctrl.mouse.y);//输入滤波
        //在pit期望值上,按遥控器或者鼠标进行增减
        gimbal.pitch.absolute_angle_set +=
                (float) rc_ctrl.rc.ch[PITCH_CHANNEL] * RC_TO_PITCH * GIMBAL_RC_MOVE_RATIO_PIT
                - mouse_in_y.out * MOUSE_Y_RADIO;  // rc_ctrl.mouse.y

        //云台绕圈时进行绝对角循环设置
        if (gimbal.yaw.absolute_angle_set >= 180) {
            gimbal.yaw.absolute_angle_set -= 360;
        }
            //当yaw期望值超过180度时，将其调整到[-180,180]的范围
        else if (gimbal.yaw.absolute_angle_set <= -180) {
            gimbal.yaw.absolute_angle_set += 360;
        }

        //对pit期望值进行动态限幅（通过陀螺仪和编码器得到动态的限位）
        gimbal.pitch.absolute_angle_set = fp32_constrain(gimbal.pitch.absolute_angle_set,
                                                         MIN_ABS_ANGLE,
                                                         MAX_ABS_ANGLE);
//    gimbal.pitch.absolute_angle_set=fp32_constrain(gimbal.pitch.absolute_angle_set,
//                                                   MIN_ABS_ANGLE+
//                                                   gimbal.pitch.absolute_angle_get - gimbal.pitch.relative_angle_get,
//                                                   MAX_ABS_ANGLE+
//                                                   gimbal.pitch.absolute_angle_get - gimbal.pitch.relative_angle_get);

    }
}


/**
  * @brief          自瞄处理得到对云台电机的控制
  * @param[in]      none
  * 获取视觉发送的角度误差转动 pitch 轴和 yaw 轴
  * @retval         返回空
  */
fp32 radio_y=0.0098f;
fp32 radio_p=0.01f;
void gimbal_auto_handle() {
    //获取视觉发送的角度误差
    first_order_filter_cali(&auto_pitch, robot_ctrl.pitch);
    first_order_filter_cali(&auto_yaw[0], sinf(robot_ctrl.yaw / 180.0f * PI));//yaw数据分解成x
    first_order_filter_cali(&auto_yaw[1], cosf(robot_ctrl.yaw / 180.0f * PI));//yaw数据分解成y
    gimbal.yaw.absolute_angle_set = atan2f(auto_yaw[0].out, auto_yaw[1].out) * 180.0f / PI;//在此处做合成
    gimbal.pitch.absolute_angle_set = auto_pitch.out;
    //云台绕圈时进行绝对角循环设置
    if(gimbal.yaw.absolute_angle_set>=180){
        gimbal.yaw.absolute_angle_set-=360;
    }
    else if(gimbal.yaw.absolute_angle_set<=-180){
        gimbal.yaw.absolute_angle_set+=360;
    }
    //对pit期望值进行动态限幅（通过陀螺仪和编码器得到动态的限位）
    gimbal.pitch.absolute_angle_set = fp32_constrain(gimbal.pitch.absolute_angle_set,
                                                     MIN_ABS_ANGLE,
                                                     MAX_ABS_ANGLE);
//    gimbal.pitch.absolute_angle_set = fp32_constrain(gimbal.pitch.absolute_angle_set,
//                                                     MIN_ABS_ANGLE +
//                                                     gimbal.pitch.absolute_angle_get - gimbal.pitch.relative_angle_get,
//                                                     MAX_ABS_ANGLE +
//                                                     gimbal.pitch.absolute_angle_get - gimbal.pitch.relative_angle_get);
//    gimbal.pitch.absolute_angle_set = 0;

}


/**
  * @brief          云台 pitch 电机闭环控制函数
  * @param[in]      none
  * 实现 Pitch 轴的控制计算，包括陀螺仪期望值的计算和电流输出的计算。
  * 对期望角度与实际角度之间的差值进行处理，确保该差值在合理范围内。
  * 避免由于角度环形性质导致PID控制过程中的不稳定性。
  * @retval         返回空
  */
void gimbal_ctrl_loop_cal(){
    //计算yaw轴的控制输出
    if(abs(gimbal.yaw.absolute_angle_set-gimbal.yaw.absolute_angle_get)<0.5){
        gimbal.yaw.gyro_set=pid_loop_calc(&gimbal.yaw.auto_angle_p,
                                          gimbal.yaw.absolute_angle_get,
                                          gimbal.yaw.absolute_angle_set,
                                          180,
                                          -180);//gimbal.yaw.absolute_angle_set

        first_order_filter_cali(&filter_yaw_gyro_in, gyro_yaw);

        gimbal.yaw.give_current = (int16_t)-pid_calc(&gimbal.yaw.auto_speed_p,
                                                     gyro_yaw,//gimbal.yaw.motor_measure->speed_rpm,
                                                     gimbal.yaw.gyro_set);
    }
    else {
        gimbal.yaw.gyro_set= pid_loop_calc(&gimbal.yaw.angle_p,
                                           gimbal.yaw.absolute_angle_get,
                                           gimbal.yaw.absolute_angle_set,
                                           180,
                                           -180);//gimbal.yaw.absolute_angle_set

        first_order_filter_cali(&filter_yaw_gyro_in, gyro_yaw);

        gimbal.yaw.give_current = (int16_t)-pid_calc(&gimbal.yaw.speed_p,
                                                    filter_yaw_gyro_in.out,//gimbal.yaw.motor_measure->speed_rpm,
                                                     gimbal.yaw.gyro_set);

    }
    if(rc_ctrl.rc.ch[3] == 0) {
        gimbal.pitch.give_current = 0;
    }
    else {
        if (gimbal.pitch.absolute_angle_get < 0 && rc_ctrl.rc.ch[3] < 0) {
            //计算pitch轴的控制输出
            gimbal.pitch.gyro_set = pid_calc(&gimbal.pitch.auto_angle_p,
                                             gimbal.pitch.absolute_angle_get,
                                             gimbal.pitch.absolute_angle_set);//Vision_info.pitch.value
            first_order_filter_cali(&filter_pitch_gyro_in, gyro_pitch);

            ///// 读取陀螺仪的角速度加在内环的期望上面
            gimbal.pitch.give_current = (int16_t) pid_calc(&gimbal.pitch.auto_speed_p,
                                                           filter_pitch_gyro_in.out,
                                                           gimbal.pitch.gyro_set);//加负号为了电机反转
        } else {
            //计算pitch轴的控制输出
            gimbal.pitch.gyro_set = pid_calc(&gimbal.pitch.angle_p,
                                             gimbal.pitch.absolute_angle_get,
                                             gimbal.pitch.absolute_angle_set);//Vision_info.pitch.value
            first_order_filter_cali(&filter_pitch_gyro_in, gyro_pitch);

            ///// 读取陀螺仪的角速度加在内环的期望上面
            gimbal.pitch.give_current = (int16_t) pid_calc(&gimbal.pitch.speed_p,
                    //gimbal.pitch.motor_measure->speed_rpm,
                                                           filter_pitch_gyro_in.out,
                                                           gimbal.pitch.gyro_set);//加负号为了电机反转
        }

    }
    //gimbal.pitch.give_current= (int16_t)Apply(&pitch_current_out,gimbal.pitch.give_current);
}


/**
  * @brief          云台 pitch 电机闭环控制函数
  * @param[in]      none
  * 实现 Pitch 轴的控制计算，包括陀螺仪期望值的计算和电流输出的计算。
  * 对期望角度与实际角度之间的差值进行处理，确保该差值在合理范围内。
  * 避免由于角度环形性质导致PID控制过程中的不稳定性。
  * @retval         返回空
  */
void gimbal_auto_ctrl_loop_cal(){
    //计算yaw轴的控制输出
    gimbal.yaw.gyro_set= pid_loop_calc(&gimbal.yaw.angle_p,
                                       gimbal.yaw.absolute_angle_get,
                                       gimbal.yaw.absolute_angle_set,
                                       180,
                                       -180);//gimbal.yaw.absolute_angle_set

    first_order_filter_cali(&filter_yaw_gyro_in, gyro_yaw);

    gimbal.yaw.give_current = (int16_t)-pid_calc(&gimbal.yaw.speed_p,
                                                 gyro_yaw,//gimbal.yaw.motor_measure->speed_rpm,
                                                 gimbal.yaw.gyro_set);

//    gimbal.pitch.give_current= (int16_t)pid_calc(&gimbal.pitch.speed_p,
//                                                 filter_pitch_gyro_in.out,
////                                                 gimbal.pitch.motor_measure->speed_rpm,
//                                                 gimbal.pitch.gyro_set);//加负号为了电机反转
    //计算pitch轴的控制输出
    gimbal.pitch.gyro_set= pid_calc(&gimbal.pitch.auto_angle_p,
                                    gimbal.pitch.absolute_angle_get,
                                    gimbal.pitch.absolute_angle_set);


    first_order_filter_cali(&filter_pitch_gyro_in,gyro_pitch);
///// 读取陀螺仪的角速度加在内环的期望上面
    gimbal.pitch.give_current= (int16_t)pid_calc(&gimbal.pitch.auto_speed_p,
                                                 filter_pitch_gyro_in.out,
                                                 gimbal.pitch.gyro_set);
}



/**
  * @brief          云台上相关电机离线 则给电机0电流
  * @param[in]      none
  * 处理云台（gimbal）和发射器（launcher）中各个电机设备离线（OFFLINE）的情况。
  * 当检测到某个设备离线时，将相应电机的给定电流设置为0，以确保在设备离线的情况下停止对应电机的运动。
  * @retval         返回空
  */
///TODO:还没有用到的
void gimbal_device_offline_handle() {
    if(detect_list[DETECT_REMOTE].status == OFFLINE){
        gimbal.pitch.give_current = 0;
        gimbal.yaw.give_current = 0;
        launcher.trigger.give_current = 0;
    }
    if (detect_list[DETECT_GIMBAL_6020_PITCH].status == OFFLINE) {
        gimbal.pitch.give_current = 0;
    }
    if (detect_list[DETECT_GIMBAL_6020_YAW].status == OFFLINE) {
        gimbal.yaw.give_current = 0;
    }
    if (detect_list[DETECT_LAUNCHER_3508_FIRE_L].status == OFFLINE) {
        launcher.fire_l.give_current = 0;
    }
    if (detect_list[DETECT_LAUNCHER_3508_FIRE_R].status == OFFLINE) {
        launcher.fire_r.give_current = 0;
    }
    if (detect_list[DETECT_LAUNCHER_3508_TRIGGER].status == OFFLINE) {
        launcher.trigger.give_current = 0;
    }
    if (detect_list[DETECT_LAUNCHER_2006_SINGLE_SHOT].status == OFFLINE) {
        launcher.single_shoot.give_current = 0;
    }
}


/**
  * @brief          云台上相关电机离线 则给电机0电流
  * @param[in]      none
  * 这段代码是一个云台（gimbal）和发射器（launcher）的电源状态监测函数。
  * 主要作用是检测裁判系统发送的机器人主电源和发射器电源的状态
  * @retval         返回空
  */
void gimbal_power_stop(){
    //检测底盘电源状态
    if(Referee.GameRobotStat.power_management_gimbal_output==0){
//        gimbal.mode=GIMBAL_RELAX;
    }

    //检测发射机构电源状态
    if(Referee.GameRobotStat.power_management_shooter_output==0){
        launcher.fire_l.give_current = 0;
        launcher.fire_r.give_current = 0;
        launcher.trigger.give_current = 0;
        launcher.single_shoot.give_current = 0;
    }
}


/**
  * @brief          UI 更新---云台模式
  * @param[in]      none
  * 更新 can_send 模式，将数据用数组保存起来
  * @retval         返回空
  */
void gimbal_can_send_back_mapping(){
    int16_t *real_motor_give_current[5];

    real_motor_give_current[0] = &launcher.fire_l.give_current;
    real_motor_give_current[1] = &gimbal.pitch.give_current;
    real_motor_give_current[2] = &launcher.fire_r.give_current;
    real_motor_give_current[3] = &launcher.trigger.give_current;
    real_motor_give_current[4] = &launcher.single_shoot.give_current;
}


/**
  * @brief          UI信息
  * @param[in]      none
  * 为用户界面（UI）封装云台（gimbal）和发射器（launcher）的信息
  * 以便将这些信息传输到UI系统中，用于显示和监控机器人状态。
  * @retval         返回空
  */
void gimbal_uiInfo_packet(){
    // 将云台的运动模式信息存储到UI系统的机器人状态结构体中的 gimbal_mode 字段中。
    ui_robot_status.gimbal_mode=gimbal.mode;
    // 将发射器的射击模式信息存储到UI系统的机器人状态结构体中的 fire_mode 字段中。
    ui_robot_status.fire_mode=launcher.fire_mode;
    // 将云台Pitch轴的绝对角度信息存储到UI系统的机器人状态结构体中的 pitch_value 字段中。
    ui_robot_status.pitch_value=gimbal.pitch.absolute_angle_get;
    // 将云台Yaw轴的相对角度信息存储到UI系统的机器人状态结构体中的 relative_yaw_value 字段中。
    ui_robot_status.relative_yaw_value=gimbal.yaw.relative_angle_get;
}