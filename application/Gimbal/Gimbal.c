
/*遥控器键位：

        S1                                                                         S0
                    |                                                   |
                    |                                                   |
                    |                                                   |
                    |                                                   |
           ------------------ 2                                ------------------ 0
                    |                                                   |
                    |                                                   |
                    |                                                   |
                    3                                                   1

*/

/******************include******************/
#include "Gimbal.h"

/******************变量******************/
extern chassis_t chassis;       // 底盘
extern gimbal_t gimbal;         //云台
extern launcher_t launcher;     //发射机构
extern trigger_t trigger;       //拨盘模式

extern RC_ctrl_t rc_ctrl;       //遥控器
extern key_board_t KeyBoard;    //键盘
extern TIM_HandleTypeDef htim1;

extern fp32 INS_angle[3];
extern fp32 INS_gyro[3];
extern fp32 INS_quat[4];

vision_t vision_data;   //发送机器人id
extern robot_ctrl_info_t robot_ctrl;
first_order_filter_type_t pitch_first_order_set;
first_order_filter_type_t pitch_current_first_order_set;
first_order_filter_type_t filter_yaw_gyro_in;
first_order_filter_type_t filter_pitch_gyro_in;
first_order_filter_type_t mouse_in_y;
first_order_filter_type_t mouse_in_x;
first_order_filter_type_t auto_pitch;
first_order_filter_type_t auto_yaw[2];
first_kalman_filter_t filter_autoYaw;

fp32 gyro_pitch = 0.0f;
fp32 gyro_yaw = 0.0f;

_Noreturn void Gimbal_task(void const*pvParameters);
static void gimbal_init();
static void gimbal_angle_update();
static void pit_offset_get();
/******** while ********/
static void chassis_mode_set();
static void gimbal_mode_set();
static void gimbal_mode_change();
static void gimbal_control();

uint8_t Referee_ID;
uint8_t Referee_Hit_Flag;
uint8_t fire_lock = 0;
/******************函数实现******************/

/**
  * @brief          云台任务
  * @param[in]      pvParameters
  * 初始化，设置模式，实现功能，接受 CAN 信号，云台的具体实现逻辑
  * @retval         返回指针
  */
void Gimbal_task(void const*pvParameters) {
    vTaskDelay(GIMBAL_TASK_INIT_TIME);  //任务初始化时间
    gimbal_init();  //云台初始化
    launcher_init();//发射机构初始化

    while(1) {
        //更新PC的控制信息
        update_pc_info();
        Send_Keyboard(KeyBoard.W.status, KeyBoard.A.status, KeyBoard.S.status, KeyBoard.D.status, trigger.state);

        if(robot_ctrl.fire_command == 1) {
            fire_lock = 1;
        }
        if(fire_lock > 0){
            // 进入开火模式,发送10次can信号
            fire_lock ++;
            if(fire_lock > 5) fire_lock = 0;
        }
        Send_Chassis_Speed(rc_ctrl.rc.ch[1], rc_ctrl.rc.ch[0], rc_ctrl.rc.ch[2], rc_ctrl.rc.s[1], KeyBoard.Mouse_l.status);
        vTaskDelay(1);

        gimbal_angle_update();  //更新绝对、相对角度接收值
        if(detect_list[DETECT_GIMBAL_6020_YAW].status == OFFLINE) {
            gimbal.yaw.relative_angle_get = 0;
            Send_Yaw_Angle(gimbal.yaw.relative_angle_get, fire_lock, KeyBoard.V.click_flag);
        } else {
            Send_Yaw_Angle(gimbal.yaw.relative_angle_get, fire_lock, KeyBoard.V.click_flag);
        }

        ////注意：校准时采用，校准完毕一定注释掉
        //pit_offset_get();

        chassis_mode_set(); //设置底盘模式
        gimbal_mode_set();  //设置云台模式
        images_mode_set();  //设置图传模式
        launcher_mode_set();//设置发射模式

        gimbal_control();  //云台模式设置实现
        launcher_control();//发射机构控制

        Send_Mode(chassis.mode, gimbal.mode, launcher.fire_mode, launcher.single_shoot_cmd);
//        Send_command(robot_ctrl.fire_command);

        //检测电机、电源是否断线  TODO:取消注释会失能，要装裁判系统
//        gimbal_device_offline_handle();//TODO:检测离线,装完裁判系统后可以打开

        gimbal_can_send_back_mapping(); // 接受can信号，传到UI
        vTaskDelay(1);

        //接收CAN信号
        CAN_cmd_motor(CAN_1,
                      CAN_MOTOR_0x1FF_ID,
                      launcher.images.give_current,     //205
                      0,                                //206
                      0,                                //207
                      0                                 //208
        );

        CAN_cmd_motor(CAN_2,
                      CAN_MOTOR_0x200_ID,
                      0,                                //201
                      0,                                //202
                      0,                                //203
                      0//launcher.fire_on.give_current     //204
        );

        CAN_cmd_motor(CAN_2,
                      CAN_MOTOR_0x1FF_ID,
                      gimbal.yaw.give_current,           //205
                      gimbal.pitch.give_current,         //206
                      0,//launcher.fire_r.give_current,      //207
                      0//launcher.fire_l.give_current       //208
        );

        gimbal_uiInfo_packet();     //更新UI云台状态

        vTaskDelay(2);
    }
}


/******************/
/**
  * @brief          云台初始化
  * @param[in]      none
  * 初始化英雄 mode 和 last_mode 的模式为失能，获取 pitch 电机数据
  * 初始化 pitch 轴和 yaw 轴电机的角度环和速度环 PID (包括自瞄)
  * 将 pitch 轴和 yaw 轴电机的电流给定为 0，即失能
  * 初始化时，云台设为未回中状态，校准 pitch 轴和 yaw 轴电机的编码值
  * @retval         返回空
  */
static void gimbal_init(){
    //初始化默认状态为失能
    gimbal.mode=gimbal.last_mode=GIMBAL_RELAX;
    chassis.mode=chassis.last_mode=CHASSIS_RELAX;

    //获取 pitch 和 yaw 电机数据
    gimbal.yaw.motor_measure = &motor_yaw_measure;
    gimbal.pitch.motor_measure=&motor_pitch_measure;

    //pit轴电机 角度环和速度环PID初始化
    pid_init(&gimbal.pitch.angle_p, GIMBAL_PITCH_ANGLE_MAX_OUT,
             GIMBAL_PITCH_ANGLE_MAX_IOUT, GIMBAL_PITCH_ANGLE_PID_KP,
             GIMBAL_PITCH_ANGLE_PID_KI, GIMBAL_PITCH_ANGLE_PID_KD);

    pid_init(&gimbal.pitch.speed_p, GIMBAL_PITCH_SPEED_MAX_OUT,
             GIMBAL_PITCH_SPEED_MAX_IOUT, GIMBAL_PITCH_SPEED_PID_KP,
             GIMBAL_PITCH_SPEED_PID_KI, GIMBAL_PITCH_SPEED_PID_KD);

    //pit轴角度环和速度环PID初始化 自瞄时
    pid_init(&gimbal.pitch.auto_angle_p, GIMBAL_PITCH_AUTO_ANGLE_MAX_OUT,
             GIMBAL_PITCH_AUTO_ANGLE_MAX_IOUT, GIMBAL_PITCH_AUTO_ANGLE_PID_KP,
             GIMBAL_PITCH_AUTO_ANGLE_PID_KI, GIMBAL_PITCH_AUTO_ANGLE_PID_KD);

    pid_init(&gimbal.pitch.auto_speed_p, GIMBAL_PITCH_AUTO_SPEED_MAX_OUT,
             GIMBAL_PITCH_AUTO_SPEED_MAX_IOUT, GIMBAL_PITCH_AUTO_SPEED_PID_KP,
             GIMBAL_PITCH_AUTO_SPEED_PID_KI, GIMBAL_PITCH_AUTO_SPEED_PID_KD);

    //yaw 轴电机角度环和速度环PID初始化
    pid_init(&gimbal.yaw.angle_p,
             GIMBAL_YAW_ANGLE_MAX_OUT, GIMBAL_YAW_ANGLE_MAX_IOUT,
             GIMBAL_YAW_ANGLE_PID_KP, GIMBAL_YAW_ANGLE_PID_KI, GIMBAL_YAW_ANGLE_PID_KD);

    pid_init(&gimbal.yaw.speed_p,
             GIMBAL_YAW_SPEED_MAX_OUT, GIMBAL_YAW_SPEED_MAX_IOUT,
             GIMBAL_YAW_SPEED_PID_KP, GIMBAL_YAW_SPEED_PID_KI, GIMBAL_YAW_SPEED_PID_KD);

    //yaw 轴电机角度环和速度环PID初始化 自瞄时
    pid_init(&gimbal.yaw.auto_angle_p,
             GIMBAL_YAW_AUTO_ANGLE_MAX_OUT, GIMBAL_YAW_AUTO_ANGLE_MAX_IOUT,
             GIMBAL_YAW_AUTO_ANGLE_PID_KP, GIMBAL_YAW_AUTO_ANGLE_PID_KI, GIMBAL_YAW_AUTO_ANGLE_PID_KD);

    pid_init(&gimbal.yaw.auto_speed_p,
             GIMBAL_YAW_AUTO_SPEED_MAX_OUT, GIMBAL_YAW_AUTO_SPEED_MAX_IOUT,
             GIMBAL_YAW_AUTO_SPEED_PID_KP, GIMBAL_YAW_AUTO_SPEED_PID_KI, GIMBAL_YAW_AUTO_SPEED_PID_KD);

    //初始时云台模式为失能，电流给定为0
    gimbal.yaw.give_current = 0;
    gimbal.pitch.give_current=0;

    //pitch轴和yaw轴电机的校准编码值
    gimbal.pitch.motor_measure->offset_ecd=3220;//2370;
    gimbal.yaw.motor_measure->offset_ecd = 6470;//6470;//7400

    //低通滤波初始化
    first_order_filter_init(&pitch_first_order_set, 0.f, 500);
    first_order_filter_init(&pitch_current_first_order_set, 5, 30);
    first_order_filter_init(&filter_yaw_gyro_in, 5, 30);

    first_order_filter_init(&mouse_in_x, 1, 40);
    first_order_filter_init(&mouse_in_y, 1, 10);

    first_order_filter_init(&filter_pitch_gyro_in, 1, 20);
    first_order_filter_init(&auto_pitch, 1, 15);
    first_order_filter_init(&auto_yaw[0], 1, 15);
    first_order_filter_init(&auto_yaw[1], 1, 15);

    first_Kalman_Create(&filter_autoYaw,1,20);  //卡尔曼滤波
}


/**
  * @brief          云台角度更新，与视觉通讯
  * @param[in]      none
  * 根据惯性导航系统的数据更新云台的角度信息，并将相应的信息发送到视觉系统。
  * 其中，角度信息通过陀螺仪测量、电机编码器数据以及惯性导航系统的角度信息等获得。
  * @retval         返回空
  */
static void gimbal_angle_update(){
    // 更新云台俯仰角度信息
    gimbal.pitch.absolute_angle_get=INS_angle[2]*MOTOR_RAD_TO_ANGLE;
    gimbal.pitch.relative_angle_get= motor_ecd_to_angle_change(gimbal.pitch.motor_measure->ecd,
                                                               gimbal.pitch.motor_measure->offset_ecd);
    gimbal.yaw.absolute_angle_get=INS_angle[0]*MOTOR_RAD_TO_ANGLE;
    gimbal.yaw.relative_angle_get= motor_ecd_to_angle_change(gimbal.yaw.motor_measure->ecd,
                                                             gimbal.yaw.motor_measure->offset_ecd);

    // 更新云台底盘相对角 | get
    gimbal.pitch.relative_up_down_get = gimbal.pitch.absolute_angle_get - gimbal.pitch.absolute_angle_get_down;

//     更新云台偏航角度信息,通过陀螺仪测量的偏航和俯仰角速度
    gyro_yaw=INS_gyro[2]*MOTOR_RAD_TO_ANGLE;
    gyro_pitch=-INS_gyro[0]*MOTOR_RAD_TO_ANGLE;

    gimbal.absolute_gyro_yaw = (fp32) INS_gyro[2];
    gimbal.absolute_gyro_pitch = (fp32) INS_gyro[0];

//    if(gimbal.yaw.relative_angle_get > -MAX_ECD_ERROR && gimbal.yaw.relative_angle_get < MAX_ECD_ERROR) {
//        gimbal.yaw.relative_angle_get=0;
//    }

    // 107:蓝 7:红
    if (Referee.GameRobotStat.robot_id<10){
        vision_data.id = 7;
    }else{
        vision_data.id = 107;
    }
    vision_data.yaw = gimbal.yaw.absolute_angle_get;
    vision_data.pitch = gimbal.pitch.absolute_angle_get;
    vision_data.shoot_speed = Referee.ShootData.bullet_speed;
    vision_data.roll = (fp32) INS_angle[1] * MOTOR_RAD_TO_ANGLE;
    for (int i = 0; i < 4; ++i) {
        vision_data.quaternion[i] = INS_quat[i];
    }
    rm_queue_data(VISION_ID, &vision_data, sizeof(vision_t));
}


/**
  * @brief          校准 pitch 绝对角
  * @param[in]      none
  * 通过这个函数，云台检测到pit绝对角为接近0时，获取当前pit_offset。在调试模式获取了offset后，在初始化函数中更改。
  * TODO:校准时采用，校准完毕一定注释掉
  * @retval         返回空
  */
static void pit_offset_get(){
    if(gimbal.pitch.absolute_angle_get>=-0.002 && gimbal.pitch.absolute_angle_get<=0.002){
        //在这一行打断点调试，触发时成功获取0度时offset
        gimbal.pitch.motor_measure->offset_ecd=motor_pitch_measure.ecd;
    }
}


/**
  * @brief          设置底盘模式
  * @param[in]      none
  * 左下 右下底盘失能CHASSIS_RELAX   左不下 右下底盘独立CHASSIS_ONLY
  * 右中底盘跟随云台CHASSIS_FOLLOW_GIMBAL 右上 E小陀螺CHASSIS_SPIN
  * 按下 Q 键，底盘刹车   UI更新底盘模式
  * @retval         返回空
  */
static void chassis_mode_set(){
    //根据遥控器设置底盘模式
        //左下 右下： 底盘失能 CHASSIS_RELAX
    if(switch_is_down(rc_ctrl.rc.s[RC_s_L])&&switch_is_down(rc_ctrl.rc.s[RC_s_R])) {
        chassis.last_mode=chassis.mode;
        chassis.mode=CHASSIS_RELAX;
    }
//    else if(rc_ctrl.rc.ch[AUTO_CHANNEL]> 50) {
//        chassis.last_mode=chassis.mode;
//        chassis.mode=CHASSIS_SPIN_1;
//    }
        //左不下 右下： 底盘独立CHASSIS_ONLY
    else if(!switch_is_down(rc_ctrl.rc.s[RC_s_L])&&switch_is_down(rc_ctrl.rc.s[RC_s_R])) {
        chassis.last_mode=chassis.mode;
        chassis.mode=CHASSIS_ONLY;
    }
        //右上：小陀螺CHASSIS_SPIN
    else if(switch_is_up(rc_ctrl.rc.s[RC_s_R])) {
        chassis.last_mode=chassis.mode;
        chassis.mode=CHASSIS_SPIN;
    }
        //右中：底盘跟随云台CHASSIS_FOLLOW_GIMBAL
    else if(switch_is_mid(rc_ctrl.rc.s[RC_s_R])) {
        chassis.last_mode=chassis.mode;
        chassis.mode=CHASSIS_FOLLOW_GIMBAL;
        //小陀螺
        if(KeyBoard.E.click_flag==KEY_DOWN) {
            chassis.last_mode=chassis.mode;
            chassis.mode=CHASSIS_SPIN;
        }
        //独立
//        else if(KeyBoard.G.click_flag==KEY_DOWN) {
//            chassis.last_mode=chassis.mode;
//            chassis.mode=CHASSIS_ONLY;
//        }
        ///底盘刹车，英雄在打基地时实现，使用键盘实现，目前未实现
        else if(KeyBoard.C.click_flag==KEY_DOWN) {
            chassis.last_mode=chassis.mode;
            chassis.mode=CHASSIS_BLOCK;
        }
        //反着站打
        if(KeyBoard.F.click_flag==KEY_DOWN) {
            chassis.last_mode=chassis.mode;
            chassis.mode=CHASSIS_INDEPENDENT_CONTROL;
        }
    }
    //UI更新---底盘模式
    ui_robot_status.chassis_mode=chassis.mode;
}


/**
  * @brief          云台模式设置（获取遥控器信息，判断模式）
  * @param[in]      none
  * 右边拨杆向下云台失能
  * 右边拨杆向中和向上得到遥控器对云台电机的控制 —— 云台回中，自瞄判定
  * gimbal_mode_change() —— 进行是否自瞄的判定
  * @retval         返回空
  */
static void gimbal_mode_set(){
    //根据遥控器设置云台模式（只使用到右边拨杆）
    switch (rc_ctrl.rc.s[RC_s_R]) {
        case RC_SW_DOWN: {
            gimbal.mode=GIMBAL_RELAX; //失能发射机构，pitch，yaw(传输)
            gimbal.last_mode=gimbal.mode;
            break;
        }
        case RC_SW_MID:
        case RC_SW_UP:{
            gimbal.last_mode=gimbal.mode;
            gimbal.mode=GIMBAL_ACTIVE;
            }break;
        default:
            break;
    }
    gimbal_mode_change();
}


/**
  * @brief          自瞄判定
  * @param[in]      none
  * 四通道向下拨，向视觉发送自瞄信息 —— 0x21
  * 如果四通道开启且视觉返回 0x31 —— 进入自瞄模式，0x32 —— 本次自瞄不进行
  * @retval         返回空
  */
static void gimbal_mode_change() {
    if (gimbal.mode == GIMBAL_ACTIVE) {   //自瞄判定
        if (rc_ctrl.rc.ch[AUTO_CHANNEL] > 50 || KeyBoard.Mouse_r.status == KEY_PRESS) {
            vision_data.mode = 0x21;
        }
        else {
            vision_data.mode = 0;
        }

        if ((KeyBoard.Mouse_r.status == KEY_PRESS && robot_ctrl.target_lock == 0x31 && (detect_list[DETECT_AUTO_AIM].status == ONLINE))
            ||(rc_ctrl.rc.ch[AUTO_CHANNEL]> 50    && robot_ctrl.target_lock == 0x31 && (detect_list[DETECT_AUTO_AIM].status == ONLINE))) {
            gimbal.last_mode = GIMBAL_ACTIVE;
            gimbal.mode = GIMBAL_AUTO;
        }
    }
    else if (gimbal.mode == GIMBAL_AUTO) {   //0x32表示自瞄数据无效
        if ((detect_list[DETECT_AUTO_AIM].status == OFFLINE) || robot_ctrl.target_lock == 0x32) {
            gimbal.last_mode = GIMBAL_AUTO;
            gimbal.mode = GIMBAL_ACTIVE;//默认回到一般模式
            vision_data.mode = 0;
        }
    }
    // TODO: 离线检测问题 记得开关
//    if(detect_list[DETECT_LAUNCHER_2006_SINGLE_SHOT].status==OFFLINE)
//    {
//        gimbal.mode=GIMBAL_RELAX;
//    }
}


/******************/
/**
  * @brief          云台模式设置实现(四种模式)
  * @param[in]      none
  * 云台失能，云台回中，云台控制，云台自瞄
  * @retval         返回空
  */
static void gimbal_control(){
    switch (gimbal.mode) {
        case GIMBAL_RELAX://云台失能（fire, pitch, single_shoot）
            gimbal_relax_handle();
            break;

        case GIMBAL_ACTIVE://云台控制
            gimbal_active_handle();  //得到遥控器对云台电机的控制
            gimbal_ctrl_loop_cal();  //云台电机闭环控制函数
            break;

        case GIMBAL_AUTO://云台自瞄模式
            gimbal_auto_handle();
            gimbal_auto_ctrl_loop_cal();  //云台电机闭环控制函数
            break;
        default:
            break;
    }
}