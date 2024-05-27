
/*ң������λ��

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

/******************����******************/
extern chassis_t chassis;       // ����
extern gimbal_t gimbal;         //��̨
extern launcher_t launcher;     //�������
extern trigger_t trigger;       //����ģʽ

extern RC_ctrl_t rc_ctrl;       //ң����
extern key_board_t KeyBoard;    //����
extern TIM_HandleTypeDef htim1;

extern fp32 INS_angle[3];
extern fp32 INS_gyro[3];
extern fp32 INS_quat[4];

vision_t vision_data;   //���ͻ�����id
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
/******************����ʵ��******************/

/**
  * @brief          ��̨����
  * @param[in]      pvParameters
  * ��ʼ��������ģʽ��ʵ�ֹ��ܣ����� CAN �źţ���̨�ľ���ʵ���߼�
  * @retval         ����ָ��
  */
void Gimbal_task(void const*pvParameters) {
    vTaskDelay(GIMBAL_TASK_INIT_TIME);  //�����ʼ��ʱ��
    gimbal_init();  //��̨��ʼ��
    launcher_init();//���������ʼ��

    while(1) {
        //����PC�Ŀ�����Ϣ
        update_pc_info();
        Send_Keyboard(KeyBoard.W.status, KeyBoard.A.status, KeyBoard.S.status, KeyBoard.D.status, trigger.state);

        if(robot_ctrl.fire_command == 1) {
            fire_lock = 1;
        }
        if(fire_lock > 0){
            // ���뿪��ģʽ,����10��can�ź�
            fire_lock ++;
            if(fire_lock > 5) fire_lock = 0;
        }
        Send_Chassis_Speed(rc_ctrl.rc.ch[1], rc_ctrl.rc.ch[0], rc_ctrl.rc.ch[2], rc_ctrl.rc.s[1], KeyBoard.Mouse_l.status);
        vTaskDelay(1);

        gimbal_angle_update();  //���¾��ԡ���ԽǶȽ���ֵ
        if(detect_list[DETECT_GIMBAL_6020_YAW].status == OFFLINE) {
            gimbal.yaw.relative_angle_get = 0;
            Send_Yaw_Angle(gimbal.yaw.relative_angle_get, fire_lock, KeyBoard.V.click_flag);
        } else {
            Send_Yaw_Angle(gimbal.yaw.relative_angle_get, fire_lock, KeyBoard.V.click_flag);
        }

        ////ע�⣺У׼ʱ���ã�У׼���һ��ע�͵�
        //pit_offset_get();

        chassis_mode_set(); //���õ���ģʽ
        gimbal_mode_set();  //������̨ģʽ
        images_mode_set();  //����ͼ��ģʽ
        launcher_mode_set();//���÷���ģʽ

        gimbal_control();  //��̨ģʽ����ʵ��
        launcher_control();//�����������

        Send_Mode(chassis.mode, gimbal.mode, launcher.fire_mode, launcher.single_shoot_cmd);
//        Send_command(robot_ctrl.fire_command);

        //���������Դ�Ƿ����  TODO:ȡ��ע�ͻ�ʧ�ܣ�Ҫװ����ϵͳ
//        gimbal_device_offline_handle();//TODO:�������,װ�����ϵͳ����Դ�

        gimbal_can_send_back_mapping(); // ����can�źţ�����UI
        vTaskDelay(1);

        //����CAN�ź�
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

        gimbal_uiInfo_packet();     //����UI��̨״̬

        vTaskDelay(2);
    }
}


/******************/
/**
  * @brief          ��̨��ʼ��
  * @param[in]      none
  * ��ʼ��Ӣ�� mode �� last_mode ��ģʽΪʧ�ܣ���ȡ pitch �������
  * ��ʼ�� pitch ��� yaw �����ĽǶȻ����ٶȻ� PID (��������)
  * �� pitch ��� yaw �����ĵ�������Ϊ 0����ʧ��
  * ��ʼ��ʱ����̨��Ϊδ����״̬��У׼ pitch ��� yaw �����ı���ֵ
  * @retval         ���ؿ�
  */
static void gimbal_init(){
    //��ʼ��Ĭ��״̬Ϊʧ��
    gimbal.mode=gimbal.last_mode=GIMBAL_RELAX;
    chassis.mode=chassis.last_mode=CHASSIS_RELAX;

    //��ȡ pitch �� yaw �������
    gimbal.yaw.motor_measure = &motor_yaw_measure;
    gimbal.pitch.motor_measure=&motor_pitch_measure;

    //pit���� �ǶȻ����ٶȻ�PID��ʼ��
    pid_init(&gimbal.pitch.angle_p, GIMBAL_PITCH_ANGLE_MAX_OUT,
             GIMBAL_PITCH_ANGLE_MAX_IOUT, GIMBAL_PITCH_ANGLE_PID_KP,
             GIMBAL_PITCH_ANGLE_PID_KI, GIMBAL_PITCH_ANGLE_PID_KD);

    pid_init(&gimbal.pitch.speed_p, GIMBAL_PITCH_SPEED_MAX_OUT,
             GIMBAL_PITCH_SPEED_MAX_IOUT, GIMBAL_PITCH_SPEED_PID_KP,
             GIMBAL_PITCH_SPEED_PID_KI, GIMBAL_PITCH_SPEED_PID_KD);

    //pit��ǶȻ����ٶȻ�PID��ʼ�� ����ʱ
    pid_init(&gimbal.pitch.auto_angle_p, GIMBAL_PITCH_AUTO_ANGLE_MAX_OUT,
             GIMBAL_PITCH_AUTO_ANGLE_MAX_IOUT, GIMBAL_PITCH_AUTO_ANGLE_PID_KP,
             GIMBAL_PITCH_AUTO_ANGLE_PID_KI, GIMBAL_PITCH_AUTO_ANGLE_PID_KD);

    pid_init(&gimbal.pitch.auto_speed_p, GIMBAL_PITCH_AUTO_SPEED_MAX_OUT,
             GIMBAL_PITCH_AUTO_SPEED_MAX_IOUT, GIMBAL_PITCH_AUTO_SPEED_PID_KP,
             GIMBAL_PITCH_AUTO_SPEED_PID_KI, GIMBAL_PITCH_AUTO_SPEED_PID_KD);

    //yaw �����ǶȻ����ٶȻ�PID��ʼ��
    pid_init(&gimbal.yaw.angle_p,
             GIMBAL_YAW_ANGLE_MAX_OUT, GIMBAL_YAW_ANGLE_MAX_IOUT,
             GIMBAL_YAW_ANGLE_PID_KP, GIMBAL_YAW_ANGLE_PID_KI, GIMBAL_YAW_ANGLE_PID_KD);

    pid_init(&gimbal.yaw.speed_p,
             GIMBAL_YAW_SPEED_MAX_OUT, GIMBAL_YAW_SPEED_MAX_IOUT,
             GIMBAL_YAW_SPEED_PID_KP, GIMBAL_YAW_SPEED_PID_KI, GIMBAL_YAW_SPEED_PID_KD);

    //yaw �����ǶȻ����ٶȻ�PID��ʼ�� ����ʱ
    pid_init(&gimbal.yaw.auto_angle_p,
             GIMBAL_YAW_AUTO_ANGLE_MAX_OUT, GIMBAL_YAW_AUTO_ANGLE_MAX_IOUT,
             GIMBAL_YAW_AUTO_ANGLE_PID_KP, GIMBAL_YAW_AUTO_ANGLE_PID_KI, GIMBAL_YAW_AUTO_ANGLE_PID_KD);

    pid_init(&gimbal.yaw.auto_speed_p,
             GIMBAL_YAW_AUTO_SPEED_MAX_OUT, GIMBAL_YAW_AUTO_SPEED_MAX_IOUT,
             GIMBAL_YAW_AUTO_SPEED_PID_KP, GIMBAL_YAW_AUTO_SPEED_PID_KI, GIMBAL_YAW_AUTO_SPEED_PID_KD);

    //��ʼʱ��̨ģʽΪʧ�ܣ���������Ϊ0
    gimbal.yaw.give_current = 0;
    gimbal.pitch.give_current=0;

    //pitch���yaw������У׼����ֵ
    gimbal.pitch.motor_measure->offset_ecd=3220;//2370;
    gimbal.yaw.motor_measure->offset_ecd = 6470;//6470;//7400

    //��ͨ�˲���ʼ��
    first_order_filter_init(&pitch_first_order_set, 0.f, 500);
    first_order_filter_init(&pitch_current_first_order_set, 5, 30);
    first_order_filter_init(&filter_yaw_gyro_in, 5, 30);

    first_order_filter_init(&mouse_in_x, 1, 40);
    first_order_filter_init(&mouse_in_y, 1, 10);

    first_order_filter_init(&filter_pitch_gyro_in, 1, 20);
    first_order_filter_init(&auto_pitch, 1, 15);
    first_order_filter_init(&auto_yaw[0], 1, 15);
    first_order_filter_init(&auto_yaw[1], 1, 15);

    first_Kalman_Create(&filter_autoYaw,1,20);  //�������˲�
}


/**
  * @brief          ��̨�Ƕȸ��£����Ӿ�ͨѶ
  * @param[in]      none
  * ���ݹ��Ե���ϵͳ�����ݸ�����̨�ĽǶ���Ϣ��������Ӧ����Ϣ���͵��Ӿ�ϵͳ��
  * ���У��Ƕ���Ϣͨ�������ǲ�������������������Լ����Ե���ϵͳ�ĽǶ���Ϣ�Ȼ�á�
  * @retval         ���ؿ�
  */
static void gimbal_angle_update(){
    // ������̨�����Ƕ���Ϣ
    gimbal.pitch.absolute_angle_get=INS_angle[2]*MOTOR_RAD_TO_ANGLE;
    gimbal.pitch.relative_angle_get= motor_ecd_to_angle_change(gimbal.pitch.motor_measure->ecd,
                                                               gimbal.pitch.motor_measure->offset_ecd);
    gimbal.yaw.absolute_angle_get=INS_angle[0]*MOTOR_RAD_TO_ANGLE;
    gimbal.yaw.relative_angle_get= motor_ecd_to_angle_change(gimbal.yaw.motor_measure->ecd,
                                                             gimbal.yaw.motor_measure->offset_ecd);

    // ������̨������Խ� | get
    gimbal.pitch.relative_up_down_get = gimbal.pitch.absolute_angle_get - gimbal.pitch.absolute_angle_get_down;

//     ������̨ƫ���Ƕ���Ϣ,ͨ�������ǲ�����ƫ���͸������ٶ�
    gyro_yaw=INS_gyro[2]*MOTOR_RAD_TO_ANGLE;
    gyro_pitch=-INS_gyro[0]*MOTOR_RAD_TO_ANGLE;

    gimbal.absolute_gyro_yaw = (fp32) INS_gyro[2];
    gimbal.absolute_gyro_pitch = (fp32) INS_gyro[0];

//    if(gimbal.yaw.relative_angle_get > -MAX_ECD_ERROR && gimbal.yaw.relative_angle_get < MAX_ECD_ERROR) {
//        gimbal.yaw.relative_angle_get=0;
//    }

    // 107:�� 7:��
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
  * @brief          У׼ pitch ���Խ�
  * @param[in]      none
  * ͨ�������������̨��⵽pit���Խ�Ϊ�ӽ�0ʱ����ȡ��ǰpit_offset���ڵ���ģʽ��ȡ��offset���ڳ�ʼ�������и��ġ�
  * TODO:У׼ʱ���ã�У׼���һ��ע�͵�
  * @retval         ���ؿ�
  */
static void pit_offset_get(){
    if(gimbal.pitch.absolute_angle_get>=-0.002 && gimbal.pitch.absolute_angle_get<=0.002){
        //����һ�д�ϵ���ԣ�����ʱ�ɹ���ȡ0��ʱoffset
        gimbal.pitch.motor_measure->offset_ecd=motor_pitch_measure.ecd;
    }
}


/**
  * @brief          ���õ���ģʽ
  * @param[in]      none
  * ���� ���µ���ʧ��CHASSIS_RELAX   ���� ���µ��̶���CHASSIS_ONLY
  * ���е��̸�����̨CHASSIS_FOLLOW_GIMBAL ���� EС����CHASSIS_SPIN
  * ���� Q ��������ɲ��   UI���µ���ģʽ
  * @retval         ���ؿ�
  */
static void chassis_mode_set(){
    //����ң�������õ���ģʽ
        //���� ���£� ����ʧ�� CHASSIS_RELAX
    if(switch_is_down(rc_ctrl.rc.s[RC_s_L])&&switch_is_down(rc_ctrl.rc.s[RC_s_R])) {
        chassis.last_mode=chassis.mode;
        chassis.mode=CHASSIS_RELAX;
    }
//    else if(rc_ctrl.rc.ch[AUTO_CHANNEL]> 50) {
//        chassis.last_mode=chassis.mode;
//        chassis.mode=CHASSIS_SPIN_1;
//    }
        //���� ���£� ���̶���CHASSIS_ONLY
    else if(!switch_is_down(rc_ctrl.rc.s[RC_s_L])&&switch_is_down(rc_ctrl.rc.s[RC_s_R])) {
        chassis.last_mode=chassis.mode;
        chassis.mode=CHASSIS_ONLY;
    }
        //���ϣ�С����CHASSIS_SPIN
    else if(switch_is_up(rc_ctrl.rc.s[RC_s_R])) {
        chassis.last_mode=chassis.mode;
        chassis.mode=CHASSIS_SPIN;
    }
        //���У����̸�����̨CHASSIS_FOLLOW_GIMBAL
    else if(switch_is_mid(rc_ctrl.rc.s[RC_s_R])) {
        chassis.last_mode=chassis.mode;
        chassis.mode=CHASSIS_FOLLOW_GIMBAL;
        //С����
        if(KeyBoard.E.click_flag==KEY_DOWN) {
            chassis.last_mode=chassis.mode;
            chassis.mode=CHASSIS_SPIN;
        }
        //����
//        else if(KeyBoard.G.click_flag==KEY_DOWN) {
//            chassis.last_mode=chassis.mode;
//            chassis.mode=CHASSIS_ONLY;
//        }
        ///����ɲ����Ӣ���ڴ����ʱʵ�֣�ʹ�ü���ʵ�֣�Ŀǰδʵ��
        else if(KeyBoard.C.click_flag==KEY_DOWN) {
            chassis.last_mode=chassis.mode;
            chassis.mode=CHASSIS_BLOCK;
        }
        //����վ��
        if(KeyBoard.F.click_flag==KEY_DOWN) {
            chassis.last_mode=chassis.mode;
            chassis.mode=CHASSIS_INDEPENDENT_CONTROL;
        }
    }
    //UI����---����ģʽ
    ui_robot_status.chassis_mode=chassis.mode;
}


/**
  * @brief          ��̨ģʽ���ã���ȡң������Ϣ���ж�ģʽ��
  * @param[in]      none
  * �ұ߲���������̨ʧ��
  * �ұ߲������к����ϵõ�ң��������̨����Ŀ��� ���� ��̨���У������ж�
  * gimbal_mode_change() ���� �����Ƿ�������ж�
  * @retval         ���ؿ�
  */
static void gimbal_mode_set(){
    //����ң����������̨ģʽ��ֻʹ�õ��ұ߲��ˣ�
    switch (rc_ctrl.rc.s[RC_s_R]) {
        case RC_SW_DOWN: {
            gimbal.mode=GIMBAL_RELAX; //ʧ�ܷ��������pitch��yaw(����)
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
  * @brief          �����ж�
  * @param[in]      none
  * ��ͨ�����²������Ӿ�����������Ϣ ���� 0x21
  * �����ͨ���������Ӿ����� 0x31 ���� ��������ģʽ��0x32 ���� �������鲻����
  * @retval         ���ؿ�
  */
static void gimbal_mode_change() {
    if (gimbal.mode == GIMBAL_ACTIVE) {   //�����ж�
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
    else if (gimbal.mode == GIMBAL_AUTO) {   //0x32��ʾ����������Ч
        if ((detect_list[DETECT_AUTO_AIM].status == OFFLINE) || robot_ctrl.target_lock == 0x32) {
            gimbal.last_mode = GIMBAL_AUTO;
            gimbal.mode = GIMBAL_ACTIVE;//Ĭ�ϻص�һ��ģʽ
            vision_data.mode = 0;
        }
    }
    // TODO: ���߼������ �ǵÿ���
//    if(detect_list[DETECT_LAUNCHER_2006_SINGLE_SHOT].status==OFFLINE)
//    {
//        gimbal.mode=GIMBAL_RELAX;
//    }
}


/******************/
/**
  * @brief          ��̨ģʽ����ʵ��(����ģʽ)
  * @param[in]      none
  * ��̨ʧ�ܣ���̨���У���̨���ƣ���̨����
  * @retval         ���ؿ�
  */
static void gimbal_control(){
    switch (gimbal.mode) {
        case GIMBAL_RELAX://��̨ʧ�ܣ�fire, pitch, single_shoot��
            gimbal_relax_handle();
            break;

        case GIMBAL_ACTIVE://��̨����
            gimbal_active_handle();  //�õ�ң��������̨����Ŀ���
            gimbal_ctrl_loop_cal();  //��̨����ջ����ƺ���
            break;

        case GIMBAL_AUTO://��̨����ģʽ
            gimbal_auto_handle();
            gimbal_auto_ctrl_loop_cal();  //��̨����ջ����ƺ���
            break;
        default:
            break;
    }
}