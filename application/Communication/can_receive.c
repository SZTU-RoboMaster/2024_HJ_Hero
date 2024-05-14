//
// Created by xhuanc on 2021/9/27.
//

#include "can_receive.h"
#include "remote.h"
#include "Referee.h"
#include "key_board.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
chassis_t chassis;       //底盘
gimbal_t gimbal;         //云台
launcher_t launcher;     //发射机构
trigger_t trigger;
extern key_board_t KeyBoard;    //键盘
extern robot_ctrl_info_t robot_ctrl;
extern uint8_t fire_lock;
RC_ctrl_t rc_ctrl;       //遥控器
cap_info_t cap_info;     //电容

void cap_info_decode(cap_info_t *cap, const uint16_t *rx_data){
    cap->  input_value=(float)rx_data[0]/100;
    cap->    cap_value=(float)rx_data[1]/100;
    cap->input_current=(float)rx_data[2]/100;
    cap-> target_power=(float)rx_data[3]/100;
}
//将接收到的数据解码并存储到 `cap_info_t` 结构体中，具体的解码方式如下：
//将 `rx_data` 数组中的第一个元素除以 100，并将结果转换为浮点数，然后将其赋值给 `cap` 结构体的 ` input_value ` 成员。
//将 `rx_data` 数组中的第二个元素除以 100，并将结果转换为浮点数，然后将其赋值给 `cap` 结构体的 `  cap_value  ` 成员。
//将 `rx_data` 数组中的第三个元素除以 100，并将结果转换为浮点数，然后将其赋值给 `cap` 结构体的 `input_current` 成员。
//将 `rx_data` 数组中的第四个元素除以 100，并将结果转换为浮点数，然后将其赋值给 `cap` 结构体的 ` target_power` 成员。
//目的是将接收到的数据解析为浮点数，并存储到 `cap_info_t` 结构体的相应成员中，以便后续在程序中使用这些数据进行计算或控制。


/******************** define *******************/

//电子数据解算
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }

//电机总编码值的计算
#define get_motor_round_cnt(ptr)                                        \
    {                                                                   \
             if(ptr.ecd-ptr.last_ecd> 4192){                            \
                ptr.round_cnt--;                                        \
             }                                                          \
             else if(ptr.ecd-ptr.last_ecd<-4192)                        \
             {                                                          \
                ptr.round_cnt++;                                        \
             }                                                          \
             ptr.total_ecd= ptr.round_cnt*8192+ptr.ecd-ptr.offset_ecd;  \
    }


/******************** variable *******************/

motor_measure_t motor_3508_measure[8];//0-3 分别对应  RF,LF,LB,RB 4,5分别对应FIRE_L,FIRE_R 6对应拨盘电机  7对应上摩擦轮
motor_measure_t motor_yaw_measure;    //yaw轴
motor_measure_t motor_pitch_measure;  //云台pitch轴

static CAN_TxHeaderTypeDef  tx_message;
static uint8_t              can_send_data[8];

//车轮电机的发送函数
void CAN_cmd_motor(CAN_TYPE can_type,can_msg_id_e CMD_ID,int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    tx_message.StdId = CMD_ID;
    tx_message.IDE = CAN_ID_STD;
    tx_message.RTR = CAN_RTR_DATA;
    tx_message.DLC = 0x08;
    can_send_data[0] = motor1 >> 8;
    can_send_data[1] = motor1;
    can_send_data[2] = motor2 >> 8;
    can_send_data[3] = motor2;
    can_send_data[4] = motor3 >> 8;
    can_send_data[5] = motor3;
    can_send_data[6] = motor4 >> 8;
    can_send_data[7] = motor4;

    if(can_type==CAN_1) {
        HAL_CAN_AddTxMessage(&hcan1, &tx_message, can_send_data, &send_mail_box);
    }
    else if(can_type==CAN_2) {
        HAL_CAN_AddTxMessage(&hcan2, &tx_message, can_send_data, &send_mail_box);
    }
}

fp32 motor_ecd_to_rad_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > HALF_ECD_RANGE) {
        relative_ecd -= ECD_RANGE;
    }
    else if (relative_ecd < -HALF_ECD_RANGE) {
        relative_ecd += ECD_RANGE;
    }
    return relative_ecd * MOTOR_ECD_TO_RAD;
}

//计算距离零点的度数  -180-180
//根据电机编码器数据和偏移量计算相对角度变化的函数
fp32 motor_ecd_to_angle_change(uint16_t ecd,uint16_t offset_ecd) {
    int16_t tmp=0;
    if(offset_ecd>=4096) {
        if(ecd>offset_ecd-4096) {
            tmp=ecd-offset_ecd;
        }
        else {
            tmp=ecd+8192-offset_ecd;
        }
    }
    else {
        if(ecd>offset_ecd+4096) {
            tmp=ecd-8192-offset_ecd;
        }
        else {
            tmp=ecd-offset_ecd;
        }
    }
    return (fp32)tmp/8192.f*360;
}

void CAN_cap(CAN_TYPE can_type,can_msg_id_e CMD_ID,uint16_t power_limit) {
    uint32_t send_mail_box;
    tx_message.StdId = CMD_ID;
    tx_message.IDE = CAN_ID_STD;
    tx_message.RTR = CAN_RTR_DATA;
    tx_message.DLC = 0x08;
    can_send_data[0] = power_limit >> 8;
    can_send_data[1] = power_limit;
    can_send_data[2] = 0;
    can_send_data[3] = 0;
    can_send_data[4] = 0;
    can_send_data[5] = 0;
    can_send_data[6] = 0;
    can_send_data[7] = 0;

    if(can_type==CAN_1) {
        HAL_CAN_AddTxMessage(&hcan1, &tx_message, can_send_data, &send_mail_box);
    }
    else if(can_type==CAN_2) {
        HAL_CAN_AddTxMessage(&hcan2, &tx_message, can_send_data, &send_mail_box);
    }
}

void CAN_cmd_cap2(cap2_info_t*cap) {
    uint32_t send_mail_box;
    tx_message.StdId = 0x002;
    tx_message.IDE = CAN_ID_STD;
    tx_message.RTR = CAN_RTR_DATA;
    tx_message.DLC = 0x06;
    for (uint8_t i = 0; i <=4 ; ++i) {
        can_send_data[i]=cap->send_data[i];
    }
    HAL_CAN_AddTxMessage(&hcan1, &tx_message, cap->send_data, &send_mail_box);
}

void CAN_Gimbal_init(){
    // 云台的CAN初始化
    // 使用过滤器 CAN1: [0~6] CAN2: [14~20]

    // 基本部分
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0xFFFF;
    can_filter_st.FilterMaskIdLow = 0x0000;

    /****************** CAN1 过滤器 ***********************/
    can_filter_st.SlaveStartFilterBank = 0;
    // 过滤器0
//    can_filter_st.FilterBank = 0;
//    can_filter_st.FilterIdHigh = 0x0000;
//    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
//
//    // 开启CAN1过滤器
//    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
//    HAL_CAN_Start(&hcan1);
//    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
//    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);
    /****************** CAN2 过滤器 ***********************/
    can_filter_st.SlaveStartFilterBank = 14;
    // single_shoot-0x204 单发限位电机
    can_filter_st.FilterIdHigh = (0x204 <<5);               // 接受ID
    can_filter_st.FilterBank = 14;                          // 过滤器编号
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;      // 将接受的的报文放入FIFO0
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);

    // Yaw-0x205 Yaw电机
    can_filter_st.FilterBank = 18;
    can_filter_st.FilterIdHigh = (0x205 << 5);
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);

    // pitch-0x206 Pitch电机
    can_filter_st.FilterIdHigh = (0x206 <<5);               // 接受ID
    can_filter_st.FilterBank = 15;                          // 过滤器编号
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;      // 将接受的的报文放入FIFO0
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);

    // fire_r-0x207 右摩擦轮电机
    can_filter_st.FilterIdHigh = (0x207 <<5);               // 接受ID
    can_filter_st.FilterBank = 16;                          // 过滤器编号
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;      // 将接受的的报文放入FIFO0
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);

    // fire_l-0x208 左摩擦轮电机
    can_filter_st.FilterIdHigh = (0x208 <<5);               // 接受ID
    can_filter_st.FilterBank = 17;                          // 过滤器编号
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;      // 将接受的的报文放入FIFO0
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);

    // 开启CAN2过滤器
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
}

void CAN_Chass_init(){
    // 底盘的CAN初始化
    // 使用过滤器 CAN1: [7~13] CAN2: [21~27]

    // 基本部分
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0xFFFF;
    can_filter_st.FilterMaskIdLow = 0x0000;

    /****************** CAN1 过滤器 ***********************/
    can_filter_st.SlaveStartFilterBank = 0;

    // motor_chassis[RF]-0x201 右上底盘电机
    can_filter_st.FilterBank = 7;
    can_filter_st.FilterIdHigh = (0x201 << 5);
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);

    // motor_chassis[LF]-0x202 左上底盘电机
    can_filter_st.FilterBank = 8;
    can_filter_st.FilterIdHigh = (0x202 << 5);
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);

    // motor_chassis[LB]-0x203 左下底盘电机
    can_filter_st.FilterBank = 9;
    can_filter_st.FilterIdHigh = (0x203 << 5);
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);

    // motor_chassis[RB]-0x204 右下底盘电机
    can_filter_st.FilterBank = 10;
    can_filter_st.FilterIdHigh = (0x204 << 5);
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);

    // TRIGGER-0x207 拨盘电机
    can_filter_st.FilterBank = 1;
    can_filter_st.FilterIdHigh = (0x207 << 5);
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);

    // 自定义数据集-[0x110~0x117]
    can_filter_st.FilterBank = 13;
    can_filter_st.FilterIdHigh = (0x110 << 5);
    can_filter_st.FilterMaskIdHigh = 0xFF00;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO1;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);

    // 开启CAN1过滤器
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);
    /****************** CAN2 过滤器 ***********************/
/*    can_filter_st.SlaveStartFilterBank = 14;
    // 过滤器0
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);

    // 开启CAN2过滤器
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);*/

}

void CAN1_RX0_IRQHandler(){
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_data);
    // 底盘电机数据
    switch (rx_header.StdId) {
        case CAN_CHASSIS_3508_MOTOR_1:  //201
            get_motor_measure(&motor_3508_measure[0], rx_data);
            detect_handle(DETECT_CHASSIS_3508_RF);
            break;
        case CAN_CHASSIS_3508_MOTOR_2:  //202
            get_motor_measure(&motor_3508_measure[1], rx_data);
            detect_handle(DETECT_CHASSIS_3508_LF);
            break;
        case CAN_CHASSIS_3508_MOTOR_3:  //203
            get_motor_measure(&motor_3508_measure[2], rx_data);
            detect_handle(DETECT_CHASSIS_3508_LB);
            break;
        case CAN_CHASSIS_3508_MOTOR_4:  //204
            get_motor_measure(&motor_3508_measure[3], rx_data);
            detect_handle(DETECT_CHASSIS_3508_RB);
            break;
        case CAN_LAUNCHER_3508_TRIGGER: //207
            get_motor_measure(&motor_3508_measure[6], rx_data);
            get_motor_round_cnt(motor_3508_measure[6]);//获取转动拨轮电机转动圈数和总编码值
            detect_handle(DETECT_LAUNCHER_3508_TRIGGER);
            break;
        case 0x211:
            cap_info_decode(&cap_info,(uint16_t *)rx_data);
            break;
        default:
            break;
    }
}

void CAN1_RX1_IRQHandler(){
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO1, &rx_header, rx_data);

    // 上下板通信数据
    switch (rx_header.StdId) {
        case 0x110:{        //上板发送遥控器数据
            union ctrl_ch ch1;
            union ctrl_ch ch0;
            union ctrl_ch ch2;
            {
                ch1.data[0] = rx_data[0];
                ch1.data[1] = rx_data[1];
                ch0.data[0] = rx_data[2];
                ch0.data[1] = rx_data[3];
                ch2.data[0] = rx_data[4];
                ch2.data[1] = rx_data[5];
            }
            rc_ctrl.rc.ch[1] = ch1.value;
            rc_ctrl.rc.ch[0] = ch0.value;
            rc_ctrl.rc.ch[2] = ch2.value;
            rc_ctrl.rc.s[1] = rx_data[6];
            KeyBoard.Mouse_l.status = rx_data[7];
        } break;

        case 0x111:{    //TODO: 没用完
            union angle angle1;
            {
                angle1.data[0] = rx_data[0];
                angle1.data[1] = rx_data[1];
                angle1.data[2] = rx_data[2];
                angle1.data[3] = rx_data[3];
                fire_lock = rx_data[4];
                KeyBoard.V.click_flag = rx_data[5];
            }
            gimbal.yaw.relative_angle_get = angle1.value;
        } break;

        case 0x112:{
            union mode mode1;
            union mode mode2;
            union mode mode3;
            union mode mode4;
            {
                mode1.data[0] = rx_data[0];
                mode1.data[1] = rx_data[1];
                mode2.data[0] = rx_data[2];
                mode2.data[1] = rx_data[3];
                mode3.data[0] = rx_data[4];
                mode3.data[1] = rx_data[5];
                mode4.data[0] = rx_data[6];
                mode4.data[1] = rx_data[7];
            }
            chassis.mode = mode1.mode;
            gimbal.mode = mode2.mode;
            launcher.fire_mode = mode3.mode;
            launcher.single_shoot_cmd = mode4.mode;
        } break;

        case 0x113:{    //TODO:没用完
            union mode trigger_motion;
            {
                KeyBoard.W.status = rx_data[0];
                KeyBoard.A.status = rx_data[1];
                KeyBoard.S.status = rx_data[2];
                KeyBoard.D.status = rx_data[3];
                trigger_motion.data[0] = rx_data[4];
                trigger_motion.data[1] = rx_data[5];
//                command.data[0] = rx_data[6];
//                command.data[1] = rx_data[7];
            }
            trigger.state = trigger_motion.mode;
//            robot_ctrl.fire_command = command.value;
        } break;

        case 0x114:{    //TODO: 没用完
            union referee power;
            {
                power.data[0] = rx_data[0];
                power.data[1] = rx_data[1];
                power.data[2] = rx_data[2];
                power.data[3] = rx_data[3];
                Referee.GameRobotStat.robot_id = rx_data[4];
            }
            Referee.PowerHeatData.chassis_power = power.referee;
        } break;

        case 0x115:{

        } break;

        case 0x116:{

        } break;

        case 0x117:{

        } break;
    }
}

void CAN2_RX0_IRQHandler(){
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &rx_header, rx_data);

    // 云台电机数据
    switch (rx_header.StdId) {
        case CAN_LAUNCHER_3508_FIRE_ON:         //204
            get_motor_measure(&motor_3508_measure[7], rx_data);
//            get_motor_round_cnt(motor_3508_measure[7]);//获取转动拨轮电机转动圈数和总编码值
            detect_handle(DETECT_LAUNCHER_3508_FIRE_ON);
            break;
        case CAN_GIMBAL_6020_YAW:                   //205
            get_motor_measure(&motor_yaw_measure, rx_data);
            detect_handle(DETECT_GIMBAL_6020_YAW);
            break;
        case CAN_GIMBAL_6020_PITCH:                 //206
            get_motor_measure(&motor_pitch_measure, rx_data);
//            get_motor_round_cnt(motor_pitch_measure);//获取转动拨轮电机转动圈数和总编码值
            detect_handle(DETECT_GIMBAL_6020_PITCH);
            break;
        case CAN_LAUNCHER_3508_FIRE_R:              //207
            get_motor_measure(&motor_3508_measure[5], rx_data);
            detect_handle(DETECT_LAUNCHER_3508_FIRE_R);
            break;
        case CAN_LAUNCHER_3508_FIRE_L:              //208
            get_motor_measure(&motor_3508_measure[4],rx_data);
            detect_handle(DETECT_LAUNCHER_3508_FIRE_L);
            break;
        default:
            break;
    }
}