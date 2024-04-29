//
// Created by xhuanc on 2022/5/22.
//

#include "Cap.h"
#include "cmsis_os.h"
#include "can_receive.h"
#include "Referee.h"
#include "key_board.h"
#include "bsp_led.h"

extern cap_info_t cap_info;
cap2_info_t cap2;
extern key_board_t KeyBoard;
extern RC_ctrl_t rc_ctrl;

void cap_info_update(){

    if(KeyBoard.SHIFT.status == KEY_PRESS||rc_ctrl.rc.s[1]==3)//V按键状态为1 为开启电容
    {
        cap2.send_data[0]=(uint8_t)0xFF;//电容开启
        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,GPIO_PIN_RESET); // 4月12日改电容LED
    }
    else{
        cap2.send_data[0]=(uint8_t)0x00;//电容关闭
        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,GPIO_PIN_SET); // 4月12日改电容LED
    }

    if(cap2.mode==0) {
//        cap2.send_data[1] = (uint8_t) Referee.GameRobotStat.max_chassis_power - 15;//需要给电容一些余量
        uint16_t ax2=65;
        cap2.send_data[1]=(uint8_t)ax2;
    }
    else{
        uint16_t ax=40;
//        cap2.send_data[1] = (uint8_t) Referee.GameRobotStat.max_chassis_power - 5;//需要给电容一些余量
        cap2.send_data[1]=(uint8_t)ax;
    }

    if(Referee.GameRobotStat.current_HP<=0){
        cap2.send_data[2]=ENABLE;
    }
    else{
        cap2.send_data[2]=DISABLE;
    }

    cap2.send_data[3]=(uint8_t)0x00;

    cap2.send_data[4]=Referee.GameState.stage_remain_time>>8;
    cap2.send_data[5]=Referee.GameState.stage_remain_time;

//    if(cap_info.ctrl_cmd==CAP_ON)//电容开亮灯
//        HAL_GPIO_WritePin(LED2_PORT,LED2_PIN,GPIO_PIN_RESET);
//    else
//        HAL_GPIO_WritePin( LED2_PORT,LED2_PIN,GPIO_PIN_SET);
//    if( cap2.send_data[0]==0xFF)//电容开亮灯
//        led.mode=CAP;
}


void cap_task(void const *pvParameters)
{
    osDelay(CAP_TASK_INIT_TIME);
    while (1)
    {
        cap_info_update();
        CAN_cmd_cap2(&cap2);
        osDelay(100);
    }
}