//
// Created by xhuanc on 2022/5/22.
//

#include "Cap.h"
#include "cmsis_os.h"
#include "Referee.h"
#include "key_board.h"
#include "bsp_led.h"

extern cap_info_t cap_info;
extern UART_HandleTypeDef huart1;
cap2_info_t cap2;
extern key_board_t KeyBoard;
extern RC_ctrl_t rc_ctrl;

extern uint8_t usart1_buf[REFEREE_BUFFER_SIZE];

void cap_info_update(){
    if(KeyBoard.SHIFT.status == KEY_PRESS)//V按键状态为1 为开启电容
    {
        cap2.send_data[0]=(uint8_t)0xFF;//电容开启
        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,GPIO_PIN_RESET); // 4月12日改电容LED
    }
    else{
        cap2.send_data[0]=(uint8_t)0x00;//电容关闭
        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,GPIO_PIN_SET); // 4月12日改电容LED
    }

    if(cap2.mode==0) {
        cap2.send_data[1] = (uint8_t) Referee.GameRobotStat.chassis_power_limit - 15;//需要给电容一些余量
    }
    else{
        cap2.send_data[1] = (uint8_t) Referee.GameRobotStat.chassis_power_limit - 5;//需要给电容一些余量
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

#ifdef GIMBAL
#else //CHASSIS

void cap_task(void const *pvParameters)
{
    osDelay(CAP_TASK_INIT_TIME);
    while (1)
    {
        send_to_cap_xidi(Referee.GameRobotStat.chassis_power_limit-3,&huart1);
        osDelay(120);
    }
}
#endif //GIMBAL
