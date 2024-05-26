#include "bsp_servo_pwm.h"
#include "main.h"

extern TIM_HandleTypeDef htim1;
// 舵机调控
void servo_pwm_set(uint16_t pwm, uint8_t i)
{
    switch(i)
    {
        case 0:
        {
                    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, pwm);
        }break;
        case 1:
        {
                    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, pwm);
        }break;
        case 2:
        {
                    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, pwm);
        }break;
        case 3:
        {
                    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, pwm);
        }break;
    }
}
