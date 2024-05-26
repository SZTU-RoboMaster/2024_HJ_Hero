//
// Created by xhuanc on 2022/5/22.
//

#ifndef DEMO1_CAP_H
#define DEMO1_CAP_H
#include "Hero.h"
#include "struct_typedef.h"
#include "can_receive.h"
#include "Referee.h"
#include "key_board.h"
#include "bsp_led.h"
#include "bsp_xidi_cap.h"
#define CAP_TASK_INIT_TIME 357
extern void cap_task(void const *pvParameters);




extern void cap_info_update();

#endif //DEMO1_CAP_H