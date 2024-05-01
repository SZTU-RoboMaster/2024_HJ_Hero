# 6020yaw轴版本

# 新增USB虚拟串口功能

## 发送上位机信息

在protocol_hero.h中添加自己的消息结构体,在data_cmd_id中按顺序加入消息ID
在task中调用 rm_queue_data()函数发送消息结构体

## 接收上位机信息

在decode.c中的函数decode_data_solve()中的case状态机按照消息ID拷贝内存

# 快捷键

ctrl+1：底盘cmd
ctrl+2：云台cmd
ctrl+3：底盘pid
ctrl+4：云台pid
ctrl+5：发射机构pid
ctrl+6：摩擦轮转速
ctrl+7：freertos临时文件

# TIM
TIM1: 
TIM3: 
TIM4: 蜂鸣器 PWM3通道
TIM5: 控制C板的LED灯
TIM8: pwm控制摩擦轮 通道1 2
TIM10: pwm通道1 imu

# key
WASD: 移动
Q: 开摩擦轮
E: 小陀螺
R: 一键掉头
F: 底盘不跟着云台动  侧着打  在坡上打前哨战用
C: 底盘刹车
G: 底盘独立

# USART
串口6: 裁判系统
串口3: 图传