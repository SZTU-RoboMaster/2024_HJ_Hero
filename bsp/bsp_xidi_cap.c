//
// Created by suheng on 2024/5/12.
//
#include "bsp_xidi_cap.h"
#ifdef GIMBAL
#else //CHASSIS
extern cap_receive_data_t capReceiveData;  //接收溪地电容反馈数据
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
fp32 Float = 0.f;//解包时某一个数据的小数部分
uint8_t Integer = 0;//解包时某一个数据的整数部分
uint8_t iCap = 0;//遍历capData指针，用于解包时索引capData
uint8_t capList_index = 0;//遍历串口缓冲区指针，用于索引串口缓冲区，移至capData
uint8_t decode_flag = 0;//允许解包标志位，若capData中出现/r/n则视作capData中存放一个完整的数据帧
uint8_t capData[REFEREE_BUFFER_SIZE];//由于溪地创新2024电容反馈内容不定，capData用于存放从串口缓冲区中提取的信息，经由此处汇总消息再解包
//将串口缓冲区内容移至capData并根据结束标志位置解包标志位decode_flag
void CapRxCallback(uint8_t *ReadFromUsart)
{
    for(uint8_t i = 0;i < REFEREE_BUFFER_SIZE && capList_index < REFEREE_BUFFER_SIZE;i++)
    {
        if(ReadFromUsart[i] == 0x0D)//  0x0D->'/r'
        {
            decode_flag = 1;
            break;
        }
        if(ReadFromUsart[i] != 0)
            capData[capList_index++] = ReadFromUsart[i];
    }
    if(decode_flag)
    {
        Cap_read_data(capData);
        memset(capData, 0, sizeof(capData));
        decode_flag = 0;
    }
    if(capList_index >= REFEREE_BUFFER_SIZE)
    {
        capList_index = 0;
        memset(capData, 0, sizeof(capData));
    }
}

//根据数据位抠出数据来
void Cap_info_decode(uint8_t cmd, uint8_t *ReadFromUsart)
{
    if(ReadFromUsart[iCap + 4] == '.')//Integer1
    {
        Integer = ReadFromUsart[iCap + 3] - '0';
        if(ReadFromUsart[iCap + 7] == ' ')//Float2
            Float = ((fp32)((ReadFromUsart[iCap + 5] - '0') * 10 + ReadFromUsart[iCap + 6] - '0')) / 100;
    }
    else if(ReadFromUsart[iCap + 5] == '.')//Integer2
    {
        Integer = ReadFromUsart[iCap + 4] - '0' + (ReadFromUsart[iCap + 3] - '0') * 10;
        if(ReadFromUsart[iCap + 8] == ' ')//Float2
            Float = ((fp32)((ReadFromUsart[iCap + 6] - '0') * 10 + ReadFromUsart[iCap + 7] - '0')) / 100;
    }
    if(Float + Integer != 0)
    {
        switch (cmd) {
            case Vi:
                capReceiveData.voltage_in = Float + Integer;
                break;
            case Vo:
                capReceiveData.voltage_out = Float + Integer;
                break;
            case Pi:
                capReceiveData.power_in = Float + Integer;
                break;
            case Ii:
                capReceiveData.current_in = Float + Integer;
                break;
            case Io:
                capReceiveData.current_out = Float + Integer;
                break;
            case Pr:
                capReceiveData.power_reference = Float + Integer;
                break;
        }
    }
}

//遍历capData，找到数据位
bool_t Cap_read_data(uint8_t *ReadFromUsart)
{
    if(ReadFromUsart == NULL)
        return 0;
    uint8_t decode_count = 0;
    for(iCap = 0;iCap < REFEREE_BUFFER_SIZE;)
    {
        if(ReadFromUsart[iCap] == 'V' && ReadFromUsart[iCap + 1] == 'i')//input voltage
        {
            Cap_info_decode(Vi, ReadFromUsart);
            decode_count++;
        }
        else if(ReadFromUsart[iCap] == 'V' && ReadFromUsart[iCap + 1] == 'o')//output voltage
        {
            Cap_info_decode(Vo, ReadFromUsart);
            decode_count++;
        }
        else if(ReadFromUsart[iCap] == 'P' && ReadFromUsart[iCap + 1] == 'i')//input power
        {
            Cap_info_decode(Pi, ReadFromUsart);
            decode_count++;
        }
        else if(ReadFromUsart[iCap] == 'I' && ReadFromUsart[iCap + 1] == 'i')//input current
        {
            Cap_info_decode(Ii, ReadFromUsart);
            decode_count++;
        }
        else if(ReadFromUsart[iCap] == 'I' && ReadFromUsart[iCap + 1] == 'o')//output current
        {
            Cap_info_decode(Io, ReadFromUsart);
            decode_count++;
        }
        else if(ReadFromUsart[iCap] == 'P' && ReadFromUsart[iCap + 1] == 's')//reference power
        {
            Cap_info_decode(Pr, ReadFromUsart);
            decode_count++;
        }
        if(decode_count == 6)
            break;
        iCap++;
    }
}

//发送最大功率给溪地电容控制板
void send_to_cap_xidi(int power_limit,UART_HandleTypeDef *huart){
    if(power_limit<=20){
        return;
    }
    if(power_limit>99){
        uint8_t send_data[7]="P100P\r\n";
        HAL_UART_Transmit(huart,send_data,sizeof (send_data),1000);
    }
    else{
        int ten=(power_limit/10)%10;
        int ge=power_limit%10;
        uint8_t send_data[7]="P000P\r\n";
        send_data[2]=ten+'0';
        send_data[3]=ge+'0';
        HAL_UART_Transmit(huart,send_data,sizeof (send_data),1000);
    }
}
#endif //GIMBAL
