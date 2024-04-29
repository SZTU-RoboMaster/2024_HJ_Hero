//#ifndef BSP_LED_H
//#define BSP_LED_H
//#include "struct_typedef.h"
//
///**
//  * @brief          aRGB show
//  * @param[in]      aRGB: 0xaaRRGGBB, 'aa' is alpha, 'RR' is red, 'GG' is green, 'BB' is blue
//  * @retval         none
//  */
///**
//  * @brief          ��ʾRGB
//  * @param[in]      aRGB:0xaaRRGGBB,'aa' ��͸����,'RR'�Ǻ�ɫ,'GG'����ɫ,'BB'����ɫ
//  * @retval         none
//  */
//extern void aRGB_led_show(uint32_t aRGB);
//extern void led_init();
//extern void led_flow();
//extern void led_off_flow();
//extern void led_task(void const *pvParameters);
//extern void led_dashboard();
//extern void led_light(uint8_t led_1,uint8_t led_2,uint8_t led_3,uint8_t led_4,uint8_t led_5,uint8_t led_6,uint8_t led_7);
//
//#define DELAY_TIME 100
//#define DELAY_LONG_TIME 200
//
//#define CHASSIS_TASK_INIT_TIME 357
////PWM�˵� io��
//#define LED1_PORT GPIOE
//#define LED1_PIN GPIO_PIN_9
//#define LED2_PORT  GPIOE
//#define LED2_PIN GPIO_PIN_11
//#define LED3_PORT  GPIOE
//#define LED3_PIN  GPIO_PIN_13
//#define LED4_PORT GPIOC
//#define LED4_PIN GPIO_PIN_6
//#define LED5_PORT GPIOI
//#define LED5_PIN GPIO_PIN_6
//#define LED6_PORT GPIOI
//#define LED6_PIN GPIO_PIN_7
//
////�û��ӿڶ˵�
//#define LED7_PORT GPIOB
//#define LED7_PIN GPIO_PIN_14
//
//typedef enum{
//    ON_FLOW=0,
//    OFF_FLOW,
//    AUTO_AIM,
//    SHOOT,
//    CAP,
//    SPIN,
//
//    MODE_NUM
//}led_mode_e;
//
//typedef struct {
//    led_mode_e mode;
//
//
//}led_t;
//
//
//
//#endif
#ifndef BSP_LED_H
#define BSP_LED_H
#include "main.h"

typedef struct			//��ɫ�ṹ��
{
    uint8_t R;
    uint8_t G;
    uint8_t B;
}RGBColor_TypeDef;

typedef enum
{
    ON_FLOW=0,
    OFF_FLOW,
    AUTO_AIM,
    SHOOT,
    CAP,
    SPIN,

    MODE_NUM
}led_mode_e;

typedef struct
{
    led_mode_e mode;
}led_t;

static uint8_t RGB_Counter = 0;	// ��SPI2������ɻص��������ļ�����
#define RGB_NUM 7	// ������������Ȼ����˺꣬����д��ʱ��Ĭ��ֻ��7�����Ե����������˵Ļ���ü��ÿһ��

// ֡������ͺ���
//void RGB_RST(void);
//void RGB_Reset(void);
// ��ɫ���ú���
void RGB_Set_Color(uint8_t LedId, RGBColor_TypeDef Color);
// RGB ˢ�º���
void RGB_Refresh(uint8_t refresh_NUM);

// ��C���Դ�LED
extern void aRGB_led_show(uint32_t aRGB);
// ui�ư�������
void led_dashboard();
// �ſ��Ч
void RGB_Single_Color(uint16_t RGB_LEN, uint8_t R, uint8_t G, uint8_t B);	// ��ɫ�㶨��Ч
void RGB_Flow(uint16_t RGB_LEN, uint8_t R, uint8_t G, uint8_t B);	// ��������ˮ��
void RGB_Rainbow(uint8_t RGB_LEN, uint8_t MODE);    // �ʺ��Ч

// ������ɫ����
void RGB_RED(uint16_t RGB_LEN);		    //��
void RGB_GREEN(uint16_t RGB_LEN);		//��
void RGB_BLUE(uint16_t RGB_LEN);		//��
void RGB_YELLOW(uint16_t RGB_LEN);		//��
void RGB_MAGENTA(uint16_t RGB_LEN);	    //��
void RGB_BLACK(uint16_t RGB_LEN);		//��
void RGB_Test();                        //
void RGB_Debug();                       //
void RGB_WHITE(uint16_t RGB_LEN);		//��
void RGB_BRIGHT(uint16_t RGB_LEN);	    //��
#endif /* __WS2812_H */
