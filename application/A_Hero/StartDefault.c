#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

extern osThreadId calibrateTaskHandle;
extern osThreadId chassisTaskHandle;
extern osThreadId gimbalTaskHandle;
extern osThreadId imuTaskHandle;
extern osThreadId detectTaskHandle;
extern osThreadId usbtaskHandle;
extern osThreadId decodetaskHandle;
extern osThreadId uipaintTaskHandle;
extern osThreadId ledTaskHandle;

void StartDefaultTask(void const * argument) {
//#define GIMBAL
#ifdef GIMBAL
    vTaskDelete(chassisTaskHandle);
    vTaskDelete(uipaintTaskHandle);
    vTaskDelete(NULL);

#else //CHASSIS
    vTaskDelete(calibrateTaskHandle);
    vTaskDelete(imuTaskHandle);
    vTaskDelete(detectTaskHandle);
    vTaskDelete(gimbalTaskHandle);
    vTaskDelete(usbtaskHandle);
    vTaskDelete(decodetaskHandle);
    vTaskDelete(ledTaskHandle);
    vTaskDelete(NULL);
#endif //GIMBAL
}