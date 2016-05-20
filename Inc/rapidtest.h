#include "stm32l4xx_hal.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void MX_ADC1_Init(void);
void MX_ADC2_Init(void);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
void init(void);
uint16_t getadc1(void);
uint16_t getadc2(void);
void sendusb(uint8_t data);