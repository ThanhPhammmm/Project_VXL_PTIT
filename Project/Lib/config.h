#ifndef _CONFIG_H_
#define _CONFIG_H_

#include "stm32f10x.h"                  // Device header

void RCC_Config(void);
void GPIO_Config(void);
void USART1_Config(void);
void TIM2_Config(void);
void TIM3_Config(void);
void SysTickConfig(uint32_t ticks);

#endif