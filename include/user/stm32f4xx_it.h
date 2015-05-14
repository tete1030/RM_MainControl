#ifndef __STM32F4xx_IT_H
#define __STM32F4xx_IT_H

#ifdef __cplusplus
extern "C"
{
#endif 

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);

#ifdef __cplusplus
}
#endif

// 遥控器接收器
#define ITP_USART2_DMA_RX_PREEMPTION 4
#define ITP_USART2_DMA_RX_SUB 0


// 电脑
#define ITP_USART3_DMA_TX_PREEMPTION 3
#define ITP_USART3_DMA_TX_SUB 2

#define ITP_USART3_DMA_RX_PREEMPTION 3
#define ITP_USART3_DMA_RX_SUB 1

#define ITP_USART3_GLOBAL_PREEMPTION 1
#define ITP_USART3_GLOBAL_SUB 0

// 与云台
#define ITP_CAN2_RX0_PREEMPTION 2
#define ITP_CAN2_RX0_SUB 0

// 与驱动
#define ITP_CAN1_RX0_PREEMPTION 2
#define ITP_CAN1_RX0_SUB 1

#define ITP_CAN1_TX_PREEMPTION 2
#define ITP_CAN1_TX_SUB 2

#define ITP_SYSTICK_PREEMPTION 0
#define ITP_SYSTICK_PREEMPTION 0

// ******** Following Not Used By Apr 20, 2015 *********

#define ITP_TIM6_DAC_PREEMPTION 10
#define ITP_TIM6_DAC_SUB 0

// Key
#define ITP_EXTI4_PREEMPTION 10
#define ITP_EXTI4_SUB 0

#define ITP_EXTI15_10_PREEMPTION 10
#define ITP_EXTI15_10_SUB 0

#define ITP_EXTI0_PREEMPTION 10
#define ITP_EXTI0_SUB 0

#define ITP_EXTI1_PREEMPTION 10
#define ITP_EXTI1_SUB 0

#define ITP_EXTI9_5_PREEMPTION 10
#define ITP_EXTI9_5_SUB 0



#endif /* __STM32F4xx_IT_H */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
