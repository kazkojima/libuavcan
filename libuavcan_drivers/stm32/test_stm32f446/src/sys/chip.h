/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

extern "C"
{
#include "stm32f446.h"

#define STM32_IRQ_CAN1TX 	19
#define STM32_IRQ_CAN1RX0	20
#define STM32_IRQ_CAN1RX1	21
#define STM32_IRQ_CAN1SCE	22

#if UAVCAN_STM32_NUM_IFACES > 1
#define STM32_IRQ_CAN2TX 	63
#define STM32_IRQ_CAN2RX0	64
#define STM32_IRQ_CAN2RX1	65
#define STM32_IRQ_CAN2SCE	66
#endif

#define STM32_IRQ_TIMX		55

#define TIM_CR1_CEN			(1 << 0)
#define TIM_CR1_URS			(1 << 2)
#define TIM_SR_UIF			(1 << 0)
#define TIM_EGR_UG			(1 << 0)
#define TIM_DIER_UIE		(1 << 0)
}
