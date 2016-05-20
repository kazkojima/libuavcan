/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#define STM32_PCLK1_FREQUENCY	(45 * 1000000)

#define PRIO_IRQ_CAN1TX  	8
#define PRIO_IRQ_CAN1RX0 	7
#define PRIO_IRQ_CAN1RX1 	7

#if UAVCAN_STM32_NUM_IFACES > 1
#define PRIO_IRQ_CAN2TX  	8
#define PRIO_IRQ_CAN2RX0 	7
#define PRIO_IRQ_CAN2RX1 	7
#endif

#define PRIO_IRQ_TIMX		10

// UAVCAN_STM32_TIMER_NUMBER=7 in Makefile
#define STM32_IRQ_TIMX		55
#define STM32_TIM7			TIM7
#define STM32_TIMCLK1		(2 * STM32_PCLK1_FREQUENCY)
