/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 * Board specific routines for chopstx thread library on stm32f446.
 * Based with the stm32f107 version of board specific routines written by
 * Pavel Kirienko <pavel.kirienko@gmail.com> for chibios.
 */

#include "board.hpp"
#include <cstring>
#include <unistd.h>

extern "C" {
#include <chopstx.h>
#include <chip.h>
#include <board.h>
static void set_led(bool on);
static void nvic_system_reset(void);
}

#define lowsyslog(...)

namespace board
{

void init()
{
}

__attribute__((noreturn))
void die(int error)
{
    lowsyslog("Fatal error %i\n", error);
    while (1)
    {
        setLed(false);
        chopstx_usec_wait (1000*1000);
        setLed(true);
        chopstx_usec_wait (1000*1000);
    }
}

void setLed(bool state)
{
    set_led(state);
}

void restart()
{
    nvic_system_reset();
}

void readUniqueID(std::uint8_t bytes[UniqueIDSize])
{
    std::memcpy(bytes, reinterpret_cast<const void*>(0x1FFFF7E8), UniqueIDSize);
}

}

/*
 * Early init
 */
extern "C"
{
void clock_init(void)
{
    // This is NOT clock setting but perhaps good time for it.
    SCB->CPACR |= (0xf << 20);
    asm volatile ("dsb\n\t"
                  "isb" : : : "memory");
#if defined(USE_FPU_AUTO_SAVE)
    // Auto save/restore fp context on exceptions.
    FPU->FPCCR = (1 << 31);
#else
    // No auto save/restore.
    FPU->FPCCR = 0;
#endif

    // HSI setup
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY))
        ;
    // Reset CR
    RCC->CR = 0x83;
    RCC->CFGR = 0;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI)
        ;
    // Reset PLLCFGR
    RCC->PLLCFGR = 0x24003010;
    // Reset PLLI2SCFGR
    RCC->PLLSAICFGR = 0x24003010;
    // Reset PLLSAICFGR
    RCC->PLLSAICFGR = 0x24003000;

    // Disable all interrupts
    RCC->CIR = 0;

    // HSE setup
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY))
        ;

    // PLL setup
    RCC->PLLCFGR = (RCC_PLLCFGR_PLLSRC_HSE
                    | (8 << 0)		// PLLM=8
                    | (360 << 6)	// PLLN=360
                    | (0 << 16)		// PLLP=DIV2 0:DIV2 1:DIV4 2:DIV6 3:DIV8
                    | (6 << 24)		// PLLQ=6
                    | (2 << 28));	// PLLR=2
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY))
        ;

    // PLLI2S setup
    RCC->PLLI2SCFGR = ((8 << 0)	    // PLLM=8 */
                       | (192 << 6)	// PLLN=192 */
                       | (0 << 16)	// PLLP=DIV2 0:DIV2 1:DIV4 2:DIV6 3:DIV8
                       | (6 << 24)	// PLLQ=6
                       | (2 << 28));// PLLR=2
    RCC->CR |= RCC_CR_PLLI2SON;
    while (!(RCC->CR & RCC_CR_PLLI2SRDY))
        ;

    // PLLSAI setup
    RCC->PLLSAICFGR = ((8 << 0)	    // PLLM=8
                       | (384 << 6)	// PLLN=384
                       | (3 << 16)	// PLLP=DIV8 0:DIV2 1:DIV4 2:DIV6 3:DIV8
                       | (6 << 24)	// PLLQ=6
                       | (2 << 28));// must be Reset value
    RCC->CR |= RCC_CR_PLLSAION;
    while (!(RCC->CR & RCC_CR_PLLSAIRDY))
        ;

    // Clock settings
    RCC->CFGR = ((0 << 30)		// MCO2 0:SYS 1:PLLI2S 2:HSE 3:PLL
                 | (6 << 27)	// MCO2PRE 4:DIV2 5:DIV3 6:DIV4 7:DIV5
                 | (4 << 24)	// MCO1PRE 4:DIV2 5:DIV3 6:DIV4 7:DIV5
                 | (0 << 23)	// I2SSRC 0:PLLI2S 1:I2S_CKIN
                 | (0 << 21)	// MCO1 0:HSI 1:LSE 2:HSE 3:PLL
                 | (0 << 16)	// RTCPRE HSE division factor
                 | RCC_CFGR_PPRE2_DIV2	// 4:DIV2 5:DIV4 6:DIV8 7:DIV16
                 | RCC_CFGR_PPRE1_DIV4
                 | (0 << 4)		// HPRE 0xxx:DIV1 4:DIV2 ... 15:DIV512
                 | (0 << 2)		// SWS read only bits
                 | (0 << 0));	// SW 0:HSI 1:HSE 2:PLL

    /*
     * We don't touch RCC->CR2, RCC->CFGR2, RCC->CFGR3, and RCC->CIR.
     */

    // Flash setup
    FLASH->ACR = (FLASH_ACR_LATENCY_5WS	// 6 CPU cycle wait
                  | FLASH_ACR_PRFTEN
                  | FLASH_ACR_ICEN
                  | FLASH_ACR_DCEN);

    // CRC
    RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;

    // Switching on the configured clock source.
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SW_PLL) != RCC_CFGR_SW_PLL)
        ;

    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    RCC->APB2RSTR = RCC_APB2RSTR_SYSCFGRST;
    RCC->APB2RSTR = 0;

    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    RCC->APB1RSTR = RCC_AHB1RSTR_DMA2RST;
    RCC->APB1RSTR = 0;

#if defined(HAVE_SYS_H)
    // Use vectors on RAM
    SYSCFG->MEMRMP = (SYSCFG->MEMRMP & ~SYSCFG_MEMRMP_MEM_MODE) | 0x3;
#endif
}

/*
 * Port A setup.
 * PA5  - LED (LED 1:ON 0:OFF)
 * PA13, PA14 - SWD(AF0)
 * PA11 - CAN1_RX (AF9)
 * PA12 - CAN1_TX (AF9)
 * PA15  - I2S1_WS (AF5)
 */
#define GPIOA_MODER   0xaa800400 // AF Pin 15,14,13,12,11,4 Output Pin5
#define GPIOA_OTYPER  0x00000000 // Push-Pull
#define GPIOA_OSPEEDR 0x03c00c00 // High speed: Pin12,11,5
#define GPIOA_PUPDR   0x00000000 // No pull-up/pull-down
#define GPIOA_AFR0    0x00000000 // AF5 Pin4
#define GPIOA_AFR1    0x50099000 // AF9 Pin12,11

/*
 * Port B setup.
 * PB3  - I2S1_CK (AF5)
 * PB5  - I2S1_SD (AF5)
 * PB8  - I2C1_SCL(AF4)
 * PB9  - I2C1_SDA(AF4)
 */
#define GPIOB_MODER   0x000a0880 // AF4 Pin9,8 AF5 Pin5,3
#define GPIOB_OTYPER  0x00000300 // Open-drain Pin9,8 otherwise Push-Pull
#define GPIOB_OSPEEDR 0x000f0cc0 // High speed: Pin9,8,5,3
#define GPIOB_PUPDR   0x00050000 // Pin9,8 pull-up otherwise no pull-up/down
#define GPIOB_AFR0    0x00505000 // AF5 Pin5,3
#define GPIOB_AFR1    0x00000044 // AF4 Pin9,8

/*
 * Port C setup.
 * PC13  - USER Button
 */
#define GPIOC_MODER   0x000000ff // Input Pin13 Analog input Pin3,2,1,0
#define GPIOC_OTYPER  0x00000000 // Push-Pull
#define GPIOC_OSPEEDR 0x00000000
#define GPIOC_PUPDR   0x00000000 // No pull-up/pull-down

#define GPIO_LED GPIOA
#define GPIO_LED_SET_TO_EMIT    5

void gpio_init(void)
{
    /*
     * Enabling the CAN controllers, then configuring GPIO functions for CAN_TX.
     * Order matters, otherwise the CAN_TX pins will twitch, disturbing the CAN bus.
     */
    RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
    RCC->APB1RSTR = RCC_APB1RSTR_CAN1RST;
    RCC->APB1RSTR = 0;

#if UAVCAN_STM32_NUM_IFACES > 1
    RCC->APB1ENR |= RCC_APB1ENR_CAN2EN;
    RCC->APB1RSTR = RCC_APB1RSTR_CAN2RST;
    RCC->APB1RSTR = 0;
#endif

    // Then enable GPIO clock
    RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN
                     | RCC_AHB1ENR_GPIOBEN
                     | RCC_AHB1ENR_GPIOCEN);
    RCC->AHB1RSTR = (RCC_AHB1RSTR_GPIOARST
                     | RCC_AHB1RSTR_GPIOBRST
                     | RCC_AHB1RSTR_GPIOCRST);
    RCC->AHB1RSTR = 0;

    GPIOA->AFR[0]  = GPIOA_AFR0;
    GPIOA->AFR[1]  = GPIOA_AFR1;
    GPIOA->OSPEEDR = GPIOA_OSPEEDR;
    GPIOA->OTYPER  = GPIOA_OTYPER;
    GPIOA->MODER   = GPIOA_MODER;
    GPIOA->PUPDR   = GPIOA_PUPDR;

    GPIOB->AFR[0]  = GPIOB_AFR0;
    GPIOB->AFR[1]  = GPIOB_AFR1;
    GPIOB->OSPEEDR = GPIOB_OSPEEDR;
    GPIOB->OTYPER  = GPIOB_OTYPER;
    GPIOB->MODER   = GPIOB_MODER;
    GPIOB->PUPDR   = GPIOB_PUPDR;

    GPIOC->OSPEEDR = GPIOC_OSPEEDR;
    GPIOC->OTYPER  = GPIOC_OTYPER;
    GPIOC->MODER   = GPIOC_MODER;
    GPIOC->PUPDR   = GPIOC_PUPDR;
}

static void
set_led (bool on)
{
  if (on)
    GPIO_LED->BSRR = (1 << GPIO_LED_SET_TO_EMIT);
  else
    GPIO_LED->BSRR = (1 << GPIO_LED_SET_TO_EMIT) << 16;
}

#define SYSRESETREQ 0x04
static void
nvic_system_reset (void)
{
    SCB->AIRCR = (0x05FA0000 | (SCB->AIRCR & 0x70) | SYSRESETREQ);
    asm volatile ("dsb");
    for (;;);
}

static void __attribute__ ((naked, section(".sys.reset")))
reset (void)
{
    /*
     * This code may not be at the start of flash ROM, because of DFU.
     * So, we take the address from PC.
     */
    asm volatile ("cpsid	i\n\t"		// Mask all interrupts
                  "ldr	r0, 1f\n\t"     // r0 = SCR
                  "mov	r1, pc\n\t"		// r1 = (PC + 0x1000) & ~0x0fff
                  "mov	r2, #0x1000\n\t"
                  "add	r1, r1, r2\n\t"
                  "sub	r2, r2, #1\n\t"
                  "bic	r1, r1, r2\n\t"
                  "str	r1, [r0, #8]\n\t"	// Set SCR->VCR
                  "ldr	r0, [r1], #4\n\t"
                  "msr	MSP, r0\n\t"	// Main (exception handler) stack
                  "ldr	r0, [r1]\n\t"	// Reset handler
                  "bx	r0\n\t"
                  ".align	2\n"
                  "1:	.word	0xe000ed00"
                  : /* no output */ : /* no input */ : "memory");
    /* Never reach here. */
}


#include <errno.h>

// Heap symbols defined with linker script
extern char __heap_base__;
extern char __heap_end__;

// Modified version of newlib implementation of _sbrk
void* _sbrk (int incr)
{
    static char* heap_end;
    char* prev_heap_end;

    if (heap_end == NULL)
        heap_end = &__heap_base__;

    prev_heap_end = heap_end;

    if (heap_end + incr > &__heap_end__)
    {
        /* Some of the libstdc++-v3 tests rely upon detecting
           out of memory errors, so do not abort here.  */
        errno = ENOMEM;
        return (void *) -1;
    }
    heap_end += incr;

    return (void *) prev_heap_end;
}

typedef void (*handler)(void);
extern uint8_t __ram_end__;

handler vector[] __attribute__ ((section(".vectors"))) = {
    (handler)&__ram_end__,
    reset,
    NULL,
};

const uint8_t sys_version[8] __attribute__((section(".sys.version"))) = {
    3*2+2,	// bLength
    0x03,	// bDescriptorType = USB_STRING_DESCRIPTOR_TYPE
    // chopstx version: "2.1"
    '2', 0, '.', 0, '1', 0,
};

const uint32_t __attribute__((section(".sys.board_id")))
sys_board_id = BOARD_ID;

const uint8_t __attribute__((section(".sys.board_name")))
sys_board_name[] = BOARD_NAME;
}
