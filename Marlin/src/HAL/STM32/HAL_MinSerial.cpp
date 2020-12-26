/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (c) 2017 Victor Perez
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#if defined(ARDUINO_ARCH_STM32) && !defined(STM32GENERIC)

#include "../../inc/MarlinConfig.h"

#if ENABLED(POST_MORTEM_DEBUGGING)

#include "../shared/HAL_MinSerial.h"
#include "watchdog.h"

/* Instruction Synchronization Barrier */
#define isb() __asm__ __volatile__ ("isb" : : : "memory")

/* Data Synchronization Barrier */
#define dsb() __asm__ __volatile__ ("dsb" : : : "memory")

// Dumb mapping over the registers of a USART device on STM32
struct USARTMin {
    volatile uint32_t SR;
    volatile uint32_t DR;
    volatile uint32_t BRR;
    volatile uint32_t CR1;
};

#if WITHIN(SERIAL_PORT, 1, 6)
  // Depending on the CPU, the serial port is different for USART1
  static const uint32_t regsAddr[] = {  
                                      /* USART1 */
                                  #if defined(STM32F1xx)
                                      0x40013800,
                                  #else
                                      0x40011000,
                                  #endif
                                      /* USART2 */ 0x40004400, /* USART3 */ 0x40004800, /* UART4_BASE */ 0x40004C00, /* UART5_BASE */ 0x40005000, /* USART6 */ 0x40011400 };
  static USARTMin * regs = (USARTMin*)regsAddr[SERIAL_PORT - 1];
#endif

static void TXBegin() {
  #if !WITHIN(SERIAL_PORT, 1, 6)
    #warning Using POST_MORTEM_DEBUGGING requires a physical U(S)ART hardware in case of severe error. 
    #warning Disabling the severe error reporting feature currently because the used serial port is not a HW port.
  #else
    // This is common between STM32F1/STM32F2 and STM32F4
    const int nvicUART[] = { /* NVIC_USART1 */ 37, /* NVIC_USART2 */ 38, /* NVIC_USART3 */ 39, /* NVIC_UART4 */ 52, /* NVIC_UART5 */ 53, /* NVIC_USART6 */ 71 };
    int nvicIndex = nvicUART[SERIAL_PORT - 1];

    struct NVICMin
    {
      volatile uint32_t ISER[32];
      volatile uint32_t ICER[32];
    };

    NVICMin * nvicBase = (NVICMin*)0xE000E100;
    nvicBase->ICER[nvicIndex / 32] |= (1UL << (nvicIndex % 32));

    // We NEED memory barriers to ensure Interrupts are actually disabled!
    // ( https://dzone.com/articles/nvic-disabling-interrupts-on-arm-cortex-m-and-the )
    dsb();
    isb();

    // Example for USART1 disable:  (RCC->APB2ENR &= ~(RCC_APB2ENR_USART1EN))
    // Too difficult to reimplement here, let's query the STM32duino macro here
    #if SERIAL_PORT == 1
      __HAL_RCC_USART1_CLK_DISABLE();
      __HAL_RCC_USART1_CLK_ENABLE();
    #elif SERIAL_PORT == 2
      __HAL_RCC_USART2_CLK_DISABLE();
      __HAL_RCC_USART2_CLK_ENABLE();
    #elif SERIAL_PORT == 3
      __HAL_RCC_USART3_CLK_DISABLE();
      __HAL_RCC_USART3_CLK_ENABLE();
    #elif SERIAL_PORT == 4
      __HAL_RCC_UART4_CLK_DISABLE(); // BEWARE: UART4 and not USART4 here
      __HAL_RCC_UART4_CLK_ENABLE();
    #elif SERIAL_PORT == 5
      __HAL_RCC_UART5_CLK_DISABLE(); // BEWARE: UART5 and not USART5 here
      __HAL_RCC_UART5_CLK_ENABLE();
    #elif SERIAL_PORT == 6
      __HAL_RCC_USART6_CLK_DISABLE();
      __HAL_RCC_USART6_CLK_ENABLE();
    #endif

    uint32_t brr = regs->BRR;
    regs->CR1 = 0; // Reset the USART
    regs->CR2 = 0; // 1 stop bit

    // If we don't touch the BRR (baudrate register), we don't need to recompute.
    regs->BRR = brr;

    regs->CR1 = (1<<3) | (1<<13); // 8 bits, no parity, 1 stop bit (TE | UE)
  #endif
}

// A SW memory barrier, to ensure GCC does not overoptimize loops
#define sw_barrier() __asm__ volatile("": : :"memory");
static void TX(char c) {
  while (!(regs->SR & USART_SR_TXE)) {
    TERN_(USE_WATCHDOG, HAL_watchdog_refresh());
    sw_barrier();
  }
  regs->DR = c;
}

void install_min_serial() {
  HAL_min_serial_init = &TXBegin;
  HAL_min_serial_out = &TX;
}

#if !ENABLED(DYNAMIC_VECTORTABLE)
extern "C"
{
  __attribute__((naked)) void JumpHandler_ASM() {
    __asm__ __volatile__ ( 
        "b CommonHandler_ASM\n" 
    );
  }
  
  void __attribute__((naked, alias("JumpHandler_ASM"))) HardFault_Handler();
  void __attribute__((naked, alias("JumpHandler_ASM"))) BusFault_Handler();
  void __attribute__((naked, alias("JumpHandler_ASM"))) UsageFault_Handler();
  void __attribute__((naked, alias("JumpHandler_ASM"))) MemManage_Handler();
  void __attribute__((naked, alias("JumpHandler_ASM"))) NMI_Handler();
}
#endif

#endif // POST_MORTEM_DEBUGGING
#endif // ARDUINO_ARCH_STM32
