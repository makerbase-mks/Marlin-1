/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
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

/***************************************************************************
 * ARM CPU Exception handler
 ***************************************************************************/

#if defined(__arm__) || defined(__thumb__)

#include "exception_hook.h"
#include "../backtrace/backtrace.h"
#include "../HAL_MinSerial.h"

#define HW_REG(X)  (*((volatile unsigned long *)(X)))

// Default function use the CPU VTOR register to get the vector table.
// Accessing the CPU VTOR register is done in assembly since it's the only way that's common to all current tool
unsigned long get_vtor() { return HW_REG(0xE000ED08); } // Even if it looks like an error, it is not an error
void * hook_get_hardfault_vector_address(unsigned vtor)  { return (void*)(vtor + 0x03); }
void * hook_get_memfault_vector_address(unsigned vtor)   { return (void*)(vtor + 0x04); }
void * hook_get_busfault_vector_address(unsigned vtor)   { return (void*)(vtor + 0x05); }
void * hook_get_usagefault_vector_address(unsigned vtor) { return (void*)(vtor + 0x06); }

// Common exception frame for ARM, should work for all ARM CPU
// Described here (modified for convenience): https://interrupt.memfault.com/blog/cortex-m-fault-debug
struct __attribute__((packed)) ContextStateFrame {
  uint32_t r0;
  uint32_t r1;
  uint32_t r2;
  uint32_t r3;
  uint32_t r12;
  uint32_t lr;
  uint32_t pc;
  uint32_t xpsr;
};

struct __attribute__((packed)) ContextSavedFrame {
  uint32_t R0;
  uint32_t R1;
  uint32_t R2;
  uint32_t R3;
  uint32_t R12;
  uint32_t LR;
  uint32_t PC;
  uint32_t XPSR;

  uint32_t CFSR;
  uint32_t HFSR;
  uint32_t DFSR;
  uint32_t AFSR;
  uint32_t MMAR;
  uint32_t BFAR;

  uint32_t ESP;
  uint32_t ELR;
};


// From https://interrupt.memfault.com/blog/cortex-m-fault-debug with addition of saving the exception's LR and the fault type
// Please notice that the fault itself is accessible in the CFSR register at address 0xE000ED28
#define WRITE_HANDLER(X) \
  __attribute__((naked)) void FaultHandler_##X() { \
    __asm__ __volatile__ ( \
        "tst lr, #4\n" \
        "ite eq\n" \
        "mrseq r0, msp\n" \
        "mrsne r0, psp\n" \
        "mov r1,lr\n" \
        "mov r2,#" #X "\n" \
        "b CommonHandler_C\n" \
    ); \
  }

WRITE_HANDLER(1);
WRITE_HANDLER(2);
WRITE_HANDLER(3);
WRITE_HANDLER(4);

// Must be a macro to avoid creating a function frame
#define HALT_IF_DEBUGGING()                              \
  do {                                                   \
    if (HW_REG(0xE000EDF0) & (1 << 0)) {                 \
      __asm("bkpt 1");                                   \
    }                                                    \
} while (0)

// Resume from a fault (if possible) so we can still use interrupt based code for serial output
// In that case, we will not jump back to the faulty code, but instead to a dumping code and then a
// basic loop with watchdog calling or manual resetting
static ContextSavedFrame savedFrame;
static uint8_t           lastCause;
bool resume_from_fault() {
  static const char* causestr[] = { "Unknown", "Hard", "Mem", "Bus", "Usage" };
  // Reinit the serial link (might only work if implemented in each of your boards)
  MinSerial::init();

  MinSerial::TX("\n\n## Software Fault detected ##\n");
  MinSerial::TX("Cause: "); MinSerial::TX(causestr[lastCause]); MinSerial::TX('\n');

  MinSerial::TX("R0   : "); MinSerial::TXHex(savedFrame.R0);   MinSerial::TX('\n');
  MinSerial::TX("R1   : "); MinSerial::TXHex(savedFrame.R1);   MinSerial::TX('\n');
  MinSerial::TX("R2   : "); MinSerial::TXHex(savedFrame.R2);   MinSerial::TX('\n');
  MinSerial::TX("R3   : "); MinSerial::TXHex(savedFrame.R3);   MinSerial::TX('\n');
  MinSerial::TX("R12  : "); MinSerial::TXHex(savedFrame.R12);  MinSerial::TX('\n');
  MinSerial::TX("LR   : "); MinSerial::TXHex(savedFrame.LR);   MinSerial::TX('\n');
  MinSerial::TX("PC   : "); MinSerial::TXHex(savedFrame.PC);   MinSerial::TX('\n');
  MinSerial::TX("PSR  : "); MinSerial::TXHex(savedFrame.XPSR); MinSerial::TX('\n');

  // Configurable Fault Status Register
  // Consists of MMSR, BFSR and UFSR
  MinSerial::TX("CFSR : "); MinSerial::TXHex(savedFrame.CFSR); MinSerial::TX('\n');

  // Hard Fault Status Register
  MinSerial::TX("HFSR : "); MinSerial::TXHex(savedFrame.HFSR); MinSerial::TX('\n');

  // Debug Fault Status Register
  MinSerial::TX("DFSR : "); MinSerial::TXHex(savedFrame.DFSR); MinSerial::TX('\n');

  // Auxiliary Fault Status Register
  MinSerial::TX("AFSR : "); MinSerial::TXHex(savedFrame.AFSR); MinSerial::TX('\n');

  // Read the Fault Address Registers. These may not contain valid values.
  // Check BFARVALID/MMARVALID to see if they are valid values
  // MemManage Fault Address Register
  MinSerial::TX("MMAR : "); MinSerial::TXHex(savedFrame.MMAR); MinSerial::TX('\n');

  // Bus Fault Address Register
  MinSerial::TX("BFAR : "); MinSerial::TXHex(savedFrame.BFAR); MinSerial::TX('\n');

  MinSerial::TX("ExcLR: "); MinSerial::TXHex(savedFrame.ELR); MinSerial::TX('\n');
  MinSerial::TX("ExcSP: "); MinSerial::TXHex(savedFrame.ESP); MinSerial::TX('\n');

  // The stack pointer is pushed by 8 words upon entering an exception, so we need to revert this
  backtrace_ex(savedFrame.ESP + 8*4, savedFrame.LR, savedFrame.PC); 

  // Call the last resort function here
  hook_last_resort_func();

  const uint32_t start = millis(), end = start + 100; // 100ms should be enough
  // We need to wait for the serial buffers to be output but we don't know for how long
  // So we'll just need to refresh the watchdog for a while and then stop for the system to reboot
  uint32_t last = start;
  while (PENDING(last, end)) {
    watchdog_refresh();
    while (millis() == last) { /* nada */ }
    last = millis();
    MinSerial::TX('.');
  }

  // Reset now by reinstantiating the bootloader's vector table
  HW_REG(0xE000ED08) = 0;
  // Restart watchdog
  #if !ENABLED(USE_WATCHDOG)
    // No watchdog, let's perform ARMv7 reset instead by writing to AIRCR register with VECTKEY set to SYSRESETREQ
    HW_REG(0xE000ED0C) = (HW_REG(0xE000ED0C) & 0x0000FFFF) | 0x05FA0004;
  #endif

  while(1) {} // Bad luck, nothing worked
}

// Make sure the compiler does not optimize the frame argument away
extern "C"
__attribute__((optimize("O0")))
void CommonHandler_C(ContextStateFrame * frame, unsigned long lr, unsigned long cause) {

  // If you are using it'll stop here
  HALT_IF_DEBUGGING();
  
  // Save the state to backtrace later on
  memcpy(&savedFrame, frame, sizeof(*frame));
  lastCause = cause;
  
  volatile uint32_t &CFSR = HW_REG(0xE000ED28);
  savedFrame.CFSR = CFSR; 
  savedFrame.HFSR = HW_REG(0xE000ED2C);  
  savedFrame.DFSR = HW_REG(0xE000ED30);  
  savedFrame.AFSR = HW_REG(0xE000ED3C);  
  savedFrame.MMAR = HW_REG(0xE000ED34);  
  savedFrame.BFAR = HW_REG(0xE000ED38);  
  savedFrame.ESP  = (unsigned long)frame; // Even on return, this should not be overwritten by the CPU
  savedFrame.ELR  = lr;        

  // First check if we can resume from this exception to our own handler safely
  // If we can, then we don't need to disable interrupts and the usual serial code
  // can be used
  
  //const uint32_t non_usage_fault_mask = 0x0000FFFF;
  //const bool non_usage_fault_occurred = (CFSR & non_usage_fault_mask) != 0;
  // the bottom 8 bits of the xpsr hold the exception number of the
  // executing exception or 0 if the processor is in Thread mode
  const bool faulted_from_exception = ((frame->xpsr & 0xFF) != 0);
  if (!faulted_from_exception) { // Not sure about the non_usage_fault, we want to try anyway, don't we ? && !non_usage_fault_occurred)
    // Try to resume to our handler here
    CFSR |= CFSR; // The ARM programmer manual says you must write to 1 all fault bits to clear them so this instruction is correct
    // The frame will not be valid when returning anymore, let's clean it    
    savedFrame.CFSR = 0;
    
    frame->pc = (uint32_t)resume_from_fault; // Patch where to return to
    frame->lr = 0xdeadbeef;  // If our handler returns (it shouldn't), let's make it trigger an exception immediately
    frame->xpsr = (1 << 24); // Need to clean the PSR register to thumb II only
    MinSerial::force_using_default_output = true;
    return; // The CPU will resume in our handler hopefully, and we'll try to use default serial output 
  }

  // Sorry, we need to emergency code here since the fault is too dangerous to recover from 
  MinSerial::force_using_default_output = false;
  resume_from_fault();
}

void hook_cpu_exceptions() {
  // On ARM 32bits CPU, the vector table is like this:
  // 0x0C => Hardfault
  // 0x10 => MemFault
  // 0x14 => BusFault
  // 0x18 => UsageFault

  // Unfortunately, it's usually run from flash, and we can't write to flash here directly to hook our instruction
  // We could set an hardware breakpoint, and hook on the fly when it's being called, but this
  // is hard to get right and would probably break debugger when attached

  // So instead, we'll allocate a new vector table filled with the previous value except
  // for the fault we are interested in.
  // Now, comes the issue to figure out what is the current vector table size
  // There is nothing telling us what is the vector table as it's per-cpu vendor specific.
  // BUT: we are being called at the end of the setup, so we assume the setup is done
  // Thus, we can read the current vector table until we find an address that's not in flash, and it would mark the
  // end of the vector table (skipping the fist entry obviously)
  // The position of the program in flash is expected to be at 0x08xxx xxxx on all known platform for ARM and the
  // flash size is available via register 0x1FFFF7E0 on STM32 family, but it's not the case for all ARM boards
  // (accessing this register might trigger a fault if it's not implemented).

  // So we'll simply mask the top 8 bits of the first handler as an hint of being in the flash or not -that's poor and will
  // probably break if the flash happens to be more than 128MB, but in this case, we are not magician, we need help from outside.

  unsigned long * vecAddr = (unsigned long*)get_vtor();

  #ifdef VECTOR_TABLE_SIZE
    uint32_t vec_size = VECTOR_TABLE_SIZE;
    alignas(128) static unsigned long vectable[VECTOR_TABLE_SIZE] ;
  #else
    #ifndef IS_IN_FLASH
      #define IS_IN_FLASH(X) (((unsigned long)X & 0xFF000000) == 0x08000000)
    #endif

    // When searching for the end of the vector table, this acts as a limit not to overcome
    #ifndef VECTOR_TABLE_SENTINEL
      #define VECTOR_TABLE_SENTINEL 80
    #endif

    // Find the vector table size
    uint32_t vec_size = 1;
    while (IS_IN_FLASH(vecAddr[vec_size]) && vec_size < VECTOR_TABLE_SENTINEL)
      vec_size++;

    // We failed to find a valid vector table size, let's abort hooking up
    if (vec_size == VECTOR_TABLE_SENTINEL) return;
    // Poor method that's wasting RAM here, but allocating with malloc and alignment would be worst
    // 128 bytes alignement is required for writing the VTOR register
    alignas(128) static unsigned long vectable[VECTOR_TABLE_SENTINEL];
  #endif

  // Copy the current vector table into the new table
  for (uint32_t i = 0; i < vec_size; i++)
    vectable[i] = vecAddr[i];

  // Let's hook now with our functions
  vectable[(unsigned long)hook_get_hardfault_vector_address(0)]  = (unsigned long)&FaultHandler_1;
  vectable[(unsigned long)hook_get_memfault_vector_address(0)]   = (unsigned long)&FaultHandler_2;
  vectable[(unsigned long)hook_get_busfault_vector_address(0)]   = (unsigned long)&FaultHandler_3;
  vectable[(unsigned long)hook_get_usagefault_vector_address(0)] = (unsigned long)&FaultHandler_4;

  // Finally swap with our own vector table
  HW_REG(0xE000ED08) = (unsigned long)vectable | (1<<29UL); // 29th bit is for telling the CPU the table is now in SRAM (should be present already)
}

#endif // __arm__ || __thumb__
