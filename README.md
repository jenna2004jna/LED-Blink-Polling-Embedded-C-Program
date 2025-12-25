# LED Blink Using Polling — Embedded C

A simple demonstration of LED blinking using polling (busy-wait) in Embedded C. This repository shows register-level GPIO configuration and a software delay loop. It is intended for learning fundamentals of GPIO control, bitwise operations, and the trade-offs of busy-wait delays vs. hardware timers/interrupts.

---

## Table of Contents
- [Overview](#overview)
- [Features](#features)
- [Hardware Requirements](#hardware-requirements)
- [Reference Wiring / Pinout](#reference-wiring--pinout)
- [Supported / Example MCUs](#supported--example-mcus)
- [Project Structure (suggested)](#project-structure-suggested)
- [Code Explanation](#code-explanation)
  - [GPIO Initialization (register-level)](#gpio-initialization-register-level)
  - [Turning LED On/Off](#turning-led-onoff)
  - [Polling Delay (software)](#polling-delay-software)
- [Timing & Calibration](#timing--calibration)
- [Build & Flash (examples)](#build--flash-examples)
- [Testing & Debugging](#testing--debugging)
- [Limitations](#limitations)
- [Future Enhancements](#future-enhancements)
- [License](#license)
- [Author / Contact](#author--contact)

---

## Overview
This project implements a continuous LED blink loop using a busy-wait delay implemented in software (polling). The code demonstrates:
- Configuring a GPIO pin as an output at register level
- Setting and clearing the output pin
- Implementing a software delay loop to produce ~1 second on/off periods

This approach is intentionally minimal to illustrate the basics before moving to hardware timers or RTOS-based task scheduling.

---

## Features
- Register-level GPIO configuration examples (AVR and STM32-style)
- Minimal, portable C structure which can be adapted to many MCUs
- Explanation of timing and calibration for software delays
- Clear list of limitations and next steps to improve power efficiency and multitasking

---

## Hardware Requirements
- Microcontroller (any general-purpose MCU: AVR, ARM Cortex-M, 8051, etc.)
- LED
- Current-limiting resistor (220–1kΩ typical)
- Breadboard / wires / power supply
- Programmer (e.g., USBasp, ST-Link, AVRISP, etc.)

Recommended safe LED wiring:
- LED anode -> MCU output pin (through resistor)
- LED cathode -> GND

---

## Reference Wiring / Pinout

Example (AVR, ATmega328P on Arduino Uno pin 13):
- MCU pin: PB5 (Arduino D13)
- Connect LED anode -> PB5 via resistor -> cathode -> GND

Example (STM32, Nucleo/BluePill PA5):
- MCU pin: PA5
- Connect LED anode -> PA5 via resistor -> cathode -> GND

Adjust the pin definitions in the code to match the MCU and board you use.

---

## Supported / Example MCUs
- AVR (e.g., ATmega328P) — uses DDRx / PORTx registers
- ARM Cortex-M (e.g., STM32F0/F1/F4) — uses RCC and GPIO registers
- Generic MCU: adapt register names to your device's reference manual

---

## Project Structure (suggested)
- README.md
- src/
  - main.c
  - delay.c
  - delay.h
  - gpio.h
- Makefile or build script
- docs/ (optional diagrams and calibration notes)

Example file responsibilities:
- main.c — application entry, initialization, blink loop
- delay.c/h — software polling delay functions (calibration constant)
- gpio.h — pin definitions and low-level macros (optional)

---

## Code Explanation

Below are short examples showing how to implement the LED blink at register level for AVR and STM32-style MCUs.

### AVR (ATmega328P) — Example
```c
// main.c (AVR example)
#include <avr/io.h>

#define LED_PIN     PB5
#define LED_DDR     DDRB
#define LED_PORT    PORTB

void delay_approx_ms(uint32_t ms) {
    volatile uint32_t i, j;
    const uint32_t inner = 800; // CALIBRATE this for your clock/optimization
    for (i = 0; i < ms; ++i)
        for (j = 0; j < inner; ++j)
            __asm__ __volatile__ ("nop");
}

int main(void) {
    // Configure PB5 as output
    LED_DDR |= (1 << LED_PIN);

    while (1) {
        // Turn LED on
        LED_PORT |= (1 << LED_PIN);
        delay_approx_ms(1000);

        // Turn LED off
        LED_PORT &= ~(1 << LED_PIN);
        delay_approx_ms(1000);
    }
}
```

Notes:
- DDRx sets pin direction; PORTx writes output.
- Calibrate `inner` to get close to 1 second for your clock and optimization level.

---

### STM32 (Cortex-M) — Minimal register-style example (conceptual)
```c
// main.c (STM32 conceptual example)
#include "stm32f4xx.h" // or vendor header

void delay_approx_ms(uint32_t ms) {
    volatile uint32_t i, j;
    const uint32_t inner = 16000; // needs calibration (depends on clock)
    for (i = 0; i < ms; ++i)
        for (j = 0; j < inner; ++j)
            __asm__ __volatile__("nop");
}

int main(void) {
    // Enable GPIOA clock (example for STM32F4)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // Set PA5 to general purpose output (MODER)
    GPIOA->MODER &= ~(3u << (5 * 2));
    GPIOA->MODER |= (1u << (5 * 2)); // 01 = output

    while (1) {
        // Set PA5
        GPIOA->ODR |= (1u << 5);
        delay_approx_ms(1000);

        // Clear PA5
        GPIOA->ODR &= ~(1u << 5);
        delay_approx_ms(1000);
    }
}
```

Notes:
- Names like RCC->AHB1ENR, GPIOA->MODER, GPIOA->ODR are vendor-supplied peripheral structures in CMSIS headers for many STM32 parts.
- Always consult your device reference manual and header files for exact names and bit positions.

---

## Polling Delay (software)
A typical software (busy-wait) delay looks like nested loops. The duration depends on:
- CPU clock frequency
- Compiler optimizations
- Number of cycles per loop iteration

Simple implementation:
```c
void delay_approx_ms(uint32_t ms) {
    volatile uint32_t i, j;
    const uint32_t inner = CALIBRATE_VALUE;
    for (i = 0; i < ms; ++i)
        for (j = 0; j < inner; ++j)
            __asm__ __volatile__("nop");
}
```

To calibrate:
1. Set `CALIBRATE_VALUE` to a rough estimate (based on clock).
2. Measure with a logic analyzer/oscilloscope or observe LED with stopwatch.
3. Adjust until timing matches desired ms.

Pros:
- Simple
Cons:
- CPU is busy and cannot do other work
- Timing varies with optimization level and CPU frequency

---

## Timing & Calibration
Rough timing formula:
- t_total ≈ ms × inner × t_iter
- t_iter = cycles per inner-loop iteration / CPU_frequency

Because t_iter depends on assembler output and optimizations, measure and adjust. Recommended procedure:
1. Compile with same optimization settings used in final project.
2. Toggle an I/O pin at the start and end of `delay_approx_ms` and measure with an oscilloscope to compute the real time per inner loop.
3. Update `CALIBRATE_VALUE` accordingly.

Example quick estimate for AVR at 16 MHz:
- If one inner iteration ≈ 4 cycles → 4 / 16e6 = 250 ns per iteration
- inner = 4000 → 1 ms (approx)
But this is only an estimate; measure to be sure.

---

## Build & Flash (examples)

AVR (ATmega328P) with avr-gcc:
```sh
# Compile
avr-gcc -mmcu=atmega328p -Os -o main.elf main.c
# Convert to hex
avr-objcopy -O ihex -R .eeprom main.elf main.hex
# Flash (example with USBasp)
avrdude -c usbasp -p m328p -U flash:w:main.hex:i
```

ARM Cortex-M with arm-none-eabi-gcc (example):
```sh
arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -Os -o main.elf main.c
arm-none-eabi-objcopy -O binary main.elf main.bin
# Use OpenOCD / ST-Link / vendor tool to flash
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
  -c "program main.elf verify reset exit"
```

Adjust flags and programmer to your board.

---

## Testing & Debugging
- Verify power and wiring first (LED orientation, resistor).
- Use a multimeter to confirm MCU supply voltages.
- If LED never lights:
  - Check pin mapping and that the pin is configured for output.
  - Try toggling more frequently for visual confirmation.
- Use an oscilloscope or logic analyzer to verify timing and pin toggles.
- If compile optimizations remove delay code, ensure volatile variables or inline assembly NOPs are used.

---

## Limitations
- Busy-wait delays block CPU — not suitable for multitasking.
- Timing accuracy is limited; affected by compiler optimization and clock changes.
- Higher power consumption during delay.
- Not suitable for precise timing or real-time tasks.

---

## Future Enhancements
- Use hardware timers for accurate delays with low CPU usage
- Implement interrupt-driven toggling (Timer Compare + ISR)
- Add build scripts / Makefile examples per MCU
- Add CI that compiles for supported targets
- Add RTOS-based task scheduling sample

---

## Contributing
Contributions, bug reports, and improvements welcome. If you want me to:
- add a Makefile,
- add a hardware schematic diagram,
- or create a pre-configured example for a specific MCU (e.g., ATmega328P, STM32F103),

tell me which MCU and toolchain and I will add it.

---

## License
Choose a license (e.g., MIT). Example header for files:
```
/*
 * Copyright (c) 2025 Jenna
 * SPDX-License-Identifier: MIT
 */
```

---

## Author / Contact
Author: Jenna  
Embedded Systems | Firmware | IoT  
GitHub: [jenna2004jna](https://github.com/jenna2004jna)

---

Notes
- This README is written to be beginner friendly but assumes access to MCU datasheets and basic toolchains.
- If you'd like, I can:
  - create and push this README to your repository,
  - add a Makefile,
  - add an AVR-specific example project layout,
  - or include a simple schematic image.
