# Udemy Course: Embedded Systems Programming on ARM Cortex-M3/M4 Processor

* [Course Link](https://www.udemy.com/embedded-system-programming-on-arm-cortex-m3m4/)
* [Course Repo]()

## Section 1 - Introduction

### Lecture 2 - Course Overview

* Course Contnt:
	* Procesor Architecture: intrnal arch, operation modes, access levels, bus arch
	* Programming Model: internal disk org, special purp registers, stack opers, reegister access
	* Memory Arch: memory model, memory ifs, bus access 
	* Interrupt Managemt: external irqs system eceptions, priorities
	* Exception Handling
	* Buttons LEDs
	* Programming and debug using KEIL
	* debug with usb logic analyzer
	* lab sessions

### Lecture 3 - Motivation to learn Cortex Family of Processors

* In a MCU used everywhere:
	* Medical
	* Automotive
	* IOT
	* Appliances
	* Automation
	* Consumer
	* Lab Equipment
* Fitbit: ARM CORTEX M3 (STM32L151C6) Ultra Low power
* TomTom wathc: ARM Cortex M7 (SAMSx) Ultra Low Power
* Most MCU manufacturers offer ARM CORTEX M because:
	* minimal cost, silicon area, consumption
	* 32bit => better performance
	* ultra low power to highend
	* customizable processor  to add  Floatpoint, DSP , MPU etc
	* Very powerfuland easy interupt system (240 ext interrupts)
	* RTOS friendly. exceptions, processor operational modes, access level cofig helps develop RTOS secure apps
	* Rich and mem efficioent instructor set. THumb instr set (16b and 32b)
	* Cortex M cannot execute ARM instr set (32bit) it excecutes THumb which is 32b in 16b format
	* good documentation from ARM
* Competitors
	* Atmel AVR (arduino)
	* TI MSP430

### Lecture 4 - Processor Core Vs Processor

* Consult [ARM Technical Reference Manual (TRM)](http://infocenter.arm.com/help/topic/com.arm.doc.100166_0001_00_en/arm_cortexm4_processor_trm_100166_0001_00_en.pdf)
* Processor != Processor Core (CPU)
* Procesor is Core + Periferrals
* The Core:
	* consists of an ALU for data computation  and result return
	* has logic to decode and execute instr
	* has many registers to store and manipulate data
	* has pipeline engine to boost instruction execution
	* has HW multiplicator and division engine 
* A CPU can hve 1 or more cores

### Lecture 5 - Processor Vs Microcontroller (MCU)

* Cortex-M Processor is designed by ARM as an RTL IP
* Microchip manufacturers build their MCUs using it
* They can modify it removing parts (e.g Mem Protection unit)
* STM32F446RE is a MCU from STmicro basedon ARM Cortex M4 core + FPU + DSP + Memory
* in its [datasheet](https://www.st.com/resource/en/datasheet/stm32f446re.pdf) Descripion we see the block diagram where the procesor (CPU) is just a small part of the SoC
* Manuf  offer many MCU flavors to cover market needs
* Cortex-M4F means floating point

### Lecture 6 - Download Source code

* [Download link](https://drive.google.com/file/d/1JWoj3s6jaVCGv6WBQFdYTo10VSI8DHLV/view?usp=sharing) for course source code

## Section 2 - Development board used in our courses

### Lecture 8 - About MCU Development board

* we have both dev boards suggested: NUCLEO-F446RE and STM32F4DISCOVERY
* choose an active dev board  with support. 
* check if board has on board debugger, most have otherwise thereis a need fro JTAG programer which is generaly Faster 
* ST Boards usually have STLlink debugger
* Make sure the peripherals we plan to use are avaiable on the board
* check mem size of microcontroller and ram 
* use >100KB SRAM
* plan on future. stick witha ref design and master the MCU
* if using discovery board use the newest one DIC1
* MCU does not use the external clock unless we tell it to