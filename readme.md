# Udemy Course: Embedded Systems Programming on ARM Cortex-M3/M4 Processor

* [Course Link](https://www.udemy.com/embedded-system-programming-on-arm-cortex-m3m4/)
* [Course Repo](https://github.com/niekiran/CortexMxProgramming)

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

### Lecture 9 - STM32F4 Discovery and Nucleo: Board Details

* if using discovery board use the newest one DIC1
* MCU does not use the external clock unless we tell it to
* Nucleo board is more streamlined (less feats)
	* no external crystal
	* arduino hat compatible
	* nucleo boards have stm32 nucleo support
	* when we connect it to pc it is enumerated as a virtual com port
* to enable virtual com stlink connectivity in discovery board we need to configure it (ST-LINK/V2-A VCP config) and use a UART to USB adapter

### Lecture 10 - ST-Link Driver Installation

* to use ST Link from our dev machine we need to install ST Link Drivers
	* Windows: use Executable from st.com
	* Mac: no driver installation needed
	* Ubuntu:
```
sudo apt-get install libusb-1.0
sudo apt-get install libc6:i386 lib32ncurses5
```
* in win we download and install prior to connecting the device
* connect device and go: Control Panel => Device Manager => COM Ports to verify the COM port it is listening to (it appears as STLink Dongle)
* IDE uses this port and we can use this port to communicate the device through terminal

### Lecture 11 - ST Link Firmware Upgrade

* in the board page in ST we download firmware upgrade
* we use the version for the platform we use  (follow the readme file)
* ST-LINK/V2-1 is same as ST-LINK/V2-A
* we upgrade our firmware (open in update mode -> upgrade)

## Section 2 - OpenSTM32 System Workbench Installation

### Lecture 13 - Downloading and Installing OpenSTM32 System-Workbench

* Go to [OpenSTM32](www.openstm32.org) and download it
* we register
* go to to the download area
* read the info in the install page

### Lecture 14 - Installing OpenSTM32 System-Workbench

* after downloading the installer we run it
* we follow installation procedure

### Lecture 16 - STM32CubeMX Installationstm32

* Download it from [here](https://www.st.com/en/development-tools/stm32cubemx.html)
* stm32cubemx is a tool to generate code for stm32 checking our config
* we run the installer (java is required)
* we open cubemx, setup connection
* Help => Help opens the hefty manual (378 pages)
* Help => Check for updates checks for latest updates (firmware for the microcontroller type etc)

## Section 4 - KEIL-MDK-5 Setup For ARM Cortex M based MCUs

### Lecture 18 - KEIL-MDK-5 Installation

* we will see how to setup a keil based microcontroller dev env 
* keil is only on windows
* go to [MDK5  homepage](http://www2.keil.com/mdk5/)
* in its core is the IDE + Debugger and a native C/C++ compiler for ARM. we can use gcc but by default it uses the ARM proprietary compiler
* OpenSTM32 is GCC+Eclipse.
* Keil gives things out of the box like the CMSIS-Core to talk to the core processor
* KEIL supports man devices from different manufacutres offering HAL, startup code and CMSIS drivers. 
* Middleware is avaialble for extra cost
* We go for free version which has a limitof 32KB of compiled code
* we install mdk5 and launch it (keil uVision5 IDE)
	* NOTE install packs in '<keil install folder>/ARM/PACK'

### Lecture 20 - KEIL-MDK-5 Pack Installation

* Keil is from ARM for ARM MCUs
* after IDE installation we need to install BSPs for the board we use and the pack of the MCU
* a dev board package contains all. example code, datasheets bsp, hal for mcu, drivers
* in kEIL uVision IDE we click 'Pack Installer'
* In opens a window with a list of manufacturers and their products grouped in families
* Devices is for MCUs, Boards is for DevBoards
* we locate our board. on right we have the avialable SW components
	* DFP is device firmware package. it contans all needed (code, firmware drivers). we need to install this
* the dfp is for MCUfamily. so if we use compatible MCU dev boards it will do the trick
* Nucleo Boards have a BSP available in Keil.
* DFP is per MCU , BSP is per Board. unless we do dev board clones BSP is of little use for custom boards
* We install DFP and BSP for Nucleo boars

### Lecture 21 - Locating Pack Installation files

* in '/Keil-V5/ARM/packs/Keil' we have our installed packs
* in '/STM32F4xx_DFP' we have
	* '/Drivers' has the MCU HAL driver. in '/Src' we have drivers for all the peripherals on the MCU. '/Inc' has te header files
* Same folders are in the Eclipse = CubeMX toolchain

### Lecture 22 - Creating a KEIL Project

* in keil uVision we go: Project => New uVision Project
* we set the directory and the name => Save
* keil asks us to spec the MCU used. we select STM32F446RE
* we are shown the Manage Runtime-Env Environment window
	* we select our board (NUCLEO-F446RE)
	* in 'Device' we click Startup to get a startup code. that  gives a warning as it requires an additional package CMSIS CORE to be installed
	* We install CMSIS CORE either clicking resolve or CMSIS => Core
	* we click OK
* a project is created. it contains:
	* Source Group 1 folder
	* CMSIS api as header
	* A device statup code as C and as assembly
* C file imports several header files. but it DOEST NOT contain a main()
* C file initializes the board
* we compile the project and get an error 'undefined main'
* the code execution starts with the reset handler of the MCU. this is whats called when the MCU powers up
* `Reset_Handler` is present in the startup assembly file . we search for it in the s file and see it is called.
* first thing it tries to do is to try to call `SystemInit`
* `SystemInit` is defined in the C file
* after SystemInit , main is called by the Reset_Handler
* so we need to define a main and include it
* in Source Group 1 we add a new file (Add new item..., C file) 'main.c' adding boilerplate code
```
int main(void) {
	return 0;
}
```
* we build and get 0 errors

## Section 5 - LED/Button Exercises using BSPs

### Lecture 23 - Exercise: LED Toggling App using Board BSP APIs

* we will flesh out the empty main
* every dev board has some user leds
* first we need to have a look at the schematic of the board to see the GPIO that is connected to it
* we look at NUCLEO board [user manual](https://www.st.com/content/ccc/resource/technical/document/user_manual/98/2e/fa/4b/e0/82/43/b7/DM00105823.pdf/files/DM00105823.pdf/jcr:content/translations/en.DM00105823.pdf)
* User LED LD2 is cconnected to pin A5 for nucleo (GPIO port A #5)
* We will use the BSP package to control the LED => click on Manage Run-Time Environment button => Board Support : select the BOARD we use => LED(API) enable to add LED support to or app
* if it asks for STM32 Cube Framework select Classic
* we get a Board Support enbtry in the project tree. under it we have a C file 'LED_Nucleo-F446RE.c' for controlling the LEDs in the NUCLEO board. it contains ready methods to control the LED on the board. the methods are for:
	* initializing the LED
	* unitialize the LED
	* turn on/off
* we add `LED_Initialize();` in the main. we get a warning as we have not an h file to declare and include the prototype befre using it (implicit declaration). it compiles but ...
* in 'LED_Nucleo-F446RE.c' where we got the method from it includes 'Board_LED.h' that contains the prototypes of the LED methods. we include it in our main.c `#include "Board_LED.h"` to resolve the warning
* we will turn the LED on with 'LED_On' passing the LED num `LED_On(0);` we give 0 as NUCLEO has only 1 user led LD2 but indexes are 0 based (assumption)
* before downloading the code to the board we have to do some settings regarding the target: In project tree RCLICK on Target1 => Options for target 'Target1' => Debug tab => Use => select used debugger (ST-LINK Debugger) => Click Settings => Enable Download options (Verify Code Download, Download to Flash) => OK => OK
* We are ready to download => Click on Load button 
* Wait load to finish and Reset the board. IT WORKS
* To toggle it we nedd to  set it on and off. we will implement a function for soft delay
```
void delay(void) {
	uint32_t i = 0;
	for(i=0;i<500000;i++);
}
```
* we include the stdint.h to use the uint32_t type `#include <stdint.h>`
* we use  a while loop and elay in main to do the toggle
```
int main(void) {
	LED_Initialize();
	while(1){
		LED_On(0);
		delay();
		LED_Off(0);
		delay();
	}
	return 0;
}
```
* we get a warning because while is perpetual and return is unreachable

### Lecture 24 - Exercise: LED Toggling App using Board BSP APIs-Nucleo

* we have done it

### Lecture 25 - Exercise : Adding button support using board BSP APIs(Nucleo)

* our goal is when we press the User button the led should toggle. when we leave the button the toggle should stop
* we check schematic: when USer button is pressed the GPIO Port C13 goes to GND (0) when it is unpressed it goes to VDD (1)
* we give it a try: Manage Runtime Env => Board Support => Button API enable => see the offered methods in 'Buttons_Nucleo-F446RE.c' => add `#include "Board_Buttons.h"` in main
* We use `Buttons_Initialize();` to initialize and `Buttons_GetState()` to get the state of the buttons (1 if it is pressed 0 otherwise).
* our main becomes
```
int main(void) {
	LED_Initialize();
	Buttons_Initialize();
	while(1){
		if (Buttons_GetState() == 1){
					LED_On(0);
					delay();
					LED_Off(0);
					delay();
		}
	}
	return 0;
}
```

## Section 6 - LED/Button Exercises with OpenSTM32 SystemWorkbench

### Lecture 26 - Creating First project using OpenSTM32 System workbench : LED Toggling App

* we launch openstm32 eclipse app and set our workspace folder
* we will create anew project using the help of cubemx software
* we open CubeMX software => New Project => (MCU selector for custom boards, Board Selector for ST dev boards or clones) => We choose Board because we use ST boards. => We selct our board from the list (NUCLEO-F446RE) => Double click on the picture to use the board for our project => We get a prompt if we want all peripherals initialized with their default mode (we click yes, but it is crucial to think about it depending on the project, no for custom boards) => We see the Pinout configuration screen (preconfigured for dev board of choice)
* We need to save the CubeMX project: Project Manager tab => Project Location (Use OpenSTM32 Workspace) => Give project a name: '001Led_Toggle' => Set toolchain: SW4STM32 (Deselect 'Generate under Root'!!!!!!!) => Use default Firmware Loation point to the latest firmware available when cubeMX was installed. if we use a more recent one we need to rreference it manualy. IT should match the 'Firmware Package name and Version'
* Then go to Code Generator Tab: make sure 'Copy only the necessary lib files' is selected
* We go to File => Save Project
* we check the pin config screen. some of the pins are initialized to default status (board based) showing Green. LED pin PA5 is set a output
* We click on GENERATE CODE, we get errors probably because our workspace in in a user folder..
* we import the project to OpenSTM32: File => Import => General => Existing Projects into Workspace => Browse for generated project => it finds the project (clicked) => Finish
* our project is added to OpensSTM32

### Lecture 27 - Writing LED Toggling Application ( For Nucleo)

* In OpenSM32 generated project tree we go to: Application => User => main.c
* it has all initializations ready and a while(1) loop
* in there we write our code
* CubeMX adds all drivers in the project: Drivers => STM32F4xx_HAL_Driver we have MCU specific peripheral drivers exposing an API
* we are interested in 'stm32f4xx_hal_gpio.c' for LED blinking.
* We will use 'HAL_GPIO_TogglePin' pasing in port and pin (PA5) `HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);`
* to autocomplete LCTRL+SPACE
* we need to add some delay. from 'stm32f4xx_hal.c' api we will use `void HAL_Delay(uint32_t Delay)` passing delay in msec `HAL_Delay(500);`
* we build our project (hammer button) and see progres in console

### Lecture 28 - Downloading and testing LED Toggling Application ( For Nucleo)

* to download select project in project tree => RCLICK => Target => Program Chip
* in the modal select (Reset after program) => OK... Chip is programmed and IT WORKS

### Lecture 29 - Writing LED Toggling Application ( For Discovery)

* same process. select the board in CubeMX and follow the same pattern
* Leds for our DISCO board are at PG13 and PG14
* code is 
```
HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_13);
HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_14);
HAL_Delay(500);
```

### Lecture 30 - Adding button support

* In CubeMX we create a New Project for Nucleo board named '003Button' 
* the project manager options are the same as before
* button is in PC13
* another way to configur e is to go to System View => GPIO => check configuration
* PC13 is External interrupt Mode (Input)
* CODE GENERATE
* we import project in OpenSTM32
* we check API on 'stm32f4xx_hal_gpio.c'. we will use 'HAL_GPIO_ReadPin'.
* CubeMX HAL reflects actual hardware. PC13 (pushbutton) is pulld down. so when  pressed Input is 0, 1 otherwose
* the code we add in  while(1) loop in main is
```
	 if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13) == GPIO_PIN_RESET) {
		 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
		 HAL_Delay(500);
		 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
	 }
```
* macros GPIO_PIN_RESET is 0 and GPIO_PIN_SET is 1

### Lecture 31 - OpenSTM32 System Workbench Debugging

* to debug a projectin OpenSTM32 sys workbench: RCLICK on project in tree => Debug As
* We choose AC6 application
* or we can choose the debug button
* Workbench asks us to change the perspectiv einto debug mode
* the re are 2 IDE perspectives (Editing, Debugging)
* we add a breakpoint in HAL_Init() and check debugging
* to return to Edit mode click C/C++ button on Top-Right corner of IDE

## Section 7 - ARM Cortex Mx Processor: Architecture Details

### Lecture 32 - Features of Cortex Mx Processor

* Operational Mode of the Processor
* Different access levels of the processor
* Register set of the processor (general purpose set, special purspose set)
* Banked stack design of Processor
* Exceptions & Exceptions handling
* Interrupt Handling
* Bus interfaces and bus matrix
* memory architecture, bit banging, memory
* endianess
* aligned adn unaligned data transfer
* bootloader and IAP (in applicaiton programming)

### Lecture 33 - Operational Modes of the Cortex Mx Processor

* this lecture applies to M0/M3/M4 cortex microprocessors
* processor gives 2 operational modes
	* thread  mode
	* handler mode
* all our app code will execute under "thread mode" of the processor. this mode is also called "user mode"
* all the exception handlers and interrupt handlers will  run under the processor "handler mode"
* procesor always starts with "thread mode"
* whenever the core meets with the system excetion or external interrupts it will change mode to handler mode to service the associated ISR

### Lecture 34 -  Operational Modes of the Cortex Mx Processor: Demonstration

* we will use a Keil project to demonstrate the operational modes of the processor
* we create a new project called 'operational_modes' in Keil for our fev board. add a main.c implementing the main func
* we add a while(1) loop
```
int main(void){
	while(1);
	return 0;
}
```
* our method just burns cycles and does othing
* we set debugger and load the project to chip. we add a brakepoint at while(1) and click the debug button to tenter in debug mode
* in register list we see the Core loaded to the board but not running
* The dissasembly shows that we are in an infinite loop. the next instruction is branch (B) to the same address
* in the register list in group Internal we see that the Mode is Thread. we cnfrm tha that the processor always starts in thread mode
* if i Run (F5) i see the processor is in thread mode
* we will trigger an exception or interrupt to see it switch to handler mode
* we will simulate watchdog interrupt in a helper function using the CMSIS APIs
* we include the CMSIS STM32F446xx Device Peripheral Access Layer Header File `#include "stm32f446xx.h"`
* first we need to enable the IRQ (ennable the interupt) apssing the interrupt id `NVIC_EnableIRQ(WWDG_IRQn);`
* now we have to trigger it `NVIC_SetPendingIRQ(WWDG_IRQn);`
* our helper menthod
```
void generate_interrupt(void) {
	// lets simulate the watchdog interrupt
	NVIC_EnableIRQ(WWDG_IRQn);
	NVIC_SetPendingIRQ(WWDG_IRQn);
}
```
* we call it in main before the while loop. the result is to fire the Watchdog Interrupt, take excution to the watchdog intrrupt handler and we expecto to see the Core going to handler mode
* we have to implement the watchdog intrrupt handler to be able to see this
* in startup assembly file we have the list of interrupt handlers. the one that interests us is 'WWDG_IRQHandler'
* we implement it adding a for loop to be able to catch it
```
void WWDG_IRQHandler(void) {
	for (int i=0;i<50;i++);
}
```
* we add a break point in main at `generate_interrupt();`. we compile flash and enter debug mode
* we are in thread mode ready to branch. we add abreak point at EnableIRQ in helper and run. we do step by step debuging and enable th iterrupt
* when we add the breakpoint in IRQhandler we run and see core in handler mode
* when it finishes service the interrupt it returns in thread mode

### Lecture 35 - Access Levels of the Cortex Mx Processor

* Processor offers 2 access levels:
	* Priviledged Access Level (PAL)
	* Non-Priviledged Access Level (NPAL)
* if our code is running with PAL, our code has full access to all the processor specific resources and restricted registers
* if our code is running with NPAL, then our code may not have access to some of the restricted registers of the Processor
* By default our code will run in PAL
* When the processor is in "Thread Mode" its possible to move processor into PAL. Once we move out of the PAL to NPAL being in thread mode, then its not possible to go back to PAL unless we change the processor operational mode to "Handler Mode"
* "Handler Mode" code execution is ALWAYS with PAL
* we can use the CONTROL register of the processor if we want to switch between Access Levels
* in our previous excersize in debug mode int eh Registers list in the Internal under mode we had the Priviledge reg that was  always Priviledged
* to change that we need to play with the control register.we can learn about them in the [Generic User Guide](http://infocenter.arm.com/help/topic/com.arm.doc.dui0553b/DUI0553.pdf) at 2-9 (p22)
* CONTROL register is part of Core registers (2.1.3)
* we see that bit0 'nPRIV' of CONTROL register defines the Thread Mode priviledge level. 0 for PAL and 1 for NPAL

### Lecture 36 - Access Levels of the Cortex Mx Processor: Demonstration Part-1

* we create a new project in KEIL 'access_levels'
* we cp the previous lecture's main.c
* we download the code and go to debug mode
* in registers in System group CONTROL reg has value 4 (bit0 is 0 PAL)
* we will implement a func to move the proc to unpriviledged mode and call it instead of generate_interrupt() `RTOS_Init();` to emulate a typical RTOS operation
* in it we call a app task method 'call_application_Task' but before calling it will change from PAL to NPAL. to do this we use a method ofCMSIS api from 'cmsis_armcc.h' `__set_CONTROL` to set bit0 to 1 (NPAL)
* first we get the value and mask it
* the code for setting the priviledge flag in COntrol reg is 
```
	uint32_t value = __get_CONTROL();
	value |= 0x01;
	__set_CONTROL(value);
```
* we compile load and debug we see the CONTROL reg changing from 4 to 5 and the Priviledge from PAL to NPAL
* what we do is common in RTOS or embeddedOS to treat application tasks as Unpriviledged

### Lecture 37 - Access Levels of the Cortex Mx Processor: Demonstration Part-2

* we will now attempt to clear the priviledge bit from CONTROL reg in the unpriviedged application task function (we should not be allowed to do it)
```
	uint32_t value = __get_CONTROL();
	value &= 0x00;
	__set_CONTROL(value);
```
* we debug it adding breakpoints. the set reg is ignored and proc remains in NPAL mode
* we will test now going to Handler mode => Thread mode to see if this returns priviledge mode PAL to NPAL
* in 'call_application_Task' we call `generate_interrupt();` and in 'WWDG_IRQHandler' we will try to move CONTROl reg back to PAL
```
void WWDG_IRQHandler(void) {
	for (int i=0;i<50;i++);
	
	/* lets try to move the processor to PAL */
	uint32_t value = __get_CONTROL();
	value &= ~0x01;
	__set_CONTROL(value);
}
```
* we debug but while in NPAL our generate_interrupt fails to reach `NVIC_SetPendingIRQ(WWDG_IRQn);`
* we suspect that while in NPAL we cannot enable interrupts
* we stop and see we are traped in a HardFault Handler. this confirms that while in NPAL we cannot play with the System Interrupt registers

### Lecture 38 - ARM Cortex Mx Core Registers Discussion : Part 1

* in the generic user guide at 2.1.3 we see the Core registers of the the Cortex M4 processor. these registers we see at debug mode of our program at the registers list, listed as Core
* They are part of the Cortex-M4 Core
* As Cortex-M4 is 32bit. the Core registers are 32bit as well.
* Registers from R0 to R12 (13 in total) are for general purpose
* The register R13 is called Stack pointer (SP). it is used to track the Stack Memory. 
* It has 2 versions called 'Banked' : PSP (Process Stack Pointer) and MSP (Main Stack Pointer)
* Always one of the versions is active and the other inactive. this depends on the mode of the Processor
* Register R14 is the Link Register (LR). Link Register  stores the return information of subroutines, function calls and exceptions. On  Reset the proc sets the LR to 0xFFFFFFFF
* to understand it we run the blinky program in Keil. we add a breakpoint at Led_On(0) and run it in Debug mode.
* i monitor LR status and the disassembly code (assembly code of our program)
* in dissassembly when we call a function we see `..... BL.W Led_On(0x080003E4)` which means branch to the address of the function.
* when it does this it stores in LR the address to return to after returning from the function it will move to. it is a way to point back to the caller.
* the actual value passed in is the address of the instruction to branch to the Function + 1
* when the function returns it does 'BX lr' so it branches to LR

### Lecture 39 - ARM Cortex Mx Core Registers Discussion : Part 2

* Register R15 is the Program Counter (PC). It contains the current program address. On reset, the proc loads the PC with the value of the reset vector which is address 0x00000004. Bit0 of the value is loaded into the EPSR T-bit at reset and must be 1.
* we test it again at blinky program. at debug mode. we see that PC holds the address of the instruction the program that is executed at the moment

### Lecture 40 - ARM Cortex Mx Core Registers Discussion : Part 3

* after the  16 R registers we have 5 special purpose registers
* PSR (Program Status Register) combines:
	* APSR (Application Program Status Register) bit 27-32. Contains the current state of previous instruction executions condition flags (ALU condition flags)
	* IPSR (Interrupt Program Status Register) bit 0-8. Contains the exception type number of the current ISR
	* EPSR (Execution Program Status Register) bit 10-15, 24-26
* These registers are mutually exclusive inthe 32b PSR. The Bit Assigmnement is:
	* bit0-8: ISR_NUMBER (IPSR) gives the number of the currently active ISR
	* bit9-15: ICI/IT (EPSR) Interruptible-Continuable Instruction/If-Then Instruction execution state bits
	* bit24: T (EPSR) Thumb State Bit
	* bit25-26: ICI/IT (EPSR) Interruptible-Continuable Instruction/If-Then Instruction execution state bits
	* bit27: Q (APSR) DSP overflow and Saturation Flag
	* bit28: V (APSR) Overflow Flag
	* bit29: C (APSR) Carry on Borrow Flag
	* bit30: Z (APSR) Zero Flag
	* bit31: N (APSR) Negative Flag
* To understand ISR_NUMBER we run operational_mode app with a brakpoint in the watchodog IRQ handler. the ISR_NUMBER is 16 when we are in the handler 16=IRQ0. according to the startup_stm32f446xx.s Vector table is the 'WWDG_IRQHandler' as i would expect
* ICI is used for an interrupted load multiple or store multiple instruction.
**T bit of EPSR**
* Various ARM processors support interworking, that means the ability to switch between ARM and Thumb instruction sets
* If "T" bit is set (1), the processor thinks that the next instruction which it is about to execute if from the "Thumb instruction set"
* If "T" bit is reset (0), then processor thinks that the next instruction which is about to execute is from "ARM instruction set"
* The Cortex-M4 processor only supports execution of instructions in Thumb state. Attempting to execute instructions when the T bit is 0 results in a fault or lockup with "User fault" exception
* The bit0 of the PC (ProG Counter) is linked to the T bit. So any address we place in PCmust have bit0 = 1
* This is taken care by the compiler so programmers have not to worry about it.
* This is why we see all vector addresses  incremented by 1 in teh vector table
* The following can clear the T bit to 0:
	* instructions BLX, BX and POP{PC}
	* restoration from the stacked xPSR value on an exception return
	* bit[0] of the vector value on an exception entry or reset.
* We confirm the T bit in Debug mode => Reset => the see the dsaasembly. Reset Handler is in address 0x08000248 but if we look at 0x00000004 (adress where we store the reset handler) the stored address is 0x08000249 so incremented by 1 for the T-bit

### Lecture 41 - Importance of 'T' bit of the EPSR

* we will demonstrate it using the 'operational_modes' project
* we add a breakpoint in `generate_interrupt();` and fire debugger
* our program is ready to branch in the function address. this address will be placed in PC
* we confirm that address is placed in PC as is. (not added 1 in bit0). the zero bit is the EPSR T-bit.
* to show this we change the way we call generate_interrupt(). from straight call to using function pointers
```
	//generate_interrupt();
	void (*jump_addr) (void) = &generate_interrupt;
	jump_addr();
```
* we fire the debugger
* the address of generate_interrupt is 0x08000374 but it is stored in jump_addr as 0x08000375 so incremented by 1
* steping in the function using the pointer we see that PC becomes 0x08000374 while Tbit is 1.
* we change the code hardcoding into the function pointer the address of the function `void (*jump_addr) (void) = (void *) 0x08000374;` now jump_addr is not incremented by 1 like before (as we hardcode it and compiler cannot do the trick )
* when we call jump_addr() we place the (hardcoded) address into PC. it gets passed but as there is not 1 bit0. Tbit becomes 0 (when we single step in the function)
* So once we step into a function bit0 of function address is copied in Tbit (and it should be 1)
* Now that Tbit is 0 the proc thinks the next instruction is from ARM instruction set. instead of `0x08000374 4C01      LDR           r4,[pc,#4]  ; @0x0800037C` we go to `0x08000252 E7FE      B             HardFault_Handler (0x08000252)` which is the hardFault_Handler and Tbit becomes 1
* if we repeat the example harcoding the address to 0x08000374+1 then problem is solved.
* The moral of the story is (DO NOT HARDCODE FUNCTION ADDRESSES) leave compiler do its job.

### Lecture 42 - Importance of PRIMASK & FAULTMASK registers: Part-1

* we move to the Exception Mask Registers PRIMASK and FAULTMASK
* PRIMASK (Priority Mask) is a Priviledged Register. it prevenets activation of all exceptions with configurable priority. The bit assignment is 
	* bit0: PRIMASK (0=no effect, 1=prevents the activation of all exceptions with configurable priority)
	* bit1-31: reserved
* we again use operational_mode project to showcase primask. if i change PRIMASK to 1 no exceptions are allowed so i disable interrupts globally.
* we use CMSIS api `__set_PRIMASK(1);` and confirm that IRQ handler is not called

### Lecture 43 - Importance of PRIMASK & FAULTMASK registers: Part-2

* even if i set PRIMASK some exceptions like hardfault exception are enabled
* FAULTMASK register prevents activation of all exceptions except from Non-Maskable Interupt(NMI) when bit0 is 1. bits1-31 are reserved 
* with primask set if we use the example of Tbit harcoding the address and setting tbit=0 we still get in the hardfault exception handler.
* if i set `__set_FAULTMASK(1);` then i dont get in the hardfault handler
* we will simulate the NMI exception to see that even masking the FAULTMASK with the method `generate_NMI_Interrupt();` using CMSIS api
```
void generate_NMI_Interrupt(void) {
	NVIC_EnableIRQ(NonMaskableInt_IRQn);
	NVIC_SetPendingIRQ(NonMaskableInt_IRQn);
}
```
* this does not work as the code we wrote is for normal interrupts not exceptions
* NVIC_SetPendingIRQ(); works for IrQn >0 (external Interrupts) but NMI has IrQn -14 (exception) so its not triggered
* to do it i have to directly manipulate SCB
```
	SCB_Type *pSCB;
	pSCB = SCB;
	pSCB->ICSR |= SCB_ICSR_NMIPENDSET_Msk;
```
* i debug and  see that i indeed am able to enter NMI handler eve if  FAULTMASK is set. NMI is registered as ISR =2 in registers
* CONTROL register controls the stack used and the priviledge level for SW execution when proc is in Thread Mode
* if implemented indicates whether the FPU state is active. Bit Asignment
	* bit3-31: reserved
	* bit2: FPCA (Floating Point Context Active) flag. M$ uss it to determine whether to preserve floating point state when processing an exception.
	* bit1: SPSEL. Defines the currently active stack pointer. In handler mode this bit reads as zero and ignores writes. The M4 updates this bit automatically on exception return (0 = MSP, 1 = PSP is the current stack pointer)
	* bit0: nPRIV defines the thread mode privilendge level (0=PAL,1=NPAL)

### Lecture 44 - ARM Cortex Mx Processor Reset Sequence

**Reset Sequence of the Cortex M4 Processor:**
* When we reset the proc, PC is loaded with val 0x00000000
* The proc reads the value @ memory location 0x00000000 in the MSP
	* MSP = value@0x00000000
	* MSP is a Main Stack pointer register
	* That means, proc first initializes the Stack Pointer
* After that proc reads the value @memory loc 0x00000004 into PC. that val is actually address of the reset handler
* PC jumps to the reset handler
* A reset handler is a C or assembly function written by the programmer to carry
 out initializations
* From the reset handler we call main() function

### Lecture 45 - ARM Cortex Mx Processor Reset Sequence : Demonstration

* we add a new project in keil 'reset_sequence' and add main.c
* we add an empty main function and load
* i enter debug mode and it automatically takes me to main
* i reset the proc and see in debug that i am in the reset handler
* in register i see that MSP has 0x20000660 which is the content of address 0x00000000
* we confirm it in the mem viewer
* we check in mem 0x00000004 and we confirm that it has the mem address of  reset handler 0x08000248 + Tbit (1)
* we confirm that this address is loaded to the PC as we are in the handler
* the startup code of the Keil lib in the reset handler loads and branches in the SystemInit function and then main
* for app developer SystemInit is the palce to add initialization code
* main is called a `__main`
* Reset_Handler is implemented as WEAK assembly function so we can add a different one with the same name
* we will implementa Reset_handler function in main.c
```
void Reset_Handler(void) {

}
```
* we load , reset and see that we are in our implemented Reset_Handler
* we emulate the assemlby Reset_handler in C
```
#include "system_stm32f4xx.h"

void Reset_Handler(void) {
SystemInit();
main();
}
```
* it works
* we can simulate header file declaration of functions with `extern` in the c file `extern int __main(void);`

## Section 8 - Memory System Architecture

### Lecture 46 - Memory System features and Memory Map

* Section objectives:
	* mem system features
	* proc mem map
	* bus interfaces
	* bus protocols
	* aligned and unaligned data transfer
	* bit banging and its advantages
	* code example
* Mem System Features:
	* All CortexM processors have 32bit mem addressing (4GB of addressable mem space)
	* The memory is one unified space which is shared by code space, data space and peripheral space
	* Proc uses Harvard bus architecture. It means concurrent instruction and data accesses using multiple bus interfaces. It can simultaneously fetch data and instruction
	* Support for little endian and big endian memory systems (config register) default is little endianess
	* Support for unaligned data transfers
	* Bit addressable memory spaces(bit-banding)
	* MPU (Mem Protection Unit) support (optional)

**ARM Cortex M3/M4 Mem Space**
* CODE Region (0x00000000 - 0x1FFFFFFF) (512MB) Physically Implemented as FLASH or ROM memory on Chip 
	* Vector Table (inital stack value, addresses of Exception handlers)
	* Application Code
* SRAM Region (0x20000000 - 0x3FFFFFFF) (512MB) Physically as On-chip RAM or SRAM, used for Stack Memory, We can execute code from here
	* Bit-Band Region (0x20000000 - 0x20100000) (1MB) every bit is addressable
	* 
	* Bit-Band Alias (0x22000000 - 0x23FFFFFF)
	*
* Peripherals Region (0x40000000 - 0x5FFFFFFF) (512MB) Physically Various On-Chip Peripherals (see the datasheet), We cannot execute code from here
	* Bit-Band Region (0x40000000 - 0x40100000) (1MB) every bit is addressable
	* 
	* Bit-Band Alias (0x42000000 - 0x43000000) 
	*
* External RAM Region(0x60000000 - 0x9FFFFFFF) (1GB) Physically on-chip or off-chip RAM (we can execute code from here)
* External Device Region (0xA0000000 - 0xDFFFFFFF) (1GB) for exteral devices and/or shared memory (non-executable)
* Private Bus "Internal" (0xE0000000 - 0xE003FFFF)
	* NVIC (0xE000E000 - 0xE000F000)
* Private Bus "External" (0xE0040000 - 0xE00FFFFF)
	* External private peripheral bus (0xE0042000 - 0xE00FF000)
	* ROM Table (0xE00FF001 - 0xE00FFFFF)
* Vendor Specific (0xE0100000 - 0xFFFFFFFF)

### Lecture 47 - Bus Protocols and Bus interfaces

**Bus Protocols** (Interconnecting Proc with Peripherals)
* AHB Lite (Main System Bus):
	* AHB Lite protocol is used for the main bus interfaces
	* AHB lite stands  for AMBA High Performance Bus which is derived from AMBA (Advanced Microcontroller Bus Architecture) spec
* APB (Peripheral Bus):
	* APB is AMBA compliant bus optimized for minimum power and reduced interface complexity
	* APB is much simpler (power optimized) and much slower bus compared to AHB
* There are multiple AHB Lite and APB interfaces:
	* I-BUS A dedicated bus used to fetch the instructions and Vector Table from CODE region (AHB Lite based). all fetches are word aligned (32bit) so if instructions are 16bit we fetch them 2 by 2
	* D-BUS is a bus dedicated for data fetches from SRAM (AHB Lite based) can access data from non word aligned mem addresses (unaligned)
	* System Bus (32bit AHB Lite) for data and instruction fetch from MEM devs like SRAM or bus masters (USB,DMA)
* For slower peripherals there is no need for fast AHB bus. AHB/APB bridge converters are used and APB for Slow Peripherals (Saves power)

### Lecture 48 - Aligned and Un-aligned data transfer

* aligned data transfer:
	* word alligned data will always be places in addresses that are multiple of 4. e.g 0x00000000 0x00000004 0x00000008
	* <32 bit data like 8bit char or 16bit short will be stored in intermediate addresses
	* this means unused memory spaces are created.
	* storing the data in aligned fashion may waste SRAM space but boosts performance because it is BUS friendly
	* processor needs only 1 cycle for every word fetch
* un-aligned data transfer:
	* data is stored using  `__packed` keyword in a struct definition. 
	* data are compacted in conscutive mem locations (no gaps) with 1byte granularity. mem efficiency
	* when an unalignewd data trasfer is issued by the processor data are actually converted to multiple aligned transfers by the processor bus interface unit
	* since it is broken into several separate aligned transfers and as a result it takes more clock cycles for a single data access and might not be good in situations where highe performance is required.
```
struct __packed mydata
{
	unsigned long data1;
	char data2;
	unsigned long data3;
	char data4[3];
	short int data5
}
```
* how an unaligned data trasfer happens?
	* direct pointer manipulation
	* accessing `__packed` data structs
	* inline assembly code
* instructions are aligned using 2byte granularity in mem space as Cortex M3/4 uses Thumb-2 instruction set. some instractions use 4bytes thoghoug
* Thumb-2 ISA is combination of 16b and 32b instructions
* processor fetches instructions in word boundary, it fetches 1 32bit instr or 2 consequent 16bit instructiosn
* AVOID UNALIGNED DATA STORAGE

### Lecture 49 - Bit-Banding

* Bit banding:
	* a capability to address a single bit of memory address
	* this feature is optional (mcu manufacturer dependent)
* Bit band regions are the regions in memory map of the processor whose each bit can be uniquely addressed by using dedicated address(bit-addressable)
* There are 2 such regions 1MB each. in SRAM and Peripehral region
* The Bit band alias region is 32MB. a mem byte in allas region is a bit in the bit-band region
* To set/reset any bit of the bit band memory region address, then we can do that using corresponsding bit address (byte address) in the bit band alias region
* if we want to set the 2nd bit of 0x20000000 address:
	* With-out bitband: read 0x20000000 to register => mask and set bit 2 => write back to 0x20000000
	* with bit-band: write 1 to 0x22000008 (this address is called the alias address for bit2 of 0x20000000). 
* under the hood when we write to bit-band alias the action is mapped to 2 bus transfers
	* read data from 0x20000000
	* write to 0x20000000 from buffer with bit2 set 
* action is 1 instruction for core so non-interruptible
* Mapping of bit-band region to bit-band alias:
	* 0x20000000 bit0 => 0x22000000 
	* 0x20000000 bit1 => 0x22000004
	* 0x20000000 bit2 => 0x22000008 
	* 0x20000004 bit0 => 0x22000080
* advantage of bit-band regions
	* faster register flag set
	* less race conditions on shared resources (bit band alias address access is atomic)

## Section 9 - LAB SESSIONS

### Lecture 51 - Lab Assignments 3: Bit-Band Operations using C

* in our program we will change the value of BIT0,BIT1 and BIT2 of the data stored in the memory address 0x20000000 using bit-band operations
* we make a new project 'bit-banding' using  the C file from course repo
* the project uses macros for region base addresses
```
#define BIT_BAND_REGION_MEM_ADDR_1           *((volatile unsigned long *)(0x20000000))
#define BIT_BAND_ALIAS_BASE_ADDRESS           (0x22000000)
```
* bit band region is the one to be ultimately modified 
* we store in macro the aliases
* our program
```
int main(void)
{
	uint32_t val;
	BIT_BAND_REGION_MEM_ADDR_1 = 0xA0;
	BIT_BAND_ALIAS_ADDRESS_1 = 0x01;

	BIT_BAND_ALIAS_ADDRESS_2 = 0x01;
	BIT_BAND_ALIAS_ADDRESS_3= 0x01;

	val = BIT_BAND_REGION_MEM_ADDR_1;
	return 0;
}
```
* we check in ebug mode.
* after first line 0x2000000 is A0 after bitbanding in alias mem for bit0 1 and 3 it becomes A7

## Section 10 - Stacks

### Lecture 52 - ARM Cortex Mx Stack Discussion : Part-1

**Banked Stack Pointers**
* Cortex M Proc physically has 3 stack pointers
	* SP (R13) (Current Stack Pointer)
	* MSP (Main Stack Pointer)
	* PSP (Process Stack Pointer)
* After Processor Reset, by default MSP will be selected as current stack pointer. That means, SP copies the contents of MSP
* Thread mode can change the current pointer to PSP by configuring the CONTRO reg SPSEL bit
* Handler mode code execution will ALWAYS use MSP as the SP. So changing the SPSEL in handler mode has no effect.

### Lecture 53 - ARM Cortex Mx Stack Discussion : Part-2

**Banked Stack Pointers (continued)** 
* MSP will be initialized automatically by the processor after reset by reading the content of the address 0x00000000
* if we want to use the PSP then we must make sure that we initialize the PSP to valid stack address in our code

### Lecture 54 - ARM Cortex Mx Stack Discussion : Part-3

* we use 'blinky' as base for our 'stack_pointer' project
* we debug and confirm that at startup SP and MSP have the same value which is the content of address 0x00000000 (reset sequence)
* in the startup assembly file 'startup_stm32f446xx.s' we see that the stack size is set to 0x00000400 (4KB or 1024 32bit words). c equivalent would be `#define STACK_SIZE 0x400`
```
Stack_Size      EQU     0x00000400

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp
```
* `__initial_sp` is the value@0x00000000 which is 0x20000660 (top pf stack) set in the Vectors table
* whenever we push to the stack it decreases from this value and stores the new value
* by the term decrease we mean that the content of 0x20000660 is pushed to 0x2000065C (4byte word aligned) and 0x20000660 gets the new value.

### Lecture 55 - ARM Cortex Mx Stack Discussion : Part 4

* As we said Stack recides in the SRAM mem region. the available area is 112KB (0x2000_0000 - 0x2001_BFFF) and is named SRAM1. for our application top of the stack is at 0x2000_0660 and has a sixe of 0x400 so the bottom of the stack is at 0x2000_0460.
* the whole area is available for the stack (we can set it programmatically) so the MAX stack we cen set is 447KB
* we will move the top of the stack to the far end of the area (0x2001BFFF). we ll program 0x2001BFFF+1 (to be word alligned aka mulitple of 4). we are safe to do that as the value first will decrement by 4 and then store
* we use the `__set_MSP()` from  'cmsis_armcc.h' passing in the address (topOfMainStack) `	__set_MSP(0x2001BFFF+1);`
* we test it in debug and it works. so we can move the top of the stack at runtime (SCARY!!!!)
* PUSH operations and POP operations (assemply) are done in the stack
* we usually  set the top of stack in initialization function

### Lecture 56 - ARM Cortex Mx Stack Discussion : Part 5

* we will set the SPSEL (bit1) of CONTROL reg to set current SP to PSP unstead of MSP (in thread mode)
```
	uint32_t value= __get_CONTROL();
	value |= (1 << 1);
	__set_CONTROL(value);
```
* we debug and it works. PSP value is 0x000000000 in our app
* this creates a serious problem as processor tries to decrement that to do the push and its impossible
* we initialize PSP top fix that. PSP stack is again reserved in the SRAM! region `__set_PSP(0x2001BFFF+1);`

### Lecture 57 - ARM Cortex Mx Stack Discussion : Part 6

* in handler mode MSP will always be used as stack pointer. we will attempt to set SPSEL in the Interrupt handler to test it. we need to add an interrupt handler for this
* we use our code from other project to generate  (WDOG_INTERRUPT)
```
void generate_interrupt(void) {
	// lets simulate the watchdog interrupt
	NVIC_EnableIRQ(WWDG_IRQn);
	NVIC_SetPendingIRQ(WWDG_IRQn);
}
```
* we also add the dummy handler 
```
void WWDG_IRQHandler(void) {
	for (int i=0;i<50;i++);
}
```
* ther we will attmept to mode the SPSEL in the handler. we confirm that nothing changes
* we also confirm that when we enter Handler mode we switch to MSP from PSP and when goes back to Thread mode it goes back to PSP.
* PSP makes sense when designing an Embedded OS. for simple apps we can use the MSP only
* In an RTOS app we can program that User tasks use the PSP stack (in isolation) while the RTOS Kernel uses the Kernel Stack on MSP. (for safety). also before switching to User Task the RTOS changes priviledge mode from PAL to NPAL

### Lecture 58 - Subroutine and Stack

* According to the Procedure Call Standard of Arm Architecture when a function is called, as per below table, registers are used for parameter passing:
	* R0 : input: first Input param => return: function return val
	* R1 : input: second Input param => return: function return val if size is 64bit
	* R2 : input: third input param
	* R3 : input: forth input param
* Caller function calls the callee function (subroutine)
* its the callee 'function' responsibility to push the contents of R4-R11,R13,R14 if the function is going to change these registers. (compiler takes care when coded in C)

### Lecture 59 - Stacking and Un-stacking during Exception

* we use a drawing board example. 
* Task A executes in THread mode using MSP.
* MSP is pointing to the last stacked item
	* used stack space
	* last stacked item
* when an exception occurs, processor pushes automatically many registers into the stack before executing the exception handler
	* used stack space
	* last stacked item
	* xPSR
	* Return Address
	* LR
	* R12
	* R3
	* R2
	* R1
	* R0
* MSP now points to R0
* processor changes mode (Handler Mode) and proc executes the exception handler
* the contents stacked in the stack memory are called 'Stack Frame'
* in total 32bytes are pushed into the stack (by default). this delay adds up to exception or interrupt latency
* To avoid this pushing and popping crap ARM7TDMI introduced FIQ(Fast Interrupt reQuest) mode, where instead of storing to the stack, internal registers are used which improves latency . Cortex does not have FIQ mode
* when proc finishes processing the handler it exits the ISR popping back and 32 bytes and MSP moves back to Last stacked item (unstacking operation)

## Section 11 - LAB SESSION

### Lecture 60 - Lab assignment 4: Stack Operations Using Different Stack Pointers (MSP/PSP)

* We will write a program to PUSH the contents of R0,R1,R2 registers using MSP as a stack pointer, and then POP the contents back using the PSP as a stack pointer
	* Proc always starts in priviledged level and uses MSP as stack pointer
	* We can change the stack pointer to PSP by writing to CONTROL reg (SPSEL)
	* In C if we want to access MSP,PSP,CONTROL register then we have to use either CMSIS API or assembly
* we start a project 'lab_exercise4' adding boilerplate main() to call our assepbly function
```
#include <stdint.h>
#include "stm32f407xx.h"

extern void do_stack_operations(void);
int  main()
{
	/* This function is implemeted in Assembly */
	do_stack_operations();
	return 0;
}
```
* we add a 'stack_ops.s' file for the assembly method
* our assembly code is
```
	AREA    STACK_OP, CODE, READONLY
    EXPORT do_stack_operations
do_stack_operations
    MOV R0,#0x11
	MOV R1,#0X22
	MOV R2,#0X33
    PUSH {R0-R2}     ; PUSH the contents of RO,R1,R2
	MRS R0,CONTROL ; Read the Contents of CONTROL register
	ORR R0,R0,#0X02 ; set the SPSEL bit to 1, to select PSP
	MSR CONTROL,R0  ; Write back to the CONTROL register 
	MRS R0,MSP
	MSR PSP,R0      ; Initialize the PSP
	POP  {R0-R2}    ; POP back 

    BX   lr           ; Return.
    END
```
* we use the extern keyword to tell linker the method is implemented somewhere else
* we see in registers and mem map the contents of the stack which indeed does what we expect
* `AREA    STACK_OP, CODE, READONLY` tells compiler to assemble a new code area for the method
* `EXPORT do_stack_operations` exports it as a C method to be called
* function is ended using 'END'
* `ORR R0,R0,#0X02 ` sets the 2nd bit
* `MSR` writes and `MRS` reads
* to copy the pointer from MSP to PSP i use R0  as temp
* `POP` moves back the stack pointer and returns values to where they were copied from
* to return we use  `BX lr`

## Section 12 - System Exceptions and Interrupts-I

### Lecture 61 - ARM Cortex Mx System Exceptions

* Cortex M processor Exception Model is the same for M0/M1/M3/M4/M7
* Exception is anything that can disrupt the normal operation of the program by changing the operational mode of the processor.
* There are 2 types of exceptions
	* system exceptions (15 in total): generated by the processor itself internally
	* interrupts (240 in total): come from the external world to the proc
* Whenever proc gets an exception oper mode changes to Hanlder

### Lecture 62 - ARM Cortex Mx : Different System Exceptions

* there is room for 15 system exceptions
* exception num 1 is Reset Exception (or system exception)
* only 9 implemented System Exceptions. 6 are reserved for the future
* Exception #16 is interrupt 1 (IRQ 1)
* In Cortex M$ Generic User Manual at 2.3 we can read about the exception model
* We see the different Exception types
	* 			 : Reset : Priority -3 : Asynch
	* IRQn = -14 : NMI : Priority -2 : Asynch
	* IRQn = -13 : Hardfault : Priority -1
	* IRQn = -12 : MemManage : Priority Configurable : Synch
	* IRQn = -11 : BusFault : Priority Configurable : Synch/Asynch
	* IRQn = -10 : UsageFault : Priority Configurable : Synch
	* IRQn = -9 to -6 : Reserved
	* IRQn = -5 : SVCall : Priority Configurable : Synch
	* IRQn = -4 to -3 : Reserved
	* IRQn = -2 : PendSV : Priority Configurable : Asynch
	* IRQn = -1 : SysTick : Priority Configurable : Asynch
* From IRQn=0 and up we have interrupts
	* IRQn = 0 : Interrupt(IRQ) : Priority Configurable : Asynch
* Configurable priority is >=0 (0 highest)
**Exception Types**	
* Reset: invoked on power up or warm reset. processor halts. when asserted operation resumes from reset_handler in vector table (PLA, thread mode)
* NMI: Peripheral or SW triggered.always enabled. cannot be masked or prempted
* HardFault: triggered by an error in exception processing or by an error cannot be managed by other exception mech.
* MemManage: triggered by a memory protection related fault.used to abort instruction access to XN (non execute mem region)
* BusFault: triggered from me related fault for instruction or data transaction
* UsageFault: triggered from instruction execution fault
	* undef instruction (if Tbit is 0)
	* illegal unaligned access
	* invalid state on instruction exec
	* exception return error
	* unaligned addr on word or half word (if proc is cofig to report it)
	* division by zero (if proc is config to report it)
* SVCall: supervisor call is triggered by SVC instuction. in OS apps use SVC inst to access OS kernel funcs and dev drivers. a way fro unpriviledged app tasks to get elevetad rights from OS to use resources in a handler
* PendSV: interrupt driven req for Sy level service. used for context switching in OS env
* SysTick: system timer triggered when it reaches zero. also SW triggered. In OS env proc can use it as System tick
* Interrupt (IRQ): triggered by a peripheral. or sw request. always asynch

### Lecture 63 - ARM Cortex Mx-System Exception’s vector addresses

* The Vector Table for System exceptions as we see in 'startup_stm32f446xx.s' are stored from address 0x00000004 and onwards (word aligned). address 0x00000000 is reserved for top of stack (MSP val) as we have seen

### Lecture 64 - ARM Cortex Mx System Exception priority

* In the context of the ARM Processor always remember that: The lowest the priority value, the highest the priority of exception, and the more urgent it is

### Lecture 65 - ARM Cortex Mx System Exception Priority Contd.

* All Cortex proc share same exception model as it is ARM based
* TI TIVA CortexM4 has same. STM32F4 the same etc

### Lecture 66 - ARM Cortex Mx-System Exception activation: Part 1

* System Exception Configuration: Activating(Pending)/Deactivating/Changing Priority
* we create a new project 'system_exception_activation' with an empty main
* even if we dont enable any system exceptions, Reset,NMI and hardFault will be enabled by default
* we will add a jump to an illegal address (NX mem region) to trigger an exception
* we use a function pointer set to zero address
```
	void (*go_address) (void);
	go_address = 0x00000000;
	go_address();
```
* We get a hardFault. we can suppress it using teh FAULTMASK `__set_FAULTMASK(1);` hardfault is suppressed
* System Exception Handlers as we can see in startup assembly file are set WEAK from cu vendor. so we can override them with our own C functions witht he sam ename
* we override them creating empty methods in our file e.g
```
void HardFault_Handler(void){
	
}
```
* we will now activate these system exceptions
* in Generic User Guide of Cortex M4 in chapter 4.3 (System control block)
* System Control Block is a set of registers of the proc providing sytem implementation info and system control(config,control and report of system exceptions)
* The register to use for activating exceptions is SHCSR

### Lecture 67 - System Exception Activation and Exception Escalation

* we will activate exceptions: MemManage, BusFault, UsageFault, SVC, PendSV
* we will use SHCSR and the respective bits(flags) setting them to activate the exceptions.
* there is a bit for every system exception
* this is a MemMap register. to set the register we use a CMSIS api call. CMSIS core header file. in our case is included in 'stm32f446xx.h' and it contains macros to access the address.
* SCB_Type struct contains the SHCSR reg
* we will create apointer of this type `SCB_Type *pSCB;`
* we will init it to base address of the control block
* in the same 'core_CM4.h' there is a macro for the base address `SCB_BASE`. there are macros for all Core Hardware MemMap areas (with their regs)  'SCB' is type casted pointer of this struct type so we use it.
* we dereference SCB to get SHCSR and set the bits to enable exceptions `SCB->SHCSR = SCB->SHCSR | (1 << 16); //enable MemManage Exception`
* enabling an exception dos not trigger it. it will be triggered when triggering conditions happen
* we divide by zero using a func.debug ans see the exception triggering. to see which exception handler is called we add breakpoints in all of them. noone is called. Cortex M treats division by 0 as 0 (result goes to R0 reg and we confirm it). 
* in 4.3.7 of the manual we see that CCR (Config and Control Register) allow us to trap divizion by zero error and unalligned accesses. DIV_0_TRP (bit4) does the trick `SCB->CCR = SCB->CCR | (1 << 4);`. i see i am trapped in UsageFault Hanlder
* if i disable ther UsageFault exception HardFault is triggered

### Lecture 68 - Usage fault Exception with unaligned data access

* in CCR apart from DIV_0_TRP (bit4) thre is UNALIGN_TRP (bit3) to trap in an exception unaligned word and halfword access. `SCB->CCR = SCB->CCR | (1 << 3);`
* to trigger it we add a pointer to a RAM location and read its contents incrementing it by 2 (content is uninitialized though)
```
	//unaligned data access
	uint32_t *p = (uint32_t*) 0x20000000;
	uint32_t var = *p;
	var++;
```
* in hisdissasmbly code generated no instructions (code optimizastion), we pass var in a func to force the compiler to generate assembly code
* another way to force the compiler to generate assembly code is using 'volatile' keyword `	uint32_t volatile *p = (uint32_t*) 0x20000000;`
* it works as we do alligned access (2000000 is multiple of 4)
* for unaligned we mod it to `uint32_t *p = (uint32_t*) 0x20000001;` we are trapped in the UsageFault handler. SUCCESS
* if ichange uint32_t to uint8_t it wont have any issue whatsoever
* assy command LDR is for 33bit word , LDRH for 16b halfword, LDRB for 8bit char
* if i dont trap unaligned using CCR no exceptions happened. Cortex M has no problem handling unaligned mem access (NOT RECOMMENDED)

### Lecture 69 - NVIC,IRQ Numbers and Enabling/Disabling Interrupts

**NVIC (Nested Vector Interrupt Controller)**
* NVIC is one of the peripheral of the Cortex M processor core
* it is used to configure the 240 external interrupts (external to the processor core)
* using the NVIC registers you can Enable/Disable/Pend Various interrupts and read the status of the active and pending interrupts
* we can configure the priority and priority grouping of various interrupts
* it is called as "nested" because, it supports pre-empting a lower priority interrupt handler when higher priority interrupt arrives
* in chapter 4.2 of the General User manual we can read about tthe NVIC and its registers

**Enable/Disable/Pend various interrupts using NVIC regs**
* Cortex M proc supports 240 interrupts
* these interrupts are managed and configured using the NVIC
* what are thise 240 interrupts?
	* anything external to the proc core
	* we should consult the [MCU device ref manual](https://www.st.com/content/ccc/resource/technical/document/reference_manual/4d/ed/bc/89/b5/70/40/dc/DM00135183.pdf/files/DM00135183.pdf/jcr:content/translations/en.DM00135183.pdf) at 10.2 we see that our MCU STM32F446RE supports 96 external interrupts from on chip peripherals (e.g SPI, GPIO, Timers, DMA etc)
* NVIC module (sitting inside the COrtex M CPU) offers 240 interrupt lines to peripherals residing on the SoC. the wiring and ordering is up to the Vendor

### Lecture 70 - NVIC Interrupt handling Registers (Set/Clear/Pend/Active)

* NVIC offers various registers.
	* NVIC_ISER0 to NVIC_ISER7 (8 32bit regs) (0xE000E100 - 0xE000E11C) is to Interrupt Set-Enable Registers for up to 256 interrupts in blocks of 32 irqs. We CANNOT use the reg to disable an interrupt. only to enable it (0 has no effect)
	* NVIC_ICER0 to NVIC_ICER7 (8 32bit regs) (0xE000E180 - 0xE000E19C) is to Interrupt Clear-Enable Registers for up to 256 interrupts in blocks of 32 irqs. We CANNOT use the reg to enable an interrupt. only to disable it (0 has no effect)
	* NVIC_ISPR0 to NVIC_ISPR7 (8 32bit regs) (0xE000E200 - 0xE000E21C) is to Interrupt Set-Pending Registers for up to 256 interrupts in blocks of 32 irqs. We CANNOT use the reg to remove an interrupt from pending state. only to force it into it (0 has no effect)
	* NVIC_ICPR0 to NVIC_ICPR7 (8 32bit regs) (0xE000E280 - 0xE000E29C) is to Interrupt Clear-Pending Registers for up to 256 interrupts in blocks of 32 irqs. We CANNOT use the reg to force an interrupt to go on pending state. only to remove it (0 has no effect). We can read this reg to see which IRQs are pending (bits are set)
	* NVIC_IABR0 to NVIC_ISBR7 (8 32bit regs) (0xE000E300 - 0xE000E31C) is to Interrupt Active-Bit Registers for up to 256 interrupts in blocks of 32 irqs. Wwhen read it indicates which interrupts are active (proc is executing its ISR).
* Pending an interrupt is to use its priority level for order of execution (or halt it till higher priority isrs are finished)

### Lecture 71 - Exercise-Enabling and Pending of an Interrupt

* we create anew keil project 'nvic_en_dis_pend' and add main.c
* we will manuall Enable and Pend the USART3 interrupt
* we import HAL h file `#include "stm32f446xx.h"`
* we add boilerplate main()
* we see the STM32F446RE ref manual and see the USART3 is IRQ39 `#define USART3_IRQ_NUM 39`
* we go to the h file of the core 'core_cm4.h' to see the NVIC reg structure and how to access them. 
* in same way like the CSR we have the NVIC_Type struct we use pointers and Base address. NVIC_BASE is type casted so we can use NVIC
```
NVIC_Type	*pNVIC = NVIC;
	pNVIC->ISER[1] |= (1 << 7);
```
* after enabling it we will pend it manually `pNVIC->ISPR[1] |= (1 << 7);`
* from startup assembly file we chen in vector table for the USART3 Handler definition and ovverrite it as C addint endless while loop
* we debug to test it. we see that we enter the Handler
* PENDING an interrupt is like activating it (respecting the priorities)
* setting the pending bit is a 4 instr (all in one) loc. the instr that sets pending and fires the ISR is `0x08000388 500A      STR           r2,[r1,r0]`

### Lecture 72 - Exercise : Enabling and Pending of an Interrupt using CMSIS APIs

* using CMSIS header files we dont have to use the stm32f446xx.h file and thus we can make our code portable among different vendors. but stm32f446xx.h file is CMSIS compatible so IT IS Portable
```
	// Enable the USART3 IRQ num 39
	NVIC_EnableIRQ(USART3_IRQn);
	
	// Lets pend the interrupt
	NVIC_SetPendingIRQ(USART3_IRQn);
```

### Lecture 72 - Priority and Interrupt Nesting

* we create a new KEil project 'nvic_prioriry'
* we will use WDOG IRQ and USART3 IRQ to show the priorities
* first we enable  WWDG IRQ and USART3 IRQ
```
	NVIC_EnableIRQ(WWDG_IRQn);
	NVIC_EnableIRQ(USART3_IRQn);
```
* then we pend the IRQs
```
	NVIC_SetPendingIRQ(WWDG_IRQn);
	NVIC_SetPendingIRQ(USART3_IRQn);
```
* we add hanbdlers for them with infinite loops inside e.g
```
void WWDG_IRQHandler(void) {
	while(1);
}
```
* we flash and debug. we see that we are locked in the WOG handler as we pend it first (USAER3 pend is not called)
* we do the nasty thinng of nesting USART SetPend in teh WDOG handler
* we pend it but program is hanging in the WDOG handler (higher priority).
* we remove infinite loop from WDOG handler where we pend the USART3 IRQ to see if we will enter after it exits from WDG ISR. SUCCESS WE ENTER
* we increase the priority of the USART3 IRQ. we use 'NVIC_SetPriority(IRQn,priority)' 
* WWDG default priority is 7 and USART3 46. so anything <7 will do. ichoose to set it for both.
```
	NVIC_SetPriority(WWDG_IRQn,5);
	NVIC_SetPriority(USART3_IRQn,0);
```
* while having infinite loops inside handler. i see that changing priority does not force WWDG ISR to halt and enter USART3 ISR if i am inside the ISR. (Handler mode)
* if i pend them after priority change still if WDOG IRQ is pended first priority has no effect. if i nest USART3 IRQ sepending inside the WWDG ISR it has effect!!!
* SO Priority affects Nested Interrupts and can preempt executing ISR. so PRIORITY > ISR > THREAD MODE
* If i make USART a finite ISR i see that execution goes back to WDOG after it exits!!!!
* IN CORtex M4 we can have 8 levels IRQ nesting

### Lecture 74 -  Interrupt Priority Register Discussion

* Priority Value for a particular interrupt is configured using a register called PR whose size is just 8 bit
* Since we have 240 IRQs we need 240, 8bit registers?
* Thats why Cortex M proc provides 60 regs of size 32bit (NVIC_IPR0 - NVIC_IPR59) (4.2.7 in GUM of Cortex M4)
* Each 32bit reg is a group of 4, 8bit PR regs holding the Priority value. 60x4=240 8bit regs
* as it is 8 bit the min val is 0 and max is 255. manufacturers do not implement a number of 8 bits to limit the depth of nesting
	* For ST STM32  bits 0-3 are NOT Implemented so only 16 programmable priorities
	* for TI TIVA bits 0-4 are NOT Implemented so only 8 programmable priorities
* what if i write the val 5 for IRQ0 (WWDG) it should not be possible as i have to wiite to bit0 and bit2. 
* the correct way to do it is IPV << (8 - NO_OF_NON_IMPL) so for STM32  `5 << 4`
* CHECK THE REF MANUAL OF YOUR DEVICE (or the header file)

### Lecture 75 - Priority Grouping

* The priority level register which we just saw is divided into 2 fileds
	* ore-empts priority field
	* sub-priority field
* There are 8 different priority groups (possible grouping combos)
	* 0 (default) : pre-empt priority field bit[7:1] : sub-priority field bit[0]
	* 1  		  : pre-empt priority field bit[7:2] : sub-priority field bit[1:0]
	* 2  		  : pre-empt priority field bit[7:3] : sub-priority field bit[2:0]
	* 3  		  : pre-empt priority field bit[7:4] : sub-priority field bit[3:0]
	* 4  		  : pre-empt priority field bit[7:5] : sub-priority field bit[4:0]
	* 5  		  : pre-empt priority field bit[7:6] : sub-priority field bit[5:0]
	* 6  		  : pre-empt priority field bit[7:7] : sub-priority field bit[6:0]
	* 7  		  : pre-empt priority field none 	 : sub-priority field bit[7:0]
* Pre-Empt Priority: when the proc is running an ISR, and another interrupt appears, then the pre-empt priority values will be compared and exception with higher pre-empt priority (less in number) will be allowed to run
* Sub Piority: this value is used only when two exceptions with same pre-empt priority level occur at the same time. in this case the exception with higher sub-priority (less in number) will b ehandled first
* Priority Grouping is configured in the Application Interrupt and Reset Control Register (AIRCR) of the SCB in bit8 to 10 (PRIGROUP)
* Priority Grouping is applied in the NVIC_IPR0-59 registers where Vendor specific non implemented bits should be considered. e.g
	* PRIGROUP = 0 (default) on STM32
	* pre-empt priority wigth is 7 bits (only 4 implemented) so 16 programmable interrupt levels
	* sub priority width 1bit (2 levels) (bit is in non implemented region so no sub-levels)
* CMSIS for PrioriryGrouping (To avoid Register mess)
	* `void NVIC_SetPriorityGrouping(uint32_t PriorityGroup)` sets the PRIGROUP in AIRCR
	* `uint32_t NVIC_GetPriorityGrouping()`
	* `uint32_t NVIC_EncodePriority(uint32_t PriorityGroup, uint32_t PreemptPriority, uint32_t SubPriority)` encodes the prioirty for an interrupt with the given priiority group preemptive and subpriority vals. returns an encoded priority to be used in SetPriority
	* `void NVIC_SetPriority(IRQn_Type IRQn,uint32_t priority)`

## Section 13 - LAB SESSION

### Lecture 76 -  Lab assignment 5 : Exception Masking/Un-masking

* Demonstrating Enable/Disable Exceptions usinf PRIMASK and BASEPRI registers
* PRIMASK disables all exceptions except RESET,NMI and HARDFAULT
* setting the priority value for BASEPRI (bit0-7) register blocks all exceptions with priority same or lower than the programmed val
* we create a keil project 'lab_exercise5' adding the source files from courseRepo
* to issue the interrupt we will use the user button on our board
* with ifdefs we control the PRIMASK (disable interrupts) and BASEPRI disasbling interups with priority num >=8
```
#if 0
	 __set_PRIMASK(1);
#endif
	
#if 0
	/* 0x80 is the priority level, any interrupt which is same or lower priority will be blocked */
	__set_BASEPRI(0X80);
#endif
```
* priority was set to 0 `NVIC->IP[EXTI0_IRQn] = 0x00;` if we set it to 0x80 it wond pass
* We should revise the exercise once we learn about GPIOs as its NOT WORKING on NUCLEO

### Lecture 77 -  Lab Assignment 6 : Getting Started with USB-Logic Analyzer

* we have a Saleae Logic Analyzer
* we connect the GPIO PIN we want to monitor to CH1 (red) and GRND cable to 
* set sampling rate and sampling duration
* we can use markers to check duration

### Lecture 78 - Lab Assignment 7 : Interrupt Priority and Pre-emption

* we will demonstrate interrupt Priority snd preepmption by configuring 2 interrupts
* we will use SysTick Timer exception and button interrupt to understand how pre-emption  works due to chaining priorities
* Arm Cortex M3/<4 has a24-bit system timer called sysTick
* the counter in the sysTick is 24bit decrement counter, when we star the counter by loading some value it starts decrementing for every processor clock cycle
* if it reaches 0 it raises sysTick Timer exception, reloads the val and continues
* it can be used for time keeping, time measuremenmt or as an interrupt source for tasks that need to be executed regularly
* in an OS envf it can be used for context switching between tasks
* in First case we will give SysTick Timer IRQ priority 0xF0 and to button priority 0x00 (higher)
* so when systick timer exception handler is running, if button interrupt arrives , it will preempt the systick exception
* we build on previous example cping code from courseRepo
* we configure Systick timer for 2000ticks
```
	//2000 ticks
	SysTick_Config(2000);//125 micro seconds (16MHz)
```
* internasl clock (HSI) is set to 1600000000 (16MHz) in system _stm32...c
* we connect logic analyzer in D13 pin
* we check the perion between interupts
* we set priority with SHP (system handler priority) reg of SCB 
* there are 12 such registers of 8bits, like for itnerrupts. SysTick is the 12th (0 based) so B setting it to highest val 0 `SCB->SHP[0x0B] = 0X00`
* button has low priority `	NVIC->IP[EXTI0_IRQn] = 0Xf0;// (low priority )`
* we swap priorities
* in the analyzer we see that whenever there is a button press systick is preempted
* when a higher priority isr runs it wont allow lower priority interrupt
* if i invert priorities, systick prempts the use rbutton handler which takes morfe time (it is preempted and resumes execution once systime finishes)

## Section 14 - System Exceptions and Interrupts-II

### Lecture 79 - Pending Interrupt Behaviour

* Case 1: Single Pended Interrupt
	* Proc in Thread mode
	* Interrupt asserted from peripheral (lo->hi)
	* Pending Bit for asserted IRQn is Set
	* Proc changes mode from Thread to Handler (stacking & vector fetch)
	* When proc executes the ISR Interrupt Active bit for IRQn is Set. Pending bit is cleared
* Programmer makes sure that system wont receive any other interrupt till ISR is finished
	* Proc finishes ISR and exits
	* Interrupt active bit is cleared
	* Proc unstacks and changes from Handler to Thread mode
* Case 2: Double Pended interrupt
	* Proc in Thread mode
	* Interrupt asserted from peripheral (lo->hi)
	* Pending Bit for asserted IRQn is Set
	* Proc changes mode from Thread to Handler (stacking & vector fetch)
	* Proc executes the ISR Interrupt Active bit for IRQn is Set. Pending bit is cleared
	* a new interrupt (same) is asserted
	* pending bit is set
	* once ISR exits proc goes to thread mode to serve the 2nd interrupt

### Lecture 80 - Exception Entry and Exit Sequence

* every time there is an exception (Exception ENtry Seqyebnce)
	* pending bit is set
	* stacking and vector fetch (push register conent to stack, fetch the address of ISR from vector table)
	* proc entry into handler and active bit is set
	* proc clears the pending status (automatically)
	* proc goest in handler mode
	* isr code gets executed
	* MSP will be used for stack operation in the handler
* Exception Exit Sequence
	* in Cortex M3/M4 procs the exception return mechanism is tiggered using a special return address called EXC_RETURN
	* EXC_RETURN is generated during exception entry and is stored in the LR reg
	* When EXC_RETURN is written to PC it triggers ther exception return
* Example:
	* Proc is running in thtread mode with PSP stack pointer
	* Exception is asserted
	* Proc is stacking with PSP and vector fetch, LR is loaded with EXC_RETURN val (0xFFFFFFFD)
	* Proc executes ISR in handler mode 
	* Proc wants to do exception return 
	* Exception return kicks in once LR val is copied back to PC
	* It does unstacking with PSP
	* Returns to threa dmode
* When EXC_RETURN is generated? Durng an exception handler entry, the value of the return address (PC) is not stored in the LR as it is done during function call. Instead the exception mech stores the Special value called EXC_REURN in LR
* Decoding the EXC_RETURN val
	* bit0 		: reserved 				: 1
	* bit1 		: reserved 				: 0
	* bit2 		: return stack  		: 1=return with PSP, 0=return with MSP
	* bit3 		: return mode 			: 1=return to thread mode, 0= return to handler mode
	* bit4 		: stack frame type 		: always 1 when floating point unit is not available
	* bit5-27	: reserved (all 1)		: 0xEFFFFF
	* bit28-31	: EXC_RETURN indicator 	: 0xF

### Lecture 81 - Exception Entry and Exit Sequences : Demonstration

*  we create anew keil project 'exception_entry_exit_seq' merging code from 'blinky' and 'operation_modes'
* we are simulating WWG IRQ and use it to turn on some leds
* in main we pend the wdog to turn off the led
* code
```
#include <stdint.h>
#include "Board_LED.h"
#include "stm32f446xx.h"

// we have to implement the watchdog interrupt handler
void WWDG_IRQHandler(void) {
	LED_Off(0);
}

void delay(void) {
	uint32_t i = 0;
	for(i=0;i<100000;i++);
}

int main(void) {
	LED_Initialize();

	while(1){
		LED_On(0);
		
		//Enable and Pend the Watchdog Interrupt here
		NVIC_EnableIRQ(WWDG_IRQn);
		NVIC_SetPendingIRQ(WWDG_IRQn);
		
		delay();
		
		LED_On(0);
		
		delay();
	}
	return 0;
}
```
* when irq triggers proc pushes regs into the stack
* we will analayze stack contents at exception entry. starting from MSP we expect to see in memory
	* R0 => R1 => R2 => R3 => R12 => LR => PC => xPSR => Last item before entry
	* To verify it we need to disale the FPU as it will add more registers in stack frame (Target Options => Floating Point HW: Not USed)
* Stack first decreases and then pushes
* After stacking Proc contains EXC_RETURN val
* in exception return EXC_RETURN is used
* When EXC_RETURN is written to PC it triggers the return
* on exit we check the STACK POINTER and the pop instruction which puts into PC the Return
* proc decodes EXC_RETURN to see the conditions of return (mode et al)
* then unstacking happens
* in main we change the Stack pointer to PSP
```
	// change the stack pointer to PSP
	uint32_t value = __get_CONTROL();
	value |= (1<<1);
	__set_CONTROL(value);
```
* we put PSP to end of SRAM1 (SRAM2_BASE) `__set_PSP(SRAM2_BASE);`
* EXC_RETURN reflects the change

## Section 15 - LAB SESSION

### Lecture 82 - Lab Assignment 8: Programming and Configuring LED using Registers

* we will learn to turn on and off LEDs from scratch using Registers
* we will use Nucleo Board. we check schematic and see that the only User Led is connected to PortA pin 5
* Steps to COnfigure LEDs
	* Enable the CLock for the GPIO Port where LED is connected (In our case is GPIOA). Peripheral Clocks are disabled by default to save power
	* Make Sure that the GPIO PIN which drives the LED is configured as Output Mode
* We start a barebone project in keil (no bsp)
* in MCU Ref Manual we go to RCC (Reset and Clock Control)
* this module controls the system clock among others
* we go to 6.3.10 'RCC AHB1 peripheral clock enable register (RCC_AHB1ENR)'
* this is a register responsible for enabling clocks of all peripherals connected to AHB1 bus (we confirm GPIOA is connected to this bus)
* we verify that bit0 is 'GPIOAEN' IO port A clock enable. we need to set it
* we dont know RCC_AHB1ENR exact address but its address offset is 0x30 `#define RCC_AHB1ENR_OFFSET 0x30` the offset is from RCC_BASE address (sec 2.2.2 of RefMan) which is 0x40023800 - 0x40023BFF `#define RCC_BASE_ADDR 0x40023800`
* we define amacro for address `#define RCC_AHB1ENR_ADDR *((volatile unsigned long *)(RCC_BASE_ADDR+RCC_AHB1ENR_OFFSET))`
* the resaon to use volatile keyword with mem address. it forces compiler to read from mem address instead of reading a copy of the data (in proc reg)
* to change the mode of the GPIO pin we need the 'GPIO port mode register (GPIOx_MODER) (x = A..H)' of GPIO registers (7.4.1)
* for each pin 2 bits are used to configure
* we see the reg offset `#define GPIOA_MODER_OFFSET	0x00`
* we need to find GPIOA base addr (from MEm Map we see it has range 0x40020000 - 0x400203FF) `#define GPIOA_BASE_ADDR			0x40020000`
* we add the macro for the address `#define GPIOA_MODER_ADDR		*((volatile unsigned long *)(GPIOA_BASE_ADDR+GPIOA_MODER_OFFSET))`
* we use 'GPIO port output data register (GPIOx_ODR) ' to write to the output 1
* we compile and test. GREAT!
* To toggle the LED we need to read the GPIO pin state using the 'GPIO port input data register (GPIOx_IDR)' offest 0x10 and depending on state clear the bit or set the bit in ODR reg
```
void toggle_led(void) {
	if(GPIOA_IDR_ADDR & (1<<5)) {
		GPIOA_ODR_ADDR &= ~(1 << 5);
	} else {
		GPIOA_ODR_ADDR |= (1 << 5);
	}
}
```
* i test in a while loop with delay

### Lecture 83 - Lab assignment 9 : Programming and Configuring External Interrupt (Buttons)-I

* we will see how to configure and program external interrupts from scratch
* we will use the NUcleo Board and its user button (GPIOC port pin 13)
* How external interrupts are connected to the proc? thrugh the NVIC module providing NVIC lines. 15 are reserved for system. so ext interrupt 0 is exception #16
* How external interrupts are delivered to NVIC??? 
* PC13 which is wired to the button is NOT connected directly to NVIC but passes through a MUX that takes all PIN13 of all MCU GPIO ports and allows only 1 line called EXTI13 to go to the NVIC 
* so only one pin among the same indexed pins of diff ports is allowed to go to NVIC to trigger an interrupt at any given time. 
* MIXing is done in EXTx[3:0] bits in the SYSCFG_EXTICR1 register (16 lines are available for all ports)
* line 13 is in EXTI15_10 (EXTI Line[15:10]) entry in vector table with position 40 and address 0x000000E0 in vector table. this is the address of the ISR (we overwrite the definition on s file). whatever address we pass in this location it will be called when IRQ is triggered
* it is the responcibility of the device startup code to put the address of the ISRs in the vector table locations. USE the vendors startup code

### Lecture 84 - Lab assignment 9 : Programming and Configuring External Interrupt (Buttons)-II

* GPIOs are peripherals and need a clock to operate
* Clocks are off by default to save power
* Steps to configure the Button Interrupt
	* Enable the Clock from the GPIO
	* Configure the pin as input
	* enable the interrupt for a pin and configure the Trigger (rising edge etc)
	* Enable the interrupt in NVIC
	* Setup the Priority (optional)
	* Vector Table initialization
* we start a barebone project 'lab_exercise9_interrupt' and cp the files from courseRepo
* we mod the code for NUCLEO to reflect leds and go to main to set the button interrupt
* enable clock
```
	/*1. Enable GPIOC clock */
	/* because BUTTON is connected to GPIOC PIN13 */
	//RCC_AHB1ENR
	RCC->AHB1ENR |= 0X04; //Enables the clock
```
* set pin to input (clear bits in MODER)
```
	/* 2. set the mode of GPIOC pin0 to "INPUT" */
	GPIOC->MODER &= ~0X0C000000;
```
* set falling edge trigger for pin13
```
	/*3. set the interrupt triggering level for pin13 (bit13)*/
	//(EXTI_FTSR
	EXTI->FTSR |= 0X2000;	
```
* enable the interrupt over ETXT13 line (MUX)
```
	/*4. enable the interrupt over EXTI13 */
	EXTI->IMR |= 0X2000;
```
* enable interrupt in NVIC
```
/*5. ENable the interrupt on NVIC for IRQ40 */
	NVIC->ISER[(((uint32_t)EXTI15_10_IRQn) >> 5)] = (uint32_t)(1 << (((uint32_t)EXTI15_10_IRQn) & 0x1F));
```
* seting ISER: NVIC supports up to 7 ISER registers to set enable interrupts
	* NVIC_ISER0 (addr 0xE000E100) is for IRQ 0 to 31 (bit31 for IRQ31)
	* NVIC_ISER1 (addr 0xE000E104) is for IRQ 32 to 63 (bit31 for IRQ63)
	* ...
* in interupt we need to clear pending bit
```
void EXTI0_IRQHandler(void)
{
	/*clear the pending bit for exti13 */
		if( (EXTI->PR & 0x2000) )
	{
		EXTI->PR = 0x2000;//Writing 1 , clears the pending bit for exti13
	
	}
	led_toggle(LED_2);
	
}
```

## Section 16 - Cortex M3/M4 OS Features

### Lecture 85 - Use of shadowed stack pointer

* Important concepts to grasp: 
	* the use of shadowed stack pointer in OS
	* SVC exception and its uses
	* PendSV Exception and its uses
* How Cortex M3/M4 Helps OS?
	* Shadowed stack pointer
	* SysTick Timer
	* SVC and PendSV exceptions
* Shadowed Stack Pointer
	* Physically 2 stack pointers are there for M3/M4 (MSP,PSP)
	* The SP(R13), which is called stack pointer points to the currently selected stack pointer
	* Value of SPSEL bit in CONTROL reg determines which stack is currently active and used in Thread mode
* How OS Benefits from SHadowed Stack Pointers
	* In SRAM Kernel has allocated different regions for different tasks stacks
	* When kernel runs it uses MSP to track its stack space (kernel stack space)
	* When kernel scheduler makes Task A to run, it changes the proc stack selection to PSP and loads the Task A's previous stack pointer to PSP to make task A resume from where it was left before
	* The kernel should keep track of all the tasks stack pointer values and load it appropriately into PSP when it schedules a particulat task to run
	* this prevents a task to corrupt the kernel stack

### Lecture 86 - SVC System Exception

* SVC stands for Supervisory Call.
* This is Triggered by SVC instruction
* Once we execute this instruction, the svc handler will execute right after the svc instruction(no delay!!!unless a higher priority exception arrive at the same time)
* Advantages of SVC exception:
* Case of Priviledged System Resource Access by the application. (Kernel => Driver Interface => HW(Priviledged Resource))
	* The App Task says "I want to use the Priviledged HW" 
	* Kernel says "What you want to do?"
	* Say the App wants to open the HW and send some data
	* Kernel says "Issue SVC with service number 4" and i will do it for you
* Kernel will not allow any application to directly open the HW, the application has to ask the service using the SVC instruction with the appropriate service number
* Case of Application Portability (Kernel_v2 => Driver_V2 => Hardware_V2(priviledged))
	* Application_v1 asks "Do I need to change my code to open the HW?"
	* Kernel_v2 replies "No, still you can use SVC 0x04"
* SVC allows apps to be developed independently from OS
* Method to Trigger SVC Exception (2 ways)
	* Direct execution of svc instruction with an immediate value: e.g SVC 0x04 in assembly issued in a C program (very efficient in latency)
	* Set the exception pending bit "System Handler COntrol and State Register (SHCSR)". This method is not recommended and there is no reason to use it
* How to extract the SVC Number
	* when svc instruction is executed, the associated immediate value(Service Number) will not be passed to SVC Exception Handler
	* The SVC Handler needs to extract the number by using the PC value which was stored on to the stack, prior coming to exception handler!
* Example:
	* we have Task A running in Thread mode
	* Task issues SVC instruction
	* proc stacks automatically the usual stack frame (Assuming FPU is disabled)
	* SVC exception handler is executed
	* in the handler: the handler must determine which SP was used for stacking operations when SVC exception occured (read LR in stack which has the EXC_RETURN). get the value of PC, because PC holds the address of the instruction after SVC instruction (the address next to the svc instruction). if i increment MSP by 6 i go to PC in the stack frame
```
Next_ins_addr_after_svc = MSP[6]
SVC_number = *((Next_ins_addr_after_svc)-2) //this gives the address where svc instruction is stored in the code memory
```
* we do this because SVC num is not passed in the handler

### Lecture 87 - PendSV System Exception-I

* In OS designes, we need to switch between different tasks to support multitasking. THis is typically called context switching
* Context switching is usually carried out in the PendSV exception handler
* It is exception type 14 and has a programmable priority level
* it is basically set to lowest priority possible
* This exception is triggered by setting its pending status by writing to the "Interrupt Control and State Register ICSR of NVIC"
* Typical use of PendSV:
	* Typically this exception is triggered inside a higher priority exception handler and it gets executed when the trigger priority handler finishes
	* Using this characteristic we can schedule the PendSV exception handler to be executed after all the other interrupt processing tasks are done
	* This is very useful for a context switching operation, which is a key operation in various OS design
* Context switching:
	* OS code runs on each systick timer exception and decides to schedule(switch to) different task (Context Switch)
* PendSV in Context Switching:
	* In typical embedded OS design, the context switching operation is carried out inside the PendSV exception handler
	* Using PendSV in context switching will be more efficient in a interrupt noisy environment
	* IN such an env we need to to delay the context switching untill all IRQ are executed
	* PendSV delays the context switching untill processor is free from other exceptions
	* If OS decides context status is needed it sets the pending status of the PendSV and carries out the context switching within the PendSV exception

### Lecture 88 - PendSV System Exception-II (Understanding with animation) 

* we have a Task Running in thread mode. while it executes SysTick Timer Timesout (Exception)
* control is given to the OS running in the Systick Handler
* OS wants to do context switch. it pends the pendSV
* there are no other higher priority interrupts in system
* after systick handler finishes control is given to pendSV handler
* context switching is done in the handler
* after PendSV handler exits TaskB executes in thread mode
* Why COntext Switching is done in pendSV handler and not in Systick imer handler?
* while TaskB runs an interrupt occurs
* control is handled over its ISR
* while its executing systick times out!
* systick premts the interrupt ISR and takes over
* we have a half serviced ISR pre-empted.
* if the OS task running in Systick handler wants to context switch from the SYsTick handler and return to thread mode then processor will raise HELL (UsageFault Exception) as we cannot return to thread mode leaving a pre-empted ISR pending
* doing context switch from pendSV (lowest priority) ensures there will never be an ISR pending when returning to thread mode
* Offloading Interrupt PRocessing using PendSV
	* If a higher priority handlers doing time consuming work, then the other lower priority interrupts will suffer and systems responsivenenss may reduce 
	* Typically Interrupts are serviced in 2 halves: the first is the time critical part that needs to be executed as part of ISR, the second half is called 'bottom half' is basically delayed execution where rest of the time consuming work will be done
* So PendSV can be used in these cases, to handle the second half execution by triggering it in the first half

## Section 17 - LAB SESSION

### Lecture 89 - Lab assignment 10 :SVC Exception and Handler implementation

* Write a C program to do arithmetic operation between 2 values by executing SVC instruction.
	* Use the svc service number to decide which operation needs to be carried out as per below table
	* Do the operation in SVC handler and return the result
	* SVC num 0x36 : Addition
	* SVC num 0x37 : Multiplication
* project 'lab_exercise10_svc_except' has the answer and the code

### Lecture 90 - Lab assignment 11 : PendSV in off

* Demonstrating Usage of PendSV to offload the Interrupt Processing
* we cp the interrupt lab exercise and add the pendSV pending from the Push button ext irq handler
```
	/*process only which is important and trigger the pendsv to 
	handle rest in the pendsv handler  */
	SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
```