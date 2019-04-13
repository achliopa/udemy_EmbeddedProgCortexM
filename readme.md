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

### Lecture 9 - STM32F4 Discovery and Nucleo: Board Details

* if using discovery board use the newest one DIC1
* MCU does not use the external clock unless we tell it to
* Nucleo board is more streamlined (less feats)
	* no external crystal
	* arduino hat compatible
	* nucleo boards have stm32 nucleo support
	* when we connect it to pc it is enumerated as a virtual com port
* to enable virtual com stlink connectivity in discovery board we need to configure it and use a UART to USB adapter

### Lecture 10 - ST-Link Driver Installation

* to use ST Link from our dev machine we need to install ST Link Drivers
	* Windows: use Executable from st.com
	* Mac: no driver installation needed
	* Ubuntu:
```
sudo apt-get install libusb-1.0
sudo apt-get install libc6:i386 lib32ncurses5
```
* in woin we download and install prior to connecting the device
* connect device and go: Control Panel => Device Manager => COM Ports to verify the COM port it is listening to
* IDE uses this port and we can use this port to communicate the device through terminal

### Lecture 11 - ST Link Firmware Upgrade

* in the board page in ST we download firmware upgrade
* we use the version for the platform we use  (follow the readme file)
* ST-LINK/V2-1 is same as ST-LINK/V2-A
* we upgrade our firmware 

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
* In opens a window with a list of manufacturers and thier products grouped in families
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
	* in 'Device' we click Startup to get a stertup code. that  gives a warning as it requires an additional package CMSIS CORE to be installed
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

* 