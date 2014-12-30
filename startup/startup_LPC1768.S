/*****************************************************************************/
/* startup_LPC1768.S: Startup file for LPC17xx device series                 */
/*****************************************************************************/
/* Version: GNU Tools for ARM Embedded Processors 4.9.3 20141119             */
/*****************************************************************************/

@				.syntax				unified					/* use modern assembler syntax + auto-generate IT instructions. Put in top of your source file */
@				.weak				label{,label}			/* allow 'label' to be undefined. If it's undefined, it will have the value NULL (0x00000000). */
@				.weakref			label,defaultLabel		/* allow 'label' to be undefined. If it's undefined, it will have the value of another label. */
@				.section			sectionName				/* all output from now on goes into a section called 'sectionName' */
@				.align				[bitposition]			/* align the output offset */
@				.long				value					/* output a 32-bit value */
@				.text										/* all output from now on goes into a section called '.text' (same as '.section .text') */
@				.func				label[,actualLabel]		/* mark the beginning of function 'label', so the linker may exclude the block if not referenced */
@				.endfunc									/* mark the end of the function */
@				.pool										/* allow the assembler to place constants here */
@				.size				label,size				/* tell the linker how long the block that this symbol points to is (in Bytes) */
@				.thumb_func			label					/* mark this as a thumb function (required if the function is called by using 'bx' or 'blx') */
@				.type				label,%type				/* specify the type of the symbol. Required if there is a pointer to the function somewhere. */
@				.cpu				cpuType					/* cpuType may for instance be cortex-m0, cortex-m3 or cortex-m4. */


				.syntax				unified					@ /* use modern assembler syntax + auto-generate IT instructions. Put in top of your source file */
				.cpu				cortex-m3				@ /* cpuType may for instance be cortex-m0, cortex-m3 or cortex-m4. */

				.section			isr_vector				@ /* Put everything in a section called "isr_vector" from now on... */
 				.align				2						@ /* Make sure the output goes on an address divisible by 4 (that's 1 << 2) */
															@ /* Address:   Exception Vector Description: */
				.long				_stack					@ /* 0x00000000 The initial stack pointer (defined by the linker-script) */
				.long				Reset_Handler			@ /* 0x00000004 The startup-code, the code that runs on power-on or RESET */
				.long				NMI_Handler				@ /* 0x00000008 Non-Masktable Interrupt, this can not be stopped, preempted or prevented */
				.long				HardFault_Handler		@ /* 0x0000000c Hard Fault, all classes of Fault */
				.long				MemManage_Handler		@ /* 0x00000010 Memory Management, MPU mismatch, including Access Violation and No Match */
				.long				BusFault_Handler		@ /* 0x00000014 Bus Fault, Pre-Fetch- Memory Access Fault, other address/memory related Fault */
				.long				UsageFault_Handler		@ /* 0x00000018 Usage Fault, i.e. Undefined Instructions, Illegal State Transitions */
				.long				0						@ /* 0x0000001c */
				.long				0						@ /* 0x00000020 */
				.long				0						@ /* 0x00000024 */
				.long				0						@ /* 0x00000028 */
				.long				SVC_Handler				@ /* 0x0000002c Supervisor Call */
				.long				DebugMon_Handler		@ /* 0x00000030 Debug Monitor */
				.long				0						@ /* 0x00000034 */
				.long				PendSV_Handler			@ /* 0x00000038 Pending Service, pending requests for system service */
				.long				SysTick_Handler			@ /* 0x0000003c System Tick Timer (this may not exist on all implementations) */

@				/* External Interrupts */
				.long				WDT_IRQHandler			@ /* 16: Watchdog Timer */
				.long				TIMER0_IRQHandler		@ /* 17: Timer0 */
				.long				TIMER1_IRQHandler		@ /* 18: Timer1 */
				.long				TIMER2_IRQHandler		@ /* 19: Timer2 */
				.long				TIMER3_IRQHandler		@ /* 20: Timer3 */
				.long				UART0_IRQHandler		@ /* 21: UART0 */
				.long				UART1_IRQHandler		@ /* 22: UART1 */
				.long				UART2_IRQHandler		@ /* 23: UART2 */
				.long				UART3_IRQHandler		@ /* 24: UART3 */
				.long				PWM1_IRQHandler			@ /* 25: PWM1 */
				.long				I2C0_IRQHandler			@ /* 26: I2C0 */
				.long				I2C1_IRQHandler			@ /* 27: I2C1 */
				.long				I2C2_IRQHandler			@ /* 28: I2C2 */
				.long				SPI_IRQHandler			@ /* 29: SPI */
				.long				SSP0_IRQHandler			@ /* 30: SSP0 */
				.long				SSP1_IRQHandler			@ /* 31: SSP1 */
				.long				PLL0_IRQHandler			@ /* 32: PLL0 Lock (Main PLL) */
				.long				RTC_IRQHandler			@ /* 33: Real Time Clock */
				.long				EINT0_IRQHandler		@ /* 34: External Interrupt 0 */
				.long				EINT1_IRQHandler		@ /* 35: External Interrupt 1 */
				.long				EINT2_IRQHandler		@ /* 36: External Interrupt 2 */
 				.long				EINT3_IRQHandler		@ /* 37: External Interrupt 3 */
				.long				ADC_IRQHandler			@ /* 38: A/D Converter */
				.long				BOD_IRQHandler			@ /* 39: Brown-Out Detect */
				.long				USB_IRQHandler			@ /* 40: USB */
				.long				CAN_IRQHandler			@ /* 41: CAN */
				.long				DMA_IRQHandler			@ /* 42: General Purpose DMA */
				.long				I2S_IRQHandler			@ /* 43: I2S */
				.long				ENET_IRQHandler			@ /* 44: Ethernet */
				.long				RIT_IRQHandler			@ /* 45: Repetitive Interrupt Timer */
				.long				MCPWM_IRQHandler		@ /* 46: Motor Control PWM */
				.long				QEI_IRQHandler			@ /* 47: Quadrature Encoder Interface */
				.long				PLL1_IRQHandler			@ /* 48: PLL1 Lock (USB PLL) */
				.long				USBActivity_IRQHandler	@ /* 49: USB Activity */
				.long				CANActivity_IRQHandler	@ /* 50: CAN Activity */

				.text										@ /* Put everything in the text-section from now on... */
				.align										@ /* Make sure address is aligned for code output */



_stack:

NMI_Handler:

HardFault_Handler:

MemManage_Handler:

BusFault_Handler:

UsageFault_Handler:

SVC_Handler:

DebugMon_Handler:

PendSV_Handler:

SysTick_Handler:

/* Reset Handler */


			.globl					Reset_Handler
			.type					Reset_Handler, %function

Reset_Handler:

			LDR						R0, =SystemInit
    		BLX						R0
			LDR						R0,=main
    		BX						R0


/* IRQ Handlers */

				.globl				Default_Handler
				.type				Default_Handler, %function

Default_Handler:
				b       			.
				.size				Default_Handler, . - Default_Handler

				.macro				IRQ handler
    			.weak				\handler
    			.set				\handler, Default_Handler
    			.endm
    			IRQ					WDT_IRQHandler
				IRQ					TIMER0_IRQHandler
    			IRQ					TIMER1_IRQHandler
    			IRQ					TIMER2_IRQHandler
    			IRQ					TIMER3_IRQHandler
    			IRQ					UART0_IRQHandler
    			IRQ					UART1_IRQHandler
    			IRQ					UART2_IRQHandler
    			IRQ					UART3_IRQHandler
    			IRQ					PWM1_IRQHandler
    			IRQ					I2C0_IRQHandler
    			IRQ					I2C1_IRQHandler
    			IRQ					I2C2_IRQHandler
    			IRQ					SPI_IRQHandler
    			IRQ					SSP0_IRQHandler
    			IRQ					SSP1_IRQHandler
    			IRQ					PLL0_IRQHandler
    			IRQ					RTC_IRQHandler
    			IRQ					EINT0_IRQHandler
    			IRQ					EINT1_IRQHandler
    			IRQ					EINT2_IRQHandler
    			IRQ					EINT3_IRQHandler
    			IRQ					ADC_IRQHandler
    			IRQ					BOD_IRQHandler
    			IRQ					USB_IRQHandler
    			IRQ					CAN_IRQHandler
    			IRQ					DMA_IRQHandler
    			IRQ					I2S_IRQHandler
    			IRQ					ENET_IRQHandler
    			IRQ					RIT_IRQHandler
   				IRQ					MCPWM_IRQHandler
    			IRQ					QEI_IRQHandler
    			IRQ					PLL1_IRQHandler
    			IRQ					USBActivity_IRQHandler
    			IRQ					CANActivity_IRQHandler
