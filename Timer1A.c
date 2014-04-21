// Timer0A.c
// Runs on LM4F120
// Use Timer0A in periodic mode to request interrupts at a particular
// period.
// Daniel Valvano
// May 13, 2013

/* This example accompanies the book
   "Embedded Systems: Introduction to ARM Cortex M Microcontrollers"
   ISBN: 978-1469998749, Jonathan Valvano, copyright (c) 2013
   Volume 1, Program 9.8

  "Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2013
   Volume 2, Program 7.5, example 7.6

 Copyright 2013 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */
#include "UART2.h"
#include "tm4c123gh6pm.h"

#define NVIC_EN0_INT21          0x00200000  // Interrupt 21 enable
/*#define NVIC_EN0_R              (*((volatile unsigned long *)0xE000E100))  // IRQ 0 to 31 Set Enable Register
#define NVIC_PRI5_R             (*((volatile unsigned long *)0xE000E414))  // IRQ 13 to 15 Priority Register
#define TIMER1_CFG_R            (*((volatile unsigned long *)0x40031000))//annyan
#define TIMER1_TAMR_R           (*((volatile unsigned long *)0x40031004))
#define TIMER1_CTL_R            (*((volatile unsigned long *)0x4003100C))
#define TIMER1_IMR_R            (*((volatile unsigned long *)0x40031018))
#define TIMER1_MIS_R            (*((volatile unsigned long *)0x40031020))
#define TIMER1_ICR_R            (*((volatile unsigned long *)0x40031024))
#define TIMER1_TAILR_R          (*((volatile unsigned long *)0x40031028))
#define TIMER1_TAPR_R           (*((volatile unsigned long *)0x40031038))
#define TIMER1_TAR_R            (*((volatile unsigned long *)0x40031048))//end of annyan
#define TIMER1_TAV_R						(*((volatile unsigned long *)0x40031050))*/
#define TIMER_CFG_16_BIT        0x00000004  // 16-bit timer configuration,
                                            // function is controlled by bits
                                            // 1:0 of GPTMTAMR and GPTMTBMR
#define TIMER_TAMR_TACDIR       0x00000010  // GPTM Timer A Count Direction
#define TIMER_TAMR_TAMR_PERIOD  0x00000002  // Periodic Timer mode
#define TIMER_CTL_TAEN          0x00000001  // GPTM TimerA Enable
#define TIMER_IMR_TATOIM        0x00000001  // GPTM TimerA Time-Out Interrupt
                                            // Mask
#define TIMER_ICR_TATOCINT      0x00000001  // GPTM TimerA Time-Out Raw
                                            // Interrupt
#define TIMER_TAILR_TAILRL_M    0x0000FFFF  // GPTM TimerA Interval Load
                                            // Register Low
//#define SYSCTL_RCGC1_R          (*((volatile unsigned long *)0x400FE104))
#define SYSCTL_RCGC1_TIMER1     0x00020000  // timer 0 Clock Gating Control

//modified by annyan and Zirui to drive a pin to high or low
/*#define GPIO_PORTD_DATA_R       (*((volatile unsigned long *)0x400073FC))
#define GPIO_PORTD_DIR_R        (*((volatile unsigned long *)0x40007400))
#define GPIO_PORTD_AFSEL_R      (*((volatile unsigned long *)0x40007420))
#define GPIO_PORTD_DEN_R        (*((volatile unsigned long *)0x4000751C))
#define GPIO_PORTD_AMSEL_R      (*((volatile unsigned long *)0x40007528))
#define GPIO_PORTD_PCTL_R       (*((volatile unsigned long *)0x4000752C))
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))*/
#define SYSCTL_RCGC2_GPIOD      0x00000008  // port D Clock Gating Control
//#define PIND0                    (*((volatile unsigned long *)0x40007004))
//end of modification

void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode
//Modified by annyan lab3
//void (*PeriodicTask1)(void);   // user function
//void (*PeriodicTask2)(void);   // user function

//Modified by annyan and Zirui to drive a pin to high or low
//void GPIO_Init(void){  volatile unsigned long delay;
//  SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOD; // 1) activate port D
//  delay = SYSCTL_RCGC2_R;           // allow time for clock to stabilize
//                                    // 2) no need to unlock PD3-0
//  GPIO_PORTD_AMSEL_R &= ~0x0F;      // 3) disable analog functionality on PD3-0
//  GPIO_PORTD_PCTL_R &= ~0x0000FFFF; // 4) GPIO
//  GPIO_PORTD_DIR_R |= 0x0F;         // 5) make PD3-0 out
//  GPIO_PORTD_AFSEL_R &= ~0x0F;      // 6) regular port function 
//  GPIO_PORTD_DEN_R |= 0x0F;         // 7) enable digital I/O on PD3-0
//}
//End of Modification
// ***************** Timer0A_Init ****************
// Activate Timer0A interrupts to run user task periodically
// Inputs:  task is a pointer to a user function
//          period in usec
// Outputs: none
/******************modified by annyan on April 21 for multiple perioidic interrupt********************/
void (*PeriodicTask1)(void);   // user function
void (*PeriodicTask2)(void);   // user function
void (*PeriodicTask3)(void);   // user function
void (*PeriodicTask4)(void);   // user function
void (*PeriodicTask5)(void);   // user function
int period_threads[5]={0,};
unsigned int count = 0;
unsigned int timera_count = 0;
void OS_AddPeriodicThread(void(*task)(void), 
   unsigned long period, unsigned long priority){
	long sr;
	period_threads[count]=period;
	count++;
	switch(count){
		case 1: PeriodicTask1 = task;break;
		case 2: PeriodicTask2 = task;break;
		case 3: PeriodicTask3 = task;break;
		case 4: PeriodicTask4 = task;break;
		case 5: PeriodicTask5 = task;break;
	}
	//GPIO_Init();//for the PIN0 output
//	sr = StartCritical(); 
		//UART_OutChar(count+'0');
		//SYSCTL_RCGC1_R |= SYSCTL_RCGC1_TIMER1+0x00080000; // 0) activate timer0
	if(count == 1){
		TIMER1_CTL_R &= ~0x00000001;     // 1) disable timer0A during setup
		TIMER1_CFG_R = 0;
		TIMER1_CFG_R = 0x00000000;       // 2) configure for 32-bit timer mode
		TIMER1_TAMR_R = 0x00000012;      // 3) configure for periodic mode, default down-up settings
		TIMER1_TAILR_R = 80000-1;       // 4) reload value 1ms as a unit
		TIMER1_ICR_R = 0x00000001;       // 6) clear timer1A timeout flag
		TIMER1_IMR_R |= 0x00000001;      // 7) arm timeout interrupt
		NVIC_PRI5_R = (NVIC_PRI5_R&0xFFFF1FFF)|(priority<<13); // 8) priority 2
		NVIC_EN0_R = NVIC_EN0_INT21;     // 9) enable interrupt 21 in NVIC
		TIMER1_CTL_R |= 0x00000001;      // 10) enable timer0A
	}
		
	//EndCritical(sr);
}
//	 void OS_AddPeriodicThread2(void(*task)(void), 
//   unsigned long period, unsigned long priority){
//	long sr;
//	//GPIO_Init();//for the PIN0 output
//	sr = StartCritical(); 
//	
//		//SYSCTL_RCGC1_R |= 0x00080000; // 0) activate timer0
//		TIMER3_CTL_R &= ~0x00000001;     // 1) disable timer0A during setup
//		TIMER3_CFG_R = 0;
//		TIMER3_CFG_R = 0x00000000;       // 2) configure for 32-bit timer mode
//		TIMER3_TAMR_R = 0x00000012;      // 3) configure for periodic mode, default down-up settings
//		TIMER3_TAILR_R = period-1;       // 4) reload value
//		//TIMER1_TAPR_R = 49;              // 5) 1us timer1A, not functional in 32-bit mode
//		TIMER3_ICR_R = 0x00000001;       // 6) clear timer1A timeout flag
//		TIMER3_IMR_R |= 0x00000001;      // 7) arm timeout interrupt
//		NVIC_PRI8_R = (NVIC_PRI8_R&0x1FFFFFFF)|(priority<<29); // 8) priority 2
//		NVIC_EN1_R |= 8;     // 9) enable interrupt 35 in NVIC
//		TIMER3_CTL_R |= 0x00000001;      // 10) enable timer0A
//		PeriodicTask2 = task;
//	EndCritical(sr);
//}

//modified by annyan lab3
void Timer1A_Handler(void){
//  PIND0=1;
//	long status;
////	status= StartCritical();
//	TIMER1_ICR_R = TIMER_ICR_TATOCINT;// acknowledge timer1A timeout
//	status= StartCritical();
//	(*PeriodicTask1)();                // execute user task
//  
//	
//	EndCritical(status);
	//UART_OutChar('a');
	timera_count ++;
	int i=0;
	for(i=0; i< count; i++){
		if (timera_count % period_threads[i]==0){
			switch(i){
				case 0: (*PeriodicTask1)(); break;
				case 1: (*PeriodicTask2)(); break;
				case 2: (*PeriodicTask3)(); break;
				case 3: (*PeriodicTask4)(); break;
				case 4: (*PeriodicTask5)(); break;		
			}
	  }
	}
  TIMER1_ICR_R = TIMER_ICR_TATOCINT;
}

//void Timer3A_Handler(void){
////  PIND0=1;
//	long status;

//	TIMER3_ICR_R = TIMER_ICR_TATOCINT;// acknowledge timer1A timeout
//	status= StartCritical();
//	(*PeriodicTask2)();                // execute user task
//  
//	
//	EndCritical(status);
// //UART_OutChar('b');
//}
