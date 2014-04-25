//Modified by Annyan and Zirui
#include "PLL.h"
#include "os.h"
#include "bank.h"
#include "UART2.h"
#include <stddef.h>
#include <stdlib.h>
#include "tm4c123gh6pm.h"

void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode
void PendSV(void);


/***************************************Lab2_Part1*****************************************/
/***************************************SYS_TICK****************************************/
#define NVIC_ST_CTRL_R          (*((volatile unsigned long *)0xE000E010))
#define NVIC_ST_RELOAD_R        (*((volatile unsigned long *)0xE000E014))
#define NVIC_ST_CURRENT_R       (*((volatile unsigned long *)0xE000E018))
#define NVIC_SYS_PRI3_R         (*((volatile unsigned long *)0xE000ED20))
#define NVIC_INT_CTRL_R         (*((volatile unsigned long *)0xE000ED04))
#define PEND0                   (*((volatile unsigned long *)0xE000E200))  
#define NVIC_ST_CTRL_COUNT      0x00010000  // Count flag
#define NVIC_ST_CTRL_CLK_SRC    0x00000004  // Clock Source
#define NVIC_ST_CTRL_INTEN      0x00000002  // Interrupt enable
#define NVIC_ST_CTRL_ENABLE     0x00000001  // Counter mode
#define NVIC_ST_RELOAD_M        0x00FFFFFF  // Counter load value
#define PIND0 (*((volatile unsigned long *)0x40007004))
/*******************************TCB**************************************************/
#define THREADS_NUM 10
#define STACKSIZE 100
#define DEAD 0xff78
#define ACTIVE 0x09dd
#define BLOCKED 0xff12
#define INVALID_PRIORITY 18
#define TIME_OUT 1000 //after 100us, thread will release semaphore

TCB_Type TCBS [THREADS_NUM];
TCB_Type *RunPt;
TCB_Type *NextPt;
TCB_Type *DeadPt;
long Stack [THREADS_NUM][STACKSIZE];

void StartOS(void);
void OS_DisableInterrupts(void);
int count_timer2 = 0;
void SetInitialStack (int i){
	TCBS [i].sp = & Stack [i][STACKSIZE - 16]; //should this warning be a problem???
	Stack [i][STACKSIZE-1] = 0x01000000;//Thumb bit
	Stack [i][STACKSIZE-3] = 0x14141414;//R14
	Stack [i][STACKSIZE-4] = 0x12121212;//R12
	Stack [i][STACKSIZE-5] = 0x03030303;//R3
	Stack [i][STACKSIZE-6] = 0x02020202;//R2
	Stack [i][STACKSIZE-7] = 0x01010101;//R1
	Stack [i][STACKSIZE-8] = 0x00000000;//R0
	Stack [i][STACKSIZE-9] = 0x11111111;//R11
	Stack [i][STACKSIZE-10] = 0x10101010;//R10
	Stack [i][STACKSIZE-11] = 0x09090909;//R9
	Stack [i][STACKSIZE-12] = 0x08080808;//R8
	Stack [i][STACKSIZE-13] = 0x07070707;//R7
	Stack [i][STACKSIZE-14] = 0x06060606;//R6
	Stack [i][STACKSIZE-15] = 0x05050505;//R5
	Stack [i][STACKSIZE-16] = 0x04040404;//R4

}

void OS_Init(){
	RunPt=&TCBS[0];
	DeadPt = NULL;
	OS_DisableInterrupts();
	//PLL_Init(); //set processor clock to 50MHZ
	NVIC_ST_CTRL_R = 0;
	NVIC_ST_CURRENT_R = 0;
	NVIC_SYS_PRI3_R =((NVIC_SYS_PRI3_R & 0x00ffffff)|(6<<29));//priority 7;
	NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R & 0xFF1FFFFF)|(7<<21);
	Timer2A_Init();
}
/*********required to be written?********************/

void OS_Launch(unsigned long theTimeSlice){
	NVIC_ST_RELOAD_R = theTimeSlice-1;
	NVIC_ST_CTRL_R = 0x00000007;//enable, core clock and interrupt arm
	StartOS();

}
void OS_Suspend (void){
	NVIC_ST_CURRENT_R = 0;
	NVIC_INT_CTRL_R = 0x04000000;//trigger systick_handler
}
//variable stackSize unused
int thread_num =0;
int thread_pt=0;
unsigned int count_die=0;
unsigned int count_num = 0;
int OS_AddThread (void(*task)(void), unsigned long stackSize, unsigned long priority){

	long status;
	status = StartCritical();
	
	
	
	/*for(i=0; i< thread_pt; i++){
		if (TCBS[i].status == DEAD){
			SetInitialStack (i);
			TCBS[i].Id = thread_num+1;
		  TCBS[i].priority = priority;
	    TCBS[i].sleep = 0;//Initialization
	    TCBS[i].blocked = 0;//Initialization
	    TCBS[i].status = ACTIVE;
			Stack[i][STACKSIZE-2] = (long)task;
			++thread_num;
			EndCritical(status);
			return 1;
		}
			
	}*/
	/*if (thread_num-count_die >=10||((count_num >=10) &&(!DeadPt))){
		EndCritical(status);
		return 0;}*/
		
		/**************comment for bank debug*************
	if (DeadPt){
		SetInitialStack (DeadPt->Id-1);
		DeadPt->priority = priority;
		DeadPt->sleep = 0;
		DeadPt->status = ACTIVE;
		//Modified by annyan for priority donation
		DeadPt->is_donated=0;
		DeadPt->priority_temp=INVALID_PRIORITY;
		//End of modification
		//Modified by annyan for Time out semaphore
		DeadPt->sema4_blocked = NULL;
		//End of modification
		Stack[DeadPt->Id - 1][STACKSIZE-2] = (long)task;
		DeadPt = NULL;

		//++count_alive;
		EndCritical(status);
		return 1;
	}********************************************************/
	
	
	SetInitialStack (thread_num);
	
	if(thread_num){
		TCBS[thread_num-1].Next = &TCBS[thread_num];
	}
	TCBS[thread_num].Next= &TCBS[0];
	TCBS[thread_num].Id = thread_num+1;//0 is reserved for special usage
	TCBS[thread_num].priority = priority;
	TCBS[thread_num].sleep = 0;//Initialization
	TCBS[thread_num].status = ACTIVE;
	//Modified by annyan for priority donation
	TCBS[thread_num].priority_temp=INVALID_PRIORITY;
	TCBS[thread_num].is_donated=0;
	//End of modification
	//Modified by annyan for Time out semaphore
	TCBS[thread_num].sema4_blocked = NULL;
	//End of modification
	Stack[thread_num][STACKSIZE-2] = (long)task; //PC
	
	/**********************bank******************************/
	bank_init_tcb(&TCBS[thread_num], thread_num);
	/*********************************************************/
	++thread_num;
//	++thread_pt;
	++count_num;
	//++count_alive;
	
	EndCritical(status);
	return 1;
}
//End of Modification


/*********************************Modified by Zirui**************************************/

unsigned long OS_TimeDifference(unsigned long start, unsigned long stop){
	//if (start<stop) return (stop-start);
	//else return start-stop;
	return stop-start;
	//return 0;
}
unsigned long OS_Time(void){
	return TIMER2_TAV_R + count_timer2*80000;//NVIC_ST_CURRENT_R;//TIMER2_TAR_R;//
	//return 0;
}


#define JITTERSIZE 64
#define JTTERAMOUNT 4
unsigned long JitterHistogram[JTTERAMOUNT][JITTERSIZE]={0,};
long MaxJitter[JTTERAMOUNT];   
unsigned static long lastTime[JTTERAMOUNT]; 
unsigned long thisTime[JTTERAMOUNT];

void jitterSetThisTime(char jitterNum){
	thisTime[jitterNum] = OS_Time();
}


void jitterSetLastTime(char jitterNum){
	lastTime[jitterNum] = thisTime[jitterNum];
}

long jitterCalc(char jitterNum, unsigned int period){
	long jitter; 
	unsigned long diff = OS_TimeDifference(lastTime[jitterNum],thisTime[jitterNum]);
	if(diff>period){
		jitter = (diff-period+4)/8;  // in 0.1 usec
	}else{
		jitter = (period-diff+4)/8;  // in 0.1 usec
	}
	if(jitter > MaxJitter[jitterNum]){
		MaxJitter[jitterNum] = jitter; // in usec
	}       // jitter should be 0
	if(jitter >= JITTERSIZE){
		jitter = JITTERSIZE-1;
	}
	JitterHistogram[jitterNum][jitter]++; 
	return jitter;
}

long maxJitter(char jitterNum){
	return MaxJitter[jitterNum];
}

long jitterHistogram(long jitter, char jitterNum){
	if (jitter<JITTERSIZE) return JitterHistogram[jitterNum][jitter];
	else return JitterHistogram[jitterNum][JITTERSIZE-1];
}

/********************************End of Modification*************************************/


//Modified by annyan
struct Sema4 BoxFree, DataValid;
unsigned long static MailBox; // zw
unsigned long OS_MailBox_Recv(void){
	static unsigned long data=0;
	OS_bWait (&DataValid);
	data = MailBox;
	OS_bSignal (&BoxFree);
	return data;
}
void OS_MailBox_Send(unsigned long data){
	OS_bWait (&BoxFree);
	MailBox = data;
	OS_bSignal (&DataValid);
	return;
}
//end of modification

// ******** OS_MailBox_Init ************
// Initialize communication channel
// Inputs:  none
// Outputs: none
//Modified by annyan
/*Initialize bsema4 DataValid, 1 stands for there is data
 *Boxfree, 1 stands for mailbox is free
 */
void OS_MailBox_Init(void){
	
	OS_InitSemaphore(&BoxFree, 1);//Initially BoxFree is 1 stands for an empty mailbox, 
	                              //Data Valid is 0 stands for no data
	OS_InitSemaphore(&DataValid, 0);
}
//end of modification
//Modified by annyan and Zirui
//unsigned long fifo;
#define FIFOSIZE 128
long volatile *fifoPutPt;
long volatile *fifoGetPt;
long static Fifo[FIFOSIZE];
struct Sema4 DataRoomLeft, mutex, DataAvailable;

void OS_Fifo_Init(unsigned long size){
	OS_InitSemaphore(&DataRoomLeft, FIFOSIZE);//size later
	OS_InitSemaphore(&DataAvailable, 0);
	OS_InitSemaphore(&mutex, 1);
	fifoGetPt=fifoPutPt=&Fifo[0];
}

long OS_Fifo_Size(void){
	return DataAvailable.Value;
}
unsigned long OS_Fifo_Get(void){
	unsigned long dataPt;
	OS_Wait (&DataAvailable);
	OS_bWait (&mutex);
	dataPt=*fifoGetPt;
	if (fifoGetPt==&Fifo[FIFOSIZE-1]) fifoGetPt=&Fifo[0]; //wrap
	else fifoGetPt++;
	OS_bSignal (&mutex);
	OS_Signal (&DataRoomLeft);
	return dataPt;
}

int OS_Fifo_Put(unsigned long data){
	int flag = 0;//0 for failed, 1 for successful
	volatile long *nextPt;
	//UART_OutChar('f');
	OS_Wait (&DataRoomLeft);
	OS_bWait (&mutex);
	//UART_OutChar('U');
	nextPt=++fifoPutPt;
	if (nextPt==&Fifo[FIFOSIZE]) nextPt=&Fifo[0]; //wrap
	*fifoPutPt=data;
	fifoPutPt=nextPt;
	flag = 1;
	OS_bSignal (&mutex);
	OS_Signal (&DataAvailable);
	return flag;
}

//end of modification
// ******** OS_Kill ************
// kill the currently running thread, release its TCB and stack
// input:  none
// output: none
void OS_Kill(void){
	//Assert (not in context switch)
	long status;
	status = StartCritical();
	count_die++;
	DeadPt = RunPt;
	RunPt->status = DEAD;
	EndCritical(status);
	OS_Suspend();//kill itself immediately
	
}

void OS_Sleep(unsigned long sleepTime){
	/*************Take it out and put into sleeplist*****************/
	long status;
	status = StartCritical();
	RunPt->sleep = sleepTime;
	EndCritical(status);
  OS_Suspend();
}

//Modified by annyan
void PortF4_Init(void){ 
                                // (a) activate clock for port F
  SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF;
  GPIO_PORTF_DIR_R &= ~0x10;    // (c) make PF4 in (built-in button)
  GPIO_PORTF_AFSEL_R &= ~0x10;  //     disable alt funct on PF4
  GPIO_PORTF_DEN_R |= 0x10;     //     enable digital I/O on PF4
                                //     configure PF4 as GPIO
  GPIO_PORTF_PCTL_R = (GPIO_PORTF_PCTL_R&0xFFF0FFFF)+0x00000000;
  GPIO_PORTF_AMSEL_R = 0;       //     disable analog functionality on PF
  GPIO_PORTF_PUR_R |= 0x10;     //     enable weak pull-up on PF4
  GPIO_PORTF_IS_R &= ~0x10;     // (d) PF4 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x10;    //     PF4 is not both edges
  GPIO_PORTF_IEV_R &= ~0x10;    //     PF4 falling edge event
  GPIO_PORTF_ICR_R = 0x10;      // (e) clear flag4
  GPIO_PORTF_IM_R |= 0x10;      // (f) arm interrupt on PF4
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00A00000; // (g) priority 5
  NVIC_EN0_R |=(1<<30);  // (h) enable interrupt 30 in NVIC
  //EnableInterrupts();           // (i) Program 5.3
	/*********************end of modification***************************************/
}

void PortF0_Init(void){ 

  SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF;


  GPIO_PORTF_LOCK_R
= 0x4C4F434B; // unlock GPIO Port F 
	GPIO_PORTF_CR_R
= 0x1F; // allow changes to
	
  GPIO_PORTF_DIR_R &= ~0x01;    // (c) make PF0 in (built-in button)
  GPIO_PORTF_AFSEL_R &= ~0x01;  //     disable alt funct on PF0
  GPIO_PORTF_DEN_R |= 0x01;     //     enable digital I/O on PF0
                                //     configure PF4 as GPIO
  GPIO_PORTF_PCTL_R = (GPIO_PORTF_PCTL_R&0xFFFFFFF0)+0x00000000;
  GPIO_PORTF_AMSEL_R = 0;       //     disable analog functionality on PF
  GPIO_PORTF_PUR_R |= 0x01;     //     enable weak pull-up on PF0
  GPIO_PORTF_IS_R &= ~0x01;     // (d) PF0 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x01;    //     PF0 is not both edges
  GPIO_PORTF_IEV_R &= ~0x01;    //     PF0 falling edge event
  GPIO_PORTF_ICR_R = 0x01;      // (e) clear flag0
  GPIO_PORTF_IM_R |= 0x01;      // (f) arm interrupt on PF0
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00A00000; // (g) priority 5
  NVIC_EN0_R |=(1<<30);  // (h) enable interrupt 30 in NVIC
  //EnableInterrupts();           // (i) Program 5.3
	/*********************end of modification***************************************/
}
// add a background task to run whenever the SW1 (PF4) button is pushed
// Inputs: pointer to a void/void background function
//         priority 0 is the highest, 5 is the lowest
// Outputs: 1 if successful, 0 if this thread can not be added
//Modified by annyan
void(* SW1_task)(void);
unsigned long SW1_priority;
int OS_AddSW1Task(void(*task)(void), unsigned long priority){
  PortF4_Init();
	SW1_task = task;
	SW1_priority = priority;
	return 1;
}
void (* SW2_task)(void);
unsigned long SW2_priority;
int OS_AddSW2Task(void(*task)(void), unsigned long priority){
	PortF0_Init();
	SW2_task = task;
	SW2_priority = priority;
	return 1;
}

unsigned long OS_Id(void){
  return RunPt->Id;
}
//End of Modification
/*******************************************Lab2 part2 modified on Feb 15th****************************************/
//Modified by annyan
void OS_bSignal(Sema4Type *semaPt){
  long status;
	status = StartCritical ();
	if(semaPt->Value >=1)
		semaPt->Value = 1;
	else
	  semaPt->Value ++;
	if (semaPt->Value<=0){
		if (semaPt->first !=NULL){
			struct Blocked_list_elem * cur = semaPt->first;
			cur = cur->Next;
			semaPt->first->Bloker->status = ACTIVE;
			/***************priority donation, modified by annyan***********/
			if(semaPt->holder->is_donated){
				if(semaPt->holder->priority < semaPt->first->Bloker->priority){
					if(semaPt->first->Bloker->is_donated)
						semaPt->first->Bloker->priority = semaPt->holder->priority;
					else{
						semaPt->first->Bloker->is_donated=1;
						semaPt->first->Bloker->priority_temp=semaPt->first->Bloker->priority;
						semaPt->first->Bloker->priority = semaPt->holder->priority;
					}
				}
				semaPt->holder->is_donated = 0;
				semaPt->holder->priority = semaPt->holder->priority_temp;
				semaPt->holder->priority_temp = INVALID_PRIORITY;
			}
			semaPt->holder=semaPt->first->Bloker;
			//end of modification
			free (semaPt->first);
			semaPt->first = cur;
		}
	}
	//using round robin, do not suspend execution
	else{
			/***************priority donation, modified by annyan***********/
			if(semaPt->holder->is_donated){
				semaPt->holder->is_donated = 0;
				semaPt->holder->priority = semaPt->holder->priority_temp;
				semaPt->holder->priority_temp = INVALID_PRIORITY;
			}
			semaPt->holder = NULL;
			//end of modification
		
	}
	EndCritical (status);
}

void OS_bWait(Sema4Type *semaPt){
	long status;
	status = StartCritical ();
	
	semaPt->Value--;//Decrement the semaphore counter

	if (semaPt->Value < 0){

		if (semaPt->first == NULL){

			semaPt->first = (struct Blocked_list_elem *)malloc (sizeof(struct Blocked_list_elem));
			semaPt->first->Bloker = RunPt;
			RunPt->status = BLOCKED;
			semaPt->first->Next = NULL;
			/***********priority donation modified by annyan*****************/
			if(RunPt->priority < semaPt->holder->priority){
				if(semaPt->holder->is_donated)
					semaPt->holder->priority=RunPt->priority;
				else{
					semaPt->holder->is_donated=1;
					semaPt->holder->priority_temp=semaPt->holder->priority;
					semaPt->holder->priority=RunPt->priority;
				}
			}
			//end of modification
			
		}
		else{
			struct Blocked_list_elem * cur = semaPt->first;
		  while (cur->Next!=NULL){
				cur = cur->Next;
			}
		cur->Next = (struct Blocked_list_elem *)malloc (sizeof(struct Blocked_list_elem));
		cur->Next->Bloker = RunPt;
		RunPt->status = BLOCKED;
		cur->Next->Next = NULL;
				/***********priority donation modified by annyan*****************/
		if(RunPt->priority < semaPt->holder->priority){
			if(semaPt->holder->is_donated)
				semaPt->holder->priority=RunPt->priority;
			else{
				semaPt->holder->is_donated=1;
				semaPt->holder->priority_temp=semaPt->holder->priority;
				semaPt->holder->priority=RunPt->priority;
			}
		}
		//end of modification
	
		}

		EndCritical (status);
		OS_Suspend();
	}
	else
		semaPt->holder = RunPt;//for priority donation, modified by annyan
	
	EndCritical (status);
}

void OS_Signal(Sema4Type *semaPt){
	long status;
	Sema4Type * pt;
	status = StartCritical();
	semaPt->Value ++;
	if (semaPt->Value<=0){
		if (semaPt->first !=NULL){
			struct Blocked_list_elem * cur = semaPt->first;
			cur = cur->Next;
			semaPt->first->Bloker->status = ACTIVE;
			/****************timout semaphore, modified by annyan************/
			pt = semaPt->first->Bloker->sema4_blocked;
			if(pt->bloked_next == NULL)
				semaPt->first->Bloker->sema4_blocked = NULL;
			else
				semaPt->first->Bloker->sema4_blocked = pt->bloked_next;
			/***************priority donation, modified by annyan***********/
			if(semaPt->holder->is_donated){
				if(semaPt->holder->priority < semaPt->first->Bloker->priority){
					if(semaPt->first->Bloker->is_donated)
						semaPt->first->Bloker->priority = semaPt->holder->priority;
					else{
						semaPt->first->Bloker->is_donated=1;
						semaPt->first->Bloker->priority_temp=semaPt->first->Bloker->priority;
						semaPt->first->Bloker->priority = semaPt->holder->priority;
					}
				}
				semaPt->holder->is_donated = 0;
				semaPt->holder->priority = semaPt->holder->priority_temp;
				semaPt->holder->priority_temp = INVALID_PRIORITY;
			}
			semaPt->holder=semaPt->first->Bloker;
			//end of modification
			free (semaPt->first);
			semaPt->first = cur;
		}
	}
	else{
			/***************priority donation, modified by annyan***********/
			if(semaPt->holder->is_donated){
				semaPt->holder->is_donated = 0;
				semaPt->holder->priority = semaPt->holder->priority_temp;
				semaPt->holder->priority_temp = INVALID_PRIORITY;
			}
			semaPt->holder = NULL;
			//end of modification
		
	}
	
	//using round robin, do not suspend execution
	EndCritical (status);
}

void OS_Wait(Sema4Type *semaPt){
	long status;
	status = StartCritical();
	semaPt->Value--;//Decrement the semaphore counter
	if (semaPt->Value < 0){
	
		if (semaPt->first == NULL){
			semaPt->first = (struct Blocked_list_elem *)malloc (sizeof(struct Blocked_list_elem));
			semaPt->first->Bloker = RunPt;
			semaPt->first->Bloker->status = BLOCKED;
			semaPt->first->Next = NULL;
			/***********priority donation modified by annyan*****************/
			if(RunPt->priority < semaPt->holder->priority){
				if(semaPt->holder->is_donated)
					semaPt->holder->priority=RunPt->priority;
				else{
					semaPt->holder->is_donated=1;
					semaPt->holder->priority_temp=semaPt->holder->priority;
					semaPt->holder->priority=RunPt->priority;
				}
			}
			//end of modification
		}
		else{
			struct Blocked_list_elem * cur = semaPt->first;
		  while (cur->Next!=NULL){
				cur = cur->Next;
			}
			cur->Next = (struct Blocked_list_elem *)malloc (sizeof(struct Blocked_list_elem));
			cur->Next->Bloker = RunPt;
			cur->Next->Bloker->status = BLOCKED;
			cur->Next->Next = NULL;
			/***********priority donation modified by annyan*****************/
			if(RunPt->priority < semaPt->holder->priority){
				if(semaPt->holder->is_donated)
					semaPt->holder->priority=RunPt->priority;
				else{
					semaPt->holder->is_donated=1;
					semaPt->holder->priority_temp=semaPt->holder->priority;
					semaPt->holder->priority=RunPt->priority;
				}
			}
			//end of modification
		}
		if(RunPt->sema4_blocked == NULL){
			RunPt ->sema4_blocked = semaPt;
			semaPt->bloked_next = NULL;
		}
		else{
			struct Sema4* pt = RunPt->sema4_blocked;
			while (pt->bloked_next != NULL)
				pt = pt->bloked_next;
			pt->bloked_next = RunPt->sema4_blocked;
			semaPt->bloked_next = NULL;
		}
		EndCritical(status);
		OS_Suspend();
	}
	else{
		semaPt->holder = RunPt;//for priority donation, modified by annyan
		semaPt->holding_time = OS_MsTime();
	}
	EndCritical(status);
}
void OS_InitSemaphore(Sema4Type *semaPt, long value){
	static int i=0;
  semaPt->Value = value;

	semaPt->first = NULL; //initialize blocked_list
	semaPt->holder = NULL;
	semaPt->holding_time = 0;
	semaPt->bloked_next = NULL;
	/***********************bank debug***********************/
	bank_init_semaphore (semaPt, i++);
	
	return;
}
//End of Modification
#define NVIC_EN0_INT21          0x00200000  // Interrupt 21 enable
#define TIMER_ICR_TATOCINT      0x00000001  // GPTM TimerA Time-Out Raw
int SW1_flag=0;
int Timer1_count=0;
/**********************************End of Not Implemented*****************************/
//change to 2A
void Timer2A_Init(void){
	long sr = StartCritical(); 
  SYSCTL_RCGC1_R |= 0x000e0000; // 0) activate timer2
  TIMER2_CTL_R &= ~0x00000001;     // 1) disable timer2A during setup
  TIMER2_CFG_R = 0x00000000;       // 2) configure for 32-bit timer mode
  TIMER2_TAMR_R = 0x00000012;      // 3) configure for periodic mode, default down-up settings
  TIMER2_TAILR_R = 80000-1;       // 4) reload value
  TIMER2_ICR_R = 0x00000001;       // 6) clear timer1A timeout flag
  TIMER2_IMR_R |= 0x00000001;      // 7) arm timeout interrupt
  NVIC_PRI5_R = (NVIC_PRI5_R&0x1fffffff)|(0<<29); // 8) priority high or low?
  NVIC_EN0_R = 0x00800000;     // 9) enable interrupt 23 in NVIC
  TIMER2_CTL_R |= 0x00000001;      // 10) enable timer2A
	EndCritical(sr);
}
unsigned long OS_MsTime(void){
  return Timer1_count;
}
void OS_ClearMsTime(void){
	Timer1_count = 0;
}
void GPIOPortF_Handler(void){
	if ((GPIO_PORTF_RIS_R&1)&&(GPIO_PORTF_MIS_R&1)){
		GPIO_PORTF_ICR_R |=0x1; //clear flag;
		(*SW2_task)();
	}
	else if (((GPIO_PORTF_RIS_R>>4)&1)&&((GPIO_PORTF_MIS_R>>4)&1)){
		GPIO_PORTF_ICR_R |=0x10; //clear flag;
		(*SW1_task)();
	}
}

void Timer2A_Handler(void){
	 TCB_Type *pt;
   long status;
	 TIMER2_ICR_R = TIMER_ICR_TATOCINT;// acknowledge timer1A timeout
	 status = StartCritical();
	if ((RunPt !=NULL)&&(RunPt->sleep>0)&&(RunPt->status == ACTIVE))
		RunPt->sleep--;
	pt = RunPt->Next;
	 
	Timer1_count++;
	count_timer2++;
	 while (pt!= RunPt){
		 if (!RunPt)
			 break;
		 if ((pt->sleep>0)&&(pt->status == ACTIVE))
			 pt->sleep --;
		 pt = pt->Next;
	 }
	
	 EndCritical (status);
}
//New scheduler
void SysTick_Handler (void){

	unsigned int priority = 7;//initialized as the lowest one
	
	TCB_Type *pt;
	struct Sema4 * sema_pt;

	/******************for timeout semaphore***************************/
	pt = RunPt;
	while(pt->Next!=RunPt){
		if(pt->status == BLOCKED){
			sema_pt = pt->sema4_blocked;
		  while(sema_pt){
			  if(OS_MsTime() - (sema_pt->holding_time) > TIME_OUT)
				  OS_Signal(sema_pt);
			  sema_pt = sema_pt->bloked_next;
		  }
		}
		pt = pt->Next;
		
	}
	/***************************end of timeout semaphore******************/
	if ((RunPt->sleep == 0) && (RunPt->status == ACTIVE))
		priority = RunPt -> priority;
	pt = RunPt->Next;
		while (pt!=RunPt){
			if ((pt->priority < priority)&&(pt->sleep==0)&&(pt->status==ACTIVE)){
				priority = pt->priority;
			}
			pt = pt->Next;
		}
		pt = RunPt->Next;
		while (1){
			if ((pt->priority == priority)&&(pt->sleep==0)&&(pt->status == ACTIVE)){
				NextPt = pt;
				break;
			}
			pt = pt->Next;
		}
	
	NVIC_INT_CTRL_R = 0x10000000;//trigger PEND_SV handler
}
