// Lab2.c
// Runs on LM4F120/TM4C123
// Real Time Operating System for Labs 2 and 3
// Lab2 Part 1: Testmain1 and Testmain2
// Lab2 Part 2: Testmain3 Testmain4  and main
// Lab3: Testmain5 Testmain6, Testmain7, and main (with SW2)

// Jonathan W. Valvano 1/31/14, valvano@mail.utexas.edu
// EE445M/EE380L.6 
// You may use, edit, run or distribute this file 
// You are free to change the syntax/organization of this file

// LED outputs to logic analyzer for OS profile 
// PF1 is preemptive thread switch
// PF2 is periodic task, samples PD3
// PF3 is SW1 task (touch PF4 button)

// Button inputs
// PF0 is SW2 task (Lab3)
// PF4 is SW1 button input

// Analog inputs
// PD3 Ain3 sampled at 2k, sequencer 3, by DAS software start in ISR
// PD2 Ain5 sampled at 250Hz, sequencer 0, by Producer, timer tigger

#include "os.h"
#include "tm4c123gh6pm.h"
#include "ST7735.h"
#include "ADCT0ATrigger.h"
#include "ADCSWTrigger.h"
#include "UART2.h"
#include <string.h> 
#include "PLL.h"
//#include "SysTick.h"
#include "Timer1A.h"
#include "bank.h"

void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode


//*********Prototype for FFT in cr4_fft_64_stm32.s, STMicroelectronics
void cr4_fft_64_stm32(void *pssOUT, void *pssIN, unsigned short Nbin);
//*********Prototype for PID in PID_stm32.s, STMicroelectronics
short PID_stm32(short Error, short *Coeff);

unsigned long NumCreated;   // number of foreground threads created
unsigned long PIDWork;      // current number of PID calculations finished
unsigned long FilterWork;   // number of digital filter calculations finished
unsigned long NumSamples;   // incremented every ADC sample, in Producer
#define FS 400            // producer/consumer sampling
#define RUNLENGTH (20*FS) // display results and quit when NumSamples==RUNLENGTH
// 20-sec finite time experiment duration 

#define PERIOD TIME_500US // DAS 2kHz sampling period in system time units
long x[64],y[64];         // input and output arrays for FFT

//---------------------User debugging-----------------------
unsigned long DataLost;     // data sent by Producer, but not received by Consumer
/*long MaxJitter;             // largest time jitter between interrupts in usec
#define JITTERSIZE 64
unsigned long const JitterSize=JITTERSIZE;
unsigned long JitterHistogram[JITTERSIZE]={0,};*/
#define PE0  (*((volatile unsigned long *)0x40024004))
#define PE1  (*((volatile unsigned long *)0x40024008))
#define PE2  (*((volatile unsigned long *)0x40024010))
#define PE3  (*((volatile unsigned long *)0x40024020))




void PortE_Init(void){ unsigned long volatile delay;
  SYSCTL_RCGC2_R |= 0x10;       // activate port E
  delay = SYSCTL_RCGC2_R;        
  delay = SYSCTL_RCGC2_R;         
  GPIO_PORTE_DIR_R |= 0x0F;    // make PE3-0 output heartbeats
  GPIO_PORTE_AFSEL_R &= ~0x0F;   // disable alt funct on PE3-0
  GPIO_PORTE_DEN_R |= 0x0F;     // enable digital I/O on PE3-0
  GPIO_PORTE_PCTL_R = ~0x0000FFFF;
  GPIO_PORTE_AMSEL_R &= ~0x0F;;      // disable analog functionality on PF
}
//Lab1 prelab: Fixed bandwidth
//------------------Task 1--------------------------------
// 2 kHz sampling ADC channel 1, using software start trigger
// background thread executed at 2 kHz
// 60-Hz notch high-Q, IIR filter, assuming fs=2000 Hz
// y(n) = (256x(n) -503x(n-1) + 256x(n-2) + 498y(n-1)-251y(n-2))/256 (2k sampling)
// y(n) = (256x(n) -476x(n-1) + 256x(n-2) + 471y(n-1)-251y(n-2))/256 (1k sampling)
long Filter(long data){
static long x[6]; // this MACQ needs twice
static long y[6];
static unsigned long n=3;   // 3, 4, or 5
  n++;
  if(n==6) n=3;     
  x[n] = x[n-3] = data;  // two copies of new data
  y[n] = (256*(x[n]+x[n-2])-503*x[n-1]+498*y[n-1]-251*y[n-2]+128)/256;
  y[n-3] = y[n];         // two copies of filter outputs too
  return y[n];
} 


//******** DAS *************** 
// background thread, calculates 60Hz notch filter
// runs 2000 times/sec
// samples channel 4, PD3,
// inputs:  none
// outputs: none
unsigned long DASoutput;
void DAS(void){ 
unsigned long input;  
//unsigned static long LastTime;  // time at previous ADC sample
//unsigned long thisTime;         // time at current ADC sample
//long jitter;                    // time between measured and expected, in us
 

	if(NumSamples < RUNLENGTH){   // finite time run
    PE0 ^= 0x01;
	  input = ADC_In();           // channel set when calling ADC_Init
    PE0 ^= 0x01;
  
		/*UART_OutChar(input/1000+'0');
		UART_OutChar((input%1000)/100+'0');
		UART_OutChar((input%100)/10+'0');
		UART_OutChar(input%10+'0');*/
		
		
		jitterSetThisTime(0);//thisTime = OS_Time();       // current time, 12.5 ns
    DASoutput = Filter(input);
    FilterWork++;        // calculation finished
    if(FilterWork>1){    // ignore timing of first interrupt
      /*unsigned long diff = OS_TimeDifference(LastTime,thisTime);
      if(diff>PERIOD){
        jitter = (diff-PERIOD+4)/8;  // in 0.1 usec
      }else{
        jitter = (PERIOD-diff+4)/8;  // in 0.1 usec
      }
      if(jitter > MaxJitter){
        MaxJitter = jitter; // in usec
      }       // jitter should be 0
      if(jitter >= JitterSize){
        jitter = JITTERSIZE-1;
      }
      JitterHistogram[jitter]++; */
			jitterCalc(0, PERIOD);
    }
		
    jitterSetLastTime(0);//LastTime = thisTime;
		
    PE0 ^= 0x01;
  }
}


//--------------end of Task 1-----------------------------
//Lab1 prelab: I/O bound
//------------------Task 2--------------------------------
// background thread executes with SW1 button
// one foreground task created with button push
// foreground treads run for 2 sec and die
// ***********ButtonWork*************
void ButtonWork(void){
unsigned long myId = OS_Id(); 
  PE1 ^= 0x02;
  ST7735_Message(1,0,"NumCreated =",NumCreated); 
  PE1 ^= 0x02;
  OS_Sleep(50);     // set this to sleep for 50msec
  ST7735_Message(1,1,"PIDWork     =",PIDWork);
  ST7735_Message(1,2,"DataLost    =",DataLost);
  ST7735_Message(1,3,"Jitter 0.1us=",maxJitter(0));

  PE1 ^= 0x02;
  OS_Kill();  // done, OS does not return from a Kill
} 

//************SW1Push*************
// Called when SW1 Button pushed
// Adds another foreground task
// background threads execute once and return
void SW1Push(void){
  if(OS_MsTime() > 20){ // debounce
    if(OS_AddThread(&ButtonWork,100,3)){
      NumCreated++; 
    }
		UART_OutChar('4');
    OS_ClearMsTime();  // at least 20ms between touches
  }
}
//************SW2Push*************
// Called when SW2 Button pushed, Lab 3 only
// Adds another foreground task
// background threads execute once and return
void SW2Push(void){
  if(OS_MsTime() > 20){ // debounce
    if(OS_AddThread(&ButtonWork,100,3)){
      NumCreated++; 
    }
		UART_OutChar('0');
    OS_ClearMsTime();  // at least 20ms between touches
  }
}
//--------------end of Task 2-----------------------------
//Lab2 prelab: Fixed bandwidth
//------------------Task 3--------------------------------
// hardware timer-triggered ADC sampling at 400Hz
// Producer runs as part of ADC ISR
// Producer uses fifo to transmit 400 samples/sec to Consumer
// every 64 samples, Consumer calculates FFT
// every 2.5ms*64 = 160 ms (6.25 Hz), consumer sends data to Display via mailbox
// Display thread updates LCD with measurement

//******** Producer *************** 
// The Producer in this lab will be called from your ADC ISR
// A timer runs at 400Hz, started by your ADC_Collect
// The timer triggers the ADC, creating the 400Hz sampling
// Your ADC ISR runs when ADC data is ready
// Your ADC ISR calls this function with a 12-bit sample 
// sends data to the consumer, runs periodically at 400Hz
// inputs:  none
// outputs: none
void Producer(unsigned long data){  
  if(NumSamples < RUNLENGTH){   // finite time run
    NumSamples++;               // number of samples
		//UART_OutChar('5');
		//ST7735_Message (1, 0, "data:", data);
    if(OS_Fifo_Put(data) == 0){ // send to consumer
			
      DataLost++;
			//UART_OutChar(DataLost+'0');
    } 
		//UART_OutChar(data/1000+'0');
		//UART_OutChar((data%1000)/100+'0');
		//  UART_OutChar((data%100)/10+'0');
		//  UART_OutChar(data%10+'0');
  } 
}
void Display(void); 

//******** Consumer *************** 
// foreground thread, accepts data from producer
// calculates FFT, sends DC component to Display
// inputs:  none
// outputs: none
void Consumer(void){ 
unsigned long data,DCcomponent;   // 12-bit raw ADC sample, 0 to 4095
unsigned long t;                  // time in 2.5 ms
unsigned long myId = OS_Id(); 
	
  ADC_Collect(5, FS, &Producer); // start ADC sampling, channel 5, PD2, 400 Hz
  NumCreated += OS_AddThread(&Display,128,3); 
  while(NumSamples < RUNLENGTH) { 
    PE2 = 0x04;
    for(t = 0; t < 64; t++){   // collect 64 ADC samples
      data = OS_Fifo_Get();    // get from producer
      x[t] = data;             // real part is 0 to 4095, imaginary part is 0
			/*UART_OutChar('|');
			UART_OutChar(data/1000+'0');
			UART_OutChar((data%1000)/100+'0');
			UART_OutChar((data%100)/10+'0');
			UART_OutChar(data%10+'0');*/
    }
    PE2 = 0x00;
		//ST7735_Message (1, 6, "DCdata:", data);// for debugging, zw
    cr4_fft_64_stm32(y,x,64);  // complex FFT of last 64 ADC values
    DCcomponent = y[0]&0xFFFF; // Real part at frequency 0, imaginary part should be zero
    OS_MailBox_Send(DCcomponent); // called every 2.5ms*64 = 160ms
		//ST7735_Message (1, 7, "DCcomp:", DCcomponent);// for debugging, zw
		//OS_MailBox_Send(3091); // for debugging, zw
		//UART_OutChar('t');
  }
  OS_Kill();  // done
}
//******** Display *************** 
// foreground thread, accepts data from consumer
// displays calculated results on the LCD
// inputs:  none                            
// outputs: none
void Display(void){ 
unsigned long data,voltage;
  ST7735_Message(0,1,"Run length = ",(RUNLENGTH)/FS);   // top half used for Display
	//UART_OutChar('s');
  while(NumSamples < RUNLENGTH) {
    //UART_OutChar('r');		
    data = OS_MailBox_Recv();
		//ST7735_Message (1, 8, "Disdata:", data);
		//UART_OutChar('1');
    voltage = 3000*data/4095;               // calibrate your device so voltage is in mV
    PE3 = 0x08;
    ST7735_Message(0,2,"v(mV) =",voltage);
    //UART_OutChar(voltage%10+'0');
		//UART_OutChar(data/1000+'0');
		//UART_OutChar((data%1000)/100+'0');
		//UART_OutChar((data%100)/10+'0');
		//UART_OutChar(data%10+'0');
    PE3 = 0x00;
  } 
  OS_Kill();  // done
} 

//--------------end of Task 3-----------------------------
//Lab2 prelab: CPU bound
//------------------Task 4--------------------------------
// foreground thread that runs without waiting or sleeping
// it executes a digital controller 
//******** PID *************** 
// foreground thread, runs a PID controller
// never blocks, never sleeps, never dies
// inputs:  none
// outputs: none
short IntTerm;     // accumulated error, RPM-sec
short PrevError;   // previous error, RPM
short Coeff[3];    // PID coefficients
short Actuator;
void PID(void){ 
short err;  // speed error, range -100 to 100 RPM
unsigned long myId = OS_Id(); 
  PIDWork = 0;
  IntTerm = 0;
  PrevError = 0;
  Coeff[0] = 384;   // 1.5 = 384/256 proportional coefficient
  Coeff[1] = 128;   // 0.5 = 128/256 integral coefficient
  Coeff[2] = 64;    // 0.25 = 64/256 derivative coefficient*
  while(NumSamples < RUNLENGTH) { 
    for(err = -1000; err <= 1000; err++){    // made-up data
      Actuator = PID_stm32(err,Coeff)/256;
			//UART_OutChar('P');
    }
    PIDWork++;        // calculation finished
  }
  for(;;){ }          // done
}
//--------------end of Task 4-----------------------------
//lab2 prelab: I/O bound
//------------------Task 5--------------------------------
// UART background ISR performs serial input/output
// Two software fifos are used to pass I/O data to foreground
// The interpreter runs as a foreground thread
// The UART driver should call OS_Wait(&RxDataAvailable) when foreground tries to receive
// The UART ISR should call OS_Signal(&RxDataAvailable) when it receives data from Rx
// Similarly, the transmit channel waits on a semaphore in the foreground
// and the UART ISR signals this semaphore (TxRoomLeft) when getting data from fifo
// Modify your intepreter from Lab 1, adding commands to help debug 
// Interpreter is a foreground thread, accepts input from serial port, outputs to serial port
// inputs:  none
// outputs: none
void Interpreter(void){
	char cmdString[8]; // global to assist in debugging
	short cmdCount=0;
	//OutCRLF();
	while (1){
		UART_OutString("Start Debugging.");
		UART_OutString("1:Histo,2:OST.");
		UART_InString(cmdString, 8);
		//OutCRLF();
		UART_OutString(cmdString);
		
		if (strcmp(cmdString,"1")==0){
			ST7735_Message (1, 8, "Histogram:", jitterHistogram(maxJitter(0), 0));
			cmdCount++;
		}
		else if (strcmp(cmdString,"2")==0){
			ST7735_Message (1, 8, "OSTime:", OS_Time());
			cmdCount++;
		}
		else UART_OutString("Wrong Command.");
	}
}  // just a prototype, link to your interpreter
// add the following commands, leave other commands, if they make sense
// 1) print performance measures 
//    time-jitter, number of data points lost, number of calculations performed
//    i.e., NumSamples, NumCreated, MaxJitter, DataLost, FilterWork, PIDwork
      
// 2) print debugging parameters 
//    i.e., x[], y[] 
//--------------end of Task 5-----------------------------



//modification starts, for testing DAS, zw
void Thread1ZW(void){       
  for(;;){
    PE1 ^= 0x02;       // heartbeat
		//UART_OutChar('a');
  }
}
void Thread2ZW(void){        
  for(;;){
    PE3 ^= 0x04;       // heartbeat
		UART_OutChar('c');
  }
}
//modification ends, for testing DAS, zw

//*******************final user main DEMONTRATE THIS TO TA**********
int main0(void){
  PLL_Init();
  UART_Init();	
	ADC_Init(4);  // sequencer 3, channel 4, PD3, sampling in DAS(), software trigger
	UART_OutChar('d');
  ST7735_InitR(INITR_REDTAB);
	ST7735_FillScreen(0);                 // set screen to black
  OS_Init();           // initialize, disable interrupts
  PortE_Init();
	
  DataLost = 0;        // lost data between producer and consumer
  NumSamples = 0;
  //MaxJitter = 0;       // in 1us units

 
//********initialize communication channels
  OS_MailBox_Init();
  OS_Fifo_Init(128);    // ***note*** 4 is not big enough*****

//*******attach background tasks***********
  OS_AddSW1Task(&SW1Push,2);
  OS_AddSW2Task(&SW2Push,2);  // add this line in Lab 3
  

  NumCreated = 0 ;
	OS_AddPeriodicThread(&DAS,PERIOD,3); // 2 kHz real time sampling of PD3
  // create initial foreground threads
 
  NumCreated += OS_AddThread(&Consumer,128,3); 
  NumCreated += OS_AddThread(&PID,128,3);  // Lab 3, make this lowest priority
	 NumCreated += OS_AddThread(&Interpreter,128,3); 
  OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}

//+++++++++++++++++++++++++DEBUGGING CODE++++++++++++++++++++++++
// ONCE YOUR RTOS WORKS YOU CAN COMMENT OUT THE REMAINING CODE
// 
//*******************Initial TEST**********
// This is the simplest configuration, test this first, (Lab 1 part 1)
// run this with 
// no UART interrupts
// no SYSTICK interrupts
// no timer interrupts
// no switch interrupts
// no ADC serial port or LCD output
// no calls to semaphores
unsigned long Count1;   // number of times thread1 loops
unsigned long Count2;   // number of times thread2 loops
unsigned long Count3;   // number of times thread3 loops
unsigned long Count4;   // number of times thread4 loops
unsigned long Count5;   // number of times thread5 loops
void Thread1(void){
  Count1 = 0;          
  for(;;){
    PE0 ^= 0x01;       // heartbeat
    Count1++;
		UART_OutChar('a');
    OS_Suspend();      // cooperative multitasking
  }
}
void Thread2(void){
  Count2 = 0;          
  for(;;){
    PE1 ^= 0x02;       // heartbeat
    Count2++;
		UART_OutChar('b');
    OS_Suspend();      // cooperative multitasking
  }
}
void Thread3(void){
  Count3 = 0;          
  for(;;){
    PE2 ^= 0x04;       // heartbeat
    Count3++;
		UART_OutChar('c');
    OS_Suspend();      // cooperative multitasking
  }
}

int Testmain1(void){  // Testmain1
	
	PLL_Init();
	UART_Init();
	UART_OutChar('d');
  OS_Init();          // initialize, disable interrupts
  PortE_Init();       // profile user threads
  NumCreated = 0 ;
  NumCreated += OS_AddThread(&Thread1,128,1); 
  NumCreated += OS_AddThread(&Thread2,128,2); 
  NumCreated += OS_AddThread(&Thread3,128,3); 
  // Count1 Count2 Count3 should be equal or off by one at all times
  OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}

//*******************Second TEST**********
// Once the initalize test runs, test this (Lab 1 part 1)
// no UART interrupts
// SYSTICK interrupts, with or without period established by OS_Launch
// no timer interrupts
// no switch interrupts
// no ADC serial port or LCD output
// no calls to semaphores
void Thread1b(void){
  Count1 = 0;          
  for(;;){
    PE0 ^= 0x01;       // heartbeat
    Count1++;
		UART_OutChar('1');
  }
}
void Thread2b(void){
  Count2 = 0;          
  for(;;){
    PE1 ^= 0x02;       // heartbeat
    Count2++;
		UART_OutChar('2');
  }
}
void Thread3b(void){
  Count3 = 0;          
  for(;;){
    PE2 ^= 0x04;       // heartbeat
    Count3++;
		UART_OutChar('3');
  }
}
int Testmain2(void){  // Testmain2
  PLL_Init();
	UART_Init();
	
	UART_OutChar('d');
  OS_Init();           // initialize, disable interrupts
  PortE_Init();       // profile user threads
  NumCreated = 0 ;
  NumCreated += OS_AddThread(&Thread1b,128,1); 
  NumCreated += OS_AddThread(&Thread2b,128,1); 
  NumCreated += OS_AddThread(&Thread3b,128,1); 
  // Count1 Count2 Count3 should be equal on average
  // counts are larger than testmain1
 
  OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}

//*******************Third TEST**********
// Once the second test runs, test this (Lab 1 part 2)
// no UART1 interrupts
// SYSTICK interrupts, with or without period established by OS_Launch
// Timer interrupts, with or without period established by OS_AddPeriodicThread
// PortF GPIO interrupts, active low
// no ADC serial port or LCD output
// tests the spinlock semaphores, tests Sleep and Kill
Sema4Type Readyc;        // set in background
int Lost;
void BackgroundThread1c(void){   // called at 1000 Hz
  Count1++;
  OS_Signal(&Readyc);
}
void Thread5c(void){
  for(;;){
    OS_Wait(&Readyc);
    Count5++;   // Count2 + Count5 should equal Count1 
    Lost = Count1-Count5-Count2;
  }
}

void Thread2c(void){
  OS_InitSemaphore(&Readyc,0);
  Count1 = 0;    // number of times signal is called      
  Count2 = 0;    
  Count5 = 0;    // Count2 + Count5 should equal Count1  
  NumCreated += OS_AddThread(&Thread5c,128,3); 
 OS_AddPeriodicThread(&BackgroundThread1c,TIME_1MS,0);

  for(;;){
    OS_Wait(&Readyc);
		UART_OutChar('s');
    Count2++;   // Count2 + Count5 should equal Count1
  }
}

void Thread3c(void){
  Count3 = 0;          
  for(;;){
    Count3++;
  }
}
void Thread4c(void){ int i;
  for(i=0;i<64;i++){
    Count4++;
		PE0 ^= 0x01;
    OS_Sleep(10);
		//PE0 ^= 0x01;
		//UART_OutChar('w');
  }
  OS_Kill();
  Count4 = 0;
}
void BackgroundThread5c(void){   // called when Select button pushed
  NumCreated += OS_AddThread(&Thread4c,128,3); 
}
      
int Testmain3(void){   // Testmain3
  Count4 = 0;     
  PLL_Init();	
	UART_Init();
  PortE_Init();
  UART_OutChar('a');	
  OS_Init();   // initialize, disable interrupts
	
// Count2 + Count5 should equal Count1
  NumCreated = 0 ;
  OS_AddSW1Task(&BackgroundThread5c,2); 
  NumCreated += OS_AddThread(&Thread2c,128,2); 
  NumCreated += OS_AddThread(&Thread3c,128,3); 
  NumCreated += OS_AddThread(&Thread4c,128,3);
  OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}

//*******************Fourth TEST**********
// Once the third test runs, run this example (Lab 1 part 2)
// Count1 should exactly equal Count2
// Count3 should be very large
// Count4 increases by 640 every time select is pressed
// NumCreated increase by 1 every time select is pressed

// no UART interrupts
// SYSTICK interrupts, with or without period established by OS_Launch
// Timer interrupts, with or without period established by OS_AddPeriodicThread
// Select switch interrupts, active low
// no ADC serial port or LCD output
// tests the spinlock semaphores, tests Sleep and Kill
Sema4Type Readyd;        // set in background
void BackgroundThread1d(void){   // called at 1000 Hz
static int i=0;
  i++;
  if(i==50){
    i = 0;         //every 50 ms
    Count1++;
    OS_bSignal(&Readyd);
  }
}
void Thread2d(void){
	//UART_OutChar('b');
  OS_InitSemaphore(&Readyd,0);
	//UART_OutChar('s');
  Count1 = 0;          
  Count2 = 0;          
  for(;;){
    OS_bWait(&Readyd);
	  UART_OutChar('w');	
    Count2++;     
  }
}
void Thread3d(void){
  Count3 = 0;          
  for(;;){
    Count3++;
		UART_OutChar('3');
  }
}
void Thread4d(void){ int i;
  for(i=0;i<640;i++){
    Count4++;
    OS_Sleep(1);
		UART_OutChar('h');
  }
	UART_OutChar((Count4%1000)/100+'0');
  OS_Kill();
}
void BackgroundThread5d(void){   // called when Select button pushed
  NumCreated += OS_AddThread(&Thread4d,128,3); 
}
int Testmain4(void){   // Testmain4
  Count4 = 0;       
  PLL_Init();
  UART_Init();
  UART_OutChar('a');	
  OS_Init();           // initialize, disable interrupts
  NumCreated = 0 ;
  OS_AddPeriodicThread(&BackgroundThread1d,PERIOD,0); 
  OS_AddSW1Task(&BackgroundThread5d,2);
  NumCreated += OS_AddThread(&Thread2d,128,2); 
  NumCreated += OS_AddThread(&Thread3d,128,3); 
  NumCreated += OS_AddThread(&Thread4d,128,3); 
	
  OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}

//******************* Lab 3 Preparation 2**********
// Modify this so it runs with your RTOS (i.e., fix the time units to match your OS)
// run this with 
// UART0, 115200 baud rate, used to output results 
// SYSTICK interrupts, period established by OS_Launch
// first timer interrupts, period established by first call to OS_AddPeriodicThread
// second timer interrupts, period established by second call to OS_AddPeriodicThread
// SW1 no interrupts
// SW2 no interrupts
unsigned long CountA;   // number of times Task A called
unsigned long CountB;   // number of times Task B called
unsigned long Count1;   // number of times thread1 loops


//*******PseudoWork*************
// simple time delay, simulates user program doing real work
// Input: amount of work in 100ns units (free free to change units
// Output: none
void PseudoWork(unsigned short work){
unsigned short startTime;
  startTime = OS_Time();    // time in 100ns units
  while(OS_TimeDifference(startTime,OS_Time()) <= work){} 
}
void Thread6(void){  // foreground thread
  Count1 = 0;          
  for(;;){
    Count1++; 
    PE0 ^= 0x01;        // debugging toggle bit 0  
  }
}
extern void Jitter(void){ // prints jitter information (write this), not Implemented annyan
	
	UART_OutString("MaxJitter1:");
	UART_OutUDec(maxJitter(1));
	//OutCRLF();
	UART_OutString("Histogram1:");
	UART_OutUDec(jitterHistogram(maxJitter(1), 1));
	//OutCRLF();
	UART_OutString("MaxJitter2:");
	UART_OutUDec(maxJitter(1));
	//OutCRLF();
	UART_OutString("Histogram2:");
	UART_OutUDec(jitterHistogram(maxJitter(1), 1));
	//UART_OutCRLF();
	return;
}
void Thread7(void){  // foreground thread
  UART_OutString("\n\rEE345M/EE380L, Lab 3 Preparation 2\n\r");
  OS_Sleep(5000);   // 10 seconds        
  Jitter();         // print jitter information
  UART_OutString("\n\r\n\r");
  OS_Kill();
}

unsigned long jitterCount1=0;
unsigned long jitterCount2=0;
#define PERIODA 2.99*TIME_1MS
#define workA 500       // {5,50,500 us} work in Task A
#define counts1us 10    // number of OS_Time counts per 1us
void TaskA(void){       // called every {1000, 2990us} in background
  PE1 = 0x02;      // debugging profile
	jitterSetThisTime(1);//zw
	if (jitterCount1>0){
		jitterCount1++;
		jitterCalc(1,PERIODA);
		//UART_OutUDec(jitterCalc(1,PERIODA));//zw
	}
  CountA++;
  PseudoWork(workA*counts1us); //  do work (100ns time resolution)
	jitterSetLastTime(1);//zw
  PE1 = 0x00;      // debugging profile  
}
#define workB 250       // 250 us work in Task B
void TaskB(void){       // called every pB in background
  PE2 = 0x04;      // debugging profile  
	jitterSetThisTime(2);//zw
	if (jitterCount2>0){
		jitterCount2++;
		jitterCalc(2,PERIODA);
		//UART_OutUDec(jitterCalc(2,PERIODA));//zw
  }
	CountB++;
  PseudoWork(workB*counts1us); //  do work (100ns time resolution)
	jitterSetLastTime(2);//zw
  PE2 = 0x00;      // debugging profile  
}

int Testmain5(void){       // Testmain5 Lab 3
  PLL_Init();
	UART_Init();
  UART_OutChar('a');
	PortE_Init();
  OS_Init();           // initialize, disable interrupts
  NumCreated = 0 ;
  NumCreated += OS_AddThread(&Thread6,128,2); 
  NumCreated += OS_AddThread(&Thread7,128,1); 
  OS_AddPeriodicThread(&TaskA,2.99*TIME_1MS,0);           // 1 ms, higher priority
  OS_AddPeriodicThread(&TaskB,2*TIME_1MS,1);         // 2 ms, lower priority
 
  OS_Launch(TIME_2MS); // 2ms, doesn't return, interrupts enabled in here
  return 0;             // this never executes
}


//******************* Lab 3 Preparation 4**********
// Modify this so it runs with your RTOS used to test blocking semaphores
// run this with 
// UART0, 115200 baud rate,  used to output results 
// SYSTICK interrupts, period established by OS_Launch
// first timer interrupts, period established by first call to OS_AddPeriodicThread
// second timer interrupts, period established by second call to OS_AddPeriodicThread
// SW1 no interrupts, 
// SW2 no interrupts
Sema4Type s;            // test of this counting semaphore
unsigned long SignalCount1;   // number of times s is signaled
unsigned long SignalCount2;   // number of times s is signaled
unsigned long SignalCount3;   // number of times s is signaled
unsigned long WaitCount1;     // number of times s is successfully waited on
unsigned long WaitCount2;     // number of times s is successfully waited on
unsigned long WaitCount3;     // number of times s is successfully waited on
#define MAXCOUNT 20000
void OutputThread(void){  // foreground thread
//	long status ;
	//long sum;
//	char sumc;
	//UART_OutChar('t');
  //UART_OutString("\n\rEE345M/EE380L, Lab 3 Preparation 4\n\r");
  while(SignalCount1+SignalCount2+SignalCount3<100*MAXCOUNT){
    OS_Sleep(1000);   // 1 second
		//status = StartCritical();
      UART_OutChar('.');
		//EndCritical (status);
  }       
  UART_OutString(" done\n\r");
  UART_OutString("Signalled="); 
	UART_OutUDec(SignalCount1+SignalCount2+SignalCount3);
  UART_OutString(", Waited="); 
	UART_OutUDec(WaitCount1+WaitCount2+WaitCount3);
  UART_OutString("\n\r");
//	sum=SignalCount1+SignalCount2+SignalCount3;
//	sum=sum%10;
//	sumc=(char) sum+'0';
//	status = StartCritical();
	//UART_OutChar('.');
	//UART_OutString(" done\n\r");
  //UART_OutString("Signalled="); 
	
	//UART_OutUDec(SignalCount1+SignalCount2+SignalCount3);
  //UART_OutString(", Waited="); 
	//UART_OutChar('.');
	//UART_OutChar(sumc);
	//UART_OutUDec(WaitCount1+WaitCount2+WaitCount3);
  //UART_OutString("\n\r");
	//EndCritical (status);
  OS_Kill();
}
void Wait1(void){  // foreground thread
  for(;;){
    OS_Wait(&s);    // three threads waiting
   // UART_OutChar('1');
		WaitCount1++; 
  }
}
void Wait2(void){  // foreground thread
  for(;;){
    OS_Wait(&s);    // three threads waiting
		//UART_OutChar('2');
    WaitCount2++; 
  }
}
void Wait3(void){   // foreground thread
  for(;;){
    OS_Wait(&s);    // three threads waiting
		//UART_OutChar('3');
    WaitCount3++; 
  }
}
void Signal1(void){      // called every 799us in background
  if(SignalCount1<MAXCOUNT){
    OS_Signal(&s);
		//UART_OutChar('A');
    SignalCount1++;
  }
}
// edit this so it changes the periodic rate
void Signal2(void){       // called every 1111us in background
	//UART_OutChar('b');
  if(SignalCount2<MAXCOUNT){
    OS_Signal(&s);
		//UART_OutChar('B');
    SignalCount2++;
  }
}
void Signal3(void){       // foreground
  while(SignalCount3<98*MAXCOUNT){
    OS_Signal(&s);
    SignalCount3++;
  }
  OS_Kill();
}

long add(const long n, const long m){
static long result;
  result = m+n;
  return result;
}
int Testmain6(void){      // Testmain6
  volatile unsigned long delay;
  PLL_Init();
	UART_Init();
  UART_OutChar('a');
	
	OS_Init();           // initialize, disable interrupts
  delay = add(3,4);
  PortE_Init();
  SignalCount1 = 0;   // number of times s is signaled
  SignalCount2 = 0;   // number of times s is signaled
  SignalCount3 = 0;   // number of times s is signaled
  WaitCount1 = 0;     // number of times s is successfully waited on
  WaitCount2 = 0;     // number of times s is successfully waited on
  WaitCount3 = 0;	  // number of times s is successfully waited on
  OS_InitSemaphore(&s,0);	 // this is the test semaphore
  OS_AddPeriodicThread(&Signal1,(799*TIME_1MS)/1000,0);   // 0.799 ms, higher priority
  //OS_AddPeriodicThread2(&Signal2,(1111*TIME_1MS)/1000,1);  // 1.111 ms, lower priority
  NumCreated = 0 ;
  NumCreated += OS_AddThread(&Thread6,128,6);    	// idle thread to keep from crashing
  NumCreated += OS_AddThread(&OutputThread,128,2); 	// results output thread
  NumCreated += OS_AddThread(&Signal3,128,2); 	// signalling thread
  NumCreated += OS_AddThread(&Wait1,128,2); 	// waiting thread
  NumCreated += OS_AddThread(&Wait2,128,2); 	// waiting thread
  NumCreated += OS_AddThread(&Wait3,128,2); 	// waiting thread
 
  OS_Launch(TIME_1MS);  // 1ms, doesn't return, interrupts enabled in here
  return 0;             // this never executes
}


//******************* Lab 3 Measurement of context switch time**********
// Run this to measure the time it takes to perform a task switch
// UART0 not needed 
// SYSTICK interrupts, period established by OS_Launch
// first timer not needed
// second timer not needed
// SW1 not needed, 
// SW2 not needed
// logic analyzer on PF1 for systick interrupt (in your OS)
//                on PE0 to measure context switch time
void Thread8(void){       // only thread running
  while(1){
    PE0 ^= 0x01;      // debugging profile  
		//UART_OutChar('1');
  }
}

int Testmain7(void){       // Testmain7
  UART_Init();
	PLL_Init();
	//UART_OutChar('d');
  PortE_Init();
  OS_Init();           // initialize, disable interrupts
  NumCreated = 0 ;
  NumCreated += OS_AddThread(&Thread8,128,2); 
  OS_Launch(TIME_1MS/10); // 100us, doesn't return, interrupts enabled in here TIME_1MS/10
  return 0;             // this never executes
}
void dummy(){
	while(1);	
}

void annyan2 (){
	UART_OutChar('1');
	//OutCRLF();
}

int Testbank(){
	int dead_lock;
	Sema4Type s0; 
	Sema4Type s1; 
	Sema4Type s2; 
	Sema4Type s3; 
	
	PLL_Init();
	UART_Init();
	OS_Init();  
	
	NumCreated = 0 ;
	NumCreated += OS_AddThread(&dummy,128,2);
	NumCreated += OS_AddThread(&dummy,128,2);
	NumCreated += OS_AddThread(&dummy,128,2);

	OS_InitSemaphore(&s0, 4);
	OS_InitSemaphore(&s1, 4);
	OS_InitSemaphore(&s2, 4);
	OS_InitSemaphore(&s3, 4);
	bank_init();
	dead_lock=bank_check();
	OS_Launch(TIME_1MS/10); // 100us, doesn't return, interrupts enabled in here TIME_1MS/10
  return 0;
}
/***********************for testing priority donation*****************/
Sema4Type testmain1;
void TestMain1_thread1(void);
void TestMain1_thread2(void);
void TestMain1_thread1() {
	int count = 0;
	NumCreated += OS_AddThread(&TestMain1_thread2,128,2);
	OS_Wait(&testmain1);
	while (count < 65576)
		count++;
	OS_Signal(&testmain1);
	OS_Kill();
}
void TestMain1_thread2(){
	int count = 0;
	OS_Wait(&testmain1);
	while (count < 65576)
		count++;
	OS_Signal(&testmain1);
	OS_Kill();
	
}
int Priority_donation_test(){
	PLL_Init();
	OS_Init();
	OS_InitSemaphore (&testmain1, 1);
	NumCreated = 0 ;
	NumCreated += OS_AddThread(&TestMain1_thread1,128,3);
	NumCreated += OS_AddThread(&dummy,128,3);
	OS_Launch(TIME_1MS/10); 
	return 0;
}
/***********************end of test****************************/
/***********************for timeout semaphore test*****************/
Sema4Type testmain2;
void TestMain2_thread1(void);
void TestMain2_thread2(void);
void TestMain2_thread1() {
	int count = 0;
	NumCreated += OS_AddThread(&TestMain2_thread2,128,2);
	OS_Wait(&testmain1);
	while (count < 65576)
		count++;
	OS_Kill();
}
void TestMain2_thread2(){
	int count = 0;
	OS_Wait(&testmain1);
	while (count < 65576)
		count++;
	OS_Signal(&testmain1);
	OS_Kill();
	
}

int timeout_semaphore(){
	PLL_Init();
	OS_Init();
	OS_InitSemaphore (&testmain1, 1);
	NumCreated = 0 ;
	NumCreated += OS_AddThread(&TestMain2_thread1,128,3);
	NumCreated += OS_AddThread(&dummy,128,3);
	OS_Launch(TIME_1MS/10); 
	return 0;
}
/***********************end of test**********************/
/**********************for mutilple periodic thread test***************/
void period1(){
	UART_OutChar ('1');
}
void period2(){
	UART_OutChar ('2');
}
void period3(){
	UART_OutChar ('3');
}
void period4(){
	UART_OutChar ('4');
}
void period5(){
	UART_OutChar ('5');
}

int main(){
	PLL_Init();
	UART_Init();
	UART_OutChar('s');
	OS_Init();
	OS_AddPeriodicThread(&period1, 
   100, 1);
	OS_AddPeriodicThread(&period2, 
   200, 1);
	OS_AddPeriodicThread(&period3, 
   300, 1);
	OS_AddPeriodicThread(&period4, 
   400, 1);
	OS_AddPeriodicThread(&period5, 
   500, 1);	
	NumCreated = 0 ;
	NumCreated += OS_AddThread(&dummy,128,3);
	OS_Launch(TIME_1MS/10); 
	return 0;
}
