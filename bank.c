#include "bank.h"
#include "os.h"
#include <stddef.h>
#include <stdlib.h>

//To find the execution of a program which deadlock can be prevented
struct bank_elem BANK_ALLOC[THREAD_NUM][RESOURCE_NUM]={0,};
struct bank_elem BANK_MAX[THREAD_NUM][RESOURCE_NUM]={0,};
struct bank_elem BANK_NEEDED[THREAD_NUM][RESOURCE_NUM]={0,};
struct resource_elem BANK_AVAIL[RESOURCE_NUM]={0,};
char judge_array[THREAD_NUM]={0,};
char execution_array[THREAD_NUM]={-1,};

void bank_init_tcb (struct TCB* thread, unsigned long thread_num){
	int i;
	for (i=0;i<RESOURCE_NUM;i++){
		BANK_ALLOC[thread_num][i].thread = thread;
		BANK_ALLOC[thread_num][i].number = 0;
		BANK_ALLOC[thread_num][i].semaphore = NULL;	
		
		BANK_MAX[thread_num][i].thread = thread;
		BANK_MAX[thread_num][i].number = 0;
		BANK_MAX[thread_num][i].semaphore = NULL;	
		
		BANK_NEEDED[thread_num][i].thread = thread;
		BANK_NEEDED[thread_num][i].number = 0;
		BANK_NEEDED[thread_num][i].semaphore = NULL;	
	}
	
}

void bank_init_semaphore (struct Sema4* semaphore, unsigned sema_num){
	int i;
	for (i=0; i<THREAD_NUM; i++){
		BANK_ALLOC[i][sema_num].semaphore = semaphore;
		BANK_MAX[i][sema_num].semaphore = semaphore;
		BANK_NEEDED[i][sema_num].semaphore = semaphore;
	}
	BANK_AVAIL[sema_num].number = semaphore->Value;
	BANK_AVAIL[sema_num].semaphore = semaphore;
}
void set_BANKMAX(int row, int num0, int num1, int num2, int num3){
	BANK_MAX[row][0].number = num0;
	BANK_MAX[row][1].number = num1;
	BANK_MAX[row][2].number = num2;
	BANK_MAX[row][3].number = num3;	
}

void get_BANKNEEDED(){
	int i,j;
	char number;
	for (i=0; i<THREAD_NUM; i++){
		for (j=0; j<RESOURCE_NUM; j++){
			number = BANK_MAX[i][j].number - BANK_ALLOC[i][j].number;
			BANK_NEEDED[i][j].number = number;
			number = number;
		}
	}
}

void bank_init(){
	set_BANKMAX(0,1,2,3,4);
	set_BANKMAX(1,2,0,0,0);
	set_BANKMAX(2,0,2,0,0);
	get_BANKNEEDED();
}
int bank_check(){
	char found = 0;
	int i,j;
	for (j=0; j<THREAD_NUM; j++){

		for (i=0; i<THREAD_NUM; i++){
		  found = 0;
			if(!judge_array[i]){
				if(BANK_NEEDED[i][0].number <= BANK_AVAIL[0].number &&
						BANK_NEEDED[i][1].number <= BANK_AVAIL[1].number &&
						BANK_NEEDED[i][2].number <= BANK_AVAIL[2].number &&
						BANK_NEEDED[i][3].number <= BANK_AVAIL[3].number
																																)
				{
					found = 1;
					judge_array[i] = 1;
					BANK_AVAIL[0].number = BANK_AVAIL[0].number + BANK_ALLOC[i][0].number;
					BANK_AVAIL[1].number = BANK_AVAIL[1].number + BANK_ALLOC[i][1].number;
					BANK_AVAIL[2].number = BANK_AVAIL[2].number + BANK_ALLOC[i][2].number;
					BANK_AVAIL[3].number = BANK_AVAIL[3].number + BANK_ALLOC[i][3].number;
					execution_array[j] = i;
					break;
				}
				
			}
		}
		if (!found)
			return -1;//dead lock
	}
	return 0;
}
