#ifndef __BANK_H__
#define __BANK_H__
#define THREAD_NUM 3
#define RESOURCE_NUM 4
struct bank_elem{
	struct TCB* thread;
	struct Sema4* semaphore;
	char number;
};
struct resource_elem{
	struct Sema4* semaphore;
	char number;
};
void bank_init_tcb (struct TCB* thread, unsigned long thread_num);
void bank_init_semaphore (struct Sema4* semaphore, unsigned sema_num);
void bank_init(void);//for testing 
int bank_check(void);//return -1 if dead lock can happen
#endif