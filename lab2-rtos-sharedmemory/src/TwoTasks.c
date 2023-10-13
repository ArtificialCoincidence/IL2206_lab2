// File: TwoTasks.c 

#include <stdio.h>
#include "includes.h"
#include <string.h>

#define DEBUG 0

/* Definition of Task Stacks */
/* Stack grows from HIGH to LOW memory */
#define   TASK_STACKSIZE       2048
OS_STK    task1_stk[TASK_STACKSIZE];
OS_STK    task2_stk[TASK_STACKSIZE];
OS_STK    stat_stk[TASK_STACKSIZE];

/* Definition of Task Priorities */
#define TASK1_PRIORITY      6  // highest priority
#define TASK2_PRIORITY      7
#define TASK_STAT_PRIORITY 12  // lowest priority

/* Parameters of shared memory */
#define nblk     2
#define blksize  (sizeof(INT8U*) + sizeof(int))

OS_EVENT *Task1Sem = NULL;
OS_EVENT *Task2Sem = NULL;

OS_MEM *SharedMem;
INT8U shared_memory[nblk][blksize];

void printStackSize(char* name, INT8U prio) 
{
  INT8U err;
  OS_STK_DATA stk_data;
    
  err = OSTaskStkChk(prio, &stk_data);
  if (err == OS_NO_ERR) {
    if (DEBUG == 1)
      printf("%s (priority %d) - Used: %d; Free: %d\n", 
	    name, prio, stk_data.OSUsed, stk_data.OSFree);
  }
  else
    {
      if (DEBUG == 1)
	      printf("Stack Check Error!\n");    
    }
}

/* Prints a message and sleeps for given time interval */
void task1(void* pdata)
{
  int timeout = 0;
  INT8U err;
  int num_sent;
  int num_received;
  int *block;

  num_sent = 1;

  /* reset semaphores for the first cycle */
  OSSemPend(Task1Sem, timeout, &err);
  OSSemPend(Task2Sem, timeout, &err);
  
  while (1)
    {
      int i;
      char text1[] = "Sending   : ";
      char text2[] = "Receiving : ";
      
      block = OSMemGet(SharedMem, &err);
      if (err != OS_ERR_NONE || block == NULL) {
	      printf("obtain memory block failed!\n");
      }

      *(block + sizeof(INT8U*)) = num_sent;
      
      err = OSMemPut(SharedMem, block);
      if (err != OS_ERR_NONE) {
	      printf("write to shared memory failed!\n");
	      printf("%d\n", err);
      }

      for (i = 0; i < strlen(text1); i++)
	      putchar(text1[i]);
      printf("%d\n", num_sent);

      err = OSSemPost(Task1Sem);
      if (err != OS_ERR_NONE) {
        printf("semaphore signal failed!");
      }

      OSSemPend(Task2Sem, timeout, &err);
      
      block = OSMemGet(SharedMem, &err);
      if (err != OS_ERR_NONE || block == NULL) {
	      printf("read from shared memory failed!\n");
      }

      num_received = *(block + sizeof(INT8U*));

      err = OSMemPut(SharedMem, block);
      if (err != OS_ERR_NONE) {
	      printf("write to shared memory failed!\n");
	      printf("%d\n", err);
      }
      
      for (i = 0; i < strlen(text2); i++)
	      putchar(text2[i]);
      printf("%d\n", num_received);

      num_sent++;
	
      OSTimeDlyHMSM(0, 0, 0, 11); /* Context Switch to next task
				   * Task will go to the ready state
				   * after the specified delay
				   */
    }
}

/* Prints a message and sleeps for given time interval */
void task2(void* pdata)
{
  int timeout = 0;
  INT8U err;
  int num_sent;
  int num_received;
  int *block;
  while (1)
    { 
      OSSemPend(Task1Sem, timeout, &err);

      block = OSMemGet(SharedMem, &err);
      if (err != OS_ERR_NONE || block == NULL) {
	      printf("read from shared memory failed!\n");
      }

      num_received = *(block + sizeof(INT8U*));
      
      num_sent = 0 - num_received;

      *(block + sizeof(INT8U*)) = num_sent;

      err = OSMemPut(SharedMem, block);
      if (err != OS_ERR_NONE) {
	      printf("write to shared memory failed!\n");
      }

      err = OSSemPost(Task2Sem);
      if (err != OS_ERR_NONE) {
	      printf("semaphore signal failed!");
      }

      OSTimeDlyHMSM(0, 0, 0, 4);
    }
}

/* Printing Statistics */
void statisticTask(void* pdata)
{
  while(1)
    {
      printStackSize("Task1", TASK1_PRIORITY);
      printStackSize("Task2", TASK2_PRIORITY);
      printStackSize("StatisticTask", TASK_STAT_PRIORITY);
    }
}

/* The main function creates two task and starts multi-tasking */
int main(void)
{
  INT8U err;

  printf("Lab 3 - Two Tasks\n");

  Task1Sem = OSSemCreate(1);
  Task2Sem = OSSemCreate(1);
  if (Task1Sem == NULL || Task2Sem == NULL) {
    printf("semaphore create failed!\n");
  } else {
    printf("semaphore create successed!\n");
  }

  SharedMem = OSMemCreate(shared_memory, nblk, blksize, &err);
  if (SharedMem == NULL) {
    printf("shared memory create failed!\n");
  } else {
    printf("shared memory create successed!\n");
  }

  OSTaskCreateExt
    ( task1,                        // Pointer to task code
      NULL,                         // Pointer to argument passed to task
      &task1_stk[TASK_STACKSIZE-1], // Pointer to top of task stack
      TASK1_PRIORITY,               // Desired Task priority
      TASK1_PRIORITY,               // Task ID
      &task1_stk[0],                // Pointer to bottom of task stack
      TASK_STACKSIZE,               // Stacksize
      NULL,                         // Pointer to user supplied memory (not needed)
      OS_TASK_OPT_STK_CHK |         // Stack Checking enabled 
      OS_TASK_OPT_STK_CLR           // Stack Cleared                                 
      );
	   
  OSTaskCreateExt
    ( task2,                        // Pointer to task code
      NULL,                         // Pointer to argument passed to task
      &task2_stk[TASK_STACKSIZE-1], // Pointer to top of task stack
      TASK2_PRIORITY,               // Desired Task priority
      TASK2_PRIORITY,               // Task ID
      &task2_stk[0],                // Pointer to bottom of task stack
      TASK_STACKSIZE,               // Stacksize
      NULL,                         // Pointer to user supplied memory (not needed)
      OS_TASK_OPT_STK_CHK |         // Stack Checking enabled 
      OS_TASK_OPT_STK_CLR           // Stack Cleared                       
      );  

  if (DEBUG == 1)
    {
      OSTaskCreateExt
	( statisticTask,                // Pointer to task code
	  NULL,                         // Pointer to argument passed to task
	  &stat_stk[TASK_STACKSIZE-1],  // Pointer to top of task stack
	  TASK_STAT_PRIORITY,           // Desired Task priority
	  TASK_STAT_PRIORITY,           // Task ID
	  &stat_stk[0],                 // Pointer to bottom of task stack
	  TASK_STACKSIZE,               // Stacksize
	  NULL,                         // Pointer to user supplied memory (not needed)
	  OS_TASK_OPT_STK_CHK |         // Stack Checking enabled 
	  OS_TASK_OPT_STK_CLR           // Stack Cleared                              
	  );
    }  

  OSStart();
  return 0;
}
