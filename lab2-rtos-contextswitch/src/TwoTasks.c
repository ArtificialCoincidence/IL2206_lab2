// File: TwoTasks.c 

#include <stdio.h>
#include "includes.h"
#include "altera_avalon_performance_counter.h"
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

#define FREQ  50000000 // frequency of the clock

OS_EVENT *Task1Sem = NULL;
OS_EVENT *Task2Sem = NULL;

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
  
  while (1)
    {
      PERF_START_MEASURING(PERFORMANCE_COUNTER_BASE);
      
      char text1[] = "Task 1 - State 0\n";
      char text2[] = "Task 1 - State 1\n";
      int i;
      alt_u64 time_switch_ticks, time_switch_nanoseconds;

      for (i = 0; i < strlen(text1); i++)
	      putchar(text1[i]);

      PERF_BEGIN(PERFORMANCE_COUNTER_BASE, 1);

      err = OSSemPost(Task1Sem);
      if (err != OS_ERR_NONE) {
        printf("semaphore signal failed!");
      }

      OSSemPend(Task2Sem, timeout, &err);

      PERF_END(PERFORMANCE_COUNTER_BASE, 2);
      time_switch_ticks = perf_get_section_time(PERFORMANCE_COUNTER_BASE, 2);
      time_switch_nanoseconds = time_switch_ticks  * 1000000000 / FREQ;
      printf("task2 -> task1: time for a context switch: %u ns\n", time_switch_nanoseconds);

      perf_print_formatted_report(PERFORMANCE_COUNTER_BASE, FREQ, 2, "1->2", "2->1");

      PERF_RESET(PERFORMANCE_COUNTER_BASE);
      for (i = 0; i < strlen(text2); i++)
	      putchar(text2[i]);
	
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
  while (1)
    { 
      char text1[] = "Task 2 - State 0\n";
      char text2[] = "Task 2 - State 1\n";
      int i;
      alt_u64 time_switch_ticks;
      alt_u64 time_switch_nanoseconds;

      OSSemPend(Task1Sem, timeout, &err);

      PERF_END(PERFORMANCE_COUNTER_BASE, 1);
      time_switch_ticks = perf_get_section_time(PERFORMANCE_COUNTER_BASE, 1);
      time_switch_nanoseconds = time_switch_ticks  * 1000000000 / FREQ;
      printf("task1 -> task2: time for a context switch: %u ns\n", time_switch_nanoseconds);
      
      for (i = 0; i < strlen(text2); i++)
	      putchar(text1[i]);

      for (i = 0; i < strlen(text2); i++)
	      putchar(text2[i]);

      PERF_START_MEASURING(PERFORMANCE_COUNTER_BASE);
      PERF_BEGIN(PERFORMANCE_COUNTER_BASE, 2);

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

  PERF_RESET(PERFORMANCE_COUNTER_BASE);

  Task1Sem = OSSemCreate(0);
  Task2Sem = OSSemCreate(0);
  if (Task1Sem == NULL || Task2Sem == NULL) {
    printf("semaphore create failed!\n");
  } else {
    printf("semaphore create successed!\n");
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
