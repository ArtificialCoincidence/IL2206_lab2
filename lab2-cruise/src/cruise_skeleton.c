/* Cruise control skeleton for the IL 2206 embedded lab
 *
 * Maintainers:  Rodolfo Jordao (jordao@kth.se), George Ungereanu (ugeorge@kth.se)
 *
 * Description:
 *
 *   In this file you will find the "model" for the vehicle that is being simulated on top
 *   of the RTOS and also the stub for the control task that should ideally control its
 *   velocity whenever a cruise mode is activated.
 *
 *   The missing functions and implementations in this file are left as such for
 *   the students of the IL2206 course. The goal is that they get familiriazed with
 *   the real time concepts necessary for all implemented herein and also with Sw/Hw
 *   interactions that includes HAL calls and IO interactions.
 *
 *   If the prints prove themselves too heavy for the final code, they can
 *   be exchanged for alt_printf where hexadecimals are supported and also
 *   quite readable. This modification is easily motivated and accepted by the course
 *   staff.
 */
#include <stdio.h>
#include "system.h"
#include "includes.h"
#include "altera_avalon_pio_regs.h"
#include "altera_avalon_performance_counter.h"
#include "sys/alt_irq.h"
#include "sys/alt_alarm.h"

#define DEBUG 0

#define HW_TIMER_PERIOD 100 /* 100ms */
#define CALIBRATION    2300 /* calibaration factor for addload  for loop */

/* Button Patterns */

#define GAS_PEDAL_FLAG      0x08
#define BRAKE_PEDAL_FLAG    0x04
#define CRUISE_CONTROL_FLAG 0x02
/* Switch Patterns */
#define SW9                 0x00000200
#define SW8                 0x00000100
#define SW7                 0x00000080
#define SW6                 0x00000040
#define SW5                 0x00000020
#define SW4                 0x00000010
#define TOP_GEAR_FLAG       0x00000002
#define ENGINE_FLAG         0x00000001

/* LED Patterns */

#define LED_RED_0  0x00000001 // Engine
#define LED_RED_1  0x00000002 // Top Gear
#define LED_RED_12 0x00001000
#define LED_RED_13 0x00002000
#define LED_RED_14 0x00004000
#define LED_RED_15 0x00008000
#define LED_RED_16 0x00010000
#define LED_RED_17 0x00020000

#define LED_GREEN_0 0x00000001 // Cruise Control activated
#define LED_GREEN_2 0x00000004 // Cruise Control Button
#define LED_GREEN_4 0x00000010 // Brake Pedal
#define LED_GREEN_6 0x00000040 // Gas Pedal

/*
 * Definition of Tasks
 */

#define TASK_STACKSIZE 2048

OS_STK StartTask_Stack[TASK_STACKSIZE]; 
OS_STK ControlTask_Stack[TASK_STACKSIZE]; 
OS_STK VehicleTask_Stack[TASK_STACKSIZE];
OS_STK ButtonIO_Stack[TASK_STACKSIZE];
OS_STK SwitchIO_Stack[TASK_STACKSIZE];
OS_STK DisplayTask_Stack[TASK_STACKSIZE];
OS_STK WatchdogTask_Stack[TASK_STACKSIZE];
OS_STK OverloadDetection_Stack[TASK_STACKSIZE];
OS_STK ExtraloadTask_Stack[TASK_STACKSIZE];

// Task Priorities
#define WATCHDOGTASK_PRIO  4
#define STARTTASK_PRIO     5
#define BUTTONIO_PRIO      8
#define SWITCHIO_PRIO      9
#define DISPLAYTASK_PRIO   10
#define VEHICLETASK_PRIO   11
#define CONTROLTASK_PRIO   12
#define EXTRALOADTASK_PRIO 14
#define OVERLOADDETECTION_PRIO   13

// Task Periods
#define HYPERPERIOD       300
#define CONTROL_PERIOD    300
#define VEHICLE_PERIOD    300
#define BUTTONIO_PERIOD   100
#define SWITCHIO_PERIOD   300
#define DISPLAY_PERIOD    300
#define WATCHDOG_PERIOD   HYPERPERIOD
#define OVERLOAD_PERIOD   HYPERPERIOD
#define EXTRALOAD_PERIOD  HYPERPERIOD

/*
 * Definition of Kernel Objects 
 */

// Mailboxes
OS_EVENT *Mbox_Throttle;
OS_EVENT *Mbox_Velocity;
OS_EVENT *Mbox_Brake;
OS_EVENT *Mbox_Engine_Control;
OS_EVENT *Mbox_Engine_Vehicle;
OS_EVENT *Mbox_TopGear;
OS_EVENT *Mbox_GasPedal;
OS_EVENT *Mbox_Cruise;
OS_EVENT *Mbox_ButtonOut;
OS_EVENT *Mbox_SwitchOut;
OS_EVENT *Mbox_PositionOut;
OS_EVENT *Mbox_ControlOut;
OS_EVENT *Mbox_WatchdogReset;

// Semaphores
OS_EVENT *VehicleSem   = NULL;
OS_EVENT *ControlSem   = NULL;
OS_EVENT *ButtonSem    = NULL;
OS_EVENT *SwitchSem    = NULL;
OS_EVENT *DisplaySem   = NULL;
OS_EVENT *WatchdogSem  = NULL;
OS_EVENT *OverloadSem  = NULL;
OS_EVENT *ExtraloadSem = NULL;
OS_EVENT *ExtraloadFinishSem = NULL;

// SW-Timer
OS_TMR *VehicleTmr   = NULL; // Timer used by vehicle task
OS_TMR *ControlTmr   = NULL; // Timer used by control task
OS_TMR *ButtonTmr    = NULL;
OS_TMR *SwitchTmr    = NULL; // Timer used by SwitchIO
OS_TMR *DisplayTmr   = NULL;
OS_TMR *WatchdogTmr  = NULL;
OS_TMR *OverloadTmr  = NULL;
OS_TMR *ExtraloadTmr = NULL;

/*
 * Types
 */
enum active {on = 2, off = 1};


/*
 * Global variables
 */
int delay; // Delay of HW-timer 
int gflag_finish[5];

/*
 * Helper functions
 */

int buttons_pressed(void)
{
  return IORD_ALTERA_AVALON_PIO_DATA(D2_PIO_KEYS4_BASE);
}

int switches_pressed(void)
{
  return IORD_ALTERA_AVALON_PIO_DATA(DE2_PIO_TOGGLES18_BASE);
}

static void ButtonIOCallback(void *ptmr, void *callback_arg)
{
  OSSemPost(ButtonSem);
}

// The interrupt service routine
static void ButtonIO(void* context, alt_u32 id)
{ 
  int btn_reg = 0;
  int out = 0;
  enum active brake_pedal, gas_pedal, cruise_control;
  cruise_control = off;
  INT8U err;
  printf("ButtonIO task created!\n");
  while (1) {
    if (DEBUG)
      printf("%s\n", __func__);
    out = 0;
    brake_pedal = gas_pedal = off;
    btn_reg = buttons_pressed();
    btn_reg = ~btn_reg;
    btn_reg = btn_reg & 0xf;

    switch (btn_reg) {
      case CRUISE_CONTROL_FLAG:                         // cruise_control button pressed
        cruise_control = (cruise_control == on)?off:on; // toggle cruise_control
        break;
      case BRAKE_PEDAL_FLAG:
        brake_pedal = on;
        break;
      case GAS_PEDAL_FLAG:
        gas_pedal = on;
        break;
      default:
        break;
    }
  
    if (cruise_control == on) {
      out += LED_GREEN_2;
    }
    if (brake_pedal == on) {
      out += LED_GREEN_4;
    }
    if (gas_pedal == on) {
      out += LED_GREEN_6;
    }

    err = OSMboxPost(Mbox_Brake, (void *)&brake_pedal);
    if (err != OS_ERR_NONE && DEBUG) {
      printf("OSMboxPost error! line %d\n", __LINE__);
    }
    err = OSMboxPost(Mbox_Cruise, (void *)&cruise_control);
    if (err != OS_ERR_NONE && DEBUG) {
      printf("OSMboxPost error! line %d\n", __LINE__);
    }
    err = OSMboxPost(Mbox_GasPedal, (void *)&gas_pedal);
    if (err != OS_ERR_NONE && DEBUG) {
      printf("OSMboxPost error! line %d\n", __LINE__);
    }

    err = OSMboxPost(Mbox_ButtonOut, (void *)&out);
    if (err != OS_ERR_NONE && DEBUG) {
      printf("OSMboxPost error! line %d\n", __LINE__);
    }
    
    gflag_finish[0] = 1;
    OSSemPend(ButtonSem, 0, &err);
  }
}

static void SwitchIOCallback(void *ptmr, void *callback_arg)
{
  OSSemPost(SwitchSem);
}

// The interrupt service for switches
static void SwitchIO(void)
{
  int btn_reg = 0;
  int out = 0;
  enum active engine_control, engine_vehicle, top_gear;
  engine_control = engine_vehicle = top_gear = off;
  INT8U err;
  printf("SwitchIO task created!\n");
  while (1) {
    if (DEBUG)
      printf("%s\n", __func__);
    engine_control = engine_vehicle = top_gear = off;
    out = 0;
    btn_reg = switches_pressed();
    btn_reg = btn_reg & 0xf;

    engine_control = engine_vehicle = (btn_reg & ENGINE_FLAG)?on:off;
    top_gear = (btn_reg & TOP_GEAR_FLAG)?on:off;

    if (engine_control == on) {
      out += LED_RED_0;
    }
    if (top_gear == on) {
      out += LED_RED_1;
    }

    err = OSMboxPost(Mbox_Engine_Control, (void *)&engine_control);
    err = OSMboxPost(Mbox_Engine_Vehicle, (void *)&engine_vehicle);
    err = OSMboxPost(Mbox_TopGear, (void *)&top_gear);

    err = OSMboxPost(Mbox_SwitchOut, (void *)&out);

    gflag_finish[1] = 1;
    OSSemPend(SwitchSem, 0, &err);
  }
}

/*
 * ISR for HW Timer
 */
alt_u32 alarm_handler(void* context)
{
  OSTmrSignal(); /* Signals a 'tick' to the SW timers */

  return delay;
}

static int b2sLUT[] = {0x40, //0
  0x79, //1
  0x24, //2
  0x30, //3
  0x19, //4
  0x12, //5
  0x02, //6
  0x78, //7
  0x00, //8
  0x18, //9
  0x3F, //-
};

/*
 * convert int to seven segment display format
 */
int int2seven(int inval){
  return b2sLUT[inval];
}

/*
 * output current velocity on the seven segement display
 */
void show_velocity_on_sevenseg(INT8S velocity){
  int tmp = velocity;
  int out;
  INT8U out_high = 0;
  INT8U out_low = 0;
  INT8U out_sign = 0;

  if(velocity < 0){
    out_sign = int2seven(10);
    tmp *= -1;
  }else{
    out_sign = int2seven(0);
  }

  out_high = int2seven(tmp / 10);
  out_low = int2seven(tmp - (tmp/10) * 10);

  out = int2seven(0) << 21 |
    out_sign << 14 |
    out_high << 7  |
    out_low;
  IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_HEX_LOW28_BASE,out);
}

/*
 * shows the target velocity on the seven segment display (HEX5, HEX4)
 * when the cruise control is activated (0 otherwise)
 */
void show_target_velocity(INT16S target_vel)
{
  int tmp = (int)target_vel;
  int out;
  INT8U out_high = 0;
  INT8U out_low = 0;

  out_high = int2seven(tmp / 10);
  out_low = int2seven(tmp - (tmp/10) * 10);

  out = int2seven(0) << 21 |
    int2seven(0) << 14 |
    out_high << 7  |
    out_low;
  IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_HEX_HIGH28_BASE,out);
}

/*
 * indicates the position of the vehicle on the track with the four leftmost red LEDs
 * LEDR17: [0m, 400m)
 * LEDR16: [400m, 800m)
 * LEDR15: [800m, 1200m)
 * LEDR14: [1200m, 1600m)
 * LEDR13: [1600m, 2000m)
 * LEDR12: [2000m, 2400m]
 */
void show_position(INT16U position, int *out)
{
  if (position < 400) {
    *out = LED_RED_17;
  } else if (position >= 400 && position < 800) {
    *out = LED_RED_16;
  } else if (position >= 800 && position < 1200) {
    *out = LED_RED_15;
  } else if (position >= 1200 && position < 1600) {
    *out = LED_RED_14;
  } else if (position >= 1600 && position < 2000) {
    *out = LED_RED_13;
  } else {
    *out = LED_RED_12;
  }
}

/*
 * The task 'VehicleTask' is the model of the vehicle being simulated. It updates variables like
 * acceleration and velocity based on the input given to the model.
 * 
 * The car model is equivalent to moving mass with linear resistances acting upon it.
 * Therefore, if left one, it will stably stop as the velocity converges to zero on a flat surface.
 * You can prove that easily via basic LTI systems methods.
 */

void VehicleCallback(void *ptmr, void *callback_arg)
{
  OSSemPost(VehicleSem);
}

void VehicleTask(void* pdata)
{ 
  // constants that should not be modified
  const unsigned int wind_factor = 1;
  const unsigned int brake_factor = 4;
  const unsigned int gravity_factor = 2;
  // variables relevant to the model and its simulation on top of the RTOS
  int position_out;
  INT8U err;  
  void* msg;
  INT8U* throttle; 
  INT16S acceleration;  
  INT16U position = 0; 
  INT16U previous_position = 0;
  INT16S velocity = 0; 
  enum active brake_pedal = off;
  enum active engine = off;

  printf("Vehicle task created!\n");

  while(1)
  {
    if (DEBUG)
      printf("%s\n", __func__);
    err = OSMboxPost(Mbox_Velocity, (void *) &velocity);

    OSSemPend(VehicleSem, 0, &err);

    /* Non-blocking read of mailbox: 
       - message in mailbox: update throttle
       - no message:         use old throttle
       */
    msg = OSMboxPend(Mbox_Throttle, 1, &err); 
    if (err == OS_ERR_NONE) 
      throttle = (INT8U*) msg;
    /* Same for the brake signal that bypass the control law */
    msg = OSMboxPend(Mbox_Brake, 0, &err); 
    if (err == OS_ERR_NONE) 
      brake_pedal = *((enum active *)msg);
    /* Same for the engine signal that bypass the control law */
    msg = OSMboxPend(Mbox_Engine_Vehicle, 0, &err); 
    if (err == OS_ERR_NONE) 
      engine = *((enum active *)msg);

    // vehichle cannot effort more than 80 units of throttle
    if (*throttle > 80) *throttle = 80;

    // brakes + wind
    if (brake_pedal == off)
    {
      // wind resistance
      acceleration = - wind_factor*velocity;
      // actuate with engines
      if (engine == on)
        acceleration += (*throttle);

      // gravity effects
      if (400 <= position && position < 800)
        acceleration -= gravity_factor; // traveling uphill
      else if (800 <= position && position < 1200)
        acceleration -= 2*gravity_factor; // traveling steep uphill
      else if (1600 <= position && position < 2000)
        acceleration += 2*gravity_factor; //traveling downhill
      else if (2000 <= position)
        acceleration += gravity_factor; // traveling steep downhill
    }
    // if the engine and the brakes are activated at the same time,
    // we assume that the brake dynamics dominates, so both cases fall
    // here.
    else 
      acceleration = - brake_factor*velocity;

    // printf("Position: %d m\n", position);
    // printf("Velocity: %d m/s\n", velocity);
    // printf("Accell: %d m/s2\n", acceleration);
    // printf("Throttle: %d V\n", *throttle);

    position = position + velocity * VEHICLE_PERIOD / 1000;
    velocity = velocity  + acceleration * VEHICLE_PERIOD / 1000.0;
    // reset the position to the beginning of the track
    if(position > 2400)
      position = 0;

    show_velocity_on_sevenseg((INT8S) velocity);
    show_position(position, (void *)&position_out);
    OSMboxPost(Mbox_PositionOut, &position_out);

    previous_position = position;

    gflag_finish[2] = 1;
  }
} 

void ControlCallback(void *ptmr, void *callback_arg)
{
  OSSemPost(ControlSem);
}

static INT8U calculate_throttle(int current_velocity, enum active gas_pedal)
{
  int i = 0;
  int throttle;
  int velocity_cur[] = {-5, 10, 20, 30, 40, 50, 60, 70, 80};
  int speed_up[]     = {30, 30, 40, 50, 70, 70, 80, 80, 70};
  int slow_down[]    = { 5,  5, 10, 20, 30, 40, 50, 60, 70};
  
  for (; i < 8; i++) {
    if (velocity_cur[i] <= current_velocity && velocity_cur[i + 1] >= current_velocity) {
      if (gas_pedal == on) {
        throttle = (current_velocity - velocity_cur[i]) / (velocity_cur[i + 1] - velocity_cur[i])
                  * (speed_up[i + 1] - speed_up[i]) + speed_up[i];
      } else {
        throttle = (current_velocity - velocity_cur[i]) / (velocity_cur[i + 1] - velocity_cur[i])
                  * (slow_down[i + 1] - slow_down[i]) + slow_down[i];
      }
      
    }
  }

  return throttle;
}

INT8U calculate_cruise(INT8U cruise_velocity, INT8U current_velocity)
{
  INT8U throttle;
  const int wind_factor = 1;
  if (cruise_velocity > current_velocity) {
    throttle = cruise_velocity * wind_factor
              + 2 * wind_factor * (cruise_velocity - current_velocity)
              / (CONTROL_PERIOD * CONTROL_PERIOD / 1000000) + 2;
  } else {
    throttle = cruise_velocity * wind_factor
              - 2 * wind_factor * (current_velocity - cruise_velocity)
              / (CONTROL_PERIOD * CONTROL_PERIOD / 1000000) - 1;
  }
}

/*
 * The task 'ControlTask' is the main task of the application. It reacts
 * on sensors and generates responses.
 */

void ControlTask(void* pdata)
{
  INT8U err;
  INT8U throttle = 40; /* Value between 0 and 80, which is interpreted as between 0.0V and 8.0V */
  void* msg;
  INT16S* current_velocity;
  INT16S previous_velocity;
  INT16S cruise_velocity = 0;
  int cruising = 0;
  int out_control = 0;

  enum active gas_pedal = off;
  enum active top_gear = off;
  enum active cruise_control = off; 
  enum active engine = off;

  printf("Control Task created!\n");

  while(1)
  {
    if (DEBUG)
      printf("%s\n", __func__);
    msg = OSMboxPend(Mbox_Velocity, 1, &err);
    if (err == OS_ERR_NONE)
      current_velocity = (INT16S*) msg;
    msg = OSMboxPend(Mbox_Cruise, 0, &err);
    if (err == OS_ERR_NONE)
      cruise_control = *((enum active *)msg);
    msg = OSMboxPend(Mbox_GasPedal, 0, &err);
    if (err == OS_ERR_NONE)
      gas_pedal = *((enum active *)msg);
    msg = OSMboxPend(Mbox_Engine_Control, 0, &err);
    if (err == OS_ERR_NONE)
      engine = *((enum active *)msg);
    msg = OSMboxPend(Mbox_TopGear, 0, &err);
    if (err == OS_ERR_NONE)
      top_gear = *((enum active *)msg);
    // Here you can use whatever technique or algorithm that you prefer to control
    // the velocity via the throttle. There are no right and wrong answer to this controller, so
    // be free to use anything that is able to maintain the cruise working properly. You are also
    // allowed to store more than one sample of the velocity. For instance, you could define
    //
    // INT16S previous_vel;
    // INT16S pre_previous_vel;
    // ...
    //
    // If your control algorithm/technique needs them in order to function. 
    if (cruise_control == on && cruising == 0 && top_gear == on && *current_velocity > 25) {
      cruising = 1;
      printf("start cruising!\n");
      cruise_velocity = *current_velocity;
      throttle = calculate_cruise(cruise_velocity, *current_velocity);
    } else if (cruising && top_gear == on && cruise_control == on) {
      throttle = calculate_cruise(cruise_velocity, *current_velocity);
    } else if (engine == on) {
      throttle = calculate_throttle(*current_velocity, gas_pedal);
      cruise_velocity = 0;
      cruising = 0;
    } else {
      throttle = 0;
    }

    show_target_velocity(cruise_velocity);

    if (cruising) {
      out_control = LED_GREEN_0;
    } else {
      out_control = 0;
    }

    // printf("current velocity: %d, cruise %d, throttle %d, top_gear %d, cruise_velocity %d\n",
    //      *current_velocity, cruising, throttle, top_gear == on, cruise_velocity);
  
    err = OSMboxPost(Mbox_Throttle, (void *) &throttle);

    err = OSMboxPost(Mbox_ControlOut, (void *) &out_control);

    gflag_finish[3] = 1;

    OSSemPend(ControlSem, 0, &err);

    previous_velocity = *current_velocity;
  }
}


void DisplayCallback(void *ptmr, void *callback_arg)
{
  OSSemPost(DisplaySem);
}
/* 
 * Display task receive message from buttons_pressed, switches_pressed, show_position, controltask.
 * Only allow display task to write to the green and red leds to avoid conflicts and blurring.
 */

void DisplayTask(void)
{
  INT8U err;
  void *msg;
  int out_button;
  int out_switch;
  int out_position;
  int out_control;
  int greenled;
  int redled;
  int greenled_prev;
  int redled_prev;

  out_button = out_switch = out_position = out_control = 0;
  greenled = greenled_prev = 0;
  redled = redled_prev = 0;

  printf("Display task created!\n");
  while (1) {
    if (DEBUG)
      printf("%s\n", __func__);
    OSSemPend(DisplaySem, 0, &err);
    msg = OSMboxPend(Mbox_ButtonOut, 1, &err);
    if (err == OS_ERR_NONE) {
      out_button = *(int *)msg;
    }
    msg = OSMboxPend(Mbox_SwitchOut, 1, &err);
    if (err == OS_ERR_NONE) {
      out_switch = *(int *)msg;
    }
    msg = OSMboxPend(Mbox_PositionOut, 1, &err);
    if (err == OS_ERR_NONE) {
      out_position = *(int *)msg;
    }
    msg = OSMboxPend(Mbox_ControlOut, 1, &err);
    if (err == OS_ERR_NONE) {
      out_control = *(int *)msg;
    }

    greenled = out_button + out_control;
    redled = out_switch + out_position;

    if (greenled != greenled_prev)
      IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_GREENLED9_BASE, greenled);
    if (redled != redled_prev)
      IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE, redled);

    greenled_prev = greenled;
    redled_prev = redled;

    gflag_finish[4] = 1;
  }
}

void WatchdogCallback(void *ptmr, void *callback_arg)
{
  OSSemPost(WatchdogSem);
}

/* Watchdog task allow to be reset during a hyperperiod.
 * If reset, it means all tasks finish running within a hyperperiod.
 * If not reset, it means there is task that have not finish running during a hyperperiod.
 */

void WatchdogTask(void)
{
  int reset = 0;
  void *msg;
  INT8U err;
  printf("Watchdog task created!\n");
  while (1) {
    if (DEBUG)
      printf("%s\n", __func__);
    PERF_RESET(PERFORMANCE_COUNTER_BASE);
    PERF_START_MEASURING(PERFORMANCE_COUNTER_BASE);
    PERF_BEGIN(PERFORMANCE_COUNTER_BASE, 1);
    msg = OSMboxPend(Mbox_WatchdogReset, HYPERPERIOD - 10, &err); /* wait for reset */
    if (err == OS_ERR_NONE) {
      reset = *(int *)msg;
      int i = 0;
      for (; i < 5; i++) {
        gflag_finish[i] = 0;
      }
    }
    if (reset == 0) {
      printf("Overload detected!\n"); /* no reset during waiting time, overload detected */
    }
    PERF_END(PERFORMANCE_COUNTER_BASE, 1);
    if (DEBUG)
      perf_print_formatted_report(PERFORMANCE_COUNTER_BASE, 50000000, 1, "SECTION"); /* print the waiting time */

    OSSemPend(WatchdogSem, 0, &err);
    reset = 0;
  }
}

void OverloadCallback(void *ptmr, void *callback_arg)
{
  OSSemPost(OverloadSem);
}

/* Overload detection task run after all other tasks have finish running in a hyperperiod.
 * Overload detection task reset watchdog to tell it all tasks have finish running.
 */

void OverloadDetection(void)
{
  int reset = 1;
  INT8U err;
  printf("Overload detection task created!\n");
  while (1) {
    if (DEBUG)
      printf("%s\n", __func__);

    OSSemPend(ExtraloadFinishSem, 0, &err);
    if (err != OS_ERR_NONE && DEBUG) {
      printf("OSSemPend error! line, %d error, %u\n", __LINE__, err);
    }

    if (gflag_finish[0] && gflag_finish[1] && gflag_finish[2] && gflag_finish[3] && gflag_finish[4]) {
      err = OSMboxPost(Mbox_WatchdogReset, (void *)&reset);
      if (err != OS_ERR_NONE && DEBUG) {
        printf("OSMboxPost error! line, %d\n", __LINE__);
      }
    }
      
    OSSemPend(OverloadSem, 0, &err);
  }
}

void addload(int time)
{
  int i, j;
  i = j = 0;
  for (; i < time; i++) {
    for (; j < CALIBRATION; j++);
  }
}

void ExtraloadCallback(void *ptmr, void *callback_arg)
{
  OSSemPost(ExtraloadSem);
}

/* Extraload task add an extra load to the system.
 * Read the value of sw4-sw9 to calculate the amount of extra load.
 */

void ExtraloadTask(void)
{
  int btn_reg;
  int workload = 0;
  int extraload;
  INT8U err;
  printf("Extraload task created!\n");
  while (1) {
    if (DEBUG)
      printf("%s\n", __func__);
    btn_reg = IORD_ALTERA_AVALON_PIO_DATA(DE2_PIO_TOGGLES18_BASE);
    btn_reg = btn_reg & 0xffffffff;

    if (btn_reg & SW4) {
      workload += 1;
    }
    if (btn_reg & SW5) {
      workload += 2;
    }
    if (btn_reg & SW6) {
      workload += 4;
    }
    if (btn_reg & SW7) {
      workload += 8;
    }
    if (btn_reg & SW8) {
      workload += 16;
    }
    if (btn_reg & SW9) {
      workload += 32;
    }

    extraload = workload * CALIBRATION;
    addload(extraload);

    OSSemPost(ExtraloadFinishSem);
    
    OSSemPend(ExtraloadSem, 0, &err);
    if (err != OS_ERR_NONE) {
      printf("OSSemPend error! line, %d error, %u\n", __LINE__, err);
    }
    OSSemSet(ExtraloadSem, 0, &err);
    if (err != OS_ERR_NONE) {
      printf("OSSemSet error! line, %d\n", __LINE__);
    }
    workload = 0;
  }
}

/* 
 * The task 'StartTask' creates all other tasks kernel objects and
 * deletes itself afterwards.
 */ 

void StartTask(void* pdata)
{
  INT8U err;
  void* context;

  static alt_alarm alarm;     /* Is needed for timer ISR function */

  /* Base resolution for SW timer : HW_TIMER_PERIOD ms */
  delay = alt_ticks_per_second() * HW_TIMER_PERIOD / 1000; 
  printf("delay in ticks %d\n", delay);

  /* 
   * Create Hardware Timer with a period of 'delay' 
   */
  if (alt_alarm_start (&alarm,
        delay,
        alarm_handler,
        context) < 0)
  {
    printf("No system clock available!n");
  }

  /* 
   * Create and start Software Timer 
   */
  VehicleTmr = OSTmrCreate(0, VEHICLE_PERIOD / 100, OS_TMR_OPT_PERIODIC,
			  &VehicleCallback, NULL, "VehicleTmr", &err);
  if (err != OS_ERR_NONE) {
    printf("timer create failed! line, %d\n", __LINE__);
  }

  ControlTmr = OSTmrCreate(0, CONTROL_PERIOD / 100, OS_TMR_OPT_PERIODIC,
        &ControlCallback, NULL, "ControlTmr", &err);
  if (err != OS_ERR_NONE) {
    printf("timer create failed! line, %d\n", __LINE__);
  }

  ButtonTmr = OSTmrCreate(0, BUTTONIO_PERIOD / 100, OS_TMR_OPT_PERIODIC,
        &ButtonIOCallback, NULL, "ButtonTmr", &err);
  if (err != OS_ERR_NONE) {
    printf("timer create failed! line, %d\n", __LINE__);
  }
  
  SwitchTmr = OSTmrCreate(0, SWITCHIO_PERIOD / 100, OS_TMR_OPT_PERIODIC,
        &SwitchIOCallback, NULL, "SwitchTmr", &err);
  if (err != OS_ERR_NONE) {
    printf("timer create failed! line, %d\n", __LINE__);
  }

  DisplayTmr = OSTmrCreate(0, DISPLAY_PERIOD / 100, OS_TMR_OPT_PERIODIC,
        &DisplayCallback, NULL, "DisplayTmr", &err);
  if (err != OS_ERR_NONE) {
    printf("timer create failed! line, %d\n", __LINE__);
  }

  WatchdogTmr = OSTmrCreate(0, WATCHDOG_PERIOD / 100, OS_TMR_OPT_PERIODIC,
        &WatchdogCallback, NULL, "WatchdogTmr", &err);
  if (err != OS_ERR_NONE) {
    printf("timer create failed! line, %d\n", __LINE__);
  }

  OverloadTmr = OSTmrCreate(0, OVERLOAD_PERIOD / 100, OS_TMR_OPT_PERIODIC,
        &OverloadCallback, NULL, "OverloadTmr", &err);
  if (err != OS_ERR_NONE) {
    printf("timer create failed! line, %d\n", __LINE__);
  }

  ExtraloadTmr = OSTmrCreate(0, EXTRALOAD_PERIOD / 100, OS_TMR_OPT_PERIODIC,
        &ExtraloadCallback, NULL, "ExtraloadTmr", &err);
  if (err != OS_ERR_NONE) {
    printf("timer create failed! line, %d\n", __LINE__);
  }

  OSTmrStart(VehicleTmr, &err);
  if (err != OS_ERR_NONE) {
    printf("timer start failed! line, %d\n", __LINE__);
  }

  OSTmrStart(ControlTmr, &err);
  if (err != OS_ERR_NONE) {
    printf("timer start failed! line, %d\n", __LINE__);
  }

  OSTmrStart(ButtonTmr, &err);
  if (err != OS_ERR_NONE) {
    printf("timer start failed! line, %d\n", __LINE__);
  }

  OSTmrStart(SwitchTmr, &err);
  if (err != OS_ERR_NONE) {
    printf("timer start failed! line, %d\n", __LINE__);
  }

  OSTmrStart(DisplayTmr, &err);
  if (err != OS_ERR_NONE) {
    printf("timer start failed! line, %d\n", __LINE__);
  }

  OSTmrStart(WatchdogTmr, &err);
  if (err != OS_ERR_NONE) {
    printf("timer start failed! line, %d\n", __LINE__);
  }

  OSTmrStart(OverloadTmr, &err);
  if (err != OS_ERR_NONE) {
    printf("timer start failed! line, %d\n", __LINE__);
  }

  OSTmrStart(ExtraloadTmr, &err);
  if (err != OS_ERR_NONE) {
    printf("timer start failed! line, %d\n", __LINE__);
  }

  /*
   * Creation of Kernel Objects
   */
  VehicleSem = OSSemCreate(0);
  if (VehicleSem == NULL) {
    printf("semaphore create failed! line, %d\n", __LINE__);
  }

  ControlSem = OSSemCreate(0);
  if (ControlSem == NULL) {
    printf("semaphore create failed! line, %d\n", __LINE__);
  }

  ButtonSem = OSSemCreate(0);
  if (ButtonSem == NULL) {
    printf("semaphore create failed! line, %d\n", __LINE__);
  }

  SwitchSem = OSSemCreate(0);
  if (SwitchSem == NULL) {
    printf("semaphore create failed! line, %d\n", __LINE__);
  }

  DisplaySem = OSSemCreate(0);
  if (DisplaySem == NULL) {
    printf("semaphore create failed! line, %d\n", __LINE__);
  }

  WatchdogSem = OSSemCreate(0);
  if (WatchdogSem == NULL) {
    printf("semaphore create failed! line, %d\n", __LINE__);
  }

  OverloadSem = OSSemCreate(0);
  if (OverloadSem == NULL) {
    printf("semaphore create failed! line, %d\n", __LINE__);
  }

  ExtraloadSem = OSSemCreate(0);
  if (ExtraloadSem == NULL) {
    printf("semaphore create failed! line, %d\n", __LINE__);
  }

  ExtraloadFinishSem = OSSemCreate(0);
  if (ExtraloadFinishSem == NULL) {
    printf("semaphore create failed! line, %d\n", __LINE__);
  }


  // Mailboxes
  Mbox_Throttle = OSMboxCreate((void*) 0); /* Empty Mailbox - Throttle */
  Mbox_Velocity = OSMboxCreate((void*) 0); /* Empty Mailbox - Velocity */
  Mbox_Brake = OSMboxCreate((void*) 0); /* Empty Mailbox - Break */
  Mbox_Engine_Control = OSMboxCreate((void*) 1); /* Empty Mailbox - Engine - to Control task */
  Mbox_Engine_Vehicle = OSMboxCreate((void*) 1); /* Empty Mailbox - Engine - to Vehicle task */
  Mbox_Cruise = OSMboxCreate((void *) 0);
  Mbox_GasPedal = OSMboxCreate((void *) 0);
  Mbox_TopGear = OSMboxCreate((void *) 0);
  Mbox_ButtonOut = OSMboxCreate((void *)0);
  Mbox_SwitchOut = OSMboxCreate((void *)0);
  Mbox_PositionOut = OSMboxCreate((void *)0);
  Mbox_ControlOut = OSMboxCreate((void *)0);
  Mbox_WatchdogReset = OSMboxCreate((void *)0);

  /*
   * Create statistics task
   */

  OSStatInit();

  /* 
   * Creating Tasks in the system 
   */


  err = OSTaskCreateExt(
      ControlTask, // Pointer to task code
      NULL,        // Pointer to argument that is
      // passed to task
      &ControlTask_Stack[TASK_STACKSIZE-1], // Pointer to top
      // of task stack
      CONTROLTASK_PRIO,
      CONTROLTASK_PRIO,
      (void *)&ControlTask_Stack[0],
      TASK_STACKSIZE,
      (void *) 0,
      OS_TASK_OPT_STK_CHK);

  err = OSTaskCreateExt(
      VehicleTask, // Pointer to task code
      NULL,        // Pointer to argument that is
      // passed to task
      &VehicleTask_Stack[TASK_STACKSIZE-1], // Pointer to top
      // of task stack
      VEHICLETASK_PRIO,
      VEHICLETASK_PRIO,
      (void *)&VehicleTask_Stack[0],
      TASK_STACKSIZE,
      (void *) 0,
      OS_TASK_OPT_STK_CHK);

  err = OSTaskCreateExt(
      ButtonIO,
      NULL,
      &ButtonIO_Stack[TASK_STACKSIZE - 1],
      BUTTONIO_PRIO,
      BUTTONIO_PRIO,
      (void *)&ButtonIO_Stack[0],
      TASK_STACKSIZE,
      (void *)0,
      OS_TaskStatStkChk);

  err = OSTaskCreateExt(
      SwitchIO,
      NULL,
      &SwitchIO_Stack[TASK_STACKSIZE - 1],
      SWITCHIO_PRIO,
      SWITCHIO_PRIO,
      (void *)&SwitchIO_Stack[0],
      TASK_STACKSIZE,
      (void *) 0,
      OS_TASK_OPT_STK_CHK);

  err = OSTaskCreateExt(
      DisplayTask,
      NULL,
      &DisplayTask_Stack[TASK_STACKSIZE - 1],
      DISPLAYTASK_PRIO,
      DISPLAYTASK_PRIO,
      (void *)&DisplayTask_Stack[0],
      TASK_STACKSIZE,
      (void *) 0,
      OS_TASK_OPT_STK_CHK);

  err = OSTaskCreateExt (
      WatchdogTask,
      NULL,
      &WatchdogTask_Stack[TASK_STACKSIZE - 1],
      WATCHDOGTASK_PRIO,
      WATCHDOGTASK_PRIO,
      (void *)&WatchdogTask_Stack[0],
      TASK_STACKSIZE,
      (void *) 0,
      OS_TASK_OPT_STK_CHK);

  err = OSTaskCreateExt (
      OverloadDetection,
      NULL,
      &OverloadDetection_Stack[TASK_STACKSIZE - 1],
      OVERLOADDETECTION_PRIO,
      OVERLOADDETECTION_PRIO,
      (void *)&OverloadDetection_Stack[0],
      TASK_STACKSIZE,
      (void *) 0,
      OS_TASK_OPT_STK_CHK);

  err = OSTaskCreateExt (
      ExtraloadTask,
      NULL,
      &ExtraloadTask_Stack[TASK_STACKSIZE - 1],
      EXTRALOADTASK_PRIO,
      EXTRALOADTASK_PRIO,
      (void *)&ExtraloadTask_Stack[0],
      TASK_STACKSIZE,
      (void *) 0,
      OS_TASK_OPT_STK_CHK);

  printf("All Tasks and Kernel Objects generated!\n");
  
  /* Task deletes itself */

  OSTaskDel(OS_PRIO_SELF);
}

/*
 *
 * The function 'main' creates only a single task 'StartTask' and starts
 * the OS. All other tasks are started from the task 'StartTask'.
 *
 */

int main(void) {
  printf("Lab: Cruise Control\n");

  OSTaskCreateExt(
      StartTask, // Pointer to task code
      NULL,      // Pointer to argument that is
      // passed to task
      (void *)&StartTask_Stack[TASK_STACKSIZE-1], // Pointer to top
      // of task stack 
      STARTTASK_PRIO,
      STARTTASK_PRIO,
      (void *)&StartTask_Stack[0],
      TASK_STACKSIZE,
      (void *) 0,  
      OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);

  OSStart();

  return 0;
}
