#include "config_1_2_3.h"
#include "gpio.h"
#include "pt_cornell_1_2_3.h"
#include <stdlib.h>
#include <math.h>
#ifndef _SUPPRESS_PLIB_WARNING
#define _SUPPRESS_PLIB_WARNING
#endif

#define NUM_STEPS_X 585
#define NUM_STEPS_Z 7850
typedef unsigned char uint8_t;
volatile uint8_t turned_on = 0;
volatile uint8_t turned_off = 0;

/*****************************[ Structs ]**************************************/
struct stepper_t stp1;
// Protothreads Structs
static struct pt pt_move, pt_blink;

/************************[ Shared Variables ]**********************************/
volatile static int keep_moving = 1; // Determines if target's been met
// Frequency of stepping (max), currently at a step every 500 uS
static const int period_xyz = 24000;
// Frequency of z stepper rotation, currently once every 1.5 msec
static const int enable_time_break = 60000;

void enable_x(void)
{
  turned_on = 0;
  mPORTASetBits(stp_1.SLEEP);
  //PT_YIELD_TIME_msec(1500);
  turned_on = 1;
}
void enable_B(void)
{
  turned_on = 0;
  mPORTBSetBits(stp_3.SLEEP);
  //PT_YIELD_TIME_msec(1500);
  turned_on = 1;
}

/*****************************[ ISRs ]*****************************************/
// == Timer 2 ISR =====================================================
// Steps the stepper motor
volatile int enable = 0;
void __ISR(_TIMER_2_VECTOR, ipl2) Timer2Handler(void)
{
//    // Turn on LED during duration of pulse
//    if (stp_1.stps_left == NUM_STEPS) 
//        set_LED();
//    else if (stp_1.stps_left == 0) 
//        clear_LED();
    // Pulse until there's no steps left
    if(turned_on && stp_1.stps_left > 0){
    //if (stp_1.stps_left > 0){
        mPORTBToggleBits(stp_1.STP);
        stp_1.stps_left--;      
    }
    if (stp_1.stps_left < 1000) set_RedLED();
    if (stp_1.stps_left < 500) set_GreenLED();
    // Clear the interrupt flag
    mT2ClearIntFlag();
}

//void __ISR(_TIMER_3_VECTOR, ipl2) Timer3Handler(void)
//{
//  if (turned_on == 1 && first_pass == 0) {
//    enable = 0;
//    first_pass = 1;
//  } else if (turned_on == 1 && first_pass == 1) {
//    enable = 1;
//    turned_on = 0;
//    first_pass = 0;
//  } else if (turned_off == 1) {
//    enable = 0;
//    turned_off = 0;
//  }
//  mT3ClearIntFlag();
//}

/*****************************[ Threads ]**************************************/
// === Blinking Thread =================================================
// Blink a LED to make sure we're alive 1 Hz
static int status = 0;
static PT_THREAD (protothread_blink(struct pt *pt))
{
    PT_BEGIN(pt);
    while(1) {
        // toggle_RedLED();
        PT_YIELD_TIME_msec(500);
    }
    PT_END(pt);
} // blink

// === Move Thread =================================================
// Move a stepper motor
static PT_THREAD (protothread_move(struct pt *pt))
{
    PT_BEGIN(pt);
    stp_1.stps_left = NUM_STEPS_X;
    clear_RedLED(); clear_GreenLED();
    while(read_mat_load() == 0) {
        set_RedLED();
    }
    clear_RedLED(); clear_GreenLED();
    set_dir_x(0);
    enable_x();
    PT_YIELD_TIME_msec(2);
    while(stp_1.stps_left >0)
      PT_YIELD(&pt_move);
    disable_x();
    //PT_YIELD_TIME_msec(2000);
//    enable_x();
//    PT_YIELD_TIME_msec(2);
////    stp_1.stps_left = NUM_STEPS;
//    while(stp_1.stps_left>0)
//      PT_YIELD(&pt_move);
    keep_moving = 1;
    //halting until the desired position is reached
    PT_YIELD_UNTIL(&pt_move, keep_moving == 0);
    while(1) {
        PT_YIELD(&pt_move);
    }
    PT_END(pt);
} // timer thread



int main(void)
{
    // === Config timer and output compares to make pulses ========
    // set up timer2 to generate the wave period
    OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_1, period_xyz);
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);
    mT2ClearIntFlag(); // and clear the interrupt flag
    
//    // Setup timer3 - Z axis
//    OpenTimer3(T3_ON | T3_SOURCE_INT | T3_PS_1_1, enable_time_break);
//    ConfigIntTimer3(T3_INT_ON | T3_INT_PRIOR_2);
//    mT3ClearIntFlag(); // and clear the interrupt flag
    
    // === now the threads ===================================
    // Setup the threads
    PT_setup();
  
    // === setup system wide interrupts  ====================
    INTEnableSystemMultiVectoredInt();
  
    // init the threads
    PT_INIT(&pt_move);
    PT_INIT(&pt_blink);
 
    init_RedLED();  init_GreenLED();
    init_steppers();
    init_limit_switches();
    
    // schedule the threads
    int q=0;
    while(q<10000000){
        q++;
    }
    while(1) {
      PT_SCHEDULE(protothread_move(&pt_move));
      PT_SCHEDULE(protothread_blink(&pt_blink));
    }
} // main
