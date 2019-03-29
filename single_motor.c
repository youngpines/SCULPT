#include "config.h"
#include "pt_cornell_1_2_3.h"
#include <stdlib.h>
#include <math.h>

#define NUM_STEPS 5000
// Macros to enable pull down resistors
#define EnablePullDownB(bits) CNPUBCLR=(bits); CNPDBSET=(bits)
#define EnablePullDownA(bits) CNPUACLR=(bits); CNPDASET=(bits)
#define LEDPIN BIT_3
#define init_LED()               \
  mPORTBSetPinsDigitalOut(LEDPIN); \
  mPORTBClearBits(LEDPIN)
#define toggle_LED() mPORTBToggleBits(LEDPIN)
#define set_LED()    mPORTBSetBits(LEDPIN)
#define clear_LED()  mPORTBClearBits(LEDPIN)

/*****************************[ Structs ]**************************************/
// Stepper Struct for DRV8825
typedef struct {
    int DIR;                 // DIRection pin
    int STP;                 // STeP pin
    int dir_move;            // Which direction to move
    volatile int steps_left; // Steps left on the current stepper
} stepper_t;
struct stepper_t stp1;
// Protothreads Structs
static struct pt pt_move, pt_blink;

/************************[ Shared Variables ]**********************************/
volatile static int keep_moving = 1; // Determines if target's been met
// Frequency of stepping (max), currently at a step ever 1 msec (2 interrupts)
static const int generate_period_xy =  20000;
// Frequency of z stepper rotation, currently once every 6 msec
static const int generate_period_z = 120000;

/*****************************[ ISRs ]*****************************************/
// == Timer 2 ISR =====================================================
// Steps the stepper motor
void __ISR(_TIMER_2_VECTOR, ipl2) Timer2Handler(void)
{
    // Turn on LED during duration of pulse
    if (stp1.stpLeft == NUM_STEPS) 
        set_LED();
    else if (stp1.stpLeft == 0) 
        clear_LED();
    // Pulse until there's no steps left
    if(stp1.stpLeft > 0){
        mPORTBToggleBits(stp1.STP);
        stp1.stpLeft--;      
    }
    // Clear the interrupt flag
    mT2ClearIntFlag();
}

/*****************************[ Threads ]**************************************/
// === Blinking Thread =================================================
// Blink a LED to make sure we're alive 1 Hz
static int status = 0;
static PT_THREAD (protothread_blink(struct pt *pt))
{
    PT_BEGIN(pt);
    while(1) {
        toggle_LED();
        PT_YIELD_TIME_msec(500);
    }
    PT_END(pt);
} // blink

// === Move Thread =================================================
// Move a stepper motor
static PT_THREAD (protothread_move(struct pt *pt))
{
    PT_BEGIN(pt);
    stp1.stpLeft = NUM_STEPS;
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
    OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_1, generate_period_xy);
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);
    mT2ClearIntFlag(); // and clear the interrupt flag
  
    // === now the threads ===================================
    // Setup the threads
    PT_setup();
  
    // === setup system wide interrupts  ====================
    INTEnableSystemMultiVectoredInt();
  
    // init the threads
    PT_INIT(&pt_move);
    PT_INIT(&pt_blink);
 
    //DECLARING BITS FOR STEPPER PINS (Stepper 1)
    mPORTBSetPinsDigitalOut(BIT_0);
    mPORTBSetPinsDigitalOut(BIT_1);
    mPORTBSetBits(BIT_0);
    mPORTBClearBits(BIT_1);
    mPORTBSetPinsDigitalOut(BIT_13);
    mPORTBSetBits(BIT_13);
    
    //DECLARING BITS FOR STEPPER PINS (Stepper 2)
    mPORTBSetPinsDigitalOut(BIT_4);
    mPORTBSetPinsDigitalOut(BIT_5);
    mPORTBSetBits(BIT_4);
    mPORTBClearBits(BIT_5);
    mPORTBSetPinsDigitalOut(BIT_12);
    mPORTBSetBits(BIT_12);
    
    
    //DECLARING BITS FOR STEPPER PINS (Stepper 3)
    mPORTBSetPinsDigitalOut(BIT_8);
    mPORTBSetPinsDigitalOut(BIT_9);
    mPORTBSetPinsDigitalOut(BIT_10);
    mPORTBSetPinsDigitalOut(BIT_11);
    mPORTBClearBits(BIT_8);
    mPORTBClearBits(BIT_9);
    mPORTBClearBits(BIT_10);
    mPORTBClearBits(BIT_11);
    
    //setting direction pins for the motors
    stp1.DIR = BIT_0; 
    stp1.STP = BIT_1;
    stp1.dirMove = 0;
    stp1.stpLeft = 0;
    
    // schedule the threads
    init_LED();
    int q=0;
    while(q<10000000){
        q++;
    }
    while(1) {
      PT_SCHEDULE(protothread_move(&pt_move));
      PT_SCHEDULE(protothread_blink(&pt_blink));
    }
} // main
