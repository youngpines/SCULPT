
#include "config.h"
#include "pt_cornell_1_2_1.h"
#include <stdlib.h>
#include <math.h>


struct Stepper_Motor{       //for easy stepper motor driver
    int DIR;    //the direction pin for a selected stepper
    int dirMove;    //the direction to move
    int STP;    //the pin connected to the step function for a stepper
    volatile int stpLeft;    //steps left on the current stepper
    
};

struct Stepper_Motor_L{     //for stepper motor driver L298N
    int DIR;        //first input pin
    int NDIR;       //second input pin, inverse of DIR
    int STP;        //third input pin
    int NSTP;       //fourth input pin, inverse of STP
    volatile int stpLeft;    //steps left on the current stepper
    
};



static struct pt pt_move;             //stepper controls


struct Stepper_Motor stp1;     //the stepper motors in use, easy stepper driver board
struct Stepper_Motor_L stp3;        // L298 stepper motor driver

//a state variable that determines if the target location has been reached
volatile static int keep_moving = 0;

// frequency of x and y motor stepping (max), currently at a step ever 1 msec (2 interrupts)
static const int generate_period_xy = 20000;

//frequency of z stepper rotation, currently once every 6 msec
static const int generate_period_z = 120000;

// == Timer 2 ISR =====================================================
// steps the x and y motors
void __ISR(_TIMER_2_VECTOR, ipl2) Timer2Handler(void)
{

      //easy driver stepper
      if(stp1.stpLeft > 0){
          mPORTBToggleBits(stp1.STP);
          stp1.stpLeft--;
    }
    
    //checking if all motors are done stepping, and if so then updating the state
    if(stp1.stpLeft <= 0 && stp3.stpLeft <= 0) keep_moving = 0;
    
    //clearing the interrupt flag
    mT2ClearIntFlag();
}

// ====================================================================



// == Timer 3 ISR =====================================================
//a state variable to track what configuration the L298N stepper driver needs for input
static volatile int ct = 0;

//this interrupt only focuses on moving the z axis (always moves first if steps left)
void __ISR(_TIMER_3_VECTOR, ipl2) Timer3Handler(void)
{    
        //checking to make sure z still needs to step
        if(stp3.stpLeft > 0){

              //determining which state the input needs and setting
              //the appropriate bits of the stepper motor driver
              if(ct==0){ //1010
                  mPORTBSetBits(stp3.DIR);
                  mPORTBSetBits(stp3.STP);
                  mPORTBClearBits(stp3.NDIR);
                  mPORTBClearBits(stp3.NSTP);
                  ct++;
              }
              else if(ct==1){ //0110
                  mPORTBSetBits(stp3.NDIR);
                  mPORTBSetBits(stp3.STP);
                  mPORTBClearBits(stp3.DIR);
                  mPORTBClearBits(stp3.NSTP);
                  ct++;
              }
              else if(ct==2){  //0101
                  mPORTBSetBits(stp3.NDIR);
                  mPORTBSetBits(stp3.NSTP);
                  mPORTBClearBits(stp3.DIR);
                  mPORTBClearBits(stp3.STP);
                  ct++;
              }
              else if(ct==3){ //1001
                  mPORTBSetBits(stp3.DIR);
                  mPORTBSetBits(stp3.NSTP);
                  mPORTBClearBits(stp3.NDIR);
                  mPORTBClearBits(stp3.STP);
                  ct = 0;
                  stp3.stpLeft--;
              }
        }
        //clear the interrupt flag
        mT3ClearIntFlag();
}
//==================================================================


// === Move Thread =================================================
//temp setup to just turn a set amount for a given motor
static PT_THREAD (protothread_move(struct pt *pt))
{
    PT_BEGIN(pt);
    
    
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //ONLY NEED TO MESS WITH THESE VALUES
    
    
    
    //modify the integer value on the right to get an appropriate travel distance
    //stp1.stpLeft = 800;
    stp3.stpLeft = 800;
    
    
    
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
    
    //setting up for the ISR to move the position
    keep_moving = 1;
    //halting until the desired position is reached
    PT_YIELD_UNTIL(&pt_move, keep_moving == 0);
           
            
    PT_END(pt);
} // timer thread



int main(void)
{
    
    // === Config timer and output compares to make pulses ========
    // set up timer2 to generate the wave period
    OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_1, generate_period_xy);
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);
    mT2ClearIntFlag(); // and clear the interrupt flag
    
    OpenTimer3(T3_ON | T3_SOURCE_INT | T3_PS_1_1,generate_period_z);
    ConfigIntTimer3(T3_INT_ON | T3_INT_PRIOR_2);
    mT3ClearIntFlag(); // and clear the interrupt flag
  
    // === now the threads ===================================
    // Setup the threads
    PT_setup();
  
    // === setup system wide interrupts  ====================
    INTEnableSystemMultiVectoredInt();
  
    // init the threads
    PT_INIT(&pt_move);
    
    
 
    //DECLARING BITS FOR STEPPER PINS (Stepper 1)
    mPORTBSetPinsDigitalOut(BIT_0);
    mPORTBSetPinsDigitalOut(BIT_1);
    mPORTBClearBits(BIT_0);
    mPORTBClearBits(BIT_1);
    
    
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
    stp3.DIR = BIT_8;
    
    //setting direction for the motors
    stp1.dirMove = 0;
    stp3.NDIR = BIT_9;
    stp3.NSTP = BIT_11;

    //setting step ppins for motors
    stp1.STP = BIT_1;
    stp3.STP = BIT_10;

    //setting the steps remaining for each motor
    stp1.stpLeft = 0;
    stp3.stpLeft = 0;

    // schedule the threads
    int q=0;
    while(q<10000000){
        q++;
    }
    PT_SCHEDULE(protothread_move(&pt_move));

} // main
