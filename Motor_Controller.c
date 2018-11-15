
#include "config.h"
// threading library
#include "pt_cornell_1_2_1.h"
// need for rand function
#include <stdlib.h>
#include <math.h>

//struct to hold info for each stepper motor
struct Stepper_Motor{
    int DIR;    //the direction pin for a selected stepper
    int dirMove;    //the direction to move
    int STP;    //the pin connected to the step function for a stepper
    volatile int stpLeft;    //steps left on the current stepper
    
};

//struct for the DC motor
struct DC_Motor{
    int on; //whether or not the motor is on
    int ENA;    //the pin for the PWM to control on duration
};

// === thread structures ============================================
//structs for threads
static struct pt pt_data,   //protothread to convert data to stepper locations
        pt_move;             //stepper calculations to get to positions


struct Stepper_Motor stp1, stp2, stp3;     //the stepper motors in use
struct DC_Motor dc1;                             //dc motor in use

volatile SpiChannel spiChn = SPI_CHANNEL2 ;	 // the SPI channel to use
volatile int spiClkDiv = 2 ;                 // 20 MHz DAC clock

//keep stepping or not
volatile static int keep_moving = 0;

// frequency of motor stepping (max)
int generate_period = 20000;

// == Timer 2 ISR =====================================================
// keeps timing for PWM
void __ISR(_TIMER_2_VECTOR, ipl2) Timer2Handler(void)
{
    
    //move z first
    if(stp1.stpLeft > 0){
        mPORTBToggleBits(stp1.STP);
        stp1.stpLeft--;
    }
    
    //otherwise move x and y
    //tentative could maybe break into x and y separately
    else {
        if(stp2.stpLeft > 0){
            mPORTBToggleBits(stp2.STP);
            stp2.stpLeft--;
        }
        if(stp3.stpLeft > 0){
            mPORTBToggleBits(stp3.STP);
            stp3.stpLeft--;
        }
    }
    if(stp1.stpLeft <= 0 &&  stp2.stpLeft <= 0 && stp3.stpLeft <= 0)
        keep_moving = 0;
    // clear the timer interrupt flag
    mT2ClearIntFlag();
}

//// == Timer 3 ISR =====================================================
//
//// ISR
//static int i;
//void __ISR(_TIMER_3_VECTOR, ipl2) Timer3Handler(void)
//{    
//    /*TODO: Implement a means to turn until the steppers have completed*/
//    for(i=0; i<3; i++){
//        
//    }
//}

// === Move Thread =================================================
/*TODO: STEPPER CALCS*/
//calculate how many rotations each stepper needs to reach the next location
//also will determine if needs to raise or not between locations
static PT_THREAD (protothread_move(struct pt *pt))
{
    PT_BEGIN(pt);

    while(1) {
        
        stp1.stpLeft = 1000;
        stp2.stpLeft = 1000;
        stp3.stpLeft = 1000;
        
        //allowing the ISR to move the position
        keep_moving = 1;
        
        //halting until the desired position is reached
        PT_YIELD_UNTIL(&pt_move, keep_moving == 0); 

        //flipping directions for lols
        stp1.dirMove = 1- stp1.dirMove;
        stp2.dirMove = 1- stp2.dirMove;
        stp3.dirMove = 1- stp3.dirMove;
        
        //handling direction changes for stp1
        if(stp1.dirMove == 1) mPORTBSetBits(stp1.DIR);
        else mPORTBClearBits(stp1.DIR);
        
        //handling direction changes for stp1
        if(stp2.dirMove == 1) mPORTBSetBits(stp2.DIR);
        else mPORTBClearBits(stp2.DIR);
        
        //handling direction changes for stp1
        if(stp3.dirMove == 1) mPORTBSetBits(stp3.DIR);
        else mPORTBClearBits(stp3.DIR);
        
        
    } // END WHILE(1)
    PT_END(pt);
} // timer thread



// === DataProcess Thread =================================================
/*TODO: Convert image info to locations*/
static PT_THREAD (protothread_data(struct pt *pt))
{
    PT_BEGIN(pt);
    while(1){

        PT_YIELD_TIME_msec(1000);
        
        //write to tft time for dubugging
    } // END WHILE(1)
  PT_END(pt);
} // animation thread

int main(void)
{
    
    // === Config timer and output compares to make pulses ========
    // set up timer2 to generate the wave period
    OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_1, generate_period);
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);
    mT2ClearIntFlag(); // and clear the interrupt flag
  
    // === now the threads ===================================
    // Setup the threads
    PT_setup();
  
    // === setup system wide interrupts  ====================
    INTEnableSystemMultiVectoredInt();
  
    // init the threads
    PT_INIT(&pt_data);
    PT_INIT(&pt_move);
    
    
    //_________________________________________________
    //ONLY FOR DEBUGGING
    
    /// SPI setup //////////////////////////////////////////
    // SCK2 is pin 26 
    // SDO2 is in PPS output group 2, could be connected to RB5 which is pin 14
//    PPSOutput(2, RPB5, SDO2);
//    // control CS for DAC
//    mPORTBSetPinsDigitalOut(BIT_4);
//    mPORTBSetBits(BIT_4);
//
//    // divide Fpb by 2, configure the I/O ports. Not using SS in this example
//    // 16 bit transfer CKP=1 CKE=1
//    // possibles SPI_OPEN_CKP_HIGH;   SPI_OPEN_SMP_END;  SPI_OPEN_CKE_REV
//    // For any given peripherial, you will need to match these
//    SpiChnOpen(spiChn, SPI_OPEN_ON | SPI_OPEN_MODE16 | SPI_OPEN_MSTEN | SPI_OPEN_CKE_REV , spiClkDiv);
//________________________________________________________________________________
//END DEBUG SECTION
    
    //DECLARING BITS FOR STEPPER PINS (Stepper 1)
    mPORTBSetPinsDigitalOut(BIT_0);
    mPORTBSetPinsDigitalOut(BIT_1);
    mPORTBClearBits(BIT_0);
    mPORTBClearBits(BIT_1);
    
    
    //DECLARING BITS FOR STEPPER PINS (Stepper 2)
    mPORTBSetPinsDigitalOut(BIT_4);
    mPORTBSetPinsDigitalOut(BIT_5);
    mPORTBClearBits(BIT_4);
    mPORTBClearBits(BIT_5);
    
    //DECLARING BITS FOR STEPPER PINS (Stepper 3)
    mPORTBSetPinsDigitalOut(BIT_8);
    mPORTBSetPinsDigitalOut(BIT_9);
    mPORTBClearBits(BIT_8);
    mPORTBClearBits(BIT_9);
    
    //setting direction pins for the motors
    stp1.DIR = BIT_0;
    stp2.DIR = BIT_4;
    stp3.DIR = BIT_8;
    
    //setting direction for the motors
    stp1.dirMove = 0;
    stp2.dirMove = 0;
    stp3.dirMove = 0;

    //setting step ppins for motors
    stp1.STP = BIT_1;
    stp2.STP = BIT_5;
    stp3.STP = BIT_9;

    //setting the steps remaining for each motor
    stp1.stpLeft = 0;
    stp2.stpLeft = 0;
    stp3.stpLeft = 0;

    //setting up the dc motor
    //dc1.ENA = BIT_8;
    //dc1.on = 0;

    // schedule the threads
    while(1) {
        PT_SCHEDULE(protothread_data(&pt_data));
        PT_SCHEDULE(protothread_move(&pt_move));
    }
} // main
