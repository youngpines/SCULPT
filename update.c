
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

struct DC_Motor{    //dc motor, controlled by two pins, no PWM as full power wanted always
    int on;     //whether or not the motor is on
    int ENA;    //enabling the motor to spin
};


static struct pt pt_data,   //protothread to convert data to stepper locations
        pt_move;             //stepper calculations to get to positions


struct Stepper_Motor stp1, stp2;     //the stepper motors in use, easy stepper driver board
struct Stepper_Motor_L stp3;        // L298 stepper motor driver
struct DC_Motor dc1;                //dc motor 

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
    //check if z axis is done moving first
    if(stp3.stpLeft > 0){
    }
   
    //move x and y if z is complete
    else {
        //y movement
        if(stp2.stpLeft > 0){
            mPORTBToggleBits(stp2.STP);
            stp2.stpLeft--;
        }
        //x movement
        if(stp1.stpLeft > 0){
            mPORTBToggleBits(stp1.STP);
            stp1.stpLeft--;
        }
    }
    
    //checking if all motors are done stepping, and if so then updating the state
    if(stp1.stpLeft <= 0 &&  stp2.stpLeft <= 0 && stp3.stpLeft <= 0) keep_moving = 0;
    
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
            }
        }
    
        //clear the interrupt flag
        mT3ClearIntFlag();
}
//==================================================================


// === Move Thread =================================================
/*TODO: STEPPER CALCS*/
//calculate how many rotations each stepper needs to reach the next location
//also will determine if needs to raise or not between locations

//this thread finds the next location to travel to, and calculates the steps needed to get there
//then it sets those steps for each axis and yields until the motion is complete
//the DC motor is turned on, or remains on, if the location is to be removed
static PT_THREAD (protothread_move(struct pt *pt))
{
    PT_BEGIN(pt);

    //infinite loop for the finding of new locations and moving to them
    while(1) {
        
        //the x and y positions in the image array
        static int x_pos, y_pos;
        
        //looping through all entries in the image array
        for(x_pos = 0; x_pos < x_max; x++){         //x coordinates iterated slowly
            for(y_pos = 0; y_pos < y_max; y++{      //individual movement along the y
                
                //TODO: CALC of DISTANCE for IMAGE ARRAY ENTRY (y and z)
                
                
                //setting up for the ISR to move the position
                keep_moving = 1;
                //halting until the desired position is reached
                PT_YIELD_UNTIL(&pt_move, keep_moving == 0); 
                
                
                
                
                
            }
                
            //handling resetting at the top of the x pos (NOT SURE WHICH DIRECTION YET)
            mPORTBSetBits(stp2.DIR);
            //mPORTBClearBits(stp2.DIR);
                
            //set the steps for moving to the x and resetting y and raising z
                
                
            //setting up for the ISR to move the position
            keep_moving = 1;
            //halting until the desired position is reached
            PT_YIELD_UNTIL(&pt_move, keep_moving == 0); 
        
        }
            
            
            
            
        stp1.stpLeft = 1000;
        stp2.stpLeft = 1000;
        stp3.stpLeft = 1000;
        

        //handling direction changes for stp1
        if(stp1.dirMove == 1) mPORTBSetBits(stp1.DIR);
        else mPORTBClearBits(stp1.DIR);
        
        //handling direction changes for stp1
        //if(stp2.dirMove == 1) mPORTBSetBits(stp2.DIR);
        ///else mPORTBClearBits(stp2.DIR);
       
        
        
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
    
    OpenTimer3(T3_ON | T3_SOURCE_INT | T3_PS_1_1,120000);
    ConfigIntTimer3(T3_INT_ON | T3_INT_PRIOR_2);
    mT3ClearIntFlag(); // and clear the interrupt flag
  
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
    mPORTBSetPinsDigitalOut(BIT_10);
    mPORTBSetPinsDigitalOut(BIT_11);
    mPORTBClearBits(BIT_8);
    mPORTBClearBits(BIT_9);
    mPORTBClearBits(BIT_10);
    mPORTBClearBits(BIT_11);
    
    //setting direction pins for the motors
    stp1.DIR = BIT_0;
    stp2.DIR = BIT_4;
    stp3.DIR = BIT_8;
    
    //setting direction for the motors
    stp1.dirMove = 0;
    stp2.dirMove = 0;
    stp3.NDIR = BIT_9;
    stp3.NSTP = BIT_11;

    //setting step ppins for motors
    stp1.STP = BIT_1;
    stp2.STP = BIT_5;
    stp3.STP = BIT_10;

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
