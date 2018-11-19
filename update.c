
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

//a state to determine if the drill is lowered currently or not
volatile static int lowered = 0;

//a state variable to determine if the image is carved yet or not
volatile static int carved = 0;

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

    //the x and y positions in the image array
    static int x_pos, y_pos;

    //looping through all entries in the image array
    for(x_pos = 0; x_pos < x_max; x++){         //x coordinates iterated slowly
        for(y_pos = 0; y_pos < y_max; y++{      //individual movement along the y

            //TODO: RECALC of DISTANCE for IMAGE ARRAY ENTRY (y and z)


            if(image[x_pos][y_pos] > 127){  //requires z to be raised

                //turn off the dc motor first
                mPORTAClearBits(dc1.ENA);
                dc1.on = 0;
                
                //checking if the z is raised or not
                if(lowered ==1){    //currently lowered and needs to raise

                    //set the state to move the opposite direction
                    lowered = 0;
                    //raise the z axis
                    z.stpLeft = 8000;

                }

                else{               //already raised
                    z.stpLeft = 0;
                }         
            }


            else{   // requires z to be lowered
                
                //turn on the dc motor first
                mPORTASetBits(dc1.ENA);
                dc1.on = 1;
                
                //checking if the z is raised or not
                if(lowered ==0){    //currently raised and needs to lower

                    //set the state to move the opposite direction
                    lowered = 1;
                    //lower the z axis
                    z.stpLeft = 8000;

                }

                else{               //already lowered
                    z.stpLeft = 0;
                }  
            }

            //setting the steps to move in y
            stp2.stpLeft = 400;

            //setting up for the ISR to move the position
            keep_moving = 1;
            //halting until the desired position is reached
            PT_YIELD_UNTIL(&pt_move, keep_moving == 0); 
        }

            
            
        //turn off the dc motor first
        mPORTAClearBits(dc1.ENA);
        dc1.on = 0;
            
        //handling resetting at the top of the x pos (NOT SURE WHICH DIRECTION YET)
        mPORTBSetBits(stp2.DIR); 
        
        //checking if z is raised or not, if not raising it
        if(lowered ==1){    //currently lowered and needs to raise

            //set the state to move the opposite direction
            lowered = 0;
            //raise the z axis
            z.stpLeft = 8000;

        }
        else    z.stpLeft = 0;      //already raised
        
        //resetting the y location
        stp2.stpLeft = 400 * y_max;
            
        //stepping the appropriate amount in x
        stp1.stpLeft = 400;

        //setting up for the ISR to move the position
        keep_moving = 1;
        //halting until the desired position is reached
        PT_YIELD_UNTIL(&pt_move, keep_moving == 0); 
            
        //return to the original direction of travel for the stepper
        mPORTBClearBits(stp2.DIR);

    }
   
            
    //final reset for z and dc motor        
    //turn off the dc motor first
    mPORTAClearBits(dc1.ENA);
    dc1.on = 0;

    //checking if z is raised or not, if not raising it
    if(lowered ==1){    //currently lowered and needs to raise

        //set the state to move the opposite direction
        lowered = 0;
        //raise the z axis
        z.stpLeft = 8000;

    }         
  
    //tell the align and data thread that the image has been carved
    carved = 1;
            
    //once done working through the image array just yield
    PT_YIELD();
            
    PT_END(pt);
} // timer thread



// === DataProcess Thread =================================================
/*TODO: Convert image info to locations*/
            
//this thread first aligns the machine as well as sets the dimensions of soap
//then it allows the image array to be iterated through in another array
//and finally returns to the default state
static PT_THREAD (protothread_data(struct pt *pt))
{
    PT_BEGIN(pt);


    //import data
    //align to edge of soap
    //yield until carving done
    PT_YIELD_UNTIL(&pt_data, carved == 1);
    //align to feed out soap

    //spin in the end
    while(1);

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
