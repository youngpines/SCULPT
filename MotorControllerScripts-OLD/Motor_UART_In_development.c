////////////////////////////////////
// clock AND protoThreads configure!
// You MUST check this file!
#include "config.h"
// threading library
#define use_uart_serial
#include "pt_cornell_1_2_3.h"
////////////////////////////////////

////////////////////////////////////
// graphics libraries
// SPI channel 1 connections to TFT
#include "tft_master.h"
#include "tft_gfx.h"
// need for rand function
#include <stdlib.h>
// need for sin function
#include <math.h>
////////////////////////////////////


//image size parameters
#define x_dim 5
#define y_dim 10

//image data array that holds pixelated info of image
static unsigned short image[x_dim][y_dim];


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

// The following threads are necessary for UART control
static struct pt pt_input, pt_output, pt_DMA_output;

struct Stepper_Motor stp1, stp2;     //the stepper motors in use, easy stepper driver board
struct Stepper_Motor_L stp3;        // L298 stepper motor driver
struct DC_Motor dc1;                //dc motor 

//a state variable that determines if the target location has been reached
volatile int keep_moving = 0;

//a state to determine if the drill is lowered currently or not
volatile int lowered = 0;

//a state variable to determine if the image is carved yet or not
volatile int carved = 0;

// frequency of x and y motor stepping (max), currently at a step ever 1 msec (2 interrupts)
#define generate_period_xy  20000

//frequency of z stepper rotation, currently once every 6 msec
#define generate_period_z  120000

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
            if(lowered = 1){    //turn one direction to lower
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
            
            else{    //turn opposite direction to raise 
                //determining which state the input needs and setting
                //the appropriate bits of the stepper motor driver
                if(ct==0){ //0101
                    mPORTBSetBits(stp3.NDIR);
                    mPORTBSetBits(stp3.NSTP);
                    mPORTBClearBits(stp3.DIR);
                    mPORTBClearBits(stp3.STP);
                    ct++;
                }
                else if(ct==1){ //1001
                    mPORTBSetBits(stp3.DIR);
                    mPORTBSetBits(stp3.NSTP);
                    mPORTBClearBits(stp3.NDIR);
                    mPORTBClearBits(stp3.STP);
                    ct++;
                }
                else if(ct==2){  //1010
                    mPORTBSetBits(stp3.DIR);
                    mPORTBSetBits(stp3.STP);
                    mPORTBClearBits(stp3.NDIR);
                    mPORTBClearBits(stp3.NSTP);
                    ct++;
                }
                else if(ct==3){ //0110
                    mPORTBSetBits(stp3.NDIR);
                    mPORTBSetBits(stp3.STP);
                    mPORTBClearBits(stp3.DIR);
                    mPORTBClearBits(stp3.NSTP);
                    ct = 0;
                }
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

//currently each change in x and y is 400 steps in the x and y direction
//
static PT_THREAD (protothread_move(struct pt *pt))
{
    PT_BEGIN(pt);

    //the x and y positions in the image array at start target positions at end of loops
    static int x_pos, y_pos;

    mPORTBSetBits(stp1.DIR);    //ensure that x is going the correct
                                //direction
    
    
    //looping through all entries in the image array
    for(x_pos = 0; x_pos < x_dim; x_pos++){         //x coordinates iterated slowly
        
        mPORTBSetBits(stp2.DIR);    //ensure that y is going the correct
                                    //direction
        
        
        //this loop looks at the next y coordinate for the current x and determines
        //if the drill needs to be raised or lowered for that entry
        
        //then the y is step forward to the future entry and the process is redone
        for(y_pos = -1; y_pos < y_dim-1; y_pos++){      //looking ahead requires the shift by 1

            //TODO: RECALC of DISTANCE for IMAGE ARRAY ENTRY (y and z)
            

            if(image[x_pos][y_pos+1] > 127){  //requires z to be raised
                
                //turn off the dc motor first
                mPORTAClearBits(dc1.ENA);
                dc1.on = 0;
                
                //checking if the z is raised or not
                if(lowered ==1){    //currently lowered and needs to raise

                    //set the state to move the opposite direction
                    lowered = 0;
                    //raise the z axis
                    stp3.stpLeft = 1000;
                }

                else{               //already raised
                    stp3.stpLeft = 0;
                }         
            }


            else{   // requires z to be lowered

                //checking if the z is raised or not
                if(lowered ==0){    //currently raised and needs to lower
                    
                    //turn on the dc motor first
                    mPORTASetBits(dc1.ENA);
                    dc1.on = 1;
                    
                    //set the state to move the opposite direction
                    lowered = 1;
                    
                    //lower the z axis
                    stp3.stpLeft = 8000;

                }

                else{               //already lowered
                    stp3.stpLeft = 0;
                }  
            }

            //setting the steps to move in y
            stp2.stpLeft = 400;

            //setting up for the ISR to move the position
            keep_moving = 1;
            //halting until the desired position is reached
            PT_YIELD_UNTIL(&pt_move, keep_moving == 0); 
        }       //end of action for every 

            
            
        //turn off the dc motor first to preserve integrity of piece 
        mPORTAClearBits(dc1.ENA);
        dc1.on = 0;
            
        //handling changing direction for y to return to top of next column
        mPORTBClearBits(stp2.DIR); 
        
        //checking if z is raised or not, if not raising it
        if(lowered ==1){    //currently lowered and needs to raise

            //set the state to move the opposite direction
            lowered = 0;
            //raise the z axis
            stp3.stpLeft = 8000;

        }
        else    stp3.stpLeft = 0;      //already raised
        
        //resetting the y location
        stp2.stpLeft = 400 * (y_dim-1);
            
        //stepping the appropriate amount in x
        stp1.stpLeft = 400;

        //setting up for the ISR to move the position
        keep_moving = 1;
        //halting until the desired position is reached
        PT_YIELD_UNTIL(&pt_move, keep_moving == 0);
    }
   
            
    //final reset for dc motor        
    //turn off the dc motor 
    mPORTAClearBits(dc1.ENA);
    dc1.on = 0;
    
    //tell the align and data thread that the image has been carved
    carved = 1;
            
    //once done working through the image array just yield
    //PT_YIELD(&pt_move);
            
    PT_END(pt);
} // timer thread



/* Demo code for interfacing TFT (ILI9340 controller) to PIC32
 * The library has been modified from a similar Adafruit library
 */
// Adafruit data:
/***************************************************
  This is an example sketch for the Adafruit 2.2" SPI display.
  This library works with the Adafruit 2.2" TFT Breakout w/SD card
  ----> http://www.adafruit.com/products/1480
  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!
  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/
// string buffer
char buffer[60];

// === thread structures ============================================
// thread control structs
// note that UART input and output are threads
static struct pt pt_timer, pt_serial;
// The following threads are necessary for UART control
static struct pt pt_input, pt_output, pt_DMA_output;

// === Timer Thread =================================================
// update a 1 second tick counter
int blink_freq = 500;
static PT_THREAD (protothread_timer(struct pt *pt))
{
    PT_BEGIN(pt);
     // set up LED to blink
     mPORTASetBits(BIT_0 );	//Clear bits to ensure light is off.
     mPORTASetPinsDigitalOut(BIT_0 );    //Set port as output
     
      while(1) {
        // yield time 1 second
        PT_YIELD_TIME_msec(blink_freq) ;
        // toggle the LED on the big board
        mPORTAToggleBits(BIT_0);
        // NEVER exit while
      } // END WHILE(1)
    PT_END(pt);
} // timer thread

//=== Serial terminal thread =================================================
static PT_THREAD (protothread_serial(struct pt *pt))
{
    PT_BEGIN(pt);
      static char cmd[30];
      static int value;
      mPORTBSetBits(BIT_0 );	//Clear bits to ensure light is off.
      mPORTBSetPinsDigitalOut(BIT_0 );    //Set port as output
      tft_fillRoundRect(0,10, 90, 14, 1, ILI9340_BLACK);// x,y,w,h,radius,color
      tft_setCursor(10, 10);
      tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(1);
      sprintf(buffer,"Starting");
      tft_writeString(buffer);
      static int count = 0;
      while(1) {
          
          //count++;
            // toggle the LED on the big board
            mPORTBToggleBits(BIT_0);
            // send the prompt via DMA to serial
            sprintf(PT_send_buffer,"cmd>");
            // by spawning a print thread
            PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output) );
            //mPORTASetBits(BIT_0);
          //spawn a thread to handle terminal input
            // the input thread waits for input
            // -- BUT does NOT block other threads
            // string is returned in "PT_term_buffer"
            PT_SPAWN(pt, &pt_input, PT_GetSerialBuffer(&pt_input) );
            // returns when the thead dies
            // in this case, when <enter> is pushed
            // now parse the string
             sscanf(PT_term_buffer, "%s %d", cmd, &value);
             count++;
             switch(cmd[0]){
                 case 'p': // set frequency of DAC sawtooth output
                     // enter frequency in HZ
                     blink_freq = value;
                     image[count%x_dim][count/y_dim] = value;
                     break;
                 case 'e': 
                     tft_fillRoundRect(50,50, 100, 100, 1, ILI9340_BLACK);// x,y,w,h,radius,color
            
                    tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);

                    tft_setCursor(50, 80);
                    sprintf(buffer,"[%d, %d] = %d", count%x_dim, count/y_dim, count);
                    tft_writeString(buffer);
                    while(1);
                 default:
                     blink_freq = 500;                   
            break;
            }
           
            // never exit while
      } // END WHILE(1)
      
     while(1);
  PT_END(pt);
} // thread 3

// === Main  ======================================================
void main(void) {
 //SYSTEMConfigPerformance(PBCLK);
  
  ANSELA = 0; ANSELB = 0; 
// === Config timer and output compares to make pulses ========
    // set up timer2 to generate the wave period
    OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_1, generate_period_xy);
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);
    mT2ClearIntFlag(); // and clear the interrupt flag
    
    OpenTimer3(T3_ON | T3_SOURCE_INT | T3_PS_1_1,generate_period_z);
    ConfigIntTimer3(T3_INT_ON | T3_INT_PRIOR_2);
    mT3ClearIntFlag(); // and clear the interrupt flag
  // === config threads ==========
  // turns OFF UART support and debugger pin, unless defines are set
  PT_setup();

  // === setup system wide interrupts  ========
  INTEnableSystemMultiVectoredInt();
    // init the display
    tft_init_hw();
    tft_begin();
    tft_setRotation(0);
    tft_fillScreen(ILI9340_BLACK);
  
  // init the threads
  PT_INIT(&pt_timer);
  PT_INIT(&pt_serial);
  PT_INIT(&pt_move);
  // round-robin scheduler for threads
  
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
    
    //DECLARING BITS FOR DC MOTOR 
    mPORTBSetPinsDigitalOut(BIT_14);
    mPORTBClearBits(BIT_14);

    //setting direction pins for the motors
    stp1.DIR = BIT_0;
    stp2.DIR = BIT_4;
    stp3.DIR = BIT_8;
    
    //setting additional pins for the third stepper
    stp3.NDIR = BIT_9;
    stp3.NSTP = BIT_11;

    //setting step pins for motors
    stp1.STP = BIT_1;
    stp2.STP = BIT_5;
    stp3.STP = BIT_10;

    //setting the steps remaining for each motor
    stp1.stpLeft = 0;
    stp2.stpLeft = 0;
    stp3.stpLeft = 0;

    //setting up the dc motor
    dc1.ENA = BIT_14;
    dc1.on = 0;
  
  while (1){
      PT_SCHEDULE(protothread_timer(&pt_timer));
      PT_SCHEDULE(protothread_serial(&pt_serial));
      PT_SCHEDULE(protothread_serial(&pt_move));  
  }
  } // main
