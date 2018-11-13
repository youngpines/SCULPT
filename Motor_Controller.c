
#include "config.h"
// threading library
#include "pt_cornell_1_2_1.h"
#include "tft_master.h"
#include "tft_gfx.h"
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

// PWM setting variables
int generate_period = 40000;
//volatile static int pwm_on_time = 4000; 

// string buffer
char buffer[60];

// == Timer 2 ISR =====================================================
// keeps timing for PWM
void __ISR(_TIMER_2_VECTOR, ipl2) Timer2Handler(void)
{
    if(stp1.stpLeft > 0){
        mPORTBToggleBits(stp1.STP);
        stp1.stpLeft--;
    }
//    if(stp2.stpLeft > 0){
//        mPORTBToggleBits(stp2.STP);
//        stp2.stpLeft--;
//    }
//    if(stp3.stpLeft > 0){
//        mPORTBToggleBits(stp3.STP);
//        stp3.stpLeft--;
//    }
    if(stp1.stpLeft <= 0 )//&&  stp2.stpLeft =< 0 && stp3.stpLeft =< 0)
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
        //stp2.stpLeft = 1000;
        //stp3.stpLeft = 1000;
        
        //allowing the ISR to move the position
        keep_moving = 1;
        //ConfigIntTimer2(T2_INT_ON );
        
        PT_YIELD_UNTIL(&pt_move, keep_moving == 0); 
        
        //ConfigIntTimer2(T2_INT_OFF );
        mPORTBToggleBits(BIT_9);
        stp1.dirMove = 1- stp1.dirMove;
        
    } // END WHILE(1)
    PT_END(pt);
} // timer thread



// === DataProcess Thread =================================================
/*TODO: Convert image info to locations*/
static PT_THREAD (protothread_data(struct pt *pt))
{
    PT_BEGIN(pt);
    while(1){
        tft_fillScreen(ILI9340_BLACK);
        // print steps left
        tft_setCursor(20, 20);
        tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
        sprintf(buffer, "Steps Left: %d", stp1.stpLeft);
        tft_writeString(buffer);
        
        
        //print direction of movement
        tft_setCursor(20, 40);
        tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
        sprintf(buffer, "Direction: %d", stp1.dirMove);
        tft_writeString(buffer);
        
        
        PT_YIELD_TIME_msec(100);
        
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
    PPSOutput(2, RPB5, SDO2);
    // control CS for DAC
    mPORTBSetPinsDigitalOut(BIT_4);
    mPORTBSetBits(BIT_4);

    // divide Fpb by 2, configure the I/O ports. Not using SS in this example
    // 16 bit transfer CKP=1 CKE=1
    // possibles SPI_OPEN_CKP_HIGH;   SPI_OPEN_SMP_END;  SPI_OPEN_CKE_REV
    // For any given peripherial, you will need to match these
    SpiChnOpen(spiChn, SPI_OPEN_ON | SPI_OPEN_MODE16 | SPI_OPEN_MSTEN | SPI_OPEN_CKE_REV , spiClkDiv);
//________________________________________________________________________________
//END DEBUG SECTION
    
    // init the display
    tft_init_hw();
    tft_begin();
    tft_fillScreen(ILI9340_BLACK);
    //240x320 vertical display
    tft_setRotation(0); // Use tft_setRotation(1) for 320x240

    //DECLARING BITS FOR STEPPER PINS
    mPORTBSetPinsDigitalOut(BIT_9);
    mPORTBSetPinsDigitalOut(BIT_15);
    mPORTBClearBits(BIT_9);
    mPORTBClearBits(BIT_15);
    
    //setting direction pins for the motors
    //stp1.DIR = BIT_9;
    //stp2.DIR = BIT_1;
    //stp3.DIR = BIT_2;
    
    //setting direction for the motors
    stp1.dirMove = 0;
    //stp2.dirMove = 0;
    //stp3.dirMove = 0;

    //setting step ppins for motors
    stp1.STP = BIT_15;
    //stp2.STP = BIT_1;
    //stp3.STP = BIT_2;

    //setting the steps remaining for each motor
    stp1.stpLeft = 50;
    //stp2.stpLeft = 0;
    //stp3.stpLeft = 0;

    //setting up the dc motor
    //dc1.ENA = BIT_8;
    //dc1.on = 0;
    // print steps left
        tft_setCursor(20, 20);
        tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
        sprintf(buffer, "Steps Left: %d", stp1.stpLeft);
        tft_writeString(buffer);
        
        
        //print direction of movement
        tft_setCursor(20, 40);
        tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
        sprintf(buffer, "Direction: %d", stp1.dirMove);
        tft_writeString(buffer);

    // schedule the threads
    while(1) {
        PT_SCHEDULE(protothread_data(&pt_data));
        PT_SCHEDULE(protothread_move(&pt_move));
    }
} // main
