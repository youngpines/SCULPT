
#include "config.h"
// threading library
#include "pt_cornell_1_2.h"
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
    int stpLeft;    //steps left on the current stepper
    
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


Stepper_Motor stp1, stp2, stp3;     //the stepper motors in use
DC_Motor dc1;                             //dc motor in use

volatile SpiChannel spiChn = SPI_CHANNEL2 ;	 // the SPI channel to use
volatile int spiClkDiv = 2 ;                 // 20 MHz DAC clock

//keep stepping or not
volatile static int keep_moving = 0;

// PWM setting variables
//int generate_period = 40000;
//volatile static int pwm_on_time = 4000; 

// string buffer
char buffer[60];

// == Timer 2 ISR =====================================================
// keeps timing for PWM
void __ISR(_TIMER_2_VECTOR, ipl2) Timer2Handler(void)
{
    
    if(stp1.stpLeft > 0){
        mPortBToggleBits(stp1.STP);
        stp1.stpLeft--;
    }
    if(stp2.stpLeft > 0){
        mPortBToggleBits(stp2.STP);
        stp2.stpLeft--;
    }
    if(stp3.stpLeft > 0){
        mPortBToggleBits(stp3.STP);
        stp3.stpLeft--;
    }
    if(stp3.stpLeft =< 0 &&  stp2.stpLeft =< 0 && stp1.stpLeft =< 0)
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
        
        stp1.stpLeft = 100;
        stp2.stpLeft = 100;
        stp3.stpLeft = 100;
        keep_moving = 1;
        
        PT_YIELD_UNTIL(keep_moving == 0));    
    } // END WHILE(1)
    PT_END(pt);
} // timer thread



// === DataProcess Thread =================================================
/*TODO: Convert image info to locations*/
static PT_THREAD (protothread_data(struct pt *pt))
{
    PT_BEGIN(pt);
    while(1){
    
        PT_YIELD_msec(1000);
        
        write to tft time for dubugging
    } // END WHILE(1)
  PT_END(pt);
} // animation thread

int main(void)
{
    ANSELA = 0; ANSELB = 0; 
    mPORTBSetPinsDigitalIn(BIT_9); // button input set as digital in
    
    // === Config timer and output compares to make pulses ========
    // set up timer2 to generate the wave period
    OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_1, generate_period);
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);
    mT2ClearIntFlag(); // and clear the interrupt flag
  
    // set up timer3 to generate the wave period
    OpenTimer3(T3_ON | T3_SOURCE_INT | T3_PS_1_1, 39999999);
    ConfigIntTimer3(T3_INT_ON | T3_INT_PRIOR_2);
    mT3ClearIntFlag(); // and clear the interrupt flag
    // set pulse to go high at 1/4 of the timer period and drop again at 1/2 the timer period
    //OpenOC1(OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, pwm_on_time, pwm_on_time);
    // OC2 is PPS group 2, map to RPB5 (pin 14)
    //PPSOutput(1, RPB7, OC1);

    // === SPI for DAC setup ==============================================
    // SCK2 is pin 26 
    // SDO2 is in PPS output group 2, could be connected to RB5 which is pin 14
    PPSOutput(2, RPB5, SDO2);
    mPORTBSetPinsDigitalOut(BIT_4);
    mPORTBSetBits(BIT_4);
    SpiChnOpen(spiChn, SPI_OPEN_ON | SPI_OPEN_MODE16 | SPI_OPEN_MSTEN | SPI_OPEN_CKE_REV , spiClkDiv);

    // === now the threads ===================================
    // Setup the threads
    PT_setup();
  
    // === setup system wide interrupts  ====================
    INTEnableSystemMultiVectoredInt();
  
    // init the threads
    PT_INIT(&pt_data);
    PT_INIT(&pt_move);
    
    // init the display
    tft_init_hw();
    tft_begin();
    tft_fillScreen(ILI9340_BLACK);
    //240x320 vertical display
    tft_setRotation(0); // Use tft_setRotation(1) for 320x240

    

    //setting direction pins for the motors
    stp1.DIR = BIT_0;
    stp2.DIR = BIT_1;
    stp3.DIR = BIT_2;
    
    //setting direction for the motors
    stp1.dirMove = 0;
    stp2.dirMove = 0;
    stp3.dirMove = 0;

    //setting step ppins for motors
    stp1.STP = BIT_0;
    stp2.STP = BIT_1;
    stp3.STP = BIT_2;

    //setting the steps remaining for each motor
    stp1.stpLeft = 0;
    stp2.stpLeft = 0;
    stp3.stpLeft = 0;

    //setting up the dc motor
    dc1.ENA = BIT_8;
    dc1.on = 0;

    // schedule the threads
    while(1) {
        PT_SCHEDULE(protothread_timer(&pt_data));
        PT_SCHEDULE(protothread_move(&pt_move));
    }
} // main
