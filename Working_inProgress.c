////////////////////////////////////
#include "config_1_2_3.h"       //modified to allow 9600 baudrate
// threading library
#define use_uart_serial
#include "pt_cornell_1_2_3.h"
////////////////////////////////////

/*Pinout for the PIC for reference*/
//RA0   -   Not Connected
//RA1   -   UART2 TX
//RA2   -   Y axis Load Switch
//RA3   -   X axis Load Switch
//RA4   -   Z axis Load Switch
//RB0   -   DIR pin for stepper 1 (x-axis)
//RB1   -   STP pin for stepper 1 (x-axis)
//RB2   -   Button to Confirm Material Load
//RB3   -   Not Used
//RB4   -   DIR pin for stepper 2 (y-axis)
//RB5   -   STP pin for stepper 2 (y-axis)
//RB6   -   DIR pin for stepper 3 (z-axis)
//RB7   -   ~DIR pin for stepper 3 (z-axis)
//RB8   -   STP pin for stepper 2 (z-axis)
//RB9   -   ~STP pin for stepper 2 (z-axis)
//RB10  -   UART2 RX
//RB11  -   DC Motor Enable pin
//RB12  -   Not Used
//RB13  -   Not Used
//RB14  -   Not Used
//RB15  -   Not Used





//enabling macros for pull down enabling
#define EnablePullDownB(bits) CNPUBCLR=bits; CNPDBSET=bits;
#define EnablePullDownA(bits) CNPUACLR=bits; CNPDASET=bits;

/*DEFINES FOR ALL CONSTANTS USED IN PROGRAM*/
//=======================================================================================
//image size parameters
#define x_dim 30
#define y_dim 40

// frequency of x and y motor stepping (max), currently at a step ever 1 msec (2 interrupts)
#define generate_period_xy  20000
//frequency of z stepper rotation, currently once every 6 msec
#define generate_period_z  120000
//=======================================================================================

/*ALL UNIQUE STRUCTS REQUIRED FOR OPERATION*/
//=======================================================================================
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
    int pos_motion;         //if the motion is towards or away from motor
    
};

struct DC_Motor{    //dc motor, controlled by two pins, no PWM as full power wanted always
    int on;     //whether or not the motor is on
    int ENA;    //enabling the motor to spin
};
//=======================================================================================

/*GLOBAL VARIABLES*/
//=======================================================================================
static struct pt pt_serial,   //protothread to import data via UART
        pt_move,             //stepper calculations to get to positions     
        pt_align;           // alignment via limit switches

// The following threads are necessary for UART control
static struct pt pt_input, pt_output, pt_DMA_output;

struct Stepper_Motor stp2;     //the stepper motors in use, easy stepper driver board
struct Stepper_Motor_L stp1,stp3;        // L298 stepper motor driver
struct DC_Motor dc1;                //dc motor 

//image data array that holds pixelated info of image
static unsigned short image[x_dim][y_dim];

//state variables for the process
volatile int keep_moving,       //a state to determine if there are steps remaining to move
        image_carved,           //a state variable to determine if the image is carved yet or not
        data_loaded,            //a state variable to determine if the image is loaded or not
        aligned,                //a state variable to determine if the device is aligned or not
        material_loaded;        //a state that determines if material has been loaded into the device
//=======================================================================================


// == Timer 2 ISR =====================================================
// steps the x and y motors IF there are steps remaining
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
    }
    //checking if all motors are done stepping, and if so then updating the state
    if(stp1.stpLeft <= 0 &&  stp2.stpLeft <= 0 && stp3.stpLeft <= 0) keep_moving = 0;
    
    //clearing the interrupt flag
    mT2ClearIntFlag();
}

// ====================================================================



// == Timer 3 ISR =====================================================
//a state variable to track what configuration the L298N stepper driver needs for input
volatile int ct = 0;

//this interrupt only focuses on moving the z axis (always moves first if steps left)
void __ISR(_TIMER_3_VECTOR, ipl2) Timer3Handler(void)
{       mPORTBClearBits(stp3.DIR);
        mPORTBClearBits(stp3.STP);
        mPORTBClearBits(stp3.NDIR);
        mPORTBClearBits(stp3.NSTP);
        //checking to make sure z still needs to step
        if(stp3.stpLeft > 0){
            mPORTBClearBits(stp3.NDIR);
            mPORTBClearBits(stp3.STP);
            mPORTBClearBits(stp3.DIR);
            mPORTBClearBits(stp3.NSTP);
            if(stp3.pos_motion == 0){    //turn one direction to lower
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
            stp3.stpLeft--;
        }
        else if(stp1.stpLeft > 0){
            if(stp1.pos_motion == 0){    //turn one direction to lower
                //determining which state the input needs and setting
                //the appropriate bits of the stepper motor driver
                if(ct==0){ //1010
                    mPORTBSetBits(stp1.DIR);
                    mPORTBSetBits(stp1.STP);
                    mPORTBClearBits(stp1.NDIR);
                    mPORTBClearBits(stp1.NSTP);
                    ct++;
                }
                else if(ct==1){ //0110
                    mPORTBSetBits(stp1.NDIR);
                    mPORTBSetBits(stp1.STP);
                    mPORTBClearBits(stp1.DIR);
                    mPORTBClearBits(stp1.NSTP);
                    ct++;
                }
                else if(ct==2){  //0101
                    mPORTBSetBits(stp1.NDIR);
                    mPORTBSetBits(stp1.NSTP);
                    mPORTBClearBits(stp1.DIR);
                    mPORTBClearBits(stp1.STP);
                    ct++;
                }
                else if(ct==3){ //1001
                    mPORTBSetBits(stp1.DIR);
                    mPORTBSetBits(stp1.NSTP);
                    mPORTBClearBits(stp1.NDIR);
                    mPORTBClearBits(stp1.STP);
                    ct = 0;
                }
            }
            
            else{    //turn opposite direction to raise 
                //determining which state the input needs and setting
                //the appropriate bits of the stepper motor driver
                if(ct==0){ //0101
                    mPORTBSetBits(stp1.NDIR);
                    mPORTBSetBits(stp1.NSTP);
                    mPORTBClearBits(stp1.DIR);
                    mPORTBClearBits(stp1.STP);
                    ct++;
                }
                else if(ct==1){ //1001
                    mPORTBSetBits(stp1.DIR);
                    mPORTBSetBits(stp1.NSTP);
                    mPORTBClearBits(stp1.NDIR);
                    mPORTBClearBits(stp1.STP);
                    ct++;
                }
                else if(ct==2){  //1010
                    mPORTBSetBits(stp1.DIR);
                    mPORTBSetBits(stp1.STP);
                    mPORTBClearBits(stp1.NDIR);
                    mPORTBClearBits(stp1.NSTP);
                    ct++;
                }
                else if(ct==3){ //0110
                    mPORTBSetBits(stp1.NDIR);
                    mPORTBSetBits(stp1.STP);
                    mPORTBClearBits(stp1.DIR);
                    mPORTBClearBits(stp1.NSTP);
                    ct = 0;
                }
            }
            stp1.stpLeft--;
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

//currently each change in x and y is 300 steps in the x and y direction
static PT_THREAD (protothread_move(struct pt *pt))
{
    PT_BEGIN(pt);
    //wait until the board is aligned, material is loaded, and the data is loaded
    PT_YIELD_UNTIL(&pt_move, material_loaded ==1);    
    PT_YIELD_UNTIL(&pt_move, data_loaded ==1); 
 
    //the x and y positions in the image array at start target positions at end of loops
    static int x_pos, y_pos;

    stp1.pos_motion = 0;    //ensure that x is going the correct
                                //direction
    
    
    
    //looping through all entries in the image array
    for(x_pos = 0; x_pos < x_dim; x_pos++){         //x coordinates iterated slowly
        
        mPORTBSetBits(stp2.DIR);    //ensure that y is going the correct
                                    //direction
        
        
        //this loop looks at the next y coordinate for the current x and determines
        //if the drill needs to be raised or lowered for that entry
        
        //then the y is step forward to the future entry and the process is redone
        for(y_pos = -1; y_pos < y_dim-1; y_pos++){      //looking ahead requires the shift by 1
            mPORTBClearBits(BIT_9);
            //TODO: RECALC of DISTANCE for IMAGE ARRAY ENTRY (y and z)
            

            if(image[x_pos][y_pos+1] > 127){  //requires z to be raised
                
                //turn off the dc motor first
                mPORTAClearBits(dc1.ENA);
                dc1.on = 0;
                
                //checking if the z is raised or not
                if(stp3.pos_motion ==1){    //currently lowered and needs to raise

                    //set the state to move the opposite direction
                    stp3.pos_motion = 0;
                    //raise the z axis
                    stp3.stpLeft = 1000;  //-----------------------------
                }

                else{               //already raised
                    stp3.stpLeft = 0;
                }         
            }


            else{   // requires z to be lowered

                //checking if the z is raised or not
                if(stp3.pos_motion ==0){    //currently raised and needs to lower
                    
                    //turn on the dc motor first
                    mPORTASetBits(dc1.ENA);
                    dc1.on = 1;
                    
                    //set the state to move the opposite direction
                    stp3.pos_motion = 1;
                    
                    //lower the z axis
                    stp3.stpLeft = 1000; //-----------------------------

                }

                else{               //already lowered
                    stp3.stpLeft = 0;
                }  
            }

            //setting the steps to move in y
            stp2.stpLeft = 300;

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
        if(stp3.pos_motion ==1){    //currently lowered and needs to raise
            
            //set the state to move the opposite direction
            stp3.pos_motion = 0;
            //raise the z axis
            stp3.stpLeft = 1000;  //-----------------------------

        }
        else    stp3.stpLeft = 0;      //already raised
        
        //resetting the y location
        stp2.stpLeft = 300 * (y_dim);
            
        //stepping the appropriate amount in x
        stp1.stpLeft = 300;

        //setting up for the ISR to move the position
        keep_moving = 1;
        //halting until the desired position is reached
        PT_YIELD_UNTIL(&pt_move, keep_moving == 0);
    }
   
            
    //final reset for dc motor        
    //turn off the dc motor 
    mPORTAClearBits(dc1.ENA);
    dc1.on = 0;
    
    //tell the align and data thread that the image has been carved and need to realign
    image_carved = 1;
    aligned =0;
    
    //once done working through the image array just yield
    PT_YIELD_UNTIL(&pt_move, image_carved ==0);
            
    
    //PT_YIELD(&pt_move);
            
    PT_END(pt);
} // timer thread


/*TODO FINISH THREAD WITH ACTUAL MEASURES and DIRECTIONS*/
//this thread will run after the serial thread and after the move thread is complete
//it will end its operation when the load switches are hit


//this thread will be responsible for feeding the material in and out of the 
//machine
static PT_THREAD (protothread_align(struct pt *pt))
{
  PT_BEGIN(pt);
  
  //wait for the initial data to be loaded before aligning
  //PT_YIELD_UNTIL(&pt_align, data_loaded ==1);
  
  //this while runs until all buttons are pressed and everything is done aligning
  while(1){
      //align on the y axis first
//      mPORTBSetBits(stp2.DIR);
//      while(mPORTAReadBits(BIT_2) == 0){
//          mPORTBSetBits(stp2.DIR);
//          stp2.stpLeft = 50;
//          stp1.stpLeft = 0;
//          stp3.stpLeft = 0;
//         keep_moving = 1;
//        //halting until the desired position is reached
//        PT_YIELD_UNTIL(&pt_move, keep_moving == 0);
//      }
      
      //finally align on z axis
      
      while(mPORTAReadBits(BIT_4) == 0){
          stp3.pos_motion = 1;
          stp3.stpLeft = 10;
          stp1.stpLeft = 0;
          stp2.stpLeft = 0;
         keep_moving = 1;
        //halting until the desired position is reached
        PT_YIELD_UNTIL(&pt_move, keep_moving == 0);
      }
      
      //align on the x axis next
      while(mPORTAReadBits(BIT_3) == 0){
          stp3.pos_motion = 1;
          stp1.stpLeft = 10;
          stp2.stpLeft = 0;
          stp3.stpLeft = 0;
         keep_moving = 1;
        //halting until the desired position is reached
        PT_YIELD_UNTIL(&pt_move, keep_moving == 0);
      }
      
      
      
      //after all alignments are done, wait for user to indicate material is loaded
      //then continue on with operation via other threads until complete
      aligned = 1;
      while(mPORTBReadBits(BIT_2) ==0); //do nothing but wait
      
        //aligning to the corner of material

        /*TODO    ALIGN WITH CORNER OF BOARD AFTER ZEROING*/
      
      
        //after finishing the alignment yield until alignment is needed again after operation
        material_loaded =1;
        PT_YIELD_UNTIL(&pt_align, aligned == 0);

  
  }  
  PT_END(pt);
} // thread 3


//=== Serial terminal thread =================================================
static PT_THREAD (protothread_serial(struct pt *pt))
{
        PT_BEGIN(pt);
      static char cmd[30];
      static int value;
      static int count = 0;
      while(1) {
            
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
                 case 'p': // load pixel values into the pixel array
                     // enter frequency in HZ
                     
                     image[count%x_dim][count/y_dim] = value;
                     break;
                 case 'e':          // all data has been loaded and terminate signal sent
                     data_loaded = 1;
                     PT_YIELD_UNTIL(&pt_serial, data_loaded ==0 );
                     break;
                 default:
                     //do nothing                  
            break;
            }
           
            // never exit while
      } // END WHILE(1) 
  PT_END(pt);
} // thread 3

// === Main  ======================================================
void main(void) {
 //SYSTEMConfigPerformance(PBCLK);

    ANSELA = 0; ANSELB = 0; 
  // === Config timer and output compares to make pulses ========
    
    // set up timer2 
    OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_1, generate_period_xy);
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);
    mT2ClearIntFlag(); // and clear the interrupt flag
    
    //setup timer3
    OpenTimer3(T3_ON | T3_SOURCE_INT | T3_PS_1_1,generate_period_z);
    ConfigIntTimer3(T3_INT_ON | T3_INT_PRIOR_2);
    mT3ClearIntFlag(); // and clear the interrupt flag
    
    // === config threads ==========
    // turns OFF UART support and debugger pin, unless defines are set
    PT_setup();

    // === setup system wide interrupts  ========
    INTEnableSystemMultiVectoredInt();

    // init the threads
    //PT_INIT(&pt_serial);
    //PT_INIT(&pt_move);
    PT_INIT(&pt_align);
    // round-robin scheduler for threads

    //DECLARING BITS FOR LOAD SWITCHES
    mPORTASetPinsDigitalIn(BIT_2);  //RB2
    mPORTASetPinsDigitalIn(BIT_3);  //RA3
    mPORTASetPinsDigitalIn(BIT_4);  //RA4
    mPORTBSetPinsDigitalIn(BIT_2);  //RA2


    //enabling pulldowns for all of the buttons
    EnablePullDownB(BIT_2);         
    EnablePullDownA(BIT_2);
    EnablePullDownA(BIT_3);
    EnablePullDownA(BIT_4);

  
     //DECLARING BITS FOR STEPPER PINS (Stepper 1)
    mPORTBSetPinsDigitalOut(BIT_0);
    mPORTBSetPinsDigitalOut(BIT_1);
    mPORTBClearBits(BIT_0);
    mPORTBClearBits(BIT_1);
     mPORTBSetPinsDigitalOut(BIT_13);
    mPORTBSetPinsDigitalOut(BIT_14);
    mPORTBClearBits(BIT_13);
    mPORTBClearBits(BIT_14 );
    
    
    //DECLARING BITS FOR STEPPER PINS (Stepper 2)
    mPORTBSetPinsDigitalOut(BIT_4);
    mPORTBSetPinsDigitalOut(BIT_5);
    mPORTBClearBits(BIT_4);
    mPORTBClearBits(BIT_5);
    
    //DECLARING BITS FOR STEPPER PINS (Stepper 3)
    mPORTBSetPinsDigitalOut(BIT_6);
    mPORTBSetPinsDigitalOut(BIT_7);
    mPORTBSetPinsDigitalOut(BIT_8);
    mPORTBSetPinsDigitalOut(BIT_9);
    mPORTBClearBits(BIT_6);
    mPORTBClearBits(BIT_7);
    mPORTBClearBits(BIT_8);
    mPORTBClearBits(BIT_9);
    
    //DECLARING BITS FOR DC MOTOR 
    mPORTBSetPinsDigitalOut(BIT_11);
    mPORTBClearBits(BIT_11);

    //setting direction pins for the motors
    stp1.DIR = BIT_0;
    stp2.DIR = BIT_4;
    stp3.DIR = BIT_6;

    //setting step pins for motors
    stp1.STP = BIT_1;
    stp2.STP = BIT_5;
    stp3.STP = BIT_8;
    
    //setting additional pins for the third stepper
    stp3.NDIR = BIT_7;
    stp3.NSTP = BIT_9;

    //setting the steps remaining for each motor
    stp1.stpLeft = 0;
    stp2.stpLeft = 0;
    stp3.stpLeft = 0;

    //setting up the dc motor
    dc1.ENA = BIT_11;
    dc1.on = 0;
    
    //setting up conditions for all threads
    data_loaded = 0;        
    image_carved = 0;       
    keep_moving =0;
    aligned=0;
    material_loaded=0;
 
  
  while (1){
      //PT_SCHEDULE(protothread_serial(&pt_serial));
      //PT_SCHEDULE(protothread_move(&pt_move)); 
      PT_SCHEDULE(protothread_align(&pt_align)); 
  }
} // main
