#include "config_1_2_3.h" // Modified to allow 9600 baudrate
// threading library
#define use_uart_serial
#include "pt_cornell_1_2_3.h"
#include "gpio.h"

#ifndef _SUPPRESS_PLIB_WARNING
#define _SUPPRESS_PLIB_WARNING
#endif
////////////////////////////////////

/*Pinout for the PIC for reference*/
//RA0   -   Enable stepper 1
//RA1   -   UART2 TX
//RA2   -   Y axis Limit Switch
//RA3   -   X axis Limit Switch
//RA4   -   Z axis Limit Switch
//RB0   -   DIR pin for stepper 1 (x-axis)
//RB1   -   STP pin for stepper 1 (x-axis)
//RB2   -   Button to Confirm Material Load
//RB3   -   Red Debug LED
//RB4   -   DIR pin for stepper 2 (y-axis)
//RB5   -   STP pin for stepper 2 (y-axis)
//RB6   -   Not on package
//RB7   -   DIR pin for stepper 3 (z-axis)
//RB8   -   STP pin for stepper 3 (z-axis)
//RB9   -   Enable stepper 3
//RB10  -   UART2 RX
//RB11  -   DC Motor Enable
//RB12  -   Not on Package
//RB13  -   Enable stepper 2
//RB14  -   UNUSED
//RB15  -   Red Debug LED

/*********************** [ Constants ] ****************************************/
// Image Size
#define X_DIM 20
#define Y_DIM 15
#define Z_DIM 20

// Stepping Frequency of X & Y, every 1 msec
#define XY_PERIOD  20000
// Stepping Frequency of X & Y, every 6 msec
#define Z_PERIOD 300000
typedef unsigned char uint8_t;

/**************** [ Global Variables ] ****************************************/
static struct pt pt_serial, // thread to import data via UART
                 pt_move,   // thread to get positions and move steppers 
                 pt_align;  // thread to align via limit switches
// The following threads are necessary for UART control
static struct pt pt_input, pt_output, pt_DMA_output;

// Data array holding pixelated info of image
static unsigned short image[X_DIM][Y_DIM] = {0};

//state variables for the process
volatile uint8_t keep_moving = 0;    //a state to determine if there are steps remaining to move
volatile uint8_t x_enable = 0;
volatile uint8_t y_enable = 0;
volatile uint8_t z_enable = 0;
volatile int image_carved = 0;   //a state variable to determine if the image is carved yet or not
volatile int data_loaded = 0;    //a state variable to determine if the image is loaded or not
volatile int aligned = 0;        // a state variable to determine if the device is aligned or not
volatile int material_loaded = 0;// a state that determines if material has been loaded into the device

/************************* [ ISRs ] *******************************************/
// == Timer 2 ISR =====================================================
// steps the x and y motors IF there are steps remaining
void __ISR(_TIMER_2_VECTOR, ipl2) Timer2Handler(void)
{
  // Move Z first
  if (z_enable && stp_3.stps_left > 0) {
    toggle_z();
    stp_3.stps_left--;
  } else if (x_enable && stp_1.stps_left > 0) {
    toggle_x();
    stp_1.stps_left--;
  } else if (y_enable && stp_2.stps_left > 0) {
    toggle_y();
    stp_2.stps_left--;
  }

  // Checking if all motors are done stepping, and if so then updating the state
  if (stp_1.stps_left <= 0 && stp_2.stps_left <= 0 && stp_3.stps_left <= 0) 
    keep_moving = 0;

  // Clearing the interrupt flag
  mT2ClearIntFlag();
}

// == Timer 3 ISR ===================================================== 
volatile uint8_t x_first_pass = 0;
volatile uint8_t y_first_pass = 0;
volatile uint8_t z_first_pass = 0;
// Timer to wait ~1 ms after turning on sleep mode
void __ISR(_TIMER_3_VECTOR, ipl2) Timer3Handler(void)
{       
  if (x_turned_on == 1 && x_first_pass == 0) {
    x_enable = 0;
    x_first_pass = 1;
  } else if (x_turned_on == 1 && x_first_pass == 1) {
    x_enable = 1;
    x_turned_on = 0;
    x_first_pass = 0;
  } else if (x_turned_off == 1) {
    x_enable = 0;
    x_turned_off = 0;
  }
  
  if (y_turned_on == 1 && y_first_pass == 0) {
    y_enable = 0;
    y_first_pass = 1;
  } else if (y_turned_on == 1 && y_first_pass == 1) {
    y_enable = 1;
    y_turned_on = 0;
    y_first_pass = 0;
  } else if (y_turned_off == 1) {
    y_enable = 0;
    y_turned_off = 0;
  }
  
  if (z_turned_on == 1 && z_first_pass == 0) {
    z_enable = 0;
    z_first_pass = 1;
  } else if (z_turned_on == 1 && z_first_pass == 1) {
    z_enable = 1;
    z_turned_on = 0;
    z_first_pass = 0;
  } else if (z_turned_off == 1) {
    z_enable = 0;
    z_turned_off = 0;
  }
  // Clear the interrupt flag
  mT3ClearIntFlag();
}
//==================================================================


// === Move Thread =================================================
// Finds next location to travel to & calculates the steps needed to get there
// Sets steps for each axis and yields until the motion is complete
// DC motor is turned on, or remains on, if the location is to be removed

//currently each change in x and y is 585 steps, z axis is a change of 2000
static PT_THREAD (protothread_move(struct pt *pt))
{
  PT_BEGIN(pt);
  // Wait until the board aligned, material loaded, & the data loaded
  PT_YIELD_UNTIL(&pt_move, data_loaded == 1);
  PT_YIELD_UNTIL(&pt_move, material_loaded == 1);    
 
  // x and y positions in the image array at start target positions at end of loops
  static int x_pos, y_pos;
  set_dir_x(1); 
  enable_x(); // TODO here?
  // Loop through all entries in the image array
  for (x_pos = 0; x_pos < X_DIM; x_pos++){         //x coordinates iterated slowly
    set_dir_y(0);
    enable_y();
    
    // Looks at the next y coordinate for the current x and determines
    // if the drill needs to be raised or lowered for that entry
    for (y_pos = -1; y_pos < Y_DIM-1; y_pos++) { // Look ahead needs shift by 1
      if (image[x_pos][y_pos+1] > 127) {  // z needs to be raised
        set_dc_state(1);
        if (stp_3.dir_move == 0) { // Currently lowered, needs to be raised
          set_dir_z(1);            // Raise Z axis
          stp_3.stps_left = 2000;  // 737 was prev value 
        } else                     // Already raised
          stp_3.stps_left = 0;
      } else {   // requires z to be lowered
          if (stp_3.dir_move == 1) { // Currently raised, needs to be lowered
            set_dc_state(1); 
            set_dir_z(1);
            stp_3.stps_left = 2000;
          } else stp_3.stps_left = 0; 
         
          // Setting the steps to move in y
          stp_2.stps_left = 585;
          // Setting up for the ISR to move the position
          keep_moving = 1;
          // Halting until the desired position is reached
          PT_YIELD_UNTIL(&pt_move, keep_moving == 0); 
      } 
      
      // DONE with y coordinate for a given x coordinate
      // Turn off the dc motor first to preserve integrity of piece 
      set_dc_state(0);
      // Return to top of next column for y
      set_dir_y(1);
      // Check if z is raised or not, if not raise it
      if(stp_3.dir_move == 0){    // Currently lowered and needs to raise
        // Set the state to move the opposite direction
        set_dir_z(1);
        // Raise the z axis
        stp_3.stps_left = 2000; 
      } else stp_3.stps_left = 0;      //already raised
        
      // Reset the y location
      stp_2.stps_left = 585 * (Y_DIM);  
      //stepping the appropriate amount in x
      stp_1.stps_left = 585;

      //setting up for the ISR to move the position
      keep_moving = 1;
      //halting until the desired position is reached
      PT_YIELD_UNTIL(&pt_move, keep_moving == 0);
  }
     
  // Turn off the dc motor 
  set_dc_state(0);
  // Tell align & data thread image has been carved and need to realign
  image_carved = 1;
  aligned = 0;
    
  // Once done working through the image array just yield forever
  PT_YIELD_UNTIL(&pt_move, image_carved == 0);          
  PT_END(pt);
  }
}

/**
 * This thread runs after serial thread & ends operation when limit switches hit
 * @param pt
 * @return 
 */
static PT_THREAD (protothread_align(struct pt *pt))
{
  PT_BEGIN(pt);
  // Wait for the initial data from serial thread before aligning
  PT_YIELD_UNTIL(&pt_align, data_loaded == 1);
  static int start = 0;
  while(1) {
    // Align on the y axis first
    while(read_limit_y() == 0) {
      set_dir_y(1);
      stp_1.stps_left = 0;
      stp_2.stps_left = 50;
      stp_3.stps_left = 0;
      enable_y();
      keep_moving = 1;
      // Halting until the desired position is reached
      PT_YIELD_UNTIL(&pt_align, keep_moving == 0);
    }
    disable_y();
      
    // Align on z axis
    while(read_limit_z() == 0){
      set_dir_z(1);
      stp_1.stps_left = 0;
      stp_2.stps_left = 10;
      stp_3.stps_left = 0;
      enable_z();
      keep_moving = 1;
      //Halting until the desired position is reached
      PT_YIELD_UNTIL(&pt_align, keep_moving == 0);
    }
    disable_z();
      
    // Align on the x axis last
    while(read_limit_x() == 0){
      set_dir_x(1);         
      stp_1.stps_left = 50;
      stp_2.stps_left = 0;
      stp_3.stps_left = 0;
      enable_x();
      keep_moving = 1;
      // Halting until the desired position is reached
      PT_YIELD_UNTIL(&pt_align, keep_moving == 0);
    }
    disable_x();
      
    // All alignments done, Wait for user to confirm material is loaded
    // Then use other threads until complete
    aligned = 1;
    while(read_mat_load == 0); //do nothing but wait
    // Realigning after material loaded only at the start of execution
    if (start == 0){  
      // Raising z to be completely clear
      set_dir_z(1);
      stp_1.stps_left = 0;
      stp_2.stps_left = 0;
      stp_3.stps_left = 2000;
      enable_z();
      keep_moving = 1;  
      //halting until the desired position is reached
      PT_YIELD_UNTIL(&pt_align, keep_moving == 0);
      disable_z();
      // Raising y to be completely clear
      set_dir_y(1);
      stp_1.stps_left = 0;
      stp_2.stps_left = 2000;
      stp_3.stps_left = 0;
      enable_y();
      keep_moving = 1;
      // Halting until the desired position is reached
      PT_YIELD_UNTIL(&pt_align, keep_moving == 0);
      disable_y();
      // Raising x to be completely clear
      set_dir_x(1);
      stp_1.stps_left = 2000;
      stp_2.stps_left = 0;
      stp_3.stps_left = 0;
      enable_x();
      keep_moving = 1;
      // Halting until the desired position is reached
      PT_YIELD_UNTIL(&pt_align, keep_moving == 0);
      disable_x();
      start = 1;
    }
    // After finishing the alignment yield until alignment is needed again
    material_loaded =1;
    PT_YIELD_UNTIL(&pt_align, aligned == 0);
  }  
  PT_END(pt);
} // alignment thread


//=== Serial thread =================================================
static PT_THREAD (protothread_serial(struct pt *pt))
{
  PT_BEGIN(pt);
  static char cmd[30];
  static int value;
  static int count = 0;
  while(1) {     
    // Send the prompt via DMA to serial
    sprintf(PT_send_buffer,"cmd>");
    // Spawning a print thread
    PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output) );
    //spawn a thread to handle terminal input
    // the input thread waits for input
    // -- BUT does NOT block other threads
    // string is returned in "PT_term_buffer"
    PT_SPAWN(pt, &pt_input, PT_GetSerialBuffer(&pt_input) );
    // returns when the thread dies
    // in this case, when <enter> is pushed
    // now parse the string
    sscanf(PT_term_buffer, "%s %d", cmd, &value);
    count++;

    switch(cmd[0]) {
    case 'p': // Load pixel values into the pixel array
      image[count%X_DIM][count/Y_DIM] = value;
      break;
    case 'e': // All data loaded, Terminate signal sent
      data_loaded = 1;
      PT_YIELD_UNTIL(&pt_serial, data_loaded == 0);
      break;
    default: // Do nothing                  
      break;
    }
    // never exit while
  } // END WHILE(1) 
  PT_END(pt);
} // serial thread

/************************** [ Main ] ******************************************/
void load_start_cond(void)
{
  data_loaded     = 0;        
  image_carved    = 0;       
  keep_moving     = 0;
  aligned         = 0;
  material_loaded = 0;
}

void main(void) { 
  // === Config timer and output compares to make pulses ========
  // Setup timer2 - X & Y axis
  OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_1, XY_PERIOD);
  ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);
  mT2ClearIntFlag(); // and clear the interrupt flag
  // Setup timer3 - Z axis
  OpenTimer3(T3_ON | T3_SOURCE_INT | T3_PS_1_1, Z_PERIOD);
  ConfigIntTimer3(T3_INT_ON | T3_INT_PRIOR_2);
  mT3ClearIntFlag(); // and clear the interrupt flag
    
  // === config threads ==========
  // turns OFF UART support and debugger pin, unless defines are set
  PT_setup();
  // === setup system wide interrupts  ========
  INTEnableSystemMultiVectoredInt();
  // init the threads
  PT_INIT(&pt_serial);
  PT_INIT(&pt_move);
  PT_INIT(&pt_align);
 
  // Init everything else
  init_LED();
  init_limit_switches();
  init_steppers();
  init_dc_motor();
  load_start_cond();
  
  // Schedule the threads
  while (1){
      PT_SCHEDULE(protothread_serial(&pt_serial));
      PT_SCHEDULE(protothread_move(&pt_move)); 
      PT_SCHEDULE(protothread_align(&pt_align)); 
  }
} // main