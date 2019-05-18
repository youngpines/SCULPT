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
//RA1   -   UART2 TX (green) // stepper 1 DIR
//RA2   -   Y axis Limit Switch
//RA3   -   X axis Limit Switch
//RA4   -   Z axis Limit Switch
//RB0   -   DIR pin for stepper 1 (x-axis) // tft
//RB1   -   STP pin for stepper 1 (x-axis) //tft
//RB2   -   Button to Confirm Material Load //tft
//RB3   -   Red Debug LED // material button
//RB4   -   DIR pin for stepper 2 (y-axis)
//RB5   -   STP pin for stepper 2 (y-axis)
//RB6   -   Not on package
//RB7   -   DIR pin for stepper 3 (z-axis)
//RB8   -   STP pin for stepper 3 (z-axis)
//RB9   -   Enable stepper 3
//RB10  -   UART2 RX (white) // stepper 1 STP
//RB11  -   DC Motor Enable // tft
//RB12  -   Not on Package
//RB13  -   Enable stepper 2
//RB14  -   Green Debug LED // tft
//RB15  -   // dc motor enable

/*********************** [ Constants ] ****************************************/
// Image Size, very very max seems to be around 175 x 175, capped at 100 x 100
#define MAX_IMAGE_SIZE 100*100
// Num ms driver waits after enable turned back on
#define SLEEP_TIME 2
// Stepper timer periods
#define XY_PERIOD  24000
// Step sizes for each stepper
#define STEP_X 180
#define STEP_Y 267
#define STEP_Z 5
// Limits of movement for each stepper
#define X_LIMIT 7030
#define Y_LIMIT 21000
#define Z_LIMIT 8000
// Starting positions for each stepper
#define X_START 0
#define Y_START 3000
#define Z_START 1750

/**************** [ Global Variables ] ****************************************/
static struct pt pt_serial, // thread to import data via UART
                 pt_move,   // thread to get positions and move steppers 
                 pt_align;  // thread to align via limit switches
// The following threads are necessary for UART control
static struct pt pt_input, pt_output, pt_DMA_output;

// Data array holding pixelated info of image
typedef unsigned char uint8_t;
typedef struct {
  uint8_t x;
  uint8_t y;
  uint8_t z;
} image_t;
int image_size = 0;
image_t image[MAX_IMAGE_SIZE] = {0};
 
//state variables for the process
volatile uint8_t keep_moving = 0;        // Determine if steps remaining to move
volatile uint8_t x_enable = 0;           // Enables x stepper
volatile uint8_t y_enable = 0;           // Enables y stepper
volatile uint8_t z_enable = 0;           // Enables z stepper
volatile static int image_carved = 0;    // Determines if image is carved or not
volatile static int data_loaded = 0;     // Determine if data loaded or not
volatile static int aligned = 0;         // Determine if device aligned or not
volatile static int material_loaded = 0; // Determines if material loaded

// Start conditions for each image carving
void load_start_cond(void)
{
  data_loaded     = 0;        
  image_carved    = 0;       
  keep_moving     = 0;
  aligned         = 0;
  material_loaded = 0;
}

/************************* [ ISRs ] *******************************************/
// == Timer 2 ISR =====================================================
// steps the x and y motors IF there are steps remaining
void __ISR(_TIMER_2_VECTOR, ipl2) Timer2Handler(void)
{
  // Move Z first
  if (z_enable && stp_3.stps_left > 0) {
    toggle_stp(&stp_3);
    stp_3.stps_left--;
    if (stp_3.dir_move == 1) {
        stp_3.pos++;                                   // Positive movement
        if (stp_3.pos == Z_LIMIT) disable_stp(&stp_3); // Disable if at limit
    } else stp_3.pos--;                                // Negative movement
    if (stp_3.stps_left == 0) disable_stp(&stp_3);     // Disable if at limit
  // Move X next
  } else if (x_enable && stp_1.stps_left > 0) {
    toggle_stp(&stp_1);
    stp_1.stps_left--;
    if (stp_1.dir_move == 1) {
        stp_1.pos++;                                   // Positive movement
        if (stp_1.pos == X_LIMIT) disable_stp(&stp_1); // Disable if at limit
    } else stp_1.pos--;                                // Negative movement
    if (stp_1.stps_left == 0) disable_stp(&stp_1);     // Disable if at limit
  } else if (y_enable && stp_2.stps_left > 0) {
    toggle_stp(&stp_2);
    stp_2.stps_left--;
    if (stp_2.dir_move == 1) {
        stp_2.pos++;                                   // Positive movement
        if (stp_2.pos == Y_LIMIT) disable_stp(&stp_2); // Disable if at limit
    } else stp_2.pos--;                                // Negative movement
    if (stp_2.stps_left == 0) disable_stp(&stp_2);     // Disable if at limit
  }

  // Checking if all motors are done stepping, and if so then updating the state
  if (stp_1.stps_left <= 0 && stp_2.stps_left <= 0 && stp_3.stps_left <= 0) 
    keep_moving = 0;

  // Clearing the interrupt flag
  mT2ClearIntFlag();
}

// === Move Thread =================================================
// Finds next location to travel to & calculates the steps needed to get there
// Sets steps for each axis and yields until the motion is complete
// DC motor is turned on, or remains on, if the location is to be removed
static int move_start = 0;
uint8_t z_start = 255;
//currently each change in x and y is 585 steps, z axis is a change of 2000
static PT_THREAD (protothread_move(struct pt *pt))
{
  PT_BEGIN(pt);
  while(1) {
  // Wait until the board aligned, material loaded, & the data loaded
  PT_YIELD_UNTIL(&pt_move, data_loaded == 1);
  PT_YIELD_UNTIL(&pt_move, material_loaded == 1); 
  move_start = 1;
 
  static int x_pos = 0; static int y_pos = 0; static int z_pos = 0;
  static int i;
  static int j;
  static image_t last_pixel;
  static image_t pixel;
  static int raise_x = 0; static int raise_y = 0 ;
  uint8_t z_start = 255;

  // Find highest position in image and start there
  for (i = 0; i < z_start+1; i=i+10) {
    for (j = 0; j < image_size; j++) {
      // Check if z position should be cut
      pixel = image[j];
      // If at first image pixel, make the pixels the same
      if (i == 15 && j == 0) last_pixel = pixel;
      // Encountered a new row, have to recalibrate
      if (j == 0) {
          raise_x = 1;
          raise_y = 1;
      }
      // Skip this pixel, greyscale value is too low to cut
       if (pixel.z <= i) { 
          // Check if x or y axes should be raised
          if (absDiff(image[j-1].y, pixel.y)> 1) raise_y = 1;
          if (absDiff(last_pixel.x, pixel.x)> 1) raise_x = 1;
          continue; 
      }
      // If at new row or skipping over more than y dim pixel, raise Z
      if (absDiff(last_pixel.x, pixel.x) >= 1 || 
          absDiff(last_pixel.y, pixel.y) > 1  ||
         (i+10 >= z_start && j == image_size-1)) {
        set_dc_state(&dc, 0);
        z_pos = Z_START;
        move(&stp_3, z_pos);
        keep_moving = 1;
        PT_YIELD_UNTIL(&pt_move, keep_moving == 0);
        disable_stp(&stp_3);
      }
      // Travel to (x, y) coordinate to drill
      // If more than 1 row difference (i.e. new row or skip row), recalib xdim
      if (j != 0 && absDiff(last_pixel.x, pixel.x)> 1 || raise_x == 1) {
        set_dc_state(&dc, 0);
        set_dir(&stp_1, 0);
        enable_stp(&stp_1);
        PT_YIELD_TIME_msec(SLEEP_TIME);
        while(read_limit_x() == 0) {
            stp_1.stps_left = 50;
            stp_1.pos = 50;
        }
        stp_1.stps_left = 0;
        stp_1.pos = 0;       
      }
      // Move x to correct position
      x_pos = pixel.x*STEP_X + X_START;
      move(&stp_1, x_pos);
      keep_moving = 1;
      PT_YIELD_UNTIL(&pt_move, keep_moving == 0);
      disable_stp(&stp_1);
      
      // If carving new row, recalibrate ydim
      if (j != 0 && absDiff(image[j-1].y, pixel.y)> 1 || raise_y == 1) {
        set_dc_state(&dc, 0);
        set_dir(&stp_2, 0);
        enable_stp(&stp_2);
        PT_YIELD_TIME_msec(SLEEP_TIME);
        while(read_limit_y() == 0) {
            stp_2.stps_left = 50;
            stp_2.pos = 50;
        }
        disable_stp(&stp_2);
        stp_2.stps_left = 0;
        stp_2.pos = 0;       
      }
      // Once y dim is recalibrated, recalibrate Z axis for every new row or 
      // when more than x or y pixels are skipped over
      // If Z level is more than 120 or less than 30, calibrate every new row
      if ( raise_x == 1 || 
         ((raise_y == 1 || absDiff(image[j-1].y, pixel.y)> 1) && (i > 120 || i < 30)) ) {
        set_dc_state(&dc, 0);
        set_dir(&stp_3, 0);
        enable_stp(&stp_3);
        PT_YIELD_TIME_msec(SLEEP_TIME);
        while(read_limit_z() == 0) {
            stp_3.stps_left = 50;
            stp_3.pos = 50;
        }
        stp_3.stps_left = 0;
        stp_3.pos = 0; 
        z_pos = Z_START;
        move(&stp_3, z_pos);
        keep_moving = 1;
        PT_YIELD_UNTIL(&pt_move, keep_moving == 0);
        disable_stp(&stp_3);
      }
      // Now, carve Y position
      y_pos = pixel.y*STEP_Y + Y_START;
      move(&stp_2, y_pos);
      keep_moving = 1;
      PT_YIELD_UNTIL(&pt_move, keep_moving == 0);
      disable_stp(&stp_2);
      
      // Start DC motor to spin end mill and drill down!
      set_dc_state(&dc, 1);
      z_pos = Z_START-i*STEP_Z;
      move(&stp_3, z_pos);
      keep_moving = 1;
      PT_YIELD_UNTIL(&pt_move, keep_moving == 0);
      disable_stp(&stp_3);
      
      // Update last tracked pixel
      last_pixel = pixel;
      raise_x = 0; raise_y = 0;
    }
  }
  // Image is done carved, turn off DC motor & turn off LEDs
  set_dc_state(&dc, 0);
  clear_RedLED(); clear_GreenLED();
  // Once done working through the image array just yield forever
  PT_YIELD_UNTIL(&pt_move, image_carved == 0); 
  set_dc_state(&dc, 0);
  // Raise Z position to start position so material can be take out
  z_pos = Z_START;
  move(&stp_3, z_pos);
  keep_moving = 1;
  PT_YIELD_UNTIL(&pt_move, keep_moving == 0);
  disable_stp(&stp_3);
  // Return to start conditions and turn on LEDs
  set_RedLED(); set_GreenLED();
  load_start_cond();
  }
  PT_END(pt);
};

/**
 * This thread runs after serial thread & ends operation when limit switches hit
 * @param pt
 * @return 
 */
static PT_THREAD (protothread_align(struct pt *pt))
{
  PT_BEGIN(pt);
  static int start = 0;
  while(1) {
    // Wait for the initial data from serial thread before aligning
    PT_YIELD_UNTIL(&pt_align, data_loaded == 1);
    // Align on the y axis first
    while(read_limit_y() == 0) {
      set_dir(&stp_2, 0);
      enable_stp(&stp_2);
      PT_YIELD_TIME_msec(SLEEP_TIME);
      stp_1.stps_left = 0;
      stp_2.stps_left = 50;
      stp_3.stps_left = 0;
      keep_moving = 1;
      // Halting until the desired position is reached
      PT_YIELD_UNTIL(&pt_align, keep_moving == 0);
    }
    disable_stp(&stp_2);
    stp_2.pos = 0;

      
    // Align on z axis
    while(read_limit_z() == 0){
      set_dir(&stp_3, 0);
      enable_stp(&stp_3);
      PT_YIELD_TIME_msec(SLEEP_TIME);
      stp_1.stps_left = 0;
      stp_2.stps_left = 0;
      stp_3.stps_left = 50;
      keep_moving = 1;
      //Halting until the desired position is reached
      PT_YIELD_UNTIL(&pt_align, keep_moving == 0);
    }
    disable_stp(&stp_3);
    stp_3.pos = 0;
   
    // Align on the x axis last
    while(read_limit_x() == 0) {
      set_dir(&stp_1, 0);
      enable_stp(&stp_1);
      PT_YIELD_TIME_msec(SLEEP_TIME);
      stp_1.stps_left = 50;
      stp_2.stps_left = 0;
      stp_3.stps_left = 0;
      keep_moving = 1;
      // Halting until the desired position is reached
      PT_YIELD_UNTIL(&pt_align, keep_moving == 0);
    }
    disable_stp(&stp_1);
    stp_1.pos = 0;

    // All alignments done, Wait for user to confirm material is loaded
    // Then use other threads until complete
    aligned = 1;
    while(read_mat_load() == 0); //do nothing but wait
    clear_RedLED(); clear_GreenLED();
    // Realigning after material loaded only at the start of execution
    if (start == 0){  
      // Raising z to be completely clear
      set_dir(&stp_3, 1);
      enable_stp(&stp_3);
      PT_YIELD_TIME_msec(SLEEP_TIME);
      stp_1.stps_left = 0;
      stp_2.stps_left = 0;
      stp_3.stps_left = Z_START;
      keep_moving = 1;  
      //halting until the desired position is reached
      PT_YIELD_UNTIL(&pt_align, keep_moving == 0);
      disable_stp(&stp_3);
      // Raising y to be completely clear
      set_dir(&stp_2, 1);
      enable_stp(&stp_2);
      PT_YIELD_TIME_msec(SLEEP_TIME);
      stp_1.stps_left = 0;
      stp_2.stps_left = Y_START;
      stp_3.stps_left = 0;
      keep_moving = 1;
      // Halting until the desired position is reached
      PT_YIELD_UNTIL(&pt_align, keep_moving == 0);
      disable_stp(&stp_2);
      // Raising x to be completely clear
      set_dir(&stp_1, 1);
      enable_stp(&stp_1);
      PT_YIELD_TIME_msec(SLEEP_TIME);
      stp_1.stps_left = X_START;
      stp_2.stps_left = 0;
      stp_3.stps_left = 0;
      keep_moving = 1;
      // Halting until the desired position is reached
      PT_YIELD_UNTIL(&pt_align, keep_moving == 0);
      disable_stp(&stp_1);
      start = 1;
    }
    // After finishing the alignment yield until alignment is needed again
    material_loaded = 1;
    PT_YIELD_UNTIL(&pt_align, aligned == 0);
  }  
  PT_END(pt);
} // alignment thread

//=== Serial thread =================================================
static PT_THREAD (protothread_serial(struct pt *pt))
{
  PT_BEGIN(pt);
  static char cmd[1];
  static int x_val, y_val, z_val;
  static int count = 0;
  while(1) { 
    set_GreenLED();
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
    sscanf(PT_term_buffer, "%s %d %d %d", cmd, &x_val, &y_val, &z_val);
    switch(cmd[0]) {
    case 's':
      image_size = x_val*y_val;  
      z_start = (uint8_t) z_val;
      break;
    case 'p': // Load pixel values into the pixel array
      image[count].x = x_val;
      image[count].y = y_val;
      image[count].z = z_val;
      count++;
      toggle_RedLED();
      break;
    case 'e': // All data loaded, Terminate signal sent
      if (count == image_size) clear_GreenLED();
      data_loaded = 1;
      PT_YIELD_UNTIL(&pt_serial, data_loaded == 0);
      count = 0;
      break;
    default: // Do nothing                  
      break;
    }
    // never exit while
  } // END WHILE(1) 
  PT_END(pt);
} // serial thread

/************************** [ Main ] ******************************************/

void main(void) { 
  // === Config timer and output compares to make pulses ========
  // Setup timer2 - X & Y axis
  OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_1, XY_PERIOD);
  ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);
  mT2ClearIntFlag(); // and clear the interrupt flag
    
  // === config threads ==========
  // turns OFF UART support and debugger pin, unless defines are set
  PT_setup();
  // === setup system wide interrupts  ========
  INTEnableSystemMultiVectoredInt();
  // init the threads
 // PT_INIT(&pt_serial);
  PT_INIT(&pt_move);
  PT_INIT(&pt_align);
 
  init_RedLED(); init_GreenLED();
  init_limit_switches();
  init_steppers(&stp_1, &stp_2, &stp_3);
  init_dc_motor(&dc);

  // Schedule the threads
  while (1){
      PT_SCHEDULE(protothread_serial(&pt_serial));
      PT_SCHEDULE(protothread_move(&pt_move)); 
      PT_SCHEDULE(protothread_align(&pt_align)); 
  }
} // main