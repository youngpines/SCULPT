#include "config_1_2_3.h" // Modified to allow 9600 baudrate
// threading library
//#define use_uart_serial
#include "pt_cornell_1_2_3.h"
#include "gpio.h"
#ifndef _SUPPRESS_PLIB_WARNING
#define _SUPPRESS_PLIB_WARNING
#endif
//////////////////////////////////////
// graphics libraries
#include "tft_master.h"
#include "tft_gfx.h"
// need for rand function
#include <stdlib.h>
// string buffer
char buffer[60];
////////////////////////////////////

////////////////////////////////////

/*Pinout for the PIC for reference*/
//RA0   -   Enable stepper 1
//RA1   -   UART2 TX // stepper 1 DIR
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
//RB10  -   UART2 RX // stepper 1 STP
//RB11  -   DC Motor Enable // tft
//RB12  -   Not on Package
//RB13  -   Enable stepper 2
//RB14  -   Green Debug LED // tft
//RB15  -   // dc motor enable

/*********************** [ Constants ] ****************************************/
// Image Size, max seems to be around 175 x 175
#define MAX_IMAGE_SIZE 100*100
#define STEP_X 400
//585
#define STEP_Y 500
//8
#define STEP_Z 10
#define SLEEP_TIME 2

// Stepping Frequency of X & Y, every 1 msec
#define XY_PERIOD  24000
// Stepping Frequency of X & Y, every 6 msec
#define Z_PERIOD 300000

#define X_LIMIT 7000
#define Y_LIMIT 15000
#define Z_LIMIT 8000
#define X_START 0 
#define Y_START 5000
#define Z_START 5000
int debug1 = 1; 
int debug2 = 1;
int debug3 = 1;
int debug4 = 0;

/**************** [ Global Variables ] ****************************************/
static struct pt pt_serial, // thread to import data via UART
                 pt_move,   // thread to get positions and move steppers 
                 pt_align;  // thread to align via limit switches
// The following threads are necessary for UART control
static struct pt pt_input, pt_output, pt_DMA_output;
static struct pt pt_tft;

// Data array holding pixelated info of image
typedef unsigned char uint8_t;
typedef struct {
  uint8_t x;
  uint8_t y;
  uint8_t z;
} image_t;
int image_size = 100;
image_t image[MAX_IMAGE_SIZE] = 
{
{  0,   0, 221}, 
{  0,   1, 221}, 
{  0,   2, 221}, 
{  0,   7, 221}, 
{  0,   8, 221}, 
{  0,   9, 221}, 
{  1,   0, 221}, 
{  1,   1, 221}, 
{  1,   2, 221}, 
{  1,   3, 221}, 
{  1,   4, 221}, 
{  1,   5, 221}, 
{  1,   6, 221}, 
{  1,   7, 221}, 
{  1,   8, 221}, 
{  1,   9, 221}, 
{  2,   0, 221}, 
{  2,   2, 221}, 
{  2,   7, 221}, 
{  2,   9, 221}, 
{  3,   7, 221}, 
{  3,   9, 221}, 
{  4,   1, 221}, 
{  4,   4, 221}, 
{  4,   5, 221}, 
{  4,   6, 221}, 
{  4,   7, 221}, 
{  4,   8, 221}, 
{  4,   9, 221}, 
{  5,   7, 221}, 
{  5,   9, 221}, 
{  6,   0, 221}, 
{  6,   2, 221}, 
{  6,   7, 221}, 
{  6,   9, 221}, 
{  7,   0, 221}, 
{  7,   1, 221}, 
{  7,   3, 221}, 
{  7,   6, 221}, 
{  7,   7, 221}, 
{  7,   8, 221}, 
{  7,   9, 221}, 
{  8,   0, 221}, 
{  8,   1, 221}, 
{  8,   2, 221}, 
{  8,   4, 221}, 
{  8,   5, 221}, 
{  8,   7, 221}, 
{  8,   8, 221}, 
{  8,   9, 221}, 
{  9,   0, 221}, 
{  9,   1, 221}, 
{  9,   2, 221}, 
{  9,   3, 221}, 
{  9,   6, 221}, 
{  9,   7, 221}, 
{  9,   8, 221}, 
{  9,   9, 221}, 
{  3,   0, 220}, 
{  3,   1, 220}, 
{  5,   0, 220}, 
{  5,   1, 220}, 
{  5,   8, 220}, 
{  7,   2, 220}, 
{  9,   4, 220}, 
{  9,   5, 220}, 
{  0,   3, 219}, 
{  0,   6, 219}, 
{  2,   1, 219}, 
{  2,   8, 219}, 
{  3,   8, 219}, 
{  4,   0, 219}, 
{  6,   1, 219}, 
{  6,   8, 219}, 
{  8,   6, 219}, 
{  0,   4, 218}, 
{  0,   5, 218}, 
{  8,   3, 218}, 
{  7,   5, 215}, 
{  7,   4, 213}, 
{  3,   2, 212}, 
{  5,   2, 204}, 
{  5,   4, 184}, 
{  4,   2, 179}, 
{  5,   5, 169}, 
{  2,   3, 167}, 
{  2,   6, 167}, 
{  3,   4, 138}, 
{  3,   5, 130}, 
{  6,   6, 130}, 
{  6,   3, 129}, 
{ 5,  6, 69}, 
{ 3,  6, 57}, 
{ 2,  5, 48}, 
{ 2,  4, 44}, 
{ 6,  4, 27}, 
{ 6,  5, 27}, 
{ 4,  3, 19}, 
{3, 3, 6}, 
{5, 3, 0}};

/*
{{  0,   0, 13}, 
{  0,   1, 13}, 
{  0,   2, 13}, 
{  0,   3, 13}, 
{  0,   4, 13}, 
{ 1,  1, 10}, 
{ 1,  3, 9}, 
{ 3,  3, 9}, 
{ 4,  2, 5}, 
{1, 2, 0}}; */
/*
{{  0,   0, 132}, 
{  0,   1, 132}, 
{  0,   2, 132}, 
{  0,   3, 132}, 
{  0,   4, 132}, 
{  1,   0, 132}, 
{  1,   4, 132}, 
{  2,   0, 132}, 
{  3,   0, 132}, 
{  3,   4, 132}, 
{  4,   0, 132}, 
{  4,   4, 132}, 
{  5,   0, 132}, 
{  5,   1, 132}, 
{  5,   2, 132}, 
{  5,   3, 132}, 
{  5,   4, 132}, 
{  2,   4, 131}, 
{  4,   1, 130}, 
{  4,   3, 126}, 
{  2,   3, 101}, 
{ 1,  1, 99}, 
{ 1,  3, 98}, 
{ 3,  3, 84}, 
{ 4,  2, 81}, 
{ 2,  2, 77}, 
{ 3,  1, 52}, 
{ 2,  1, 23}, 
{3, 2, 4}, 
{1, 2, 0}};

*/

//state variables for the process
volatile uint8_t keep_moving = 0;    //a state to determine if there are steps remaining to move
volatile uint8_t x_enable = 0;
volatile uint8_t y_enable = 0;
volatile uint8_t z_enable = 0;
volatile int image_carved = 0;   //a state variable to determine if the image is carved yet or not
volatile int data_loaded = 0;    //a state variable to determine if the image is loaded or not
volatile int aligned = 0;        // a state variable to determine if the device is aligned or not
volatile int material_loaded = 0;// a state that determines if material has been loaded into the device

void create_dummy_image() {
//    int i, j, z;
//    for (i = 0; i < X_DIM; i++) {
//        for (j = 0; j < Y_DIM; j++) {
//            if (i == 0) image[i][j] = 3;
//            if (i == 1) image[i][j] = 2;
//            if (i == 2) image[i][j] = 1;
//            if (i == 3) image[i][j] = 0;
//            
//        }
//    }


    //clear_GreenLED();
    data_loaded = 1;
}
void load_start_cond(void)
{
  data_loaded     = 0;        
  image_carved    = 0;       
  keep_moving     = 0;
  aligned         = 0;
  material_loaded = 0;
}

volatile int debug = 0;

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
        stp_3.pos++;
        if (stp_3.pos == Z_LIMIT) disable_stp(&stp_3);
    } else {
        stp_3.pos--;
        if (stp_3.pos == 0) disable_stp(&stp_3);
    }
    if (stp_3.stps_left == 0) disable_stp(&stp_3);
  } else if (x_enable && stp_1.stps_left > 0) {
    toggle_stp(&stp_1);
    stp_1.stps_left--;
    if (stp_1.dir_move == 1) stp_1.pos++;
    else stp_1.pos--;
    if (stp_1.stps_left == 0) disable_stp(&stp_1);
  } else if (y_enable && stp_2.stps_left > 0) {
    toggle_stp(&stp_2);
    stp_2.stps_left--;
    if (stp_2.dir_move == 1) stp_2.pos++;
    else stp_2.pos--;
    if (stp_2.stps_left == 0) disable_stp(&stp_2);
  }

  // Checking if all motors are done stepping, and if so then updating the state
  if (stp_1.stps_left <= 0 && stp_2.stps_left <= 0 && stp_3.stps_left <= 0) 
    keep_moving = 0;

  // Clearing the interrupt flag
  mT2ClearIntFlag();
}

// === Move Thread =================================================
// Finds next location to travel to & calculates the steps needed to get there
// Sets steps for each axis and yiel3ds until the motion is complete
// DC motor is turned on, or remains on, if the location is to be removed
static int move_start = 0;
static int debug10, debug11;
static int debug12, debug13, debug14, debug15;
static int debug16, debug17, debug18;
//currently each change in x and y is 585 steps, z axis is a change of 2000
static PT_THREAD (protothread_move(struct pt *pt))
{
  PT_BEGIN(pt);
  while(1) {
  // Wait until the board aligned, material loaded, & the data loaded
  PT_YIELD_UNTIL(&pt_move, data_loaded == 1);
  PT_YIELD_UNTIL(&pt_move, material_loaded == 1); 
  move_start = 1;
  //set_RedLED();
  //PT_YIELD_TIME_msec(500);
  //clear_RedLED();
  //clear_GreenLED();
 
  static int x_pos = 0; static int y_pos = 0; static int z_pos = 0;
  static int i;
  static int j;
  static image_t last_pixel;
  static image_t pixel;
  uint8_t z_start = image[0].z;
//  // Move Z to start position
//  move(&stp_3, Z_START);
//  keep_moving = 1;
//  PT_YIELD_UNTIL(&pt_move, keep_moving == 0);
//  disable_stp(&stp_3);

  // Find highest position in image and start there
  for (i = 0; i < z_start+1; i++) {
      debug10 = i;
    for (j = 0; j < image_size; j++) {
        debug11 = j;
      // Check if z position should be cut
      pixel = image[j];
      debug12 = pixel.x; debug13 = pixel.y; debug14 = pixel.z;
      if (i == 0 && j == 0) last_pixel = pixel;
      if (pixel.z < i) break;
      if ((absDiff(last_pixel.x, pixel.x)+absDiff(last_pixel.y, pixel.y)) > 1) {
          debug15++;
        set_dc_state(&dc, 0);
        z_pos = Z_START;
        move(&stp_3, z_pos);
        keep_moving = 1;
        PT_YIELD_UNTIL(&pt_move, keep_moving == 0);
        disable_stp(&stp_3);
      }
      // Now, cut x & y 
      x_pos = pixel.x*STEP_X;
      move(&stp_1, x_pos);
      keep_moving = 0;
      PT_YIELD_UNTIL(&pt_move, keep_moving == 0);
      disable_stp(&stp_1);
      y_pos = pixel.y*STEP_Y;
      move(&stp_2, y_pos);
      keep_moving = 0;
      PT_YIELD_UNTIL(&pt_move, keep_moving == 0);
      disable_stp(&stp_2);
      
      set_dc_state(&dc, 1);
      z_pos = (z_start-i)*STEP_Z;
      move(&stp_3, z_pos);
      keep_moving = 0;
      PT_YIELD_UNTIL(&pt_move, keep_moving == 0);
      disable_stp(&stp_3);
      
      // Update the pixels
      last_pixel = pixel;
      debug16 = last_pixel.x; debug17 = last_pixel.y; debug18 = last_pixel.z;
    }
  }
    
  // Once done working through the image array just yield forever
  PT_YIELD_UNTIL(&pt_move, image_carved == 0); 
  //load_start_cond();
  while(1) PT_YIELD(&pt_move);
  }
  PT_END(pt);
}

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
  // set_tftLED();
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
      stp_3.stps_left = 10;
      keep_moving = 1;
      //Halting until the desired position is reached
      PT_YIELD_UNTIL(&pt_align, keep_moving == 0);
    }
    disable_stp(&stp_3);
    stp_3.pos = 0;
   
    // Align on the x axis last
    while(read_limit_x() == 0){
      set_dir(&stp_1, 0 );
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
  static uint8_t x_val, y_val, z_val;
  static int count = 0;
  while(1) { 
     // set_GreenLED();
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
    sscanf(PT_term_buffer, "%s %c %c %c", cmd, &x_val, &y_val, &z_val);

    switch(cmd[0]) {
    case 's':
      image_size = z_val;  
    case 'p': // Load pixel values into the pixel array
      image[count].x = x_val;
      image[count].y = y_val;
      image[count].z = z_val;
      count++;
      //toggle_RedLED();
      break;
    case 'e': // All data loaded, Terminate signal sent
      //if (count == image_size) clear_GreenLED();
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


static PT_THREAD (protothread_tft(struct pt *pt))
{
    PT_BEGIN(pt);
     tft_setCursor(0, 0);
     tft_setTextColor(ILI9340_WHITE);  tft_setTextSize(1);
     tft_writeString("Time in seconds since boot\n");
      while(1) {
        // yield time 1 second
        PT_YIELD_TIME_msec(1000) ;
        // draw sys_time
        tft_fillRoundRect(0, 50, 320, 200, 1, ILI9340_BLACK);// x,y,w,h,radius,color
        tft_setCursor(0, 10);
        tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
        tft_writeString(buffer);

        if (debug3) {
            tft_setCursor(60, 30);
            tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
            sprintf(buffer,"toggle");
            tft_writeString(buffer);
        }
        if (aligned) {
            tft_setCursor(0, 30);
            tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
            sprintf(buffer,"aligned");
            tft_writeString(buffer);
        }
        if (debug3 != 1) {
            tft_setCursor(0, 70);
            tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
            sprintf(buffer,"x %u y %u z %u d %c", debug1, debug2, debug3, debug4);
            tft_writeString(buffer);
        }
        if (aligned) {
            tft_setCursor(0, 50);
            tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
            sprintf(buffer,"moved res %d", debug15);
            tft_writeString(buffer);
            tft_setCursor(0, 90);
            tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
            sprintf(buffer,"i %d j %d", debug10, debug11);
            tft_writeString(buffer);
            tft_setCursor(0, 110);
            tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
            sprintf(buffer,"x: pos %d, s %d, d %d", stp_1.pos, stp_1.stps_left, stp_1.dir_move);
            tft_writeString(buffer);
            tft_setCursor(0, 130);
            tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
            sprintf(buffer,"y: pos %d, s %d, d %d", stp_2.pos, stp_2.stps_left, stp_2.dir_move);
            tft_writeString(buffer);
            tft_setCursor(0, 150);
            tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
            sprintf(buffer,"z: pos %d, s %d, d %d", stp_3.pos, stp_3.stps_left, stp_3.dir_move);
            tft_writeString(buffer);
            tft_setCursor(0, 170);
            tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
            sprintf(buffer,"pix x %d y %d z %d", debug12, debug13, debug14);
            tft_writeString(buffer);
            tft_setCursor(0,190);
            tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
            sprintf(buffer,"last x %d y %d z %d", debug16, debug17, debug18);
            tft_writeString(buffer);
        }
        // NEVER exit while
      } // END WHILE(1)
  PT_END(pt);
} // timer thread

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
  PT_INIT(&pt_tft);
 
  // Init everything else
   // init the display
  tft_init_hw();
  tft_begin();
  tft_fillScreen(ILI9340_BLACK);
  //240x320 vertical display
  tft_setRotation(1); // Use tft_setRotation(1) for 320x240
  //init_RedLED(); init_GreenLED();
 // init_tftLED();
  init_limit_switches();
  init_steppers(&stp_1, &stp_2, &stp_3);
  init_dc_motor(&dc);
  load_start_cond();
  create_dummy_image();  
  // Schedule the threads
  while (1){
   //   PT_SCHEDULE(protothread_serial(&pt_serial));
      PT_SCHEDULE(protothread_move(&pt_move)); 
      PT_SCHEDULE(protothread_align(&pt_align)); 
      PT_SCHEDULE(protothread_tft(&pt_tft));
  }
} // main