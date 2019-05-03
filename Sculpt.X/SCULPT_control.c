#include "config_1_2_3.h" // Modified to allow 9600 baudrate
// threading library
#define use_uart_serial
#include "pt_cornell_1_2_3.h"
#include "gpio.h"
#ifndef _SUPPRESS_PLIB_WARNING
#define _SUPPRESS_PLIB_WARNING
#endif
//////////////////////////////////////
#ifdef TFT
// graphics libraries
#include "tft_master.h"
#include "tft_gfx.h"
// need for rand function
#include <stdlib.h>
// string buffer
char buffer[60];
#endif // TFT
////////////////////////////////////

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
// Image Size, max seems to be around 175 x 175
#define MAX_IMAGE_SIZE 100*100
#define STEP_X 180
//585
#define STEP_Y 267
//8
#define STEP_Z 9
#define SLEEP_TIME 2

// Stepping Frequency of X & Y, every 1 msec
#define XY_PERIOD  24000
// Stepping Frequency of X & Y, every 6 msec
#define Z_PERIOD 300000

#define X_LIMIT 7030
#define Y_LIMIT 21000
#define Z_LIMIT 8000
#define X_START 350
#define Y_START 4500
#define Z_START 2750
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
#ifdef TFT
static struct pt pt_tft;
#endif // TFT

// Data array holding pixelated info of image
typedef unsigned char uint8_t;
typedef struct {
  uint8_t x;
  uint8_t y;
  uint8_t z;
} image_t;
int image_size = 0;
image_t image[MAX_IMAGE_SIZE] = {0};
        /*
{
{0, 0, 8}, 
{ 0,  1, 50}, 
{ 0,  2, 67}, 
{0, 3, 0}, 
{0, 4, 1}, 
{ 1,  0, 90}, 
{  1,   1, 171}, 
{  1,   2, 151}, 
{ 1,  3, 88}, 
{  1,   4, 108}, 
{ 2,  0, 60}, 
{  2,   1, 124}, 
{  2,   2, 139}, 
{  2,   3, 119}, 
{  2,   4, 161}, 
{3, 0, 0}, 
{3, 1, 4}, 
{ 3,  2, 39}, 
{3, 3, 0}, 
{ 3,  4, 27}};
*/
/*
{
{0, 0, 255}, 
{0, 1, 255}, 
{0, 2, 15}, 
{0, 3, 0}, 
{0, 4, 0}, 
{0, 5, 0}, 
{0, 6, 255}, 
{0, 7, 255}, 
{0, 8, 255}, 
{0, 9, 255}, 
{1, 0, 255}, 
{1, 1, 15}, 
{1, 2, 0}, 
{1, 3, 0}, 
{1, 4, 0}, 
{1, 5, 0}, 
{1, 6, 0}, 
{1, 7, 255}, 
{1, 8, 0}, 
{1, 9, 0}, 
{2, 0, 255}, 
{2, 1, 0}, 
{2, 2, 0}, 
{2, 3, 0}, 
{2, 4, 0}, 
{2, 5, 0}, 
{2, 6, 0}, 
{2, 7, 0}, 
{2, 8, 0}, 
{2, 9, 0}};
*/


//state variables for the process
volatile uint8_t keep_moving = 0;    //a state to determine if there are steps remaining to move
volatile uint8_t x_enable = 0;
volatile uint8_t y_enable = 0;
volatile uint8_t z_enable = 0;
volatile static int image_carved = 0;   //a state variable to determine if the image is carved yet or not
volatile static int data_loaded = 0;    //a state variable to determine if the image is loaded or not
volatile static int aligned = 0;        // a state variable to determine if the device is aligned or not
volatile static int material_loaded = 0;// a state that determines if material has been loaded into the device

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
static int debug19, debug20;
static int debug21, debug22, debug23;
static int debug24, debug25, debug26;
static int debug27, debug28, debug29;
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
  //set_RedLED();
  //PT_YIELD_TIME_msec(500);
  //clear_RedLED();
  //clear_GreenLED();
 
  static int x_pos = 0; static int y_pos = 0; static int z_pos = 0;
  static int i;
  static int j;
  static image_t last_pixel;
  static image_t pixel;
  static int raise_x = 0; static int raise_y = 0 ;
  uint8_t z_start = 255;
  debug19 = z_start; debug20 = image_size;
//  // Move Z to start position
//  move(&stp_3, Z_START);
//  keep_moving = 1;
//  PT_YIELD_UNTIL(&pt_move, keep_moving == 0);
//  disable_stp(&stp_3);

  // Find highest position in image and start there
  for (i = 0; i < z_start+1; i=i+10) {
      debug10 = i; debug19 = z_start;
 //     PT_YIELD_TIME_msec(2000);
    for (j = 0; j < image_size; j++) {
        debug11 = j; debug20 = image_size;
        
  //       PT_YIELD_TIME_msec(2000);
      // Check if z position should be cut
      pixel = image[j];
      debug12 = pixel.x; debug13 = pixel.y; debug14 = pixel.z;

      
      if (j == 0) {
        if (i == 0) last_pixel = pixel;
        else {
          raise_x = 1;
          raise_y = 1;
        }
      }
       if (pixel.z <= i) { 
          if (absDiff(image[j-1].y, pixel.y)> 1) raise_y = 1;
          if (absDiff(last_pixel.x, pixel.x)> 1) raise_x = 1;
          continue; 
      }
      if (absDiff(last_pixel.x, pixel.x) >= 1 || 
          absDiff(last_pixel.y, pixel.y) > 1  ||
          (i+10 >= z_start && j == image_size-1)) {
          debug15++;
        set_dc_state(&dc, 0);
        z_pos = Z_START;
        move(&stp_3, z_pos);
        keep_moving = 1;
        PT_YIELD_UNTIL(&pt_move, keep_moving == 0);
        disable_stp(&stp_3);
      }
      // Travel to (x, y) coordinate to drill
      //debug24 = last_pixel.x; debug25 = pixel.x;
      debug24 = j != 0 && absDiff(last_pixel.x, pixel.x)> 1;
      debug25 = raise_x == 1;
      if (j != 0 && absDiff(last_pixel.x, pixel.x)> 1 || raise_x == 1) {
          debug26++;
        set_dc_state(&dc, 0);
        set_dir(&stp_1, 0);
        enable_stp(&stp_1);
        PT_YIELD_TIME_msec(SLEEP_TIME);
        while(read_limit_x() == 0) {
         // stp_1.stps_left = 0;
          stp_1.stps_left = 50;
         // stp_3.stps_left = 0;
          //keep_moving = 1;
          // Halting until the desired position is reached
         // PT_YIELD_UNTIL(&pt_align, keep_moving == 0);
        }
        stp_1.stps_left = 0;
        stp_1.pos = 0;       
      }
      x_pos = pixel.x*STEP_X + X_START; debug1 = pixel.x;
      move(&stp_1, x_pos);
      keep_moving = 1;
      PT_YIELD_UNTIL(&pt_move, keep_moving == 0);
      disable_stp(&stp_1);
      
      debug21 = image[j-1].y; debug22 = pixel.y;
      debug23 = (j != 0 && absDiff(image[j-1].y, pixel.y) > 1);
      
      if (j != 0 && absDiff(image[j-1].y, pixel.y)> 1 || raise_y == 1) {
        set_dc_state(&dc, 0);
        set_dir(&stp_2, 0);
        enable_stp(&stp_2);
        PT_YIELD_TIME_msec(SLEEP_TIME);
        while(read_limit_y() == 0) {
         // stp_1.stps_left = 0;
          stp_2.stps_left = 50;
         // stp_3.stps_left = 0;
          //keep_moving = 1;
          // Halting until the desired position is reached
         // PT_YIELD_UNTIL(&pt_align, keep_moving == 0);
        }
        disable_stp(&stp_2);
        stp_2.stps_left = 0;
        stp_2.pos = 0;       
      }
      if ( (j != 0 && absDiff(last_pixel.x, pixel.x)> 1 || raise_x == 1) ||
          ((j != 0 && absDiff(image[j-1].y, pixel.y)> 1 || raise_y == 1)) && i > 120 ) {
          debug29++;
        set_dc_state(&dc, 0);
        set_dir(&stp_3, 0);
        enable_stp(&stp_3);
        PT_YIELD_TIME_msec(SLEEP_TIME);
        while(read_limit_z() == 0) {
         // stp_1.stps_left = 0;
          stp_3.stps_left = 50;
         // stp_3.stps_left = 0;
          //keep_moving = 1;
          // Halting until the desired position is reached
         // PT_YIELD_UNTIL(&pt_align, keep_moving == 0);
        }
        stp_3.stps_left = 0;
        stp_3.pos = 0;  
        z_pos = Z_START;
        move(&stp_3, z_pos);
        keep_moving = 1;
        PT_YIELD_UNTIL(&pt_move, keep_moving == 0);
        disable_stp(&stp_3);
      }
      y_pos = pixel.y*STEP_Y + Y_START; debug2 = pixel.y;
      move(&stp_2, y_pos);
      keep_moving = 1;
      PT_YIELD_UNTIL(&pt_move, keep_moving == 0);
      disable_stp(&stp_2);
      
      set_dc_state(&dc, 1); debug4 = 1;
      z_pos = Z_START-i*STEP_Z;
      debug14 = z_pos;
      move(&stp_3, z_pos);
      keep_moving = 1;
      PT_YIELD_UNTIL(&pt_move, keep_moving == 0);
      disable_stp(&stp_3);
      
      last_pixel = pixel;
      raise_x = 0; raise_y = 0;
      debug16 = last_pixel.x; debug17 = last_pixel.y; debug18 = last_pixel.z;
    }
  }
  
  set_dc_state(&dc, 0);
  // Once done working through the image array just yield forever
  clear_RedLED(); clear_GreenLED();
  set_dc_state(&dc, 0);
  // Once done working through the image array just yield forever
  PT_YIELD_UNTIL(&pt_move, image_carved == 0); 
  set_dc_state(&dc, 0);
  z_pos = Z_START;
  move(&stp_3, z_pos);
  keep_moving = 1;
  PT_YIELD_UNTIL(&pt_move, keep_moving == 0);
  disable_stp(&stp_3);
  set_RedLED(); set_GreenLED();
  load_start_cond();
  //while(1) PT_YIELD(&pt_move);
  }
  PT_END(pt);
};
/*
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
  clear_RedLED();
  clear_GreenLED();
 
  static int x_pos = 0; static int y_pos = 0; static int z_pos = 0;
  static int i;
  static int j;
  static image_t last_pixel;
  static image_t pixel;
  static int raise_x = 0; static int raise_y = 0 ;
  debug19 = z_start; debug20 = image_size;
//  // Move Z to start position
//  move(&stp_3, Z_START);
//  keep_moving = 1;
//  PT_YIELD_UNTIL(&pt_move, keep_moving == 0);
//  disable_stp(&stp_3);
 set_RedLED(); set_GreenLED();
    PT_YIELD_TIME_msec(500);
    clear_RedLED(); clear_GreenLED();
    PT_YIELD_TIME_msec(500);
    set_RedLED(); set_GreenLED();
    PT_YIELD_TIME_msec(500);
    if (image_size == 20 || z_start == 171) clear_RedLED();
    if (!(image[0].x == 0 && image[0].y == 0 && image[0].z == 8)) clear_GreenLED();
    if (!(image[1].x == 0 && image[1].y == 1 && image[1].z == 50)) clear_GreenLED();
    if (!(image[2].x == 0 && image[2].y == 2 && image[2].z == 67)) clear_GreenLED();
    if (!(image[3].x == 0 && image[3].y == 3 && image[3].z == 0)) clear_GreenLED();
    if (!(image[4].x == 0 && image[4].y == 4 && image[4].z == 1)) clear_GreenLED();
    if (!(image[5].x == 1 && image[5].y == 0 && image[5].z == 90)) clear_GreenLED();
    if (!(image[6].x == 1 && image[6].y == 1 && image[6].z == 171)) clear_GreenLED();
    if (!(image[7].x == 1 && image[7].y == 2 && image[7].z == 151)) clear_GreenLED();
    if (!(image[8].x == 1 && image[8].y == 3 && image[8].z == 88)) clear_GreenLED();
    if (!(image[9].x == 1 && image[9].y == 4 && image[9].z == 108)) clear_GreenLED();
    if (!(image[10].x == 2 && image[10].y == 0 && image[10].z == 60)) clear_GreenLED();
    if (!(image[11].x == 2 && image[11].y == 1 && image[11].z == 124)) clear_GreenLED();
    if (!(image[12].x == 2 && image[12].y == 2 && image[12].z == 139)) clear_GreenLED();
    if (!(image[13].x == 2 && image[13].y == 3 && image[13].z == 119)) clear_GreenLED();
    if (!(image[14].x == 2 && image[14].y == 4 && image[14].z == 161)) clear_GreenLED();
    if (!(image[15].x == 3 && image[15].y == 0 && image[15].z == 0)) clear_GreenLED();
    if (!(image[16].x == 3 && image[16].y == 1 && image[16].z == 4)) clear_GreenLED();
    if (!(image[17].x == 3 && image[17].y == 2 && image[17].z == 39)) clear_GreenLED();
    if (!(image[18].x == 3 && image[18].y == 3 && image[18].z == 0)) clear_GreenLED();
    if (!(image[19].x == 3 && image[19].y == 4 && image[19].z == 27)) clear_GreenLED();
  // Find highest position in image and start there
  for (i = 0; i < z_start+1; i=i+10) {
      debug10 = i; debug19 = z_start;
    //  toggle_RedLED();
    //  if (i > 20) set_RedLED();
     // if (i > 80) clear_RedLED();
      PT_YIELD_TIME_msec(2000);
    for (j = 0; j < image_size; j++) {
        debug11 = j; debug20 = image_size;
    //    clear_GreenLED();
         PT_YIELD_TIME_msec(2000);
      // Check if z position should be cut
      pixel = image[j];
      debug12 = pixel.x; debug13 = pixel.y; debug14 = pixel.z;
      if (image_size == 20 || z_start == 171) clear_RedLED();
      if (j == 0 && !(image[0].x == 0 && image[0].y == 0 && image[0].z == 8)) clear_GreenLED();
    if (j == 1 && !(image[1].x == 0 && image[1].y == 1 && image[1].z == 50)) clear_GreenLED();
    if (j == 2 && !(image[2].x == 0 && image[2].y == 2 && image[2].z == 67)) clear_GreenLED();
    if (j == 3 && !(image[3].x == 0 && image[3].y == 3 && image[3].z == 0)) clear_GreenLED();
    if (j == 4 && !(image[4].x == 0 && image[4].y == 4 && image[4].z == 1)) clear_GreenLED();
    if (j == 5 && !(image[5].x == 1 && image[5].y == 0 && image[5].z == 90)) clear_GreenLED();
    if (j == 6 && !(image[6].x == 1 && image[6].y == 1 && image[6].z == 171)) clear_GreenLED();
    if (j == 7 && !(image[7].x == 1 && image[7].y == 2 && image[7].z == 151)) clear_GreenLED();
    if (j == 8 && !(image[8].x == 1 && image[8].y == 3 && image[8].z == 88)) clear_GreenLED();
    if (j == 9 && !(image[9].x == 1 && image[9].y == 4 && image[9].z == 108)) clear_GreenLED();
    if (j == 10 && !(image[10].x == 2 && image[10].y == 0 && image[10].z == 60)) clear_GreenLED();
    if (j == 11 && !(image[11].x == 2 && image[11].y == 1 && image[11].z == 124)) clear_GreenLED();
    if (j == 12 && !(image[12].x == 2 && image[12].y == 2 && image[12].z == 139)) clear_GreenLED();
    if (j == 13 && !(image[13].x == 2 && image[13].y == 3 && image[13].z == 119)) clear_GreenLED();
    if (j == 14 && !(image[14].x == 2 && image[14].y == 4 && image[14].z == 161)) clear_GreenLED();
    if (j == 15 && !(image[15].x == 3 && image[15].y == 0 && image[15].z == 0)) clear_GreenLED();
    if (j == 16 && !(image[16].x == 3 && image[16].y == 1 && image[16].z == 4)) clear_GreenLED();
    if (j == 17 && !(image[17].x == 3 && image[17].y == 2 && image[17].z == 39)) clear_GreenLED();
    if (j == 18 && !(image[18].x == 3 && image[18].y == 3 && image[18].z == 0)) clear_GreenLED();
    if (j == 19 && !(image[19].x == 3 && image[19].y == 4 && image[19].z == 27)) clear_GreenLED();
      if (j == 0) {
          if (i == 0) last_pixel = pixel;
          else {
              raise_y = 1; 
              raise_x = 1;
          }
      }
      else {
        if (absDiff(image[j-1].y, pixel.y)> 1) raise_y = 1;
        if (absDiff(last_pixel.x, pixel.x)> 1) raise_x = 1;
      }
      debug16 = last_pixel.x; debug17 = last_pixel.y; debug18 = last_pixel.z;
      if (pixel.z <= i) continue; 
      set_GreenLED();
      if (absDiff(last_pixel.x, pixel.x) >= 1 || absDiff(last_pixel.y, pixel.y) > 1  ||
          (i == z_start && j == 0)) {
          debug15++;
        set_dc_state(&dc, 0);
        z_pos = Z_START;
        move(&stp_3, z_pos);
        keep_moving = 1;
        PT_YIELD_UNTIL(&pt_move, keep_moving == 0);
        disable_stp(&stp_3);
      }
      // Travel to (x, y) coordinate to drill
      //debug24 = last_pixel.x; debug25 = pixel.x;
      debug24 = j != 0 && absDiff(last_pixel.x, pixel.x)> 1;
      debug25 = raise_x == 1;
      if (raise_x == 1) {
          debug26++;
        set_dc_state(&dc, 0);
        set_dir(&stp_1, 0);
        enable_stp(&stp_1);
        PT_YIELD_TIME_msec(SLEEP_TIME);
        while(read_limit_x() == 0) {
         // stp_1.stps_left = 0;
          stp_1.stps_left = 50;
         // stp_3.stps_left = 0;
          //keep_moving = 1;
          // Halting until the desired position is reached
         // PT_YIELD_UNTIL(&pt_align, keep_moving == 0);
        }
        stp_1.stps_left = 0;
        stp_1.pos = 0;       
      }
      x_pos = pixel.x*STEP_X + X_START; debug1 = pixel.x;
      move(&stp_1, x_pos);
      keep_moving = 1;
      PT_YIELD_UNTIL(&pt_move, keep_moving == 0);
      disable_stp(&stp_1);
      
      debug21 = image[j-1].y; debug22 = pixel.y;
      if (raise_y == 1) {
          debug23++;
        set_dc_state(&dc, 0);
        set_dir(&stp_2, 0);
        enable_stp(&stp_2);
        PT_YIELD_TIME_msec(SLEEP_TIME);
        while(read_limit_y() == 0) {
         // stp_1.stps_left = 0;
          stp_2.stps_left = 50;
         // stp_3.stps_left = 0;
          //keep_moving = 1;
          // Halting until the desired position is reached
         // PT_YIELD_UNTIL(&pt_align, keep_moving == 0);
        }
        disable_stp(&stp_2);
        stp_2.stps_left = 0;
        stp_2.pos = 0;       
      }
      if ( raise_x == 1 || (raise_y == 1 && i > 120) ) {
        debug29++;
        if (stp_2.pos < Y_START) {
          set_dc_state(&dc, 0);
          set_dir(&stp_3, 0);
          enable_stp(&stp_3);
          PT_YIELD_TIME_msec(SLEEP_TIME);
          while(read_limit_z() == 0) {
           // stp_1.stps_left = 0;
            stp_3.stps_left = 50;
           // stp_3.stps_left = 0;
           //keep_moving = 1;
           // Halting until the desired position is reached
          // PT_YIELD_UNTIL(&pt_align, keep_moving == 0);
          }
          stp_3.stps_left = 0;
          stp_3.pos = 0;  
          z_pos = Z_START;
          move(&stp_3, z_pos);
          keep_moving = 1;
          PT_YIELD_UNTIL(&pt_move, keep_moving == 0);
          disable_stp(&stp_3);
        }
      }
      y_pos = pixel.y*STEP_Y + Y_START; debug2 = pixel.y;
      move(&stp_2, y_pos);
      keep_moving = 1;
      PT_YIELD_UNTIL(&pt_move, keep_moving == 0);
      disable_stp(&stp_2);
      
      set_dc_state(&dc, 1); debug4 = 1;
      z_pos = Z_START-i*STEP_Z;
      //debug14 = z_pos;
      move(&stp_3, z_pos);
      keep_moving = 1;
      PT_YIELD_UNTIL(&pt_move, keep_moving == 0);
      disable_stp(&stp_3);
      
      last_pixel = pixel;
      raise_x = 0; raise_y = 0;
      debug16 = last_pixel.x; debug17 = last_pixel.y; debug18 = last_pixel.z;
    }
  }
  clear_RedLED(); clear_GreenLED();
  set_dc_state(&dc, 0);
  // Once done working through the image array just yield forever
  PT_YIELD_UNTIL(&pt_move, image_carved == 0); 
  set_dc_state(&dc, 0);
  z_pos = Z_START;
  move(&stp_3, z_pos);
  keep_moving = 1;
  PT_YIELD_UNTIL(&pt_move, keep_moving == 0);
  disable_stp(&stp_3);
  set_RedLED(); set_GreenLED();
  load_start_cond();
  //while(1) PT_YIELD(&pt_move);
  }
  PT_END(pt);
}
*/
/**
 * This thread runs after serial thread & ends operation when limit switches hit
 * @param pt
 * @return 
 */
static int debug40 = 0;
static PT_THREAD (protothread_align(struct pt *pt))
{
  PT_BEGIN(pt);
  static int start = 0;
  while(1) {
    // Wait for the initial data from serial thread before aligning
    PT_YIELD_UNTIL(&pt_align, data_loaded == 1);
    // Align on the y axis first
    //2set_RedLED();
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
    aligned = 1; debug40++;
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


static int debug30, debug31, debug32, debug33;
//=== Serial thread =================================================
static PT_THREAD (protothread_serial(struct pt *pt))
{
  PT_BEGIN(pt);
  static char cmd[1];
  static int x_val, y_val, z_val;
  static int count = 0;
  debug30 = count;
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
    debug19 = image_size; debug20 = z_start;
    switch(cmd[0]) {
    case 's':
      image_size = x_val*y_val;  
      z_start = (uint8_t) z_val;
      break;
    case 'p': // Load pixel values into the pixel array
      image[count].x = x_val;
      image[count].y = y_val;
      image[count].z = z_val;
      debug31 = image[count].x; debug32 = image[count].y; debug33 = image[count].z;
      count++;
      debug30 = count;
      toggle_RedLED();
      break;
    case 'e': // All data loaded, Terminate signal sent
      if (count == 45*381) clear_GreenLED();
      debug30 = count;
      data_loaded = 1;
      PT_YIELD_UNTIL(&pt_serial, data_loaded == 0);
      count = 0;
      break;
    default: // Do nothing                  
      break;
    }
    //PT_YIELD(&pt_serial);
    // never exit while
  } // END WHILE(1) 
  PT_END(pt);
} // serial thread


#ifdef TFT
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
            tft_setCursor(100, 30);
            tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
            sprintf(buffer,"toggle");
            tft_writeString(buffer);
        }
        if (aligned) {
            tft_setCursor(0, 30);
            tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
            sprintf(buffer,"aligned %d", debug40);
            tft_writeString(buffer);
        }
//        if (debug3) {
//            tft_setCursor(60, 70);
//            tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
//            sprintf(buffer,"y %d z %d x %d", debug100, debug101, debug102);
//            tft_writeString(buffer);
//        }
        if (debug3 != 1) {
            tft_setCursor(0, 70);
            tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
            sprintf(buffer,"x %u y %u z %u d %c", debug1, debug2, debug3, debug4);
            tft_writeString(buffer);
        }
        if (1) {
            tft_setCursor(0, 50);
            tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
            sprintf(buffer,"moved res %d zst %d isz %d", debug15, debug19, debug20);
            //sprintf(buffer,"moved zst %d isz %d c %d", debug19, debug20, debug30);
            tft_writeString(buffer);
            tft_setCursor(0, 90);
            tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
            sprintf(buffer,"i %d j %d", debug10, debug11);
            //sprintf(buffer,"x %d y %d z %d", debug31, debug32, debug33);
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
            tft_setCursor(0,210);
            tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
            // 21 22 23
            sprintf(buffer,"y-1 %d y %d r %d z %d", debug21, debug22, debug23, debug29);
            tft_writeString(buffer);
             tft_setCursor(0,230);
            tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
            sprintf(buffer,"dat %d a %d m %d k %d", data_loaded, aligned, material_loaded, keep_moving);
            tft_writeString(buffer);
        }
        // NEVER exit while
      } // END WHILE(1)
  PT_END(pt);
} // timer thread
#endif //TFT

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
  
#ifdef TFT
  PT_INIT(&pt_tft);
  // Init everything else
   // init the display
  tft_init_hw();
  tft_begin();
  tft_fillScreen(ILI9340_BLACK);
  //240x320 vertical display
  tft_setRotation(1); // Use tft_setRotation(1) for 320x240
 // init_tftLED();
#endif // TFT
  init_RedLED(); init_GreenLED();
  init_limit_switches();
  init_steppers(&stp_1, &stp_2, &stp_3);
  init_dc_motor(&dc);
 // load_start_cond();
 // create_dummy_image();  
  // Schedule the threads
  while (1){
      PT_SCHEDULE(protothread_serial(&pt_serial));
      PT_SCHEDULE(protothread_move(&pt_move)); 
      PT_SCHEDULE(protothread_align(&pt_align)); 
#ifdef TFT
      PT_SCHEDULE(protothread_tft(&pt_tft));
#endif // TFT
  }
} // main