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
#define STEP_X 390
//585
#define STEP_Y 585
//8
#define STEP_Z 8
#define SLEEP_TIME 2

// Stepping Frequency of X & Y, every 1 msec
#define XY_PERIOD  24000
// Stepping Frequency of X & Y, every 6 msec
#define Z_PERIOD 300000

#define X_LIMIT 7000
#define Y_LIMIT 15000
#define Z_LIMIT 8000
#define X_START 0 
#define Y_START 4000
#define Z_START 3600
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
int image_size = 17*21;
image_t image[MAX_IMAGE_SIZE] = 
        {
{  0,   0, 255}, 
{  0,   1, 255}, 
{  0,   2, 255}, 
{  0,   3, 255}, 
{  0,   4, 255}, 
{  0,   5, 253}, 
{  0,   6, 252}, 
{  0,   7, 255}, 
{  0,   8, 255}, 
{  0,   9, 255}, 
{  0,  10, 251}, 
{  0,  11, 254}, 
{  0,  12, 255}, 
{  0,  13, 255}, 
{  0,  14, 255}, 
{  0,  15, 255}, 
{  0,  16, 255}, 
{  1,   0, 255}, 
{  1,   1, 255}, 
{  1,   2, 255}, 
{  1,   3, 255}, 
{  1,   4, 253}, 
{  1,   5, 255}, 
{  1,   6, 255}, 
{  1,   7, 250}, 
{  1,   8, 241}, 
{  1,   9, 254}, 
{  1,  10, 255}, 
{  1,  11, 255}, 
{  1,  12, 253}, 
{  1,  13, 255}, 
{  1,  14, 255}, 
{  1,  15, 255}, 
{  1,  16, 255}, 
{  2,   0, 255}, 
{  2,   1, 255}, 
{  2,   2, 255}, 
{  2,   3, 253}, 
{  2,   4, 255}, 
{  2,   5, 221}, 
{  2,   6, 102}, 
{ 2,  7, 38}, 
{ 2,  8, 26}, 
{ 2,  9, 45}, 
{  2,  10, 119}, 
{  2,  11, 235}, 
{  2,  12, 255}, 
{  2,  13, 254}, 
{  2,  14, 255}, 
{  2,  15, 255}, 
{  2,  16, 255}, 
{  3,   0, 255}, 
{  3,   1, 255}, 
{  3,   2, 253}, 
{  3,   3, 255}, 
{  3,   4, 200}, 
{ 3,  5, 21}, 
{3, 6, 0}, 
{3, 7, 0}, 
{3, 8, 0}, 
{3, 9, 0}, 
{ 3, 10,  0}, 
{ 3, 11, 39}, 
{  3,  12, 224}, 
{  3,  13, 254}, 
{  3,  14, 253}, 
{  3,  15, 255}, 
{  3,  16, 255}, 
{  4,   0, 255}, 
{  4,   1, 254}, 
{  4,   2, 254}, 
{  4,   3, 239}, 
{ 4,  4, 32}, 
{4, 5, 0}, 
{4, 6, 0}, 
{ 4,  7, 11}, 
{ 4,  8, 44}, 
{ 4,  9, 12}, 
{ 4, 10,  0}, 
{ 4, 11,  0}, 
{ 4, 12, 62}, 
{  4,  13, 252}, 
{  4,  14, 253}, 
{  4,  15, 255}, 
{  4,  16, 255}, 
{  5,   0, 255}, 
{  5,   1, 251}, 
{  5,   2, 255}, 
{  5,   3, 136}, 
{5, 4, 0}, 
{5, 5, 0}, 
{ 5,  6, 63}, 
{  5,   7, 218}, 
{  5,   8, 250}, 
{  5,   9, 218}, 
{ 5, 10, 58}, 
{ 5, 11,  3}, 
{ 5, 12,  0}, 
{  5,  13, 187}, 
{  5,  14, 255}, 
{  5,  15, 252}, 
{  5,  16, 255}, 
{  6,   0, 255}, 
{  6,   1, 254}, 
{  6,   2, 253}, 
{ 6,  3, 48}, 
{6, 4, 1}, 
{ 6,  5, 19}, 
{  6,   6, 231}, 
{  6,   7, 255}, 
{  6,   8, 251}, 
{  6,   9, 255}, 
{  6,  10, 214}, 
{ 6, 11,  0}, 
{ 6, 12,  0}, 
{  6,  13, 109}, 
{  6,  14, 255}, 
{  6,  15, 251}, 
{  6,  16, 255}, 
{  7,   0, 253}, 
{  7,   1, 255}, 
{  7,   2, 221}, 
{7, 3, 9}, 
{7, 4, 0}, 
{ 7,  5, 98}, 
{  7,   6, 255}, 
{  7,   7, 250}, 
{  7,   8, 254}, 
{  7,   9, 253}, 
{  7,  10, 254}, 
{ 7, 11, 96}, 
{ 7, 12, 59}, 
{  7,  13, 131}, 
{  7,  14, 255}, 
{  7,  15, 253}, 
{  7,  16, 255}, 
{  8,   0, 252}, 
{  8,   1, 255}, 
{  8,   2, 186}, 
{8, 3, 0}, 
{8, 4, 0}, 
{  8,   5, 157}, 
{  8,   6, 254}, 
{  8,   7, 251}, 
{  8,   8, 255}, 
{  8,   9, 255}, 
{  8,  10, 254}, 
{  8,  11, 255}, 
{  8,  12, 255}, 
{  8,  13, 255}, 
{  8,  14, 254}, 
{  8,  15, 255}, 
{  8,  16, 255}, 
{  9,   0, 251}, 
{  9,   1, 255}, 
{  9,   2, 165}, 
{9, 3, 0}, 
{9, 4, 0}, 
{  9,   5, 186}, 
{  9,   6, 255}, 
{  9,   7, 252}, 
{  9,   8, 255}, 
{  9,   9, 255}, 
{  9,  10, 255}, 
{  9,  11, 253}, 
{  9,  12, 254}, 
{  9,  13, 254}, 
{  9,  14, 255}, 
{  9,  15, 255}, 
{  9,  16, 255}, 
{ 10,   0, 251}, 
{ 10,   1, 255}, 
{ 10,   2, 157}, 
{10,  3,  0}, 
{10,  4,  0}, 
{ 10,   5, 195}, 
{ 10,   6, 255}, 
{ 10,   7, 252}, 
{ 10,   8, 255}, 
{ 10,   9, 255}, 
{ 10,  10, 255}, 
{ 10,  11, 254}, 
{ 10,  12, 253}, 
{ 10,  13, 254}, 
{ 10,  14, 255}, 
{ 10,  15, 255}, 
{ 10,  16, 255}, 
{ 11,   0, 251}, 
{ 11,   1, 255}, 
{ 11,   2, 164}, 
{11,  3,  0}, 
{11,  4,  0}, 
{ 11,   5, 189}, 
{ 11,   6, 255}, 
{ 11,   7, 252}, 
{ 11,   8, 255}, 
{ 11,   9, 255}, 
{ 11,  10, 255}, 
{ 11,  11, 255}, 
{ 11,  12, 255}, 
{ 11,  13, 255}, 
{ 11,  14, 255}, 
{ 11,  15, 255}, 
{ 11,  16, 255}, 
{ 12,   0, 252}, 
{ 12,   1, 255}, 
{ 12,   2, 184}, 
{12,  3,  0}, 
{12,  4,  0}, 
{ 12,   5, 163}, 
{ 12,   6, 254}, 
{ 12,   7, 251}, 
{ 12,   8, 255}, 
{ 12,   9, 255}, 
{ 12,  10, 254}, 
{ 12,  11, 235}, 
{ 12,  12, 227}, 
{ 12,  13, 235}, 
{ 12,  14, 255}, 
{ 12,  15, 255}, 
{ 12,  16, 255}, 
{ 13,   0, 253}, 
{ 13,   1, 255}, 
{ 13,   2, 219}, 
{13,  3,  8}, 
{13,  4,  0}, 
{ 13,   5, 111}, 
{ 13,   6, 255}, 
{ 13,   7, 250}, 
{ 13,   8, 254}, 
{ 13,   9, 253}, 
{ 13,  10, 254}, 
{13, 11, 54}, 
{13, 12,  9}, 
{13, 13, 88}, 
{ 13,  14, 255}, 
{ 13,  15, 252}, 
{ 13,  16, 255}, 
{ 14,   0, 255}, 
{ 14,   1, 254}, 
{ 14,   2, 252}, 
{14,  3, 45}, 
{14,  4,  1}, 
{14,  5, 29}, 
{ 14,   6, 243}, 
{ 14,   7, 255}, 
{ 14,   8, 251}, 
{ 14,   9, 255}, 
{ 14,  10, 213}, 
{14, 11,  5}, 
{14, 12,  0}, 
{ 14,  13, 116}, 
{ 14,  14, 255}, 
{ 14,  15, 251}, 
{ 14,  16, 255}, 
{ 15,   0, 255}, 
{ 15,   1, 251}, 
{ 15,   2, 255}, 
{ 15,   3, 131}, 
{15,  4,  0}, 
{15,  5,  0}, 
{15,  6, 84}, 
{ 15,   7, 233}, 
{ 15,   8, 254}, 
{ 15,   9, 224}, 
{15, 10, 61}, 
{15, 11,  2}, 
{15, 12,  0}, 
{ 15,  13, 191}, 
{ 15,  14, 255}, 
{ 15,  15, 252}, 
{ 15,  16, 255}, 
{ 16,   0, 255}, 
{ 16,   1, 254}, 
{ 16,   2, 254}, 
{ 16,   3, 235}, 
{16,  4, 27}, 
{16,  5,  0}, 
{16,  6,  0}, 
{16,  7, 24}, 
{16,  8, 52}, 
{16,  9, 17}, 
{16, 10,  0}, 
{16, 11,  0}, 
{16, 12, 62}, 
{ 16,  13, 253}, 
{ 16,  14, 253}, 
{ 16,  15, 255}, 
{ 16,  16, 255}, 
{ 17,   0, 255}, 
{ 17,   1, 255}, 
{ 17,   2, 253}, 
{ 17,   3, 255}, 
{ 17,   4, 194}, 
{17,  5, 16}, 
{17,  6,  0}, 
{17,  7,  0}, 
{17,  8,  0}, 
{17,  9,  0}, 
{17, 10,  0}, 
{17, 11, 36}, 
{ 17,  12, 222}, 
{ 17,  13, 254}, 
{ 17,  14, 253}, 
{ 17,  15, 255}, 
{ 17,  16, 255}, 
{ 18,   0, 255}, 
{ 18,   1, 255}, 
{ 18,   2, 255}, 
{ 18,   3, 253}, 
{ 18,   4, 255}, 
{ 18,   5, 215}, 
{18,  6, 93}, 
{18,  7, 34}, 
{18,  8, 24}, 
{18,  9, 44}, 
{ 18,  10, 115}, 
{ 18,  11, 233}, 
{ 18,  12, 255}, 
{ 18,  13, 254}, 
{ 18,  14, 255}, 
{ 18,  15, 255}, 
{ 18,  16, 255}, 
{ 19,   0, 255}, 
{ 19,   1, 255}, 
{ 19,   2, 255}, 
{ 19,   3, 255}, 
{ 19,   4, 253}, 
{ 19,   5, 255}, 
{ 19,   6, 255}, 
{ 19,   7, 248}, 
{ 19,   8, 239}, 
{ 19,   9, 254}, 
{ 19,  10, 255}, 
{ 19,  11, 255}, 
{ 19,  12, 253}, 
{ 19,  13, 255}, 
{ 19,  14, 255}, 
{ 19,  15, 255}, 
{ 19,  16, 255}, 
{ 20,   0, 255}, 
{ 20,   1, 255}, 
{ 20,   2, 255}, 
{ 20,   3, 255}, 
{ 20,   4, 255}, 
{ 20,   5, 253}, 
{ 20,   6, 252}, 
{ 20,   7, 255}, 
{ 20,   8, 255}, 
{ 20,   9, 255}, 
{ 20,  10, 251}, 
{ 20,  11, 254}, 
{ 20,  12, 255}, 
{ 20,  13, 255}, 
{ 20,  14, 255}, 
{ 20,  15, 255}, 
{ 20,  16, 255}};
        /*
{
{  0,   0, 255}, 
{  0,   1, 254}, 
{  0,   2, 255}, 
{  0,   3, 255}, 
{  0,   4, 238}, 
{  0,   5, 241}, 
{  0,   6, 255}, 
{  0,   7, 255}, 
{  0,   8, 255}, 
{  0,   9, 255}, 
{  1,   0, 253}, 
{  1,   1, 255}, 
{  1,   2, 224}, 
{ 1,  3, 76}, 
{1, 4, 7}, 
{1, 5, 9}, 
{ 1,  6, 88}, 
{  1,   7, 237}, 
{  1,   8, 254}, 
{  1,   9, 254}, 
{  2,   0, 253}, 
{  2,   1, 250}, 
{ 2,  2, 44}, 
{2, 3, 5}, 
{  2,   4, 149}, 
{  2,   5, 149}, 
{2, 6, 0}, 
{ 2,  7, 52}, 
{  2,   8, 254}, 
{  2,   9, 253}, 
{  3,   0, 255}, 
{  3,   1, 190}, 
{3, 2, 0}, 
{  3,   3, 169}, 
{  3,   4, 255}, 
{  3,   5, 255}, 
{  3,   6, 164}, 
{ 3,  7, 73}, 
{  3,   8, 245}, 
{  3,   9, 255}, 
{  4,   0, 255}, 
{  4,   1, 143}, 
{4, 2, 2}, 
{  4,   3, 227}, 
{  4,   4, 251}, 
{  4,   5, 249}, 
{  4,   6, 255}, 
{  4,   7, 255}, 
{  4,   8, 255}, 
{  4,   9, 255}, 
{  5,   0, 255}, 
{  5,   1, 142}, 
{5, 2, 2}, 
{  5,   3, 228}, 
{  5,   4, 251}, 
{  5,   5, 249}, 
{  5,   6, 255}, 
{  5,   7, 255}, 
{  5,   8, 254}, 
{  5,   9, 255}, 
{  6,   0, 255}, 
{  6,   1, 188}, 
{6, 2, 0}, 
{  6,   3, 178}, 
{  6,   4, 255}, 
{  6,   5, 255}, 
{  6,   6, 151}, 
{ 6,  7, 43}, 
{  6,   8, 241}, 
{  6,   9, 255}, 
{  7,   0, 254}, 
{  7,   1, 249}, 
{ 7,  2, 40}, 
{ 7,  3, 12}, 
{  7,   4, 160}, 
{  7,   5, 153}, 
{7, 6, 0}, 
{ 7,  7, 58}, 
{  7,   8, 254}, 
{  7,   9, 253}, 
{  8,   0, 253}, 
{  8,   1, 255}, 
{  8,   2, 222}, 
{ 8,  3, 71}, 
{8, 4, 6}, 
{8, 5, 9}, 
{ 8,  6, 85}, 
{  8,   7, 236}, 
{  8,   8, 254}, 
{  8,   9, 253}, 
{  9,   0, 255}, 
{  9,   1, 254}, 
{  9,   2, 255}, 
{  9,   3, 255}, 
{  9,   4, 237}, 
{  9,   5, 240}, 
{  9,   6, 255}, 
{  9,   7, 255}, 
{  9,   8, 255}, 
{  9,   9, 255}};
        /*
{
{  0,   0, 221}, 
{  0,   1, 221}, 
{  0,   2, 221}, 
{  0,   3, 219}, 
{  0,   4, 218}, 
{  0,   5, 218}, 
{  0,   6, 219}, 
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
{  2,   1, 219}, 
{  2,   2, 221}, 
{  2,   3, 167}, 
{ 2,  4, 44}, 
{ 2,  5, 48}, 
{  2,   6, 167}, 
{  2,   7, 221}, 
{  2,   8, 219}, 
{  2,   9, 221}, 
{  3,   0, 220}, 
{  3,   1, 220}, 
{  3,   2, 212}, 
{3, 3, 6}, 
{  3,   4, 138}, 
{  3,   5, 130}, 
{ 3,  6, 57}, 
{  3,   7, 221}, 
{  3,   8, 219}, 
{  3,   9, 221}, 
{  4,   0, 219}, 
{  4,   1, 221}, 
{  4,   2, 179}, 
{ 4,  3, 19}, 
{  4,   4, 221}, 
{  4,   5, 221}, 
{  4,   6, 221}, 
{  4,   7, 221}, 
{  4,   8, 221}, 
{  4,   9, 221}, 
{  5,   0, 220}, 
{  5,   1, 220}, 
{  5,   2, 204}, 
{5, 3, 0}, 
{  5,   4, 184}, 
{  5,   5, 169}, 
{ 5,  6, 69}, 
{  5,   7, 221}, 
{  5,   8, 220}, 
{  5,   9, 221}, 
{  6,   0, 221}, 
{  6,   1, 219}, 
{  6,   2, 221}, 
{  6,   3, 129}, 
{ 6,  4, 27}, 
{ 6,  5, 27}, 
{  6,   6, 130}, 
{  6,   7, 221}, 
{  6,   8, 219}, 
{  6,   9, 221}, 
{  7,   0, 221}, 
{  7,   1, 221}, 
{  7,   2, 220}, 
{  7,   3, 221}, 
{  7,   4, 213}, 
{  7,   5, 215}, 
{  7,   6, 221}, 
{  7,   7, 221}, 
{  7,   8, 221}, 
{  7,   9, 221}, 
{  8,   0, 221}, 
{  8,   1, 221}, 
{  8,   2, 221}, 
{  8,   3, 218}, 
{  8,   4, 221}, 
{  8,   5, 221}, 
{  8,   6, 219}, 
{  8,   7, 221}, 
{  8,   8, 221}, 
{  8,   9, 221}, 
{  9,   0, 221}, 
{  9,   1, 221}, 
{  9,   2, 221}, 
{  9,   3, 221}, 
{  9,   4, 220}, 
{  9,   5, 220}, 
{  9,   6, 221}, 
{  9,   7, 221}, 
{  9,   8, 221}, 
{  9,   9, 221}};
         * */

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
static int debug19, debug20;
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
  debug19 = z_start; debug20 = image_size;
//  // Move Z to start position
//  move(&stp_3, Z_START);
//  keep_moving = 1;
//  PT_YIELD_UNTIL(&pt_move, keep_moving == 0);
//  disable_stp(&stp_3);

  // Find highest position in image and start there
  for (i = 0; i < z_start+1; i=i+20) {
      debug10 = i; debug19 = z_start;
 //     PT_YIELD_TIME_msec(2000);
    for (j = 0; j < image_size; j++) {
        debug11 = j; debug20 = image_size;
        
  //       PT_YIELD_TIME_msec(2000);
      // Check if z position should be cut
      pixel = image[j];
      debug12 = pixel.x; debug13 = pixel.y; debug14 = pixel.z;
      if (i == 0 && j == 0) last_pixel = pixel;
      if (pixel.z < i) continue;
      if ((absDiff(last_pixel.x, pixel.x)+absDiff(last_pixel.y, pixel.y)) > 1) {
          debug15++;
        set_dc_state(&dc, 0);
        z_pos = Z_START;
        move(&stp_3, z_pos);
        keep_moving = 1;
        PT_YIELD_UNTIL(&pt_move, keep_moving == 0);
        disable_stp(&stp_3);
      }
      // Travel to (x, y) coordinate to drill
      x_pos = pixel.x*STEP_X; debug1 = pixel.x;
      move(&stp_1, x_pos);
      keep_moving = 1;
      PT_YIELD_UNTIL(&pt_move, keep_moving == 0);
      disable_stp(&stp_1);
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
      debug16 = last_pixel.x; debug17 = last_pixel.y; debug18 = last_pixel.z;
    }
  }
  
  set_dc_state(&dc, 0);
  // Once done working through the image array just yield forever
  PT_YIELD_UNTIL(&pt_move, image_carved == 0); 
  load_start_cond();
  //while(1) PT_YIELD(&pt_move);
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
        if (aligned) {
            tft_setCursor(0, 50);
            tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
            sprintf(buffer,"moved res %d zst %d isz %d", debug15, debug19, debug20);
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