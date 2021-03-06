#include "config_1_2_3.h" // Modified to allow 9600 baudrate
// threading library
//#define use_uart_serial
#include "pt_cornell_1_2_3.h"
#include "gpio.h"
#ifndef _SUPPRESS_PLIB_WARNING
#define _SUPPRESS_PLIB_WARNING
#endif
//////////////////////////////////////
#define TFT 10
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
#define X_DIM 20
#define Y_DIM 20
#define Z_DIM 20
// was 400
#define STEP_X 400
//#define STEP_Y 585
//#define STEP_Z 1000
//#define STEP_X 1000
#define STEP_Y 585
// was 50
#define STEP_Z 8
#define SLEEP_TIME 2

// Stepping Frequency of X & Y, every 1 msec
#define XY_PERIOD  24000
// Stepping Frequency of X & Y, every 6 msec
#define Z_PERIOD 300000

#define X_LIMIT 7000
#define Y_LIMIT 15000
#define Z_LIMIT 8000
//#define X_START 0
#define X_START 0
#define Y_START 3500
//#define Y_START 1500
//#define Z_START 2500
#define Z_START 5000
//#define Z_LIMIT 5000
//#define Y_LIMIT 7850
typedef unsigned char uint8_t;
volatile int debug1 = -1; 
volatile int debug2 = -1;
int debug3 = -1;
int debug4 = 0;

/**************** [ Global Variables ] ****************************************/
static struct pt pt_serial, // thread to import data via UART
                 pt_move,   // thread to get positions and move steppers 
                 pt_align;  // thread to align via limit switches
// The following threads are necessary for UART control
static struct pt pt_input, pt_output, pt_DMA_output;
static struct pt pt_tft;

// Data array holding pixelated info of image
static unsigned char image[X_DIM][Y_DIM] = {
{
22,
18,
19,
19,
20,
22,
21,
21,
22,
22,
22,
22,
23,
22,
21,
21,
19,
18,
18,
17,
},
{
9,
4,
5,
6,
7,
9,
8,
8,
8,
7,
8,
9,
9,
8,
7,
7,
5,
5,
4,
3,
},
{
10,
7,
4,
8,
11,
7,
9,
11,
10,
10,
11,
7,
11,
8,
10,
10,
9,
8,
6,
5,
},
{
10,
4,
21,
11,
8,
23,
11,
10,
10,
11,
9,
23,
12,
24,
9,
10,
9,
7,
8,
7,
},
{
12,
0,
158,
58,
0,
162,
49,
2,
12,
10,
10,
153,
47,
154,
11,
9,
10,
7,
8,
7,
},
{
11,
0,
182,
67,
0,
187,
59,
1,
13,
11,
12,
174,
58,
175,
15,
11,
11,
9,
6,
4,
},
{
10,
0,
175,
66,
3,
180,
58,
0,
4,
3,
12,
167,
55,
168,
15,
1,
1,
2,
7,
5,
},
{
10,
0,
174,
54,
0,
177,
52,
22,
199,
103,
1,
169,
55,
170,
3,
85,
198,
42,
2,
7,
},
{
10,
0,
170,
111,
62,
193,
37,
124,
211,
194,
27,
161,
56,
162,
23,
196,
222,
153,
0,
8,
},
{
10,
0,
165,
249,
242,
225,
40,
164,
22,
141,
74,
149,
58,
151,
71,
167,
20,
181,
16,
3,
},
{
9,
0,
164,
162,
131,
197,
58,
164,
33,
138,
91,
144,
59,
145,
94,
137,
0,
167,
38,
0,
},
{
8,
0,
166,
50,
0,
165,
64,
183,
215,
216,
62,
148,
58,
143,
99,
128,
2,
164,
43,
0,
},
{
8,
0,
164,
62,
3,
166,
65,
159,
62,
38,
13,
156,
55,
142,
92,
130,
0,
168,
38,
0,
},
{
8,
0,
161,
59,
0,
165,
52,
157,
11,
61,
67,
143,
56,
143,
70,
146,
10,
181,
19,
1,
},
{
8,
0,
165,
59,
0,
172,
38,
132,
148,
176,
59,
147,
55,
155,
28,
178,
174,
162,
0,
5,
},
{
9,
0,
131,
44,
0,
137,
32,
38,
218,
148,
4,
129,
39,
132,
1,
107,
225,
66,
1,
6,
},
{
7,
1,
12,
6,
5,
16,
10,
3,
24,
14,
8,
19,
11,
18,
8,
9,
27,
4,
6,
3,
},
{
6,
2,
2,
5,
7,
5,
8,
10,
5,
9,
11,
9,
10,
8,
10,
10,
5,
10,
6,
3,
},
{
3,
0,
1,
2,
4,
5,
6,
5,
8,
8,
8,
8,
8,
8,
6,
6,
8,
6,
4,
1,
},
{
18,
13,
16,
16,
18,
19,
20,
20,
20,
22,
22,
21,
21,
22,
20,
20,
21,
22,
19,
16,
},
};
/*
{
    {0, 0, 0,2,  3,   3, 2,   0, 0, 0}, // x = 0 
    {0, 0, 0,        0,   0,   0,          0,   0, 0, 0}, // x = 1
    {0, 2, 0,        54,  177, 173,        54,  0, 2, 0}, // x = 2
    {1, 1, 9,        215, 83,  91,         164, 0, 2, 0}, // x = 3
    {2, 0, 42,       202, 0,   0,          0,   0, 0, 0}, // x = 4
    {1, 1, 17,       221, 37,  52,         152, 0, 1, 0}, // x = 5
    {0, 2, 0,        92,  194, 194,        91,  0, 2, 0}, // x = 6 
    {0, 0, 1,        0,   8,   6,          0,   0, 0, 0}, // x = 7
    {0, 0, 0,        3,   0,   0,          2,   0, 0, 0}, // x = 8
    {0, 0, 0,        0,   1,   1,          0,   0, 0, 0 } // x = 9
};
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
//currently each change in x and y is 585 steps, z axis is a change of 2000
static PT_THREAD (protothread_move(struct pt *pt))
{
  PT_BEGIN(pt);
  // Wait until the board aligned, material loaded, & the data loaded
  PT_YIELD_UNTIL(&pt_move, data_loaded == 1);
  PT_YIELD_UNTIL(&pt_move, material_loaded == 1); 
  move_start = 1;
  //set_RedLED();
  PT_YIELD_TIME_msec(500);
  //clear_RedLED();
  //clear_GreenLED();
 
  // x and y positions in the image array at start target positions at end of loops
  static int x_pos, y_pos, z_pos;
  // Loop through all entries in the image 
  debug1 = 0; debug2 = 0; PT_YIELD(&pt_move);
  for (x_pos = 0; x_pos < X_DIM; x_pos++) { // x coordinates iterated slowly    
    // Looks at the next y coordinate for the current x and determines
    // if the drill needs to be raised or lowered for that entry
      debug1 = x_pos;
    for (y_pos = 0; y_pos < Y_DIM; y_pos++) { // Look ahead needs shift by 1
      //toggle_RedLED();
      debug2 = y_pos;
      set_dc_state(&dc, 1);
      z_pos = image[x_pos][y_pos] * STEP_Z;
      debug3 = z_pos;
      move(&stp_3, z_pos);
      PT_YIELD_TIME_msec(2);
      // Setting up for the ISR to move the position
      keep_moving = 1;
      // Halting until the desired position is reached
      PT_YIELD_UNTIL(&pt_move, keep_moving == 0); 
      disable_stp(&stp_3);
      // Setting the steps to move in y
      int y_pos = stp_2.pos;
      move(&stp_2, y_pos + STEP_Y);
      PT_YIELD_TIME_msec(SLEEP_TIME);
      // Setting up for the ISR to move the position
      keep_moving = 1;
      // Halting until the desired position is reached
      PT_YIELD_UNTIL(&pt_move, keep_moving == 0); 
      disable_stp(&stp_2);
//      PT_YIELD_TIME_msec(2000);
    } 
      // DONE with y coordinate for a given x coordinate
      // Turn off the dc motor first to preserve integrity of piece
      //toggle_GreenLED();
      // Check if z is raised or not, if not raise it
      move(&stp_3, Z_START);
      PT_YIELD_TIME_msec(SLEEP_TIME);
      // Setting up for the ISR to move the position
      keep_moving = 1;
      // Halting until the desired position is reached
      PT_YIELD_UNTIL(&pt_move, keep_moving == 0); 
      disable_stp(&stp_3);
      set_dc_state(&dc, 0);
      move(&stp_2, Y_START);
      // Setting up for the ISR to move the position
      keep_moving = 1;
      // Halting until the desired position is reached
      PT_YIELD_UNTIL(&pt_move, keep_moving == 0); 
      disable_stp(&stp_2);
      int x_pos = stp_1.pos;
      move(&stp_1, x_pos + STEP_X);
      // Setting up for the ISR to move the position
      keep_moving = 1;
      // Halting until the desired position is reached
      PT_YIELD_UNTIL(&pt_move, keep_moving == 0); 
      disable_stp(&stp_1);
   //   PT_YIELD_TIME_msec(2000);
  }
     
  // Turn off the dc motor 
  //set_dc_state(&dc, 0);
  // Tell align & data thread image has been carved and need to realign
  image_carved = 1;
  load_start_cond();
    
  // Once done working through the image array just yield forever
  PT_YIELD_UNTIL(&pt_move, image_carved == 0);          
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
  static char cmd[30];
  static int value;
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
    sscanf(PT_term_buffer, "%s %d", cmd, &value);

    switch(cmd[0]) {
    case 'p': // Load pixel values into the pixel array
      image[count%X_DIM][count/Y_DIM] = value;
      count++;
      //toggle_RedLED();
      break;
    case 'e': // All data loaded, Terminate signal sent
      if (count == X_DIM*Y_DIM) clear_GreenLED();
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
        tft_fillRoundRect(0, 70, 300, 80, 1, ILI9340_BLACK);// x,y,w,h,radius,color
        tft_setCursor(0, 10);
        tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
//        sprintf(buffer,"%d", debug_3);
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
        if (debug1 != -1) {
            tft_setCursor(0, 70);
            tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
            sprintf(buffer,"x %d", debug1);
            tft_writeString(buffer);
        }
        if (debug2 != -1) {
            tft_setCursor(50, 70);
            tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
            sprintf(buffer,"y %d", debug2);
            tft_writeString(buffer);
        }
        if (debug3 != -1) {
            tft_setCursor(100, 70);
            tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
            sprintf(buffer,"z %d", debug3);
            tft_writeString(buffer);
        }
        if (aligned) {
            tft_setCursor(0, 50);
            tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
            sprintf(buffer,"moved");
            tft_writeString(buffer);
            tft_setCursor(0, 90);
            tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
            sprintf(buffer,"x: pos %d, s %d, d %d", stp_1.pos, stp_1.stps_left, stp_1.dir_move);
            tft_writeString(buffer);
            tft_setCursor(0, 110);
            tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
            sprintf(buffer,"y: pos %d, s %d, d %d", stp_2.pos, stp_2.stps_left, stp_2.dir_move);
            tft_writeString(buffer);
            tft_setCursor(0, 130);
            tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
            sprintf(buffer,"z: pos %d, s %d, d %d", stp_3.pos, stp_3.stps_left, stp_3.dir_move);
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