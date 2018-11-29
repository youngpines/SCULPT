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
          count++;
          
          
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

             switch(cmd[0]){
                 case 'p': // set frequency of DAC sawtooth output
                     // enter frequency in HZ
                     blink_freq = value;
                     break;
                 case 'e': 
                     tft_fillRoundRect(50,50, 100, 100, 1, ILI9340_BLACK);// x,y,w,h,radius,color
            
                    tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(4);

                    tft_setCursor(50, 80);
                    sprintf(buffer,"%d",count);
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
  // round-robin scheduler for threads
  while (1){
      PT_SCHEDULE(protothread_timer(&pt_timer));
      PT_SCHEDULE(protothread_serial(&pt_serial));
      }
  } // main

// === end  ======================================================
