#include "gpio.h"
#include <plib.h>
volatile uint8_t just_turned_on = 0;

// Init the limit switch pins
void init_limit_switches(void)
{
  mPORTASetPinsDigitalIn(BIT_2);  // RA2 - Y axis
  mPORTASetPinsDigitalIn(BIT_3);  // RA3 - X axis
  mPORTASetPinsDigitalIn(BIT_4);  // RA4 - Z axis
  mPORTBSetPinsDigitalIn(BIT_2);  // RB3 - Confirm Material Loaded
  
  // Enabling pulldowns for all buttons        
  EnablePullDownA(BIT_2);
  EnablePullDownA(BIT_3);
  EnablePullDownA(BIT_4);
  EnablePullDownB(BIT_2); 
}

// Init Stepper pins
void init_steppers(stepper_t* stp_1, stepper_t* stp_2, stepper_t* stp_3)
{
  // Stepper 1 (X)
  stp_1->DIR   = BIT_0;
  stp_1->STP   = BIT_1;
  stp_1->SLEEP = BIT_0;
  stp_1->dir_move = 0;
  stp_1->stps_left = 0;
  stp_1->pos = 0;
  stp_1->stp_num = 1;
  mPORTBSetPinsDigitalOut(stp_1->DIR);
  mPORTBClearBits(stp_1->DIR);
  mPORTBSetPinsDigitalOut(stp_1->STP);
  mPORTASetPinsDigitalOut(stp_1->SLEEP);
  mPORTBClearBits(stp_1->STP);
  mPORTAClearBits(stp_1->SLEEP);
    
  // Stepper 2 (Y)
  stp_2->DIR   = BIT_4;
  stp_2->STP   = BIT_5;
  stp_2->SLEEP = BIT_13;
  stp_2->dir_move = 0;
  stp_2->stps_left = 0;
  stp_2->pos = 0;
  stp_2->stp_num = 2;
  mPORTBSetPinsDigitalOut(stp_2->DIR);  // DIR
  mPORTBSetPinsDigitalOut(stp_2->STP);  // STP
  mPORTBSetPinsDigitalOut(stp_2->SLEEP); // SLEEP
  mPORTBClearBits(stp_2->DIR);
  mPORTBClearBits(stp_2->STP);
  mPORTBClearBits(stp_2->SLEEP);
  
  // Stepper 3 (Z)
  stp_3->DIR   = BIT_7;
  stp_3->STP   = BIT_8;
  stp_3->SLEEP = BIT_9;
  stp_3->dir_move = 0;
  stp_3->stps_left = 0;
  stp_3->pos = 0;
  stp_3->stp_num = 3;
  mPORTBSetPinsDigitalOut(stp_3->DIR); // DIR
  mPORTBSetPinsDigitalOut(stp_3->STP); // STP
  mPORTBSetPinsDigitalOut(stp_3->SLEEP); // SLEEP
  mPORTBClearBits(stp_3->DIR);
  mPORTBClearBits(stp_3->STP);
  mPORTBClearBits(stp_3->SLEEP);
}

// Init DC motor pins
void init_dc_motor(dc_t* dc)
{
  dc->on = 0;
  dc->ENABLE = BIT_11;
  mPORTBSetPinsDigitalOut(dc->ENABLE);
  mPORTBClearBits(dc->ENABLE);
}

// Read X limit switch
uint8_t read_limit_x(void) {return mPORTAReadBits(BIT_3);}
// Read Y limit switch
uint8_t read_limit_y(void) {return mPORTAReadBits(BIT_2);}
// Read Z limit switch
uint8_t read_limit_z(void) {return mPORTAReadBits(BIT_4);}
// Read material load confirmation limit switch
uint8_t read_mat_load(void) {return mPORTBReadBits(BIT_2);}

// Set direction of steppers
void set_dir(stepper_t* stp, uint8_t pos_mvmt) 
{
  // Set stepper direction
  if (pos_mvmt == 1) stp->dir_move = 1;
  else stp->dir_move = 0;
  // Set bits depending on stepper to drive
  switch(stp->stp_num) {
    case 1:
      if (pos_mvmt == 1) mPORTBSetBits(stp->DIR);
      else mPORTBClearBits(stp->DIR);
      break;
    case 2:
      if (pos_mvmt == 1) mPORTBSetBits(stp->DIR);
      else mPORTBClearBits(stp->DIR);
      break;
    case 3:
      if (pos_mvmt == 1) mPORTBSetBits(stp->DIR);
      else mPORTBClearBits(stp->DIR);
      break;
    default:
      break;
  }
}

// Toggle STP pin to make driver move stepper
void toggle_stp(stepper_t* stp)
{
  switch(stp->stp_num) {
    case 1:
      mPORTBToggleBits(stp->STP);
      break;
    case 2:
      mPORTBToggleBits(stp->STP);
      break;
    case 3:
      mPORTBToggleBits(stp->STP);
      break;
    default:
      break;
  }
}

// Disable the stepper driver
void disable_stp(stepper_t* stp) 
{
  switch(stp->stp_num) {
    case 1:
      x_enable = 0;
      mPORTAClearBits(stp->SLEEP);
      break;
    case 2:
      y_enable = 0;
      mPORTBClearBits(stp->SLEEP);
      break;
    case 3:
      z_enable = 0;
      mPORTBClearBits(stp->SLEEP);
      break;
    default:
      break;
  }
}

// Enable the stepper driver
void enable_stp(stepper_t* stp)
{
  switch(stp->stp_num) {
    case 1:
      mPORTASetBits(stp->SLEEP);
      x_enable = 1;
      break;
    case 2:
      mPORTBSetBits(stp->SLEEP);
      y_enable = 1;
      break;
    case 3:
      mPORTBSetBits(stp->SLEEP);
      z_enable = 1;
      break;
    default:
      break;
  }
}

// Set DC motor on or off
void set_dc_state(dc_t* dc, uint8_t on_or_off)
{
  if (on_or_off == 1) {
    mPORTBSetBits(dc->ENABLE);
    dc->on = 1;
  } else {
    mPORTBClearBits(dc->ENABLE);
    dc->on = 0;
  }
}

// Moves a stepper to a target position
void move(stepper_t* stp, int target_pos)
{
  int init_pos = stp->pos;
  // Need to lower
  if (init_pos > target_pos) { 
    set_dir(stp, 0);
    stp->stps_left = init_pos - target_pos;
  // Need to raise
  } else {
    set_dir(stp, 1);
    stp->stps_left = target_pos - init_pos;
  }
  // Enable the stepper driver
  enable_stp(stp);
}