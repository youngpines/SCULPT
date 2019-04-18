#include "gpio.h"
#include <plib.h>

void init_limit_switches2(void)
{
  mPORTASetPinsDigitalIn(BIT_2);  // RA2 - Y axis
  mPORTASetPinsDigitalIn(BIT_3);  // RA3 - X axis
  mPORTASetPinsDigitalIn(BIT_4);  // RA4 - Z axis
  mPORTBSetPinsDigitalIn(BIT_2);  // RB2 - Confirm Material Loaded
  
  // Enabling pulldowns for all buttons        
  EnablePullDownA(BIT_2);
  EnablePullDownA(BIT_3);
  EnablePullDownA(BIT_4);
  EnablePullDownB(BIT_2); 
}

void init_steppers2(void)
{
  // Stepper 1 (X)
  stp_1.DIR   = BIT_0;
  stp_1.STP   = BIT_1;
  stp_1.SLEEP = BIT_0;
  stp_1.dir_move = 0;
  stp_1.stps_left = 0;
  stp_1.pos = 0;
  mPORTBSetPinsDigitalOut(stp_1.DIR);
  mPORTBSetPinsDigitalOut(stp_1.STP);
  mPORTASetPinsDigitalOut(stp_1.SLEEP);
  mPORTBClearBits(stp_1.DIR);
  mPORTBClearBits(stp_1.STP);
  mPORTAClearBits(stp_1.SLEEP);
    
  // Stepper 2 (Y)
  stp_2.DIR   = BIT_4;
  stp_2.STP   = BIT_5;
  stp_2.SLEEP = BIT_13;
  stp_2.dir_move = 0;
  stp_2.stps_left = 0;
  mPORTBSetPinsDigitalOut(stp_2.DIR);  // DIR
  mPORTBSetPinsDigitalOut(stp_2.STP);  // STP
  mPORTBSetPinsDigitalOut(stp_2.SLEEP); // SLEEP
  mPORTBClearBits(stp_2.DIR);
  mPORTBClearBits(stp_2.STP);
  mPORTBClearBits(stp_2.SLEEP);
  
  // Stepper 3 (Z)
  stp_3.DIR   = BIT_7;
  stp_3.STP   = BIT_8;
  stp_3.SLEEP = BIT_9;
  stp_3.dir_move = 0;
  stp_3.stps_left = 0;
  mPORTBSetPinsDigitalOut(stp_3.DIR); // DIR
  mPORTBSetPinsDigitalOut(stp_3.STP); // STP
  mPORTBSetPinsDigitalOut(stp_3.SLEEP); // SLEEP
  mPORTBClearBits(stp_3.DIR);
  mPORTBClearBits(stp_3.STP);
  mPORTBClearBits(stp_3.SLEEP);
}

void init_dc_motor(void)
{
  dc.on = 0;
  dc.ENABLE = BIT_11;
  mPORTBSetPinsDigitalOut(dc.ENABLE);
  mPORTBSetBits(dc.ENABLE);
}

uint8_t read_limit_x(void) {return mPORTAReadBits(BIT_3);}
uint8_t read_limit_y(void) {return mPORTAReadBits(BIT_2);}
uint8_t read_limit_z(void) {return mPORTAReadBits(BIT_4);}
uint8_t read_mat_load(void) {return mPORTBReadBits(BIT_2);}

void set_dir_x(uint8_t pos_mvmt) {
  if (pos_mvmt == 1) 
    mPORTBSetBits(stp_1.DIR);
  else mPORTBClearBits(stp_1.DIR);
}
void set_dir_y(uint8_t pos_mvmt) {
  if (pos_mvmt == 1) 
    mPORTBSetBits(stp_2.DIR);
  else mPORTBClearBits(stp_2.DIR);
}
void set_dir_z(uint8_t pos_mvmt) {
  if (pos_mvmt == 1) 
    mPORTBSetBits(stp_3.DIR);
  else mPORTBClearBits(stp_3.DIR);
}

void toggle_x(void) {mPORTBToggleBits(stp_1.STP);}
void toggle_y(void) {mPORTBToggleBits(stp_2.STP);}
void toggle_z(void) {mPORTBToggleBits(stp_3.STP);}

void disable_x(void) 
{
  mPORTAClearBits(stp_1.SLEEP);
  turned_on = 0;
}
void disable_y(void) 
{
  mPORTBClearBits(stp_2.SLEEP);
  turned_off = 1;
}
void disable_z(void) {
  mPORTBClearBits(stp_3.SLEEP);
  turned_off = 1;
}


void enable_y(void)
{
  mPORTBSetBits(stp_2.SLEEP);
  turned_on = 1;
}
void enable_z(void)
{
  mPORTBSetBits(stp_3.SLEEP);
  turned_on = 1;
}

void set_dc_state(uint8_t on_or_off) {
    if (on_or_off == 1) {
        mPORTBSetBits(dc.ENABLE);
        dc.on = 1;
    } else {
        mPORTBClearBits(dc.ENABLE);
        dc.on = 0;
    }
}

void move2(int stepper_num, struct stepper_t* stepper, int target_pos) {
    /*
  if (stepper->pos < target_pos) {  // z needs to be raised
    set_dc_state(1);
    switch(stepper_num) {
      case 1:
        set_dir_x(1);
        enable_x();
        break;
      case 2:
        set_dir_y(1);
        enable_y();
        break;
      case 3:
        set_dir_z(1);
        enable_z();
        break;
      default:
        break;
    }
    stepper->stps_left = target_pos - stepper->pos;  // 737 was prev value 
  } else if (stepper->pos > target_pos) {   // requires z to be lowered
    switch(stepper_num) {
      case 1:
        set_dir_x(0);
        enable_x();
        break;
      case 2:
        set_dir_y(0);
        enable_y();
        break;
      case 3:
        set_dir_z(0);
        enable_z();
        break;
      default:
        break;
    }
    stepper->stps_left = stepper->pos - target_pos;  // 737 was prev value
  }
     * */
   switch(stepper_num) {
      case 1:
        enable_x();
        break;
      case 2:
        enable_y();
        break;
      case 3:
        enable_z();
        break;
      default:
        break;
    }
   stepper->stps_left = target_pos;
}
