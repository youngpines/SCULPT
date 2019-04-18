#ifndef GPIO_H
#define GPIO_H
typedef unsigned char uint8_t;

/********************* [ Structs ] ********************************************/
struct stepper_motor_t { // DRV8825 Driver
  int DIR;      // DIRection pin for driver
  int STP;      // STeP      pin for driver
  int SLEEP;    // SLEEP     pin for driver
  int stp_num;
  volatile int pos;
  volatile int dir_move; // Direction to move, 0 CCW, 1 CW
  volatile int stps_left; // Num steps left to go through
};
typedef struct stepper_motor_t stepper_t;

struct dc_motor_t { // DC motor
    int ENABLE; // Enable the motor, when 0 ignores `on`
    int on;     // Turn motor on (1) or off (0)
};
typedef struct dc_motor_t dc_t;

// Structs
stepper_t stp_1, stp_2, stp_3; // x, y, z
dc_t dc;                 //dc motor 
extern volatile uint8_t x_enable;
extern volatile uint8_t y_enable;
extern volatile uint8_t z_enable;

/******************************* [ Macros ] ***********************************/
// Enable Pulldowns
#define EnablePullDownB(bits) \
  CNPUBCLR = bits;            \
  CNPDBSET = bits
#define EnablePullDownA(bits) \
  CNPUACLR = bits;            \
  CNPDASET = bits
// Debug LED Functions
#define RED_LED_PIN BIT_3
#define GREEN_LED_PIN BIT_14
#define init_RedLED()                 \
  mPORTBSetPinsDigitalOut(RED_LED_PIN); \
  mPORTBClearBits(RED_LED_PIN)
#define set_RedLED()   mPORTBSetBits(RED_LED_PIN)
#define clear_RedLED() mPORTBClearBits(RED_LED_PIN)
#define toggle_RedLED() mPORTBToggleBits(RED_LED_PIN)
#define init_GreenLED()                 \
  mPORTBSetPinsDigitalOut(GREEN_LED_PIN); \
  mPORTBClearBits(GREEN_LED_PIN)
#define set_GreenLED()   mPORTBSetBits(GREEN_LED_PIN)
#define clear_GreenLED() mPORTBClearBits(GREEN_LED_PIN)
#define toggle_GreenLED() mPORTBToggleBits(GREEN_LED_PIN)

#define TFT_LED BIT_15
#define init_tftLED()                 \
  mPORTBSetPinsDigitalOut(TFT_LED); \
  mPORTBClearBits(TFT_LED)
#define set_tftLED()   mPORTBSetBits(TFT_LED)
#define clear_tftLED() mPORTBClearBits(TFT_LED)
#define toggle_tftLED() mPORTBToggleBits(TFT_LED)
volatile int debug3;

void init_limit_switches(void);
void init_steppers(stepper_t* stp_1, stepper_t* stp_2, stepper_t* stp_3);
void init_dc_motor(dc_t* dc);

uint8_t read_limit_x(void);
uint8_t read_limit_y(void);
uint8_t read_limit_z(void);
uint8_t read_mat_load(void);

void set_dir(stepper_t* stp, uint8_t pos_mvmt);
void get_dir(stepper_t* stp);
void toggle_stp(stepper_t* stp);
void disable_stp(stepper_t* stp);
void enable_stp(stepper_t* stp);
void set_dc_state(dc_t* dc, uint8_t on_or_off);
void move(stepper_t* stp, int target_pos);
#endif // GPIO_H