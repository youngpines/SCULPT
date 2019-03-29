#ifndef GPIO_H
#define GPIO_H
typedef unsigned char uint8_t;

// Structs
struct stepper_t  stp_1, stp_2, stp_3; // x, y, z
struct dc_motor_t dc;                 //dc motor 
volatile uint8_t x_turned_off;
volatile uint8_t y_turned_off;
volatile uint8_t z_turned_off;
volatile uint8_t x_turned_on;
volatile uint8_t y_turned_on;
volatile uint8_t z_turned_on;


/********************* [ Structs ] ********************************************/
struct stepper_t { // DRV8825 Driver
  int DIR;      // DIRection pin for driver
  int STP;      // STeP      pin for driver
  int SLEEP;    // SLEEP     pin for driver
  int dir_move; // Direction to move, 0 CCW, 1 CW
  volatile int stps_left; // Num steps left to go through
};

struct dc_motor_t { // DC motor
    int ENABLE; // Enable the motor, when 0 ignores `on`
    int on;     // Turn motor on (1) or off (0)
};

/******************************* [ Macros ] ***********************************/
// Enable Pulldowns
#define EnablePullDownB(bits) \
  CNPUBCLR = bits;            \
  CNPDBSET = bits
#define EnablePullDownA(bits) \
  CNPUACLR = bits;            \
  CNPDASET = bits
// Debug LED Functions
#define LEDPIN BIT_3
#define init_LED()                 \
  mPORTBSetPinsDigitalOut(LEDPIN); \
  mPORTBClearBits(LEDPIN)
#define set_LED()   mPORTBSetBits(LEDPIN)
#define clear_LED() mPORTBClearBits(LEDPIN)
#define toggle_LED() mPORTBToggleBits(LEDPIN)

void init_limit_switches(void);
void init_steppers(void);
void init_dc_motor(void);
void load_start_cond(void);

uint8_t read_limit_x(void);
uint8_t read_limit_y(void);
uint8_t read_limit_z(void);
uint8_t read_mat_load(void);
void set_dir_x(uint8_t pos_mvmt);
void set_dir_y(uint8_t pos_mvmt);
void set_dir_z(uint8_t pos_mvmt);
void toggle_x(void);
void toggle_y(void);
void toggle_z(void);
void disable_x(void);
void disable_y(void);
void disable_z(void);
void enable_x(void);
void enable_y(void);
void enable_z(void);
void set_dc_state(uint8_t on_or_off);

#endif // GPIO_H