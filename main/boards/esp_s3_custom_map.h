#ifndef ESP_S3_CUSTOM_MAP_H
#define ESP_S3_CUSTOM_MAP_H

// --- STEPPER MOTORS ---
#define X_STEP_PIN GPIO_NUM_1
// #define Y_STEP_PIN GPIO_NUM_6
// #define Z_STEP_PIN GPIO_NUM_4


// temp fix ----------------------
#define Y_STEP_PIN GPIO_NUM_46
#define Y_DIRECTION_PIN GPIO_NUM_10
// temp fix ----------------------


#define X_DIRECTION_PIN GPIO_NUM_2
// #define Y_DIRECTION_PIN GPIO_NUM_7
// #define Z_DIRECTION_PIN GPIO_NUM_5

#define STEPPERS_ENABLE_PIN GPIO_NUM_0

// --- LIMIT SWITCHES ---
// NOTE: IO46 (Y_LIMIT) might need a physical 10k pull-up resistor
#define X_LIMIT_PIN GPIO_NUM_21
// #define Y_LIMIT_PIN GPIO_NUM_46
// #define Z_LIMIT_PIN GPIO_NUM_10


// --- SPINDLE & COOLANT ---
#define SPINDLE_PULSE_PIN GPIO_NUM_39
#define SPINDLE_INDEX_PIN GPIO_NUM_40

#if SPINDLE0_ENABLE
#define SPINDLE_ENCODER_PIN SPINDLE_PULSE_PIN
#define SPINDLE_TRIGGER_PIN SPINDLE_INDEX_PIN
#define SPINDLE_PWM_PIN GPIO_NUM_13
#define SPINDLE_DIRECTION_PIN GPIO_NUM_12
#ifndef SPINDLE_ENABLE_PIN
#define SPINDLE_ENABLE_PIN -1
#endif
#endif

#ifdef LINEAR_MOTOR
#define PWM_PIN_LINEAR_MOTOR GPIO_NUM_13 
#else
#define COOLANT_FLOOD_PIN GPIO_NUM_45
#endif 


#if EMBROIDERY_ENABLE
#define AUXINPUT0_PIN GPIO_NUM_40  // SPINDLE TRIGGER PIN
// #define AUXINPUT2_PIN GPIO_NUM_8
// #define AUXINPUT3_PIN GPIO_NUM_9
// #define AUXINPUT1_PIN GPIO_NUM_38
#endif


#if PWM_SERVO_ENABLE
#define AUXOUTPUT0_PWM_PIN GPIO_NUM_11
#define SERVO0_PIN GPIO_NUM_11
#endif

// --- USER CONTROLS (Buttons) ---
// NOTE: IO0 (FEED_HOLD) is a strapping pin; ensure button is NOT pressed during boot
#if USER_CONTROL_ENABLE
#define RESET_PIN               GPIO_NUM_6
#define FEED_HOLD_PIN           GPIO_NUM_5
#define CYCLE_START_PIN         GPIO_NUM_7
#endif


// --- CAN BUS ---
#if CANBUS_ENABLE

#endif



// --- I2C (For Displays or Keypads) ---
#if I2C_ENABLE
#define I2C_PORT I2C_NUM_1
#define I2C_SDA GPIO_NUM_8
#define I2C_SCL GPIO_NUM_9
#define I2C_CLOCK 100000
#endif

// --- EXTERNAL SD CARD (Optional) ---
#if SDCARD_ENABLE
#define SD_CS_PIN               GPIO_NUM_14
#define SPI_MOSI_PIN            GPIO_NUM_3
#define SPI_SCK_PIN             GPIO_NUM_20
#define SPI_MISO_PIN            GPIO_NUM_19


// works with long module
// #define SPI_MISO_PIN                GPIO_NUM_16
// #define SPI_MOSI_PIN                GPIO_NUM_15
// #define SPI_SCK_PIN                 GPIO_NUM_48 // in s3 UNO pin labelled 47 is actually 48 and vice versa
// #define SD_CS_PIN                   GPIO_NUM_47


// works with long module
// #define SPI_MISO_PIN GPIO_NUM_46
// #define SPI_MOSI_PIN GPIO_NUM_10
// #define SPI_SCK_PIN GPIO_NUM_11
// #define SD_CS_PIN GPIO_NUM_13

#endif

#endif