#ifndef ESP_S3_UNO_MAP_H
#define ESP_S3_UNO_MAP_H

// --- STEPPER MOTORS ---
// Matches Shield D2, D3, D4
#define X_STEP_PIN GPIO_NUM_18
// #define Y_STEP_PIN GPIO_NUM_17
// #define Z_STEP_PIN GPIO_NUM_19
// #define Y_STEP_PIN -1
#define Y_STEP_PIN GPIO_NUM_8

// Matches Shield D5, D6, D7
#define X_DIRECTION_PIN GPIO_NUM_20
// #define Y_DIRECTION_PIN GPIO_NUM_3
// #define Z_DIRECTION_PIN GPIO_NUM_14
// #define Y_DIRECTION_PIN -1
#define Y_DIRECTION_PIN GPIO_NUM_9

// Matches Shield D8
#define STEPPERS_ENABLE_PIN GPIO_NUM_21

// --- LIMIT SWITCHES ---
// Matches Shield D9, D10, D11
// NOTE: IO46 (X_LIMIT) might need a physical 10k pull-up resistor
#define X_LIMIT_PIN GPIO_NUM_46
// #define Y_LIMIT_PIN GPIO_NUM_10
// #define Z_LIMIT_PIN GPIO_NUM_11




// --- SPINDLE & COOLANT ---
// Matches Shield D12, D13, A3
#if SPINDLE0_ENABLE
#define SPINDLE_ENCODER_PIN GPIO_NUM_10
#define SPINDLE_TRIGGER_PIN GPIO_NUM_11
#define SPINDLE_PWM_PIN GPIO_NUM_13
#define SPINDLE_DIRECTION_PIN GPIO_NUM_12
#ifndef SPINDLE_ENABLE_PIN
#define SPINDLE_ENABLE_PIN -1
#endif
#endif

#ifdef LINEAR_MOTOR
#define PWM_PIN_LINEAR_MOTOR GPIO_NUM_6 
#else
#define COOLANT_FLOOD_PIN GPIO_NUM_6
#endif 


#if EMBROIDERY_ENABLE
#define AUXINPUT0_PIN SPINDLE_TRIGGER_PIN
#define AUXINPUT2_PIN GPIO_NUM_8
#define AUXINPUT3_PIN GPIO_NUM_9
#define AUXINPUT1_PIN GPIO_NUM_38
#endif


#if PWM_SERVO_ENABLE
#define AUXOUTPUT0_PWM_PIN GPIO_NUM_6
#define SERVO0_PIN GPIO_NUM_6
#endif

// --- USER CONTROLS (Buttons) ---
// Matches Shield A0, A1, A2
// NOTE: IO0 (FEED_HOLD) is a strapping pin; ensure button is NOT pressed during boot
#if USER_CONTROL_ENABLE
#define RESET_PIN               GPIO_NUM_2
#define FEED_HOLD_PIN           GPIO_NUM_0
#define CYCLE_START_PIN         GPIO_NUM_7
#endif



#if EMBROIDERY
#define RPM_ENCODER_PIN   GPIO_NUM_8// interrupt
#define STICH_ENCODER_PIN GPIO_NUM_9// interrupt
#define THREAD_ENCODER_PIN GPIO_NUM_38// interrupt
#define STEP_BACK_PIN  GPIO_NUM_39// push  pull up 100k 
#define STEP_FORWARD_PIN GPIO_NUM_40// push pull up 100k
// 41, 42 safe
#endif 


// --- I2C (For Displays or Keypads) ---
// Matches Shield A4, A5
#if I2C_ENABLE
#define I2C_PORT I2C_NUM_1
#define I2C_SDA GPIO_NUM_5
#define I2C_SCL GPIO_NUM_4
#define I2C_CLOCK 100000
#endif

// --- EXTERNAL SD CARD (Optional) ---
// Uses the "Extra" free pins you identified (non-conflicting)
#if SDCARD_ENABLE
#define SD_CS_PIN               GPIO_NUM_16
#define SPI_MOSI_PIN            GPIO_NUM_15
#define SPI_SCK_PIN             GPIO_NUM_48
#define SPI_MISO_PIN            GPIO_NUM_47


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