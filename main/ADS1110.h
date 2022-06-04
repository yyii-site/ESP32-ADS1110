#ifndef __ADS1110_h
#define __ADS1110_h

#include <i2cdev.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ADS1110A0 = 0x48,      // B01001000
    ADS1110A1 = 0x49,      // B01001001
    ADS1110A2 = 0x4A,      // B01001010
    ADS1110A3 = 0x4B,      // B01001011
    ADS1110A4 = 0x4C,      // B01001100
    ADS1110A5 = 0x4D,      // B01001101
    ADS1110A6 = 0x4E,      // B01001110
    ADS1110A7 = 0x4F,      // B01001111
} addr_t;

typedef enum {
    GAIN_MASK = 0x03,      // 3 - B00000011
    GAIN_1    = 0x00,      // 0 - B00000000 (Default)
    GAIN_2    = 0x01,      // 1 - B00000001
    GAIN_4    = 0x02,      // 2 - B00000010
    GAIN_8    = 0x03       // 3 - B00000011
} gain_t;

typedef enum {
    SPS_MASK = 0x0C,       // 12 - B00001100
    SPS_15   = 0x0C,       // 12 - B00001100 (Default)
    SPS_30   = 0x08,       //  8 - B00001000
    SPS_60   = 0x04,       //  4 - B00000100
    SPS_240  = 0x00        //  0 - B00000000
} sample_rate_t;

typedef enum {
    CONT          = 0x00,  // B00000000 (Defualt)
    SINGLE        = 0x10   // B00010000
} con_mode_t;

typedef enum {
    MIN_CODE_240 = 0x01,   //  1 - Minimal Data Value for 240_SPS / -2048  (12-BIT)
    MIN_CODE_60  = 0x04,   //  4 - Minimal Data Value for 60_SPS  / -2048  (14-BIT)
    MIN_CODE_30  = 0x08,   //  8 - Minimal Data Value for 30_SPS  / -2048  (15-BIT)
    MIN_CODE_15  = 0x10    // 16 - Minimal Data Value for 15_SPS  / -2048  (16-BIT) (Default)
} min_code_t;

typedef enum {
    RES_12,                // 12-BIT Resolution
    RES_14,                // 14-BIT Resolution
    RES_15,                // 15-BIT Resolution
    RES_16                 // 16-BIT Resolution (Default)
} res_t;

typedef enum {
    INT_REF =    0,        // Inernal Reference:  Pin Vin- is connected to GND (Default)
    EXT_REF = 2048         // External Reference: Pin Vin- is connected to 2.048V source
} vref_t;

typedef struct {
    i2c_dev_t dev;
    uint8_t config;
    uint8_t vref;
}ADS1110_t;

int ads1110_i2c_init (i2c_dev_t *dev, addr_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);
int ads1110_register (ADS1110_t *ads, i2c_dev_t dev);
int ads1110_getVoltage (ADS1110_t *ads);

void ads1110_setGain(ADS1110_t *ads, gain_t newGain);

#ifdef __cplusplus
}
#endif

#endif