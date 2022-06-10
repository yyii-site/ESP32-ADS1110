#include "ADS1110.h"
#include "tools.h"

#define REG_INFO

#define I2C_FREQ_HZ 1000000 // Max 1MHz for esp32

const uint8_t DEFAULT_CONFIG   =  12;      // B00001100 (16-BIT, 15 SPS, GAIN x1, CONTINUOUS)
const uint8_t DEFAULT_DATA     =   0;      // default value of Raw Data registers
const uint8_t START_CONVERSION = 128;      // B10000000 (employed in 'Single-Shot' Conversion Mode)
const uint8_t COM_SUCCESS      =   0;      // I2C Communication Success (No Error)
const uint8_t MIN_CON_TIME     =   5;      // minimum ADC Comversion time (in mS)
const uint8_t NUM_BYTES        =   3;      // fixed number of bytes requested from the device
const int     MAX_NUM_ATTEMPTS =   3;      // number of attempts to get new data from device

void delay(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

int ads1110_i2c_init (i2c_dev_t *dev, addr_t addr, i2c_port_t port,
        gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    if (dev == 0)
        return -1;
    
    dev->port = port;
    dev->addr = addr;
    dev->cfg.sda_io_num = sda_gpio;
    dev->cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->cfg.master.clk_speed = I2C_FREQ_HZ;
#endif

    if (i2c_dev_create_mutex(dev) == ESP_OK)
        return 0;
    else
        return -1;
}

int ads1110_register (ADS1110_t *ads, i2c_dev_t dev)
{
    ads->dev = dev;
    ads->config = DEFAULT_CONFIG;
    ads->vref = INT_REF;
    return 0;
}

uint8_t readFromDevice(ADS1110_t *ads, uint8_t *buf, size_t len)
{
    const i2c_dev_t *dev = &ads->dev;
    uint8_t reg[3]={0};
    uint8_t i;
    
    if (i2c_dev_read(dev, NULL, 0, reg, 3) == ESP_OK)
    {
#ifdef REG_INFO
        printf("output register (D15~D0): 0x%02x 0x%02x\n", reg[0], reg[1]);
        printf("configuration register  : 0x%02x\n", reg[2]);
#endif

        if (len > 3) {
            len = 3;
        }
        i = len;   
        while (i) {
            i--;
            *(buf+i) = reg[i];
        }
        return len;        
    } else {
        return 0;
    }
}

int writeToDevice(ADS1110_t *ads, uint8_t buf)
{
    const i2c_dev_t *dev = &ads->dev;
    esp_err_t res = i2c_dev_write(dev, NULL, 0, &buf, 1);
    if (res == ESP_OK) {
        return 0;
    } else {
        return -1;
    }    
}

/*
 * GET CONFIGURATION SETTINGS (FROM DEVICE)
 */
uint8_t getConfig(ADS1110_t *ads) {
    uint8_t devConfig;
    uint8_t buf[3];
    uint8_t len;

    len = readFromDevice(ads, buf, sizeof(buf));            // request 3 bytes from device
    if (len == NUM_BYTES) {
        devConfig = buf[2];
    } else {
        devConfig = ads->config;                            // if 3 bytes were not recieved...
    }
    return devConfig;                                       // return device config uint8_t
}

/*
 * SET CONFIGURATION REGISTER
 */
void setConfig(ADS1110_t *ads, uint8_t newConfig) {
    int status = writeToDevice(ads, newConfig);
    if (status == 0) ads->config = newConfig;
}

/*
 * GET GAIN (1 = GAIN x1 / 2 = GAIN x2 / 4 = GAIN x4 / 8 = GAIN x8)
 */
uint8_t getGain(ADS1110_t *ads) {
    return (1 << (ads->config & GAIN_MASK));
}

/*
 * GET SAMPLE RATE (15 = 15 SPS / 30 = 30 SPS / 60 = 60 SPS / 240 = 240 SPS)
 */
uint8_t getSampleRate(ADS1110_t *ads) {
    switch (ads->config & SPS_MASK) {
        case (SPS_15):  return  15; break;
        case (SPS_30):  return  30; break;
        case (SPS_60):  return  60; break;
        case (SPS_240): return 240; break;
    }
    return 15;
}

/*
 * GET CONVERSION MODE (0 = CONTINUOUS / 1 = SINGLE-SHOT)
 */
uint8_t getConMode(ADS1110_t *ads) {
    return bitRead(ads->config, 4);
}

/*
 * GET RESOLUTION (12 = 12-BIT / 14 = 14-BIT / 15 = 15-BIT / 16 = 16-BIT)
 */
uint8_t getRes(ADS1110_t *ads) {
    switch (ads->config & SPS_MASK) {
        case (SPS_15):  return 16; break;
        case (SPS_30):  return 15; break;
        case (SPS_60):  return 14; break;
        case (SPS_240): return 12; break;
    }
    return 16;
}

/*
 * GET VOLTAGE REFERENCE (0 = INTERNAL / 2048 = EXTERNAL)
 */
int getVref(ADS1110_t *ads) {
    return ads->vref;
}

/*
 * SET GAIN
 */
void setGain(ADS1110_t *ads, gain_t newGain) {                          // PARAMS: GAIN_1 / GAIN_2 / GAIN_4 / GAIN_8
    setConfig(ads, (ads->config & ~GAIN_MASK) | (newGain & GAIN_MASK));
}

/*
 * SET SAMPLE RATE (IN SAMPLES PER SECOND)
 */
void setSampleRate(ADS1110_t *ads, sample_rate_t newRate) {             // PARAMS: SPS_15 / SPS_30 / SPS_60 / SPS_240
    setConfig(ads, (ads->config & ~SPS_MASK) | (newRate & SPS_MASK));
}

/*
 * SET CONVERSION MODE
 */
void setConMode(ADS1110_t *ads, con_mode_t newConMode) {                            // PARAMS: CONT / SINGLE
    setConfig(ads, newConMode ? bitSet(ads->config, 4) : bitClear(ads->config, 4));
}

/*
 * SET RESOLUTION
 */
void setRes(ADS1110_t *ads, res_t newRes) {                             // PARAMS: 12_BIT / 14_BIT / 15_BIT / 16_BIT
    switch (newRes) {
        case (RES_12): setConfig(ads, (ads->config & ~SPS_MASK) | (SPS_240 & SPS_MASK)); break;
        case (RES_14): setConfig(ads, (ads->config & ~SPS_MASK) | (SPS_60  & SPS_MASK)); break;
        case (RES_15): setConfig(ads, (ads->config & ~SPS_MASK) | (SPS_30  & SPS_MASK)); break;
        case (RES_16): setConfig(ads, (ads->config & ~SPS_MASK) | (SPS_15  & SPS_MASK)); break;
    }
}

/*
 * SET VOLTAGE REFERENCE
 */
void setVref(ADS1110_t *ads, vref_t newVref) {                         // PARAMS: INT_REF / EXT_REF
    ads->vref = newVref;
}

/*
 * RESET
 */
void reset(ADS1110_t *ads) {
    setConfig(ads, DEFAULT_CONFIG);
    ads->vref = INT_REF;
}

/*
 * FIND MINIMAL CODE (BASED ON SAMPLE RATE)
 */
uint8_t findMinCode(sample_rate_t sampleRate) {
    switch (sampleRate) {
        case (SPS_15) : return MIN_CODE_15;  break;
        case (SPS_30) : return MIN_CODE_30;  break;
        case (SPS_60) : return MIN_CODE_60;  break;
        case (SPS_240): return MIN_CODE_240; break;
    }
    return MIN_CODE_15;
}

/*
 * GET DATA FROM DEVICE
 */
int getData(ADS1110_t *ads) {
    uint8_t devConfig;
    uint16_t regData;
    int16_t valueData;
    uint8_t attemptCount = 0;
    uint8_t buf[3];
    uint8_t len;
    if (bitRead(ads->config, 4)) {                                  // if device is in 'SINGLE-SHOT' mode...
        setConfig(ads, ads->config | START_CONVERSION);             // add start conversion command to config uint8_t                                             // issue start conversion command
        delay(MIN_CON_TIME * findMinCode(ads->config & SPS_MASK));  // wait for conversion to complete
    }
    while (attemptCount < MAX_NUM_ATTEMPTS) {                   // make up to 3 attempts to get new data
        len = readFromDevice(ads, buf, sizeof(buf));            // request 3 bytes from device
        if (len == NUM_BYTES) {                                 // if 3 bytes were recieved...
            regData = buf[0];
            regData <<= 8;
            regData |= buf[1];                                      // read data register
            valueData = *(int16_t *)(&regData);
            devConfig = buf[2];                                 // read config register
            /*
            if (bitRead(devConfig, 7)) {                        // check if new data available...
                delay(MIN_CON_TIME);                            // if not available yet, wait a bit longer
                attemptCount++;                                 // increment attemps count
            } else return devData;                              // if new data is available, return conversion result
            */
           ads->config = devConfig;
           return valueData;
        } else {                                                // if 3 bytes were not recieved...
            //emptyBuffer();                                      // empty I2C buffer
            //_comBuffer = ping();                                // store I2C error code to find out what went wrong
            attemptCount = MAX_NUM_ATTEMPTS;                    // exit while loop
        }
    }
    return 0;                                                   // if operation unsuccessful, return 0
}

int round_d(double number)
{
    return (number >= 0) ? (int)(number + 0.5) : (int)(number - 0.5);
}

/*
 * GET VOLTAGE (mV)
 */
    // Vin+ = (output_code / (my_min_code  * GAIN)) + _Vreg
    // Output_Code = raw data from device (int)
    // my_min_code = 16 (15_SPS; 16-BIT) / 8 (30_SPS; 15-BIT) / 4 (60_SPS; 14-BIT) / 1 (240_SPS; 12-BIT)
    // _Vref = Vin- (in mV, depends on whether Vin- is connected to GND or to a 2048mV reference source)
    // Vin+ = 0 - 2048mV when Pin Vin- (=_Vref) is connected to GND
    // Vin+ = 0 - 4096mV when Pin Vin- (=_Vref) is connected to an external 2.048V reference source
int ads1110_getVoltage(ADS1110_t *ads) {
    uint8_t gain = (1 << (ads->config & GAIN_MASK));
    uint8_t minCode = findMinCode(ads->config & SPS_MASK);
    return (round_d((float)getData(ads) / (float)(minCode * gain)) + ads->vref);
}

/*
 * MAP FLOATING POINT HELPER FUNCTION (FOR PERCENT CALCULATION)
 */
double mapf(double val, double in_min, double in_max, double out_min, double out_max) {
    return ((val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

/*
 * GET PERCENTAGE (0-100%)
 */
uint8_t getPercent(ADS1110_t *ads) {
    int lowerLimit = (findMinCode(ads->config & SPS_MASK) << 11) * -1;
    int upperLimit = (findMinCode(ads->config & SPS_MASK) << 11) - 1;
    return round_d(mapf(getData(ads), lowerLimit, upperLimit, 0, 100));
}

void ads1110_setGain(ADS1110_t *ads, gain_t newGain)
{
    setGain(ads, newGain);
}