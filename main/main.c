#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <i2cdev.h>
#include <stdio.h>
#include "ADS1110.h"

#define I2C1_PORT           0
#define I2C1_MASTER_SDA     19
#define I2C1_MASTER_SCL     21

#define I2C2_PORT           1
#define I2C2_MASTER_SDA     5
#define I2C2_MASTER_SCL     18

ADS1110_t adc[2] = { 0 };
i2c_dev_t dev[2] = { 0 };

void task(void *ignore)
{
    uint32_t count = 0;
    ads1110_i2c_init(&dev[0], ADS1110A0, I2C1_PORT, I2C1_MASTER_SDA, I2C1_MASTER_SCL);
    ads1110_register(&adc[0], dev[0]);
    ads1110_i2c_init(&dev[1], ADS1110A0, I2C2_PORT, I2C2_MASTER_SDA, I2C2_MASTER_SCL);
    ads1110_register(&adc[1], dev[1]);
    
    while (1)
    {
        count++;
        printf("%d\n", count);
        printf("bus0 ads1110 voltage = %d mV\n", ads1110_getVoltage(&adc[0]));
        printf("bus1 ads1110 voltage = %d mV\n", ads1110_getVoltage(&adc[1]));
        printf("\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main()
{
    // Init i2cdev library
    ESP_ERROR_CHECK(i2cdev_init());
    // Start task
    xTaskCreate(task, "i2c_scanner", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL);
}
