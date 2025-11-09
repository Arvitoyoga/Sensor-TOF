#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define TAG "VL53L0X"

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_FREQ_HZ 100000
#define VL53L0X_ADDR 0x29
#define XSHUT_GPIO 14

static void i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

static esp_err_t i2c_write(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_write_to_device(I2C_MASTER_NUM, VL53L0X_ADDR, buf, 2, pdMS_TO_TICKS(10));
}

static esp_err_t i2c_read(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, VL53L0X_ADDR, &reg, 1, data, len, pdMS_TO_TICKS(10));
}

void vl53l0x_start_continuous(void)
{
    i2c_write(0x80, 0x01);
    i2c_write(0xFF, 0x01);
    i2c_write(0x00, 0x00);
    i2c_write(0x91, 0x3C); 
    i2c_write(0x00, 0x01);
    i2c_write(0xFF, 0x00);
    i2c_write(0x80, 0x00);

    i2c_write(0x00, 0x02); 
}

uint16_t vl53l0x_read_distance(void)
{
    uint8_t range_status = 0;
    uint8_t buffer[12];
    uint16_t dist = 0;

    for (int i = 0; i < 10; i++)
    {
        i2c_read(0x13, &range_status, 1);
        if (range_status & 0x07)
            break;
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    if (i2c_read(0x14, buffer, 12) == ESP_OK)
    {
        dist = (buffer[10] << 8) | buffer[11];
    }

    i2c_write(0x0B, 0x01);
    return dist;
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting VL53L0X continuous test...");

    gpio_set_direction(XSHUT_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(XSHUT_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(50));

    i2c_master_init();

    vl53l0x_start_continuous();

    while (1)
    {
        uint16_t dist = vl53l0x_read_distance();
        dist = dist -65;
        ESP_LOGI(TAG, "Distance: %d mm", dist);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
