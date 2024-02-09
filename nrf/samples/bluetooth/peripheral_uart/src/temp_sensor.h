#include <zephyr/logging/log.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/util.h>

/* I2C Configurations */
#define MY_TWIM DT_NODELABEL(i2c0)
#define chip_address    0x70
#define TH_SENSOR_TYPE    5


extern struct device *nrfx_twis_dev1;
extern int config_result;

extern uint8_t hum_tx_data[10];
extern uint8_t hum_rx_data[10];
extern unsigned int raw_temperature;


typedef struct sensor_data_s
{
    uint16_t latest_humidity;
    int16_t latest_temperature;
}sensor_data_t;

extern sensor_data_t sensor_data, sensor_data_to_send;

void read_temperature_and_humidity(void);
int twi_init(void);