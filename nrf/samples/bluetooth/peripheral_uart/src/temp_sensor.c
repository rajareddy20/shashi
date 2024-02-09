#include "temp_sensor.h"
#include <zephyr/device.h>
#include <stdint.h>
#include <math.h>



struct device *nrfx_twis_dev1 = DEVICE_DT_GET(MY_TWIM);
int config_result = false;

uint8_t hum_tx_data[10] = {0};
uint8_t hum_rx_data[10] = {0};
unsigned int raw_temperature;

uint16_t latest_temperature = 0;
uint16_t latest_humidity = 0;
uint8_t read_buffer[10] = {0};
uint8_t valid_temperature = 1;

extern uint8_t temp_hum_data[20];

sensor_data_t sensor_data, sensor_data_to_send;

void read_temperature_and_humidity(void)
{
   float temperature;
   float humidity;
   uint16_t temp_value = 0;

   hum_tx_data[0] = 0x35;
   hum_tx_data[1] = 0x17;

   valid_temperature = i2c_write(nrfx_twis_dev1, hum_tx_data, 2, chip_address);
   k_msleep(1);

   if (valid_temperature == 0)
   {
      hum_tx_data[0] = 0x64;
      hum_tx_data[1] = 0x58;

      valid_temperature = i2c_write(nrfx_twis_dev1, hum_tx_data, 2, chip_address);
      k_msleep(10);
      
      if (valid_temperature == 0)
      {
         valid_temperature = i2c_read(nrfx_twis_dev1, hum_rx_data, 6, chip_address);
         
         if(valid_temperature == 0)
         {
            temp_value = hum_rx_data[1] | (hum_rx_data[0] << 8);
            temperature = ((float)(175 * temp_value) / (float)65536) - (float)45.00;
            temperature *= 10.0;
            // latest_temperature =  (int16_t)temperature;
            latest_temperature =  (int16_t)(temperature * 10);

            temp_value = hum_rx_data[4] | (hum_rx_data[3] << 8);
            humidity = ((float)(100 * temp_value) / 65535);                   
            latest_humidity =  (uint16_t)humidity; 

            temp_hum_data[0] = (latest_temperature & 0xFF00) >> 8;
            temp_hum_data[1] = (latest_temperature & 0x00FF);
            temp_hum_data[2] = (latest_humidity & 0x00FF);
         }
      }
   }

   if (valid_temperature != 0)
   {
      latest_temperature = 1000;;

      temp_hum_data[0] = (latest_temperature & 0xFF00) >> 8;
      temp_hum_data[1] = (latest_temperature & 0x00FF);

      temp_hum_data[2] = 0;
   }
   
   hum_tx_data[0] = 0xB0;
   hum_tx_data[1] = 0x98;

   i2c_write(nrfx_twis_dev1, hum_tx_data, 2, chip_address);

   k_msleep(5);
}

int twi_init(void)
{
   if(!device_is_ready(nrfx_twis_dev1))
	{
		return -1;  
	}

   return 0;
}