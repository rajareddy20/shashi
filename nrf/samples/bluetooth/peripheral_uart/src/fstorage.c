#include "fstorage.h"
#include <zephyr/device.h>
#include "temp_sensor.h"

struct nvs_fs fs;
struct samples_info latest_samples_info;


int nrf_fstorage_init(void)
{
    int rc = 0;

    fs.flash_device = NVS_PARTITION_DEVICE;
    if (!device_is_ready(fs.flash_device)) 
    {
        return -1;
    }

    fs.sector_size = SECTOR_SIZE;
    fs.sector_count = SECTOR_COUNT;

    fs.offset = FSTORAGE_START_ADDRESS;

    rc = nvs_mount(&fs);
    if (rc) 
    {
        return -1;
    }

    return 0;
}

void nrf_fstorage_read(struct nvs_fs *fs, long int address, struct samples_info *latest_samples_info, uint32_t size)
{
    flash_read(fs->flash_device, address, (uint8_t *)latest_samples_info, size);
}

void nrf_fstorage_write(struct nvs_fs *fs, long int address, struct samples_info *latest_samples_info, uint32_t size)
{
    flash_write(fs->flash_device, address, (uint8_t *)latest_samples_info, size);
}

void nrf_fstorage_erase(struct nvs_fs *fs, long int address, uint32_t size)
{
    flash_erase(fs->flash_device, address, (size * SECTOR_SIZE));
}

void write_samples_info_to_flash(void)
{
   uint32_t next_address = 0, next_page_address = 0;

   next_address = SAMPLES_INFO_START_ADDRESS + (0x20 * (current_sample_info_position - 1));
   
   next_page_address = SAMPLES_INFO_START_ADDRESS + 0x1000;
   
   if(next_address == next_page_address)
   {
      current_sample_info_position = 1;
      next_address = SAMPLES_INFO_START_ADDRESS;

      nrf_fstorage_erase(&fs, next_address, 1);
   }

   latest_samples_info.dirty_flag = 0xAA;
   nrf_fstorage_write(&fs, next_address, &latest_samples_info, sizeof(latest_samples_info));
   current_sample_info_position++;
}


void read_samples_info_from_flash(void)
{
   int index = 0, current_page_address = 0, next_address = 0;

   current_page_address = SAMPLES_INFO_START_ADDRESS;

   for(index = 0; index < NUMBER_OF_SAMPLES_INFO_IN_A_PAGE; index++)
   {
      next_address = current_page_address + (0x20 * index);

      nrf_fstorage_read(&fs, next_address, &latest_samples_info, sizeof(latest_samples_info));

      if(latest_samples_info.dirty_flag != 0xAA)
      {
         if(index == 0)
         {
            index = 1;
            latest_samples_info.dirty_flag = 0xAA;
            latest_samples_info.total_number_of_samples_stored = 0;
            latest_samples_info.total_number_of_samples_transmitted = 0;
            latest_samples_info.reset_counter = 0;
            current_sample_info_position = 1;

            write_samples_info_to_flash();
         }
         else
         {
            next_address = current_page_address + (0x20 * (index - 1));
            nrf_fstorage_read(&fs, next_address, &latest_samples_info, sizeof(latest_samples_info));
            current_sample_info_position = index;
         }
         break;
      }
   }
}

void read_corresponding_data_from_flash(void)
{
   uint32_t data[4] = {0};
   uint32_t read_address = 0;
   read_counter = latest_samples_info.total_number_of_samples_transmitted % MAX_NUMBER_OF_SAMPLES;
   read_address = (SAMPLES_START_ADDRESS + (16 * read_counter));

   flash_read(fs.flash_device, read_address, data, 16);
   memcpy(read_sample_data, data, 16);
}


void fstorage_write(struct nvs_fs *fs, long int address, uint8_t *data, int size)
{
   flash_write(fs->flash_device, address, data, 16);
}