#include <zephyr/sys/reboot.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/nvs.h>



#define NVS_PARTITION		storage_partition
#define NVS_PARTITION_DEVICE	FIXED_PARTITION_DEVICE(NVS_PARTITION)
#define NVS_PARTITION_OFFSET	FIXED_PARTITION_OFFSET(NVS_PARTITION)

#define SECTOR_SIZE     4096
#define SECTOR_COUNT    40


#define FSTORAGE_START_ADDRESS          0x42000 
#define FSTORAGE_END_ADDRESS            0x78000
#define SAMPLES_START_ADDRESS           0x50000
#define SAMPLE_SIZE                     16
#define MAX_NUMBER_OF_SAMPLES           9728
#define NUMBER_OF_SAMPLES_INFO_IN_A_PAGE 128
#define SAMPLES_INFO_START_ADDRESS      FSTORAGE_START_ADDRESS


extern struct nvs_fs fs;

struct samples_info
{
   int total_number_of_samples_stored;
   int total_number_of_samples_transmitted;
   int lifetime_message_counter;
   uint8_t time_set;
   int sensing_interval_in_seconds;
   uint8_t reset_counter;
   uint8_t dirty_flag;
};

extern struct samples_info latest_samples_info;

extern uint16_t current_sample_info_position;
extern uint32_t read_counter;
extern uint8_t read_sample_data[100];



int nrf_fstorage_init(void);

void nrf_fstorage_erase(struct nvs_fs *fs, long int address, uint32_t size);
void nrf_fstorage_write(struct nvs_fs *fs, long int address, struct samples_info *latest_samples_info, uint32_t size);
void nrf_fstorage_read(struct nvs_fs *fs, long int address, struct samples_info *latest_samples_info, uint32_t size);

void write_samples_info_to_flash(void);
void read_samples_info_from_flash(void);
void read_corresponding_data_from_flash(void);
void fstorage_write(struct nvs_fs *fs, long int address, uint8_t *data, int size);