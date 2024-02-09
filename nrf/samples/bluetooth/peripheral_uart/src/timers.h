#include "led.h"

#define TIMER_INTERRUPT_DURATION    1000

extern struct k_timer my_timer;
extern uint8_t time_update_received;
extern int send_instant_packet;
extern bool data_logging;


#define TRANSMIT_WINDOW_THRESHOLD       600
#define TEMP_SENSING_INTERVAL_SECONDS   600
#define TIME_SLOT_FOR_EACH_SENSOR       5
#define MAX_SENSORS                     35

void my_work_handler(struct k_work *work);
void my_timer_handler(struct k_timer *dummy);
void timers_init(void);
void start_timer(void);