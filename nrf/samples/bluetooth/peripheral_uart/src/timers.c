#include "timers.h"
#include <zephyr/kernel.h>
#include "fstorage.h"


struct k_timer my_timer;
uint8_t start_adv_slots = 0;
uint8_t sleep_count = 0;
uint8_t start_sleep_time_ = 0;
extern uint32_t global_time;
uint32_t local_timer;
extern uint32_t temp_sensing_timer_counter;
extern uint8_t got_time_from_scanner;
extern uint8_t time_to_advertise;
uint16_t global_transmission_window;
extern uint8_t first_sample;
extern uint8_t time_to_read_temp_sensor;
extern uint8_t time_slot;

volatile  uint8_t sensor_number = 0xFF; 

void timer_handler(struct k_work *work)
{
   if(!time_update_received)
   {
      global_time++;
   }
   if(start_sleep_time_){
      sleep_count++;
   }
   if(start_adv_slots){
      local_timer++;
   }

   if((got_time_from_scanner == 0) )
   {
      if((global_time % 15) == 0)
      {
        time_to_advertise = 1;
      }
   }
   else
   {
      if((global_time % TRANSMIT_WINDOW_THRESHOLD) == 0)
      {
         global_transmission_window = 1;
         if(first_sample)
         {
            first_sample = 0;
         }
      } 

      if(((global_time % TEMP_SENSING_INTERVAL_SECONDS)== 0) && (global_transmission_window)  && (data_logging))
      {
         time_to_read_temp_sensor = 1;
         start_sleep_time_ = 1;
         sleep_count = 0;
         start_adv_slots = 0;
      } 
      
      if(sleep_count == 55){
         sleep_count = 0;
         start_sleep_time_ = 0;
         local_timer = 0;
         start_adv_slots = 1;
      }

      if(global_transmission_window && start_adv_slots)
      {         
        sensor_number = ((local_timer / TIME_SLOT_FOR_EACH_SENSOR) % MAX_SENSORS);
        if((sensor_number == time_slot) && ((global_time % TIME_SLOT_FOR_EACH_SENSOR) == 0) && (data_logging))
        {
            time_to_advertise = 1;
        }
      }
   }
}

K_WORK_DEFINE(my_work, timer_handler);

void my_timer_handler(struct k_timer *dummy)
{
    k_work_submit(&my_work);
}


void timers_init(void)
{
    k_timer_init(&my_timer, my_timer_handler, NULL);
}

void start_timer(void)
{
    k_timer_start(&my_timer,  K_MSEC(0),  K_MSEC(TIMER_INTERRUPT_DURATION));
}