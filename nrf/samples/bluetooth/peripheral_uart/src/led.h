#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <dk_buttons_and_leds.h>
#include <hal/nrf_gpio.h>
#include <zephyr/drivers/gpio.h>


#define GREEN_LED 20
#define BLUE_LED 17
#define RED_LED 15

#define SLEEP_TIME_MS 1000

void leds_init(void);
void led_blink(uint8_t pin, uint8_t counter);
void turn_on_led(uint8_t pin);
void turn_off_led(uint8_t pin);