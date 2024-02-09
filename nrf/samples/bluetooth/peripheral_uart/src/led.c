#include "led.h"
#include <zephyr/kernel.h>

void leds_init(void)
{
    nrf_gpio_cfg_output(BLUE_LED);
	nrf_gpio_cfg_output(GREEN_LED);
	nrf_gpio_cfg_output(RED_LED);
}


void led_blink(uint8_t pin, uint8_t counter)
{
	uint8_t blink = 0;

	while(blink < counter)
	{
		nrf_gpio_pin_set(pin);
		k_msleep(500);
		nrf_gpio_pin_clear(pin);
		k_msleep(500);
		blink++;
	}
}

void turn_on_led(uint8_t pin)
{
    nrf_gpio_pin_set(pin);
}

void turn_off_led(uint8_t pin)
{
    nrf_gpio_pin_clear(pin);
}