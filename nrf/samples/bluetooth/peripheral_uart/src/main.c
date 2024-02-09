/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
/** @file
 *  Author : THEJA
 */

#include "uart_async_adapter.h"
#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>
#include <soc.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <bluetooth/services/nus.h>
#include <zephyr/settings/settings.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

#include "temp_sensor.h"
#include "fstorage.h"
#include "led.h"
#include "timers.h"

#define LOG_MODULE_NAME peripheral_uart
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define STACKSIZE CONFIG_BT_NUS_THREAD_STACK_SIZE
#define PRIORITY 7

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN	(sizeof(DEVICE_NAME) - 1)

#define RUN_STATUS_LED DK_LED1
#define RUN_LED_BLINK_INTERVAL 1000

#define CON_STATUS_LED DK_LED2

#define KEY_PASSKEY_ACCEPT DK_BTN1_MSK
#define KEY_PASSKEY_REJECT DK_BTN2_MSK

#define UART_BUF_SIZE CONFIG_BT_NUS_UART_BUFFER_SIZE
#define UART_WAIT_FOR_BUF_DELAY K_MSEC(50)
#define UART_WAIT_FOR_RX CONFIG_BT_NUS_UART_RX_WAIT_TIME

#define DEVICE_ID                       "THI1A0000"


#define   SWITCH_PIN                   5
#define BUTTON0_NODE	DT_NODELABEL(button0)

#define NUMBER_OF_MESSAGES_PER_BATTERY_LIFE    225000

#define BLE_HCI_STATUS_CODE_INVALID_BTLE_COMMAND_PARAMETERS 0x12  /**< Invalid BLE Command Parameters. */
#define BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION           0x13  /**< Remote User Terminated Connection. */
#define BLE_HCI_REMOTE_DEV_TERMINATION_DUE_TO_LOW_RESOURCES 0x14  /**< Remote Device Terminated Connection due to low resources.*/
#define BLE_HCI_REMOTE_DEV_TERMINATION_DUE_TO_POWER_OFF     0x15  /**< Remote Device Terminated Connection due to power off. */
#define BLE_HCI_LOCAL_HOST_TERMINATED_CONNECTION            0x16  /**< Local Host Terminated Connection. */

#define  JAN_FIRST_2022       1640995200
#define  JAN_FIRST_2040       2208988800

#define VERSION_NO			5
#define UART_ENABLE       	1

#if UART_ENABLE 
	uint8_t time_bedubg[50] = {'\0'};  
	uint8_t time_for_sense[50] = {'\0'};
#endif

static K_SEM_DEFINE(ble_init_ok, 0, 1);

static struct bt_conn *current_conn;
static struct bt_conn *auth_conn;

uint8_t data_to_be_transmitted[50] = {'\0'};
uint8_t data_to_send[1200] = {'\0'};
uint8_t connected_to_gateway = 0;
uint8_t time_to_read_temp_sensor = 0;
uint8_t time_to_advertise = 0;
uint8_t data_to_be_sent[100] = {0};
uint8_t index = 0, time_slot = 0;

static K_FIFO_DEFINE(fifo_uart_tx_data);
static K_FIFO_DEFINE(fifo_uart_rx_data);

uint16_t current_sample_info_position = 0;
int maximum_samples_storage_possible = 0;

bool data_logging = true;

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(BUTTON0_NODE, gpios);
static struct gpio_callback button_cb_data;
volatile uint8_t user_switch_detected = 0;

static uint8_t char_2_data[320];
uint8_t battery_percentage = 100;
uint32_t global_time = 1234560000;
uint32_t temp_sensing_timer_counter = 0;
uint8_t temp_hum_data[20];
static uint32_t store_data[16] = {0};
uint32_t write_counter = 0;
uint32_t offset_for_number_of_samples_transmitted = 0;
uint32_t read_counter = 0;
uint8_t advertising_in_progress = 0;
uint8_t time_request_sent = 0;
uint8_t read_sample_data[100];
volatile uint8_t ack_received_for_last_packet = 0;
uint32_t maximum_duration_to_wait_for_ack = 0;
uint8_t time_received_from_scanner = 0;
uint8_t got_time_from_scanner = 0;
int sensing_interval_in_seconds = 0;
uint8_t config_update_received = 0;
uint8_t first_sample = 1;
uint8_t time_update_received = 0;
uint8_t temp_arr[10] = {'\0'};
uint8_t  time_slot_index = 0;
volatile uint8_t data_from_scanner = 0;
volatile uint8_t scanner_data[40] ={'\0'};

uint32_t last_packet_creation_time = 0;

int temp_time = 0;

#if UART_ENABLE
static uint8_t rx_buf[10] = {0};
const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart0));

const struct uart_config uart0_cfg = 
{
	.baudrate = 115200,
	.parity = UART_CFG_PARITY_NONE,
	.stop_bits = UART_CFG_STOP_BITS_1,
	.data_bits = UART_CFG_DATA_BITS_8,
	.flow_ctrl = UART_CFG_FLOW_CTRL_NONE
};
#endif
//to here

struct uart_data_t 
{
	void *fifo_reserved;
	uint8_t data[UART_BUF_SIZE];
	uint16_t len;
};


static struct bt_conn_le_phy_param phy = {
    .options = BT_CONN_LE_PHY_OPT_CODED_S8,
    .pref_rx_phy = BT_GAP_LE_PHY_CODED,
    .pref_tx_phy = BT_GAP_LE_PHY_CODED
};

static const char *phy2str(uint8_t phy)
{
	switch (phy) {
	case 0: return "No packets";
	case BT_GAP_LE_PHY_1M: return "LE 1M";
	case BT_GAP_LE_PHY_2M: return "LE 2M";
	case BT_GAP_LE_PHY_CODED: return "LE Coded";
	default: return "Unknown";
	}
}

static const struct bt_data ad[] = 
{
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = 
{
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};



static void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	user_switch_detected = 1;
}

void configure_interrupt_for_user_switch()
{
    gpio_pin_configure_dt(&button, GPIO_INPUT);
	gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_FALLING);
	gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	int8_t debug[50] = {'\0'};
	char addr[BT_ADDR_LE_STR_LEN];

	if (err) {
		LOG_ERR("Connection failed (err %u)", err);
		return;
	}
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Connected %s", addr);
	current_conn = bt_conn_ref(conn);
	turn_on_led(BLUE_LED);

	err = bt_conn_le_phy_update(conn, &phy);
	if (err) {
		#if UART_ENABLE
			sprintf(debug,"bt_conn_le_phy_update() returned %d\n", err);
			send_through_uart(debug);
		#endif
	}

	connected_to_gateway = 1;
	advertising_stop();
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Disconnected: %s (reason %u)", addr, reason);

	if (auth_conn) {
		bt_conn_unref(auth_conn);
		auth_conn = NULL;
	}
	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}
	turn_off_led(BLUE_LED);
	connected_to_gateway = 0;
}

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
static void security_changed(struct bt_conn *conn, bt_security_t level,
			     enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!err) {
		LOG_INF("Security changed: %s level %u", addr, level);
	} else {
		LOG_WRN("Security failed: %s level %u err %d", addr,
			level, err);
	}
}
#endif

static void phy_updated(struct bt_conn *conn, struct bt_conn_le_phy_info *param)
{	
	#if UART_ENABLE
		int8_t debug[60] = {'\0'};
		sprintf(debug,"PHY updated. RX PHY: %s, TX PHY: %s\n", phy2str(param->tx_phy), phy2str(param->rx_phy));
		send_through_uart(debug);
	#endif
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected    = connected,
	.disconnected = disconnected,
#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
	.security_changed = security_changed,
#endif
#if CONFIG_BT_USER_PHY_UPDATE
	.le_phy_updated = phy_updated,
#endif
};

#if defined(CONFIG_BT_NUS_SECURITY_ENABLED)
static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Passkey for %s: %06u", addr, passkey);
}

static void auth_passkey_confirm(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	auth_conn = bt_conn_ref(conn);

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Passkey for %s: %06u", addr, passkey);
	LOG_INF("Press Button 1 to confirm, Button 2 to reject.");
}


static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing cancelled: %s", addr);
}


static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing completed: %s, bonded: %d", addr, bonded);
}


static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing failed conn: %s, reason %d", addr, reason);
}


static struct bt_conn_auth_cb conn_auth_callbacks = {
	.passkey_display = auth_passkey_display,
	.passkey_confirm = auth_passkey_confirm,
	.cancel = auth_cancel,
};

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed
};
#else
static struct bt_conn_auth_cb conn_auth_callbacks;
static struct bt_conn_auth_info_cb conn_auth_info_callbacks;
#endif


static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data, uint16_t len)
{
	int err;
	char addr[BT_ADDR_LE_STR_LEN] = {0};

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, ARRAY_SIZE(addr));
	memset(scanner_data, 0, sizeof(scanner_data));
	memcpy(scanner_data, data, len);        
	data_from_scanner = 1;
	
}

static struct bt_nus_cb nus_cb = 
{
	.received = bt_receive_cb,
};

void error(void)
{
	dk_set_leds_state(DK_ALL_LEDS_MSK, DK_NO_LEDS_MSK);

	while (true) {
		/* Spin for ever */
		k_sleep(K_MSEC(1000));
	}
}

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
static void num_comp_reply(bool accept)
{
	if (accept) {
		bt_conn_auth_passkey_confirm(auth_conn);
		LOG_INF("Numeric Match, conn %p", (void *)auth_conn);
	} else {
		bt_conn_auth_cancel(auth_conn);
		LOG_INF("Numeric Reject, conn %p", (void *)auth_conn);
	}

	bt_conn_unref(auth_conn);
	auth_conn = NULL;
}

void button_changed(uint32_t button_state, uint32_t has_changed)
{
	uint32_t buttons = button_state & has_changed;

	if (auth_conn) {
		if (buttons & KEY_PASSKEY_ACCEPT) {
			num_comp_reply(true);
		}

		if (buttons & KEY_PASSKEY_REJECT) {
			num_comp_reply(false);
		}
	}
}
#endif 


void bluetooth_init(void)
{
	int err = 0;

	if (IS_ENABLED(CONFIG_BT_NUS_SECURITY_ENABLED)) 
	{
		err = bt_conn_auth_cb_register(&conn_auth_callbacks);
		if (err) 
		{
			printk("Failed to register authorization callbacks.\n");
			return;
		}

		err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
		if (err) 
		{
			printk("Failed to register authorization info callbacks.\n");
			return;
		}
	}

	err = bt_enable(NULL);
	if (err) 
	{
		error();
	}

	LOG_INF("Bluetooth initialized");

	k_sem_give(&ble_init_ok);

	if (IS_ENABLED(CONFIG_SETTINGS)) 
	{
		settings_load();
	}

	err = bt_nus_init(&nus_cb);
	if (err) 
	{
		LOG_ERR("Failed to initialize UART service (err: %d)", err);
		return;
	}
}

void wait_for_connection()
{
	uint8_t counter_for_connection = 0;
	while(!connected_to_gateway)
	{
		counter_for_connection++;
		k_msleep(20);
		if(counter_for_connection == 100)
		{
			break;
		}
	}
}

static void advertising_start(void)
{
    advertising_in_progress = 1;

	bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
}

static void advertising_stop_when_not_connected(void)
{
   if (advertising_in_progress)
   {
		advertising_in_progress = 0;
		bt_le_adv_stop();
   }
}

int update_char_2(void)
{
	int err_code;
	#if UART_ENABLE
		uint8_t buff[30];
	#endif
	memset(&data_to_send, '\0', sizeof(data_to_send));
	data_to_send[0] = 0xC1;
	data_to_send[1025] = 0xa1;
	data_to_send[1026] = 0xa1;
	data_to_send[1027] = 0xa1;
	data_to_send[1028] = 0xa1;
	data_to_send[1029] = 0xa1; 
	data_to_send[1030] = VERSION_NO;

	memcpy(&data_to_send[1], DEVICE_ID, 10);

	memcpy(&data_to_send[11], read_sample_data, 8);
	data_to_send[19] = read_sample_data[11];
	memcpy(&data_to_send[20], &read_sample_data[8], 3);
	
	uint8_t packet_counter = 0;
   
	for( packet_counter = 0; packet_counter < 26; packet_counter++)
	{
		memcpy(data_to_be_transmitted, &data_to_send[packet_counter * 40], 40);
		err_code = bt_nus_send(NULL, data_to_be_transmitted, 40);
		k_msleep(5);
		if(err_code != 0)
		{
			#if UART_ENABLE
				sprintf(buff, "failed=%d,counter=%d\r\n", err_code, packet_counter);
				send_through_uart(buff);
			#endif
			break;
		}
	}

	latest_samples_info.total_number_of_samples_transmitted++;
	#if UART_ENABLE
		if(packet_counter == 26)
		{
			send_through_uart("data sent succesfully\r\n");
		}
	#endif
	return err_code;
}


void get_time_from_scanner(void)
{
	read_sample_data[0] = 0xC2;
	read_sample_data[1] = 0xC2; 
	read_sample_data[2] = 0xC2;

	int res = bt_nus_send(NULL, read_sample_data, 16);
	if(res != 0){
		#if UART_ENABLE
			#if UART_ENABLE
			// TODO: need to remove
			uint8_t buff[50] = {'\0'};
			sprintf(buff,"failed to send time req=%d\r\n", res);
			send_through_uart(buff);
			#endif
		#endif
	}
}


void transmit_the_pending_samples(void)
{
	#if UART_ENABLE
		uint8_t debuggeer[100] = {'\0'};
	#endif
   while(latest_samples_info.total_number_of_samples_transmitted < latest_samples_info.total_number_of_samples_stored)
   {
		read_corresponding_data_from_flash();
		ack_received_for_last_packet = 0;
		update_char_2();
		wait_for_ack_from_scanner();

		if(!ack_received_for_last_packet)
		{
			advertising_stop();
			bt_conn_disconnect(current_conn, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
			fem_disable();
			connected_to_gateway = 0;
			#if UART_ENABLE
				send_through_uart("ACK not received\r\n");
			#endif
			if(latest_samples_info.total_number_of_samples_transmitted > 0)
			{
				latest_samples_info.total_number_of_samples_transmitted--;
			}
			break;
		}
		#if UART_ENABLE
			send_through_uart("ACK Last packet\r\n");
		#endif
    }    

   if(latest_samples_info.total_number_of_samples_transmitted == latest_samples_info.total_number_of_samples_stored)
   {
		
		advertising_stop();
		bt_conn_disconnect(current_conn, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
		fem_disable();
		connected_to_gateway = 0;
		#if UART_ENABLE
			sprintf(debuggeer, "Stored=%d, trans=%d\r\n", latest_samples_info.total_number_of_samples_stored,latest_samples_info.total_number_of_samples_transmitted);
			send_through_uart(debuggeer);
		#endif
   }
}

void calculate_the_time_slot(void)
{
	#if UART_ENABLE
		uint8_t data_[20] = {'\0'};
	#endif
    uint8_t device_id[10] = {0};
    memcpy(device_id, DEVICE_ID, 10);  
    time_slot = (((device_id[6] - 0x30) * 100) + ((device_id[7] - 0x30) * 10) + (device_id[8] - 0x30)) % MAX_SENSORS;

	// if(time_slot == 0){
	// 	time_slot = MAX_SENSORS - 1;
	// }else{
	// --time_slot;
	// }
	#if UART_ENABLE
		sprintf(data_, "time_slot = %d\r\n", time_slot);
		send_through_uart(data_);
	#endif
}


void get_battery_percentage(void)
{
  battery_percentage = 100 - ((latest_samples_info.lifetime_message_counter * 100) / NUMBER_OF_MESSAGES_PER_BATTERY_LIFE);
}


void store_the_sample_sensor(uint32_t *data)
{
   int next_address = 0, sub_counter = 0, iterations = 0;

   write_counter = latest_samples_info.total_number_of_samples_stored % MAX_NUMBER_OF_SAMPLES;
   
   if(write_counter == 0)
   {
       write_counter = MAX_NUMBER_OF_SAMPLES;
   }

   next_address = SAMPLES_START_ADDRESS + ((write_counter - 1) * 16);


   if(write_counter % 256 == 1)
   {
	  nrf_fstorage_erase(&fs, next_address, 1);

      if(latest_samples_info.total_number_of_samples_stored - latest_samples_info.total_number_of_samples_transmitted > MAX_NUMBER_OF_SAMPLES)
      {
          offset_for_number_of_samples_transmitted = (((next_address - SAMPLES_START_ADDRESS) / 0x1000) * 256 ) + 256;
          iterations = latest_samples_info.total_number_of_samples_transmitted / MAX_NUMBER_OF_SAMPLES;
          if(offset_for_number_of_samples_transmitted > read_counter)
          {
             latest_samples_info.total_number_of_samples_transmitted = (iterations * MAX_NUMBER_OF_SAMPLES) + offset_for_number_of_samples_transmitted;
          }
      }
   }
   fstorage_write(&fs, next_address, data, 16);
	
}

void create_packet(void)
{  
	if((latest_samples_info.time_set))
	{
		latest_samples_info.total_number_of_samples_stored++;
		latest_samples_info.lifetime_message_counter++;
		
		get_battery_percentage();

		char_2_data[0] = (latest_samples_info.lifetime_message_counter & 0xFF000000) >> 24;
		char_2_data[1] = (latest_samples_info.lifetime_message_counter & 0x00FF0000) >> 16;
		char_2_data[2] = (latest_samples_info.lifetime_message_counter & 0x0000FF00) >> 8;
		char_2_data[3] = (latest_samples_info.lifetime_message_counter & 0x000000FF);

		char_2_data[4] = (global_time & 0xFF000000) >> 24;
		char_2_data[5] = (global_time & 0x00FF0000) >> 16;
		char_2_data[6] = (global_time & 0x0000FF00) >> 8;
		char_2_data[7] = (global_time & 0x000000FF);

		char_2_data[8] = temp_hum_data[0];
		char_2_data[9] = temp_hum_data[1];
		char_2_data[10] = temp_hum_data[2];
		char_2_data[11] = battery_percentage;
		char_2_data[12] = TH_SENSOR_TYPE;
		char_2_data[13] = 0xC1;

		memcpy(store_data, char_2_data, 16);
		store_the_sample_sensor(store_data);
	}
}

void process_scanner_data()
{
	if(scanner_data[0] == 0xAA)
	{
		global_time = 0;
		global_time |= scanner_data[1] << 24;
		global_time |= scanner_data[2] << 16;
		global_time |= scanner_data[3] << 8;
		global_time |= scanner_data[4];

		if((global_time > JAN_FIRST_2022) && (global_time < JAN_FIRST_2040))
		{
			time_received_from_scanner = 1;   
		} 	    
	}
	else if(scanner_data[0] == 0xAB)
	{
		temp_time = 0;
		temp_time |= scanner_data[1] << 24;
		temp_time |= scanner_data[2] << 16;
		temp_time |= scanner_data[3] << 8;
		temp_time |= scanner_data[4];


		 if((temp_time > JAN_FIRST_2022) && (temp_time < JAN_FIRST_2040))
         {
            time_update_received = 1;
         }


		ack_received_for_last_packet = 	1;
	}
}
void advertising_stop()
{
	advertising_in_progress = 0;
	bt_le_adv_stop();
}

static void reset_battery_percentage_to_full(void)
{
   latest_samples_info.lifetime_message_counter = 0;
   write_samples_info_to_flash();
   k_msleep(1000);
}

void wait_for_time_from_gateway()
{
	while(1)
  	{
		if(user_switch_detected)
		{
			k_msleep(5000);
			
			if(!nrf_gpio_pin_read(SWITCH_PIN))
			{
				led_blink(RED_LED, 5);
				reset_battery_percentage_to_full();
			}
			user_switch_detected = 0;
		}
		if(time_to_advertise)
		{
			#if UART_ENABLE
				send_through_uart("advertising for time\r\n");
			#endif
			fem_enable();
			advertising_start();
			wait_for_connection();
			time_to_advertise = 0;
		}
		if(connected_to_gateway)
		{	
			#if UART_ENABLE
				send_through_uart("connected\r\n");
			#endif
			k_msleep(800);

			get_time_from_scanner();
			time_request_sent = 1;

			k_msleep(2000);
			advertising_stop();
			bt_conn_disconnect(current_conn, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
			fem_disable();
			k_msleep(100);
			if(data_from_scanner)
			{
				data_from_scanner = 0;
				process_scanner_data();
			}
			
			if(time_received_from_scanner)
			{
				#if UART_ENABLE
					send_through_uart("Got time\r\n");
				#endif
				time_received_from_scanner = 0;
				got_time_from_scanner = 1;
				latest_samples_info.time_set = 1;
				latest_samples_info.sensing_interval_in_seconds = sensing_interval_in_seconds;
				write_samples_info_to_flash();

				break;
			}
		}
		else
		{
			advertising_stop_when_not_connected();
			fem_disable();
		}
		
  	}
}




void wait_for_ack_from_scanner()
{
   maximum_duration_to_wait_for_ack = 0;

   while(!ack_received_for_last_packet)
   {
	    maximum_duration_to_wait_for_ack++;
		k_msleep(10);
		if(data_from_scanner)
		{
			data_from_scanner = 0;
			process_scanner_data();
		}
		if(maximum_duration_to_wait_for_ack == 200)
		{          
			break;
		}
   }
}


void fem_enable()
{
    nrf_gpio_cfg_output(10);
    nrf_gpio_cfg_output(2);
    nrf_gpio_cfg_output(4);
    nrf_gpio_cfg_output(20);

	nrf_gpio_pin_set(10);
	nrf_gpio_pin_clear(2);
	nrf_gpio_pin_set(4);
	nrf_gpio_pin_set(20);
}

void fem_disable()
{
	
	nrf_gpio_pin_dir_set(2, NRF_GPIO_PIN_DIR_INPUT);
	nrf_gpio_pin_dir_set(4, NRF_GPIO_PIN_DIR_INPUT);
	nrf_gpio_pin_dir_set(20, NRF_GPIO_PIN_DIR_INPUT);
	
	nrf_gpio_pin_clear(10);
}
#if UART_ENABLE
int uart_cfg()
{
	int err = uart_configure(uart, &uart0_cfg);

	if (err == -ENOSYS) 
	{
		return -ENOSYS;
	}
}

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
	switch (evt->type) 
	{	
		case UART_RX_RDY:
			
			break;
		
		case UART_RX_DISABLED:
		
			break;
			
		default:
			break;
	}
}

static int uart_init(void)
{
	if (!device_is_ready(uart)) 
	{
		printk("UART device not ready\r\n");
		return 1;
	}

	int err = uart_callback_set(uart, uart_cb, NULL);
	if (err) 
	{
		return err;
	}

	uart_rx_enable(uart ,rx_buf, sizeof(rx_buf), 150);
}

void send_through_uart(uint8_t *data)
{
	uart_tx(uart, data, strlen(data), SYS_FOREVER_MS);
	k_msleep(10);
}
#endif


void main(void)
{
	#if UART_ENABLE
		uint8_t debugger[80] = {'\0'};
		uart_cfg();
		uart_init();
		
		send_through_uart("sensor_started\r\n");
		sprintf(debugger, "ID = %s, date = %s, time = %s\r\n",DEVICE_ID, __DATE__, __TIME__);
		send_through_uart(debugger);
	#endif

	leds_init();
	timers_init();
	nrf_fstorage_init();
	bluetooth_init();
	k_msleep(500);

	configure_interrupt_for_user_switch();
	twi_init();

	turn_on_led(RED_LED);
	k_msleep(5000);
	turn_off_led(RED_LED);

	read_temperature_and_humidity();
	read_samples_info_from_flash();
	latest_samples_info.reset_counter++;
	write_samples_info_to_flash();

	start_timer();
	latest_samples_info.time_set = 0;
    maximum_samples_storage_possible = ((FSTORAGE_END_ADDRESS - SAMPLES_START_ADDRESS) / SAMPLE_SIZE);
   	latest_samples_info.sensing_interval_in_seconds = 60;

	fem_enable();
	k_msleep(100);
	fem_disable();

	calculate_the_time_slot();
    wait_for_time_from_gateway();
	time_received_from_scanner = 1;
	got_time_from_scanner = 1;
	data_logging = true;

	for (;;) 
	{
		if(user_switch_detected)
		{
			k_msleep(5000);

			if(!nrf_gpio_pin_read(SWITCH_PIN))
			{
				if(!data_logging)
				{
					#if UART_ENABLE
						send_through_uart("OFF->ON\r\n");
					#endif
					data_logging = true;
					turn_on_led(BLUE_LED);
					k_msleep(5000);
					turn_off_led(BLUE_LED);
				}
				else
				{
					#if UART_ENABLE
						send_through_uart("ON->OFF\r\n");
					#endif
					data_logging = false;
					led_blink(BLUE_LED, 5);

					nrf_fstorage_erase(&fs, SAMPLES_START_ADDRESS, 1);
					latest_samples_info.total_number_of_samples_stored = 0;
					latest_samples_info.total_number_of_samples_transmitted = 0;
					current_sample_info_position = 1;
					write_samples_info_to_flash();
				}
			}
			user_switch_detected = 0;
		}

		if(data_logging)
		{
			if(time_to_read_temp_sensor)
			{
				#if UART_ENABLE
					sprintf(time_for_sense, "Sensing temp and Hum =%d\r\n", global_time);
					send_through_uart(time_for_sense);
				#endif
				time_to_read_temp_sensor = 0;
				read_temperature_and_humidity();
				create_packet();
				write_samples_info_to_flash();
			}
			if(time_to_advertise)
			{
				if((!latest_samples_info.time_set) && (!advertising_in_progress))
				{
					#if UART_ENABLE
						send_through_uart("Advertisng for time\r\n");
					#endif
					time_request_sent = 0;
					fem_enable();
					advertising_start();
					wait_for_connection();
					time_to_advertise = 0;
				}
				if((!advertising_in_progress) && (latest_samples_info.total_number_of_samples_stored > latest_samples_info.total_number_of_samples_transmitted))
				{
					#if UART_ENABLE
						sprintf(time_bedubg, "Advertisng for packet = %d\r\n", global_time);
						send_through_uart(time_bedubg);
					#endif
					fem_enable();
					advertising_start();
					wait_for_connection();
					time_to_advertise = 0;
				}
				time_to_advertise = 0;
			}
			if((advertising_in_progress) && (!connected_to_gateway))
			{
				#if UART_ENABLE
					send_through_uart("failed to connect\r\n");
				#endif
				advertising_stop_when_not_connected();
				fem_disable();
			}

			if(connected_to_gateway)
			{
				#if UART_ENABLE
					send_through_uart("Connected\r\n");
				#endif
				k_msleep(800);
				
				if((latest_samples_info.time_set) && (latest_samples_info.total_number_of_samples_stored > 0))
				{
					transmit_the_pending_samples();
					write_samples_info_to_flash();
				}
				else
				{  
					if(!time_request_sent)
					{
						get_time_from_scanner();
						time_request_sent = 1;
						k_msleep(2000);

						if (data_from_scanner)
						{
							data_from_scanner = 0;
							process_scanner_data();
						}
						advertising_stop();
						bt_conn_disconnect(current_conn, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
						fem_disable();
						connected_to_gateway = 0;
					}
					else
					{
						advertising_stop();
						bt_conn_disconnect(current_conn, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
						fem_disable();
						connected_to_gateway = 0;
					}
				}
			}
			if(time_received_from_scanner)
			{
				time_received_from_scanner = 0;
				got_time_from_scanner = 1;
				latest_samples_info.time_set = 1;
				latest_samples_info.sensing_interval_in_seconds = sensing_interval_in_seconds;
				write_samples_info_to_flash();
			}

			if(time_update_received)
			{
				global_time = temp_time;
				time_update_received = 0;
			}
			k_msleep(100);			
		}
		k_msleep(100);
	}
}
