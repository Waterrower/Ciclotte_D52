/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Product variant, same switches as the AVR firmware (atmegamain.c):
 * BIKE alone reads the brake position from the analog resistance knob,
 * BIKE + CICLOTTE drives a brake servo from the GUI +/- buttons via PWM.
 */
#define NOHRD 1
#define BIKE 1
#define CICLOTTE 1

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#ifdef CICLOTTE
#include <zephyr/drivers/pwm.h>
#else
#include <zephyr/drivers/adc.h>
#endif
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/fs/nvs.h>
#include <zephyr/storage/flash_map.h>
#ifdef CONFIG_BOOTLOADER_MCUBOOT
#include <zephyr/dfu/mcuboot.h>
#endif

#include "ble_hr.h"
#include "ble_ftms.h"
#include "displayEnums.h"

/*
 * NCS Partition Manager remaps storage_partition to nvs_storage when CONFIG_NVS
 * is enabled. This app uses the board DTS storage_partition directly for NVS.
 */
#ifdef storage_partition
#undef storage_partition
#endif

#define DISPLAY_UNUSED __attribute__((unused))
#define DISPLAY_RX_MAX 12
#define DISPLAY_POLL_MS 5
#define UART_RX_RING_SIZE 512
#define DISPLAY_REARM_INTERVAL_MS 5000U
#define DISPLAY_IDLE_DIM_MS 60000U
#define DISPLAY_IDLE_OFF_MS 120000U
#define DISPLAY_WAKE_SPLASH_MS 3500U
#define DISPLAY_BRIGHTNESS_FULL 15U
#define DISPLAY_BRIGHTNESS_DIM 10U
#define DISPLAY_BRIGHTNESS_OFF 0U
#define DEV_MODE_UNLOCK_TAPS 8U
#define DEV_PAGE_FLASH_TEST_OFFSET 0x72000U
#define DEV_PAGE_FLASH_TEST_SIZE 4096U
#define UART_DEVICE_NODE DT_NODELABEL(uart0)
#define USER_NODE DT_PATH(zephyr_user)
#define STORAGE_NODE DT_NODELABEL(storage_partition)
#define ENABLE_SIMULATED_BIKE_TRAINING 0
#define USE_REAL_RESISTANCE_INPUT 1
#define USE_REAL_CADENCE_INPUT 1
#define SIMULATED_BIKE_NOTIFY_MS 1000
#define RESISTANCE_SAMPLE_MS 250
#define FLYWHEEL_TO_PEDAL_RATIO_X100 781U
#define RESISTANCE_CAL_NVS_ID 1
#define SESSION_RING_PREF_NVS_ID 2
#define UI_SETTINGS_NVS_ID 3
#define RESISTANCE_CAL_MAGIC 0x5243414cU
#define SESSION_RING_PREF_MAGIC 0x52494e47U
#define UI_SETTINGS_MAGIC 0x55495346U
#define SESSION_DEFAULT_TIME_SEC (30U * 60U)
#define SESSION_DEFAULT_DISTANCE_M 5000U
#define SESSION_MIN_TIME_SEC 60U
#define SESSION_MAX_TIME_SEC (10U * 60U * 60U)
#define SESSION_INACTIVITY_TIMEOUT_SEC (5U * 60U)
#define SESSION_TIME_THRESH_SEC (60U * 60U)
#define SESSION_SMALL_TIME_STEP_LOW_SEC 60U
#define SESSION_BIG_TIME_STEP_LOW_SEC (5U * 60U)
#define SESSION_SMALL_TIME_STEP_HIGH_SEC (10U * 60U)
#define SESSION_BIG_TIME_STEP_HIGH_SEC (30U * 60U)
#define SESSION_MIN_DISTANCE_M 100U
#define SESSION_MAX_DISTANCE_M 999000U
#define SESSION_DISTANCE_THRESH_M 10000U
#define SESSION_SMALL_DISTANCE_STEP_LOW_M 100U
#define SESSION_BIG_DISTANCE_STEP_LOW_M 1000U
#define SESSION_SMALL_DISTANCE_STEP_HIGH_M 1000U
#define SESSION_BIG_DISTANCE_STEP_HIGH_M 5000U
#define BT_SESSION_END_DISCONNECT_DELAY_MS 750U
#define APP_LANGUAGE_US 4U
#define METERS_PER_MILE 1609U
#define MAX_DISPLAY_PACE_SEC (30U * 60U)
#ifdef CICLOTTE
#define SERVO_PULSE_MIN_US 500U
#define SERVO_PULSE_MAX_US 2500U
#define SERVO_MAX_ANGLE_DEG 180U
#define SERVO_LEVEL_COUNT 10U
#endif

BUILD_ASSERT(IMG_BG_SS_2 == 0xcf, "IMG_BG_SS_2 must match display asset id 0xcf");
BUILD_ASSERT(IMG_TILE3_BLANK_LIGHT == 0x06, "IMG_TILE3_BLANK_LIGHT must be 0x06");
BUILD_ASSERT(IMG_TILE3_BLANK_DARK == 0x0b, "IMG_TILE3_BLANK_DARK must be 0x0b");
BUILD_ASSERT(BTN_TILE3 == 4, "BTN_TILE3 must be 4");
BUILD_ASSERT(BTN_ACTION_TILE3 == 20, "BTN_ACTION_TILE3 must be 20");

static const struct gpio_dt_spec led = {
	.port = DEVICE_DT_GET(DT_NODELABEL(gpio0)),
	.pin = 17,
	.dt_flags = GPIO_ACTIVE_HIGH,
};
static const struct gpio_dt_spec tick_input =
	GPIO_DT_SPEC_GET(USER_NODE, tick_gpios);
#ifdef CICLOTTE
static const struct pwm_dt_spec brake_servo =
	PWM_DT_SPEC_GET_BY_IDX(USER_NODE, 0);
#else
static const struct adc_dt_spec resistance_adc =
	ADC_DT_SPEC_GET_BY_IDX(USER_NODE, 0);
#endif

enum user_state {
	USER_NONE,
	USER_PRESS,
	USER_RELEASE,
	USER_SWIPE_UP,
	USER_SWIPE_DOWN,
	USER_SWIPE_LEFT,
	USER_SWIPE_RIGHT,
};

enum parse_state {
	PENDING,
	COLLECTING,
	FINISHED,
	ABORTED,
	STARTED,
};

struct display_message {
	uint8_t prefix;
	uint8_t payload[40];
	uint8_t suffix[4];
	uint8_t payload_length;
	uint8_t expected_length;
	bool complete;
};

struct parser_context {
	enum parse_state state;
	struct display_message current_message;
	uint8_t payload_index;
};

struct ring_buffer {
	uint16_t size;
	volatile uint16_t start;
	volatile uint16_t end;
	volatile uint8_t buffer[UART_RX_RING_SIZE];
};

enum display_power_state {
	DISPLAY_POWER_ACTIVE,
	DISPLAY_POWER_DIM,
	DISPLAY_POWER_OFF,
};

static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);
static bool app_darkmode;
static bool app_bt_enabled = true;
static uint8_t app_language = 1;
static enum user_state user_state;
static enum user_state last_user_state;
static uint8_t app_response_id;
static struct ring_buffer display_rx_buffer;
static struct parser_context display_parser;
static volatile bool uart_rx_overflow;
static volatile bool uart_irq_seen;
static uint32_t custom_time_sec = SESSION_DEFAULT_TIME_SEC;
static uint32_t custom_distance_m = SESSION_DEFAULT_DISTANCE_M;
static bool countdown_active;
static uint8_t countdown_mode;
static uint8_t countdown_value;
static uint32_t countdown_last_ms;
static bool bt_connection_session_active;
static bool last_ftms_connected;
static bool confirm_stop_was_paused;
static struct gpio_callback tick_input_cb;
static volatile uint32_t cadence_edge_count;
static volatile bool cadence_wake_pending;
static uint8_t measured_resistance_percent;
static uint8_t measured_cadence_rpm;
#ifdef CICLOTTE
/* Servo angle per brake level (shown as 1..10), taken from the AVR firmware. */
static const uint8_t servo_level_angle_deg[SERVO_LEVEL_COUNT] = {
	53, 67, 82, 97, 110, 115, 123, 130, 150, 180,
};
static uint8_t servo_level;
static bool servo_output_enabled;
#else
static uint16_t resistance_raw;
static uint16_t resistance_cal_raw_100 = 210;
static uint16_t resistance_cal_raw_0 = 980;
static bool resistance_calibrated;
#endif
static bool resistance_nvs_ready;
static enum display_power_state display_power_state = DISPLAY_POWER_ACTIVE;
static uint32_t display_last_activity_ms;
static uint8_t display_brightness = 0xffU;
static uint8_t wake_splash_index;
static int hr_last_pairing_error;

static void bt_disconnect_work_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(bt_disconnect_work, bt_disconnect_work_handler);

enum hr_search_mode {
	HR_SEARCH_OFF,
	HR_SEARCH_BLE,
	HR_SEARCH_COUNT,
};

enum app_page_id {
	PAGE_MAIN_MENU,
	PAGE_TIME_SELECTION,
	PAGE_DISTANCE_SELECTION,
	PAGE_SETTINGS_1,
	PAGE_SETTINGS_2,
	PAGE_SETUP,
	PAGE_HR_CONNECT,
	PAGE_CALIBRATION,
	PAGE_INFORMATION,
	PAGE_IMAGE_BROWSER,
	PAGE_SESSION,
	PAGE_BT_CONNECTION,
	PAGE_SUMMARY,
	PAGE_CONFIRM_STOP,
	PAGE_DEVELOPER,
};

enum session_mode {
	SESSION_NONE,
	SESSION_FREE,
	SESSION_TIME,
	SESSION_DISTANCE,
};

enum session_metric_id {
	SESSION_METRIC_WATTS,
	SESSION_METRIC_CADENCE,
	SESSION_METRIC_RESISTANCE,
	SESSION_METRIC_SPEED,
	SESSION_METRIC_PACE,
	SESSION_METRIC_TIME,
	SESSION_METRIC_DISTANCE,
	SESSION_METRIC_CALORIES,
	SESSION_METRIC_HEARTRATE,
	SESSION_METRIC_COUNT,
};

enum summary_metric_id {
	SUMMARY_METRIC_AVG_WATTS,
	SUMMARY_METRIC_AVG_HEARTRATE,
	SUMMARY_METRIC_MAX_HEARTRATE,
	SUMMARY_METRIC_AVG_SPEED,
	SUMMARY_METRIC_AVG_PACE,
	SUMMARY_METRIC_TIME,
	SUMMARY_METRIC_DISTANCE,
	SUMMARY_METRIC_CALORIES,
	SUMMARY_METRIC_HEARTRATE,
	SUMMARY_METRIC_COUNT,
};

struct menu_button {
	uint8_t button_id;
	uint8_t action_id;
	uint8_t image_id;
	uint8_t press_image_id;
	uint8_t text_field_id;
	const char *text;
	const char *const *localized_text;
	uint8_t top_layer;
};

struct menu_page {
	enum app_page_id page_id;
	const uint8_t *images;
	size_t image_count;
	const struct menu_button *buttons;
	size_t button_count;
};

struct session_ring {
	uint8_t idx;
	uint8_t value_field;
	uint8_t label_field;
	uint8_t first_image;
	uint8_t blank_image;
};

struct session_state {
	enum session_mode mode;
	bool active;
	bool paused;
	bool summary_active;
	uint32_t elapsed_sec;
	uint32_t distance_m;
	uint32_t watt_seconds;
	uint32_t heartrate_sum;
	uint32_t heartrate_samples;
	uint32_t inactive_sec;
	uint16_t watts;
	uint8_t resistance_percent;
	uint8_t cadence_rpm;
	uint16_t speed_centi_kmh;
	uint16_t calories;
	uint8_t heartrate_bpm;
	uint8_t max_heartrate_bpm;
};

struct resistance_cal_data {
	uint32_t magic;
	uint16_t raw_100;
	uint16_t raw_0;
};

struct session_ring_pref_data {
	uint32_t magic;
	uint8_t idx[3];
};

struct ui_settings_data {
	uint32_t magic;
	uint8_t language;
	uint8_t darkmode;
	uint8_t bt_enabled;
	uint8_t hr_search_mode;
};

static struct nvs_fs resistance_nvs;
static uint8_t image_browser_index;
#ifndef CICLOTTE
static struct menu_button calibration_button;
static uint8_t calibration_step;
static bool calibration_restore_darkmode;
#endif
static enum hr_search_mode hr_search_mode = HR_SEARCH_BLE;
static bool update_resistance_input(void);

static const uint8_t main_menu_images[] = {
	IMG_BG_GRID_LIGHT,
	IMG_TILE0_NOHRD_LOGO,
};

static const uint8_t settings1_images[] = {
	IMG_BG_GRID_LIGHT,
	IMG_TILE0_SETTINGS,
};

static const uint8_t time_selection_images[] = {
	IMG_BG_HALFGRID_LIGHT,
	IMG_TILE0_TIME,
	IMG_TILE2_BLANK_LIGHT,
};

static const uint8_t distance_selection_images[] = {
	IMG_BG_HALFGRID_LIGHT,
	IMG_TILE0_DISTANCE,
	IMG_TILE2_BLANK_LIGHT,
};

static const uint8_t settings2_images[] = {
	IMG_BG_GRID_LIGHT,
	IMG_TILE0_SETTINGS,
};

static const uint8_t setup_images[] = {
	IMG_BG_GRID_LIGHT,
	IMG_TILE0_SETUP_LOGO,
};

static const uint8_t hr_connect_images[] = {
	IMG_BG_LIGHT_HT40_NEUTRAL,
	IMG_TILE0_SETTINGS,
	IMG_TILE1_BLANK_LIGHT,
	IMG_TILE3_HR_ICON_LIGHT,
	IMG_HR_INDICATOR_ICON_OFF,
};

#ifndef CICLOTTE
static const uint8_t calibration_images[] = {
	IMG_BG_CALIBRATION,
	IMG_TILE0_NOHRD_LOGO,
};
#endif

static const uint8_t information_images[] = {
	IMG_BG_GRID_LIGHT,
	IMG_TILE0_INFO_ICON,
	IMG_TILE1_NOHRD_INFO,
	IMG_TILE3_QRCODE,
	IMG_HALFTILE41_HOME_ICON_LIGHT,
	IMG_HALFTILE40_VERSION_NUMBER,
	IMG_HR_INDICATOR_ICON_OFF,
};

static const uint8_t developer_images[] = {
	IMG_BG_GRID_LIGHT,
	IMG_TILE0_INFO_ICON,
	IMG_TILE1_NOHRD_INFO,
	IMG_TILE3_BLANK_LIGHT,
	IMG_HALFTILE41_HOME_ICON_LIGHT,
};

static const uint8_t image_browser_images[] = {
	IMG_BG_GRID_LIGHT,
};

static const uint8_t session_images[] = {
	IMG_BG_GRID_LIGHT,
	IMG_TILE1_RING_1_9,
	IMG_TILE2_RING_2_9,
	IMG_TILE3_RING_3_9,
};

static const uint8_t bt_connection_images[] = {
	IMG_BG_GRID_LIGHT,
	IMG_TILE0_NOHRD_LOGO,
	IMG_HR_INDICATOR_ICON_OFF,
};

static const uint8_t summary_images[] = {
	IMG_BG_GRID_LIGHT,
	IMG_TILE0_FINISH,
	IMG_TILE1_RING_1_9,
	IMG_TILE2_RING_2_9,
	IMG_TILE3_RING_3_9,
};

static const uint8_t confirm_stop_images[] = {
	IMG_BG_LIGHT_HT40_NEUTRAL,
	IMG_TILE0_NOHRD_LOGO,
};

static const char *const text_time[] = {"Time", "Zeit", "Temps", "Tiempo", "Time"};
static const char *const text_distance[] = {"Distance", "Distanz", "Distance", "Distancia", "Distance"};
static const char *const text_free[] = {"Free Ride", "Freie Fahrt", "Sortie", "Libre", "Free Ride"};
static const char *const text_settings[] = {"Settings", "Einstellung", "Param#tres", "Ajustes", "Settings"};
static const char *const text_info[] = {"Info", "Information", "Informations", "Informaci@n", "Info"};
static const char *const text_home[] = {"Home", "Hauptmen}", "principal", "principal", "Home"};
static const char *const text_language[] = {"English(km)", "Deutsch", "Francais", "Espanol", "English(mi)"};
static const char *const text_light[] = {"Light Mode", "Hell", "Mode clair", "Claro", "Light Mode"};
static const char *const text_dark[] = {"Dark Mode", "Dunkel", "Mode sombre", "Oscuro", "Dark Mode"};
static const char *const text_battery[] = {"Battery", "Akku", "Batterie", "Bateria", "Battery"};
static const char *const text_back[] = {"Go Back", "Zur}ck", "reour", "devolver", "Go Back"};
static const char *const text_hr[] = {"HR Connect", "Puls", "Cardio", "Pulso", "HR Connect"};
static const char *const text_bt_off[] = {"BT Off", "BT Aus", "BT off", "BT off", "BT Off"};
static const char *const text_bt_on[] = {"BT On", "BT An", "BT on", "BT on", "BT On"};
#ifndef CICLOTTE
static const char *const text_calibration[] = {"Calibration", "Kalibrierung", "Calibration", "Calibracion", "Calibration"};
static const char *const text_cal_reset[] = {"Cal Reset", "Kal. Reset", "Cal. reset", "Cal. reset", "Cal Reset"};
static const char *const text_reset_error[] = {"Reset Error", "Reset Fehler", "Erreur reset", "Error reset", "Reset Error"};
#endif
static const char *const text_start[] = {"Start", "Start", "Depart", "Inicio", "Start"};
static const char *const text_next[] = {"Next", "Weiter", "Suivant", "Siguiente", "Next"};
static const char *const text_ble[] = {"BLE", "BLE", "BLE", "BLE", "BLE"};
static const char *const text_info_line0[] = {"For more", "F}r mehr", "Pour plus", "para saber mas", "For more"};
static const char *const text_info_line1[] = {"Inform.", "Infos", "d'informations", "Informaci@n", "Inform."};
static const char *const text_info_line2[] = {"Scan", "Scannen", "scannez", "Escanear", "Scan"};
static const char *const text_info_line3[] = {"QR-Code", "Sie den", "le", "el c@digo", "QR-Code"};
static const char *const text_info_line4[] = {"below", "QR-Code", "code QR", "QR-Code", "below"};
#ifndef CICLOTTE
static const char *const text_step[] = {"Step", "Schritt", "$tape", "Paso", "Step"};
static const char *const text_cal_line0[] = {"Turn the", "Stellen", "R$glez la", "ajuste", "Turn the"};
static const char *const text_cal_line1[] = {"resistance", "Sie den", "r$sistance a", "la", "resistance"};
static const char *const text_cal_line2[] = {"knob to the", "Widerstand", "maximum", "resistencia", "knob to the"};
static const char *const text_cal_line3_max[] = {"right", "auf Max.", "usqu'a la", "al maximo", "right"};
static const char *const text_cal_line4_max[] = {"all the way", "bis Anschlag", "but$e", "para detener", "all the way"};
static const char *const text_cal_line3_min[] = {"left", "auf Min.", "Au minimum", "sobre Min.", "left"};
static const char *const text_cal_line4_min[] = {"all the way", "bis Anschlag", "jusqu'a la but$e", "para detener", "all the way"};
static const char *const text_cal_line0_end[] = {"Calibration", "Kalibration", "Calibrage", "Calibraci@n", "Calibration"};
static const char *const text_cal_line1_end[] = {"completed.", "fertig.", "fini", "listo", "completed."};
#endif
static const char *const text_minutes[] = {"Minutes", "Minuten", "Minutes", "Minutos", "Minutes"};
static const char *const text_meter[] DISPLAY_UNUSED = {"Meter", "Meter", "Metres", "Metros", "Miles"};
static const char *const text_watts[] = {"Watts", "Watt", "Watts", "Vatios", "Watts"};
static const char *const text_cadence[] = {"Cadence", "Kadenz", "Cadence", "Cadencia", "Cadence"};
static const char *const text_resistance[] = {"Resistance", "Widerstand", "Resistance", "Resistencia", "Resistance"};
static const char *const text_speed[] = {"Speed", "Tempo", "Vitesse", "Velocidad", "mph"};
static const char *const text_pace[] = {"Pace", "Pace", "Allure", "Ritmo", "Pace"};
static const char *const text_session_time[] = {"Time", "Zeit", "Temps", "Tiempo", "Time"};
static const char *const text_session_distance[] = {"Distance", "Distanz", "Distance", "Distancia", "Distance"};
static const char *const text_calories[] = {"Calories", "Kalorien", "Calories", "Calorias", "Calories"};
static const char *const text_heartrate[] = {"Heart Rate", "Puls", "Cardio", "Pulso", "Heart Rate"};
static const char *const text_summary[] = {"Summary", "Ergebnis", "R$sultat", "resultado", "Summary"};
static const char *const text_avg_watts[] = {"Avg Watts", "{ Watt", "{ Watts", "{ Watios", "Avg Watts"};
static const char *const text_max_heartrate[] = {"Max HR", "max HF", "max BPM", "max BPM", "Max HR"};
static const char *const text_avg_heartrate[] = {"Avg HR", "{ HF", "{ BPM", "{ BPM", "Avg HR"};
static const char *const text_avg_speed[] = {"Avg km/h", "{ km/h", "{ Km/h", "{ km/h", "Avg mph"};
static const char *const text_avg_pace[] = {"Avg Pace", "{ Pace", "{ Allure", "{ Ritmo", "Avg Pace"};
static const char *const text_yes[] = {"Yes", "Ja", "Oui", "Si", "Yes"};
static const char *const text_no[] = {"No", "Nein", "Non", "No", "No"};
static const char *const text_quit[] = {"Quit?", "Beenden?", "Quitter?", "Salir?", "Quit?"};
#ifndef CICLOTTE
static const char *const text_end[] = {"End", "Ende", "Fin", "Fin", "End"};
#endif
static const char *const text_bt[] = {"Bluetooth", "Bluetooth", "Bluetooth", "Bluetooth", "Bluetooth"};
static const char text_distance_mi[] = "Distance/mi";
static const char text_version_firmware[] = "FW 4.0.0";
static const char text_version_display_fw[] = "DFW 1.0";

static const char *const *const session_metric_labels[SESSION_METRIC_COUNT] = {
	text_watts,
	text_cadence,
	text_resistance,
	text_speed,
	text_pace,
	text_session_time,
	text_session_distance,
	text_calories,
	text_heartrate,
};

static const char *const *const summary_metric_labels[SUMMARY_METRIC_COUNT] = {
	text_avg_watts,
	text_avg_heartrate,
	text_max_heartrate,
	text_avg_speed,
	text_avg_pace,
	text_session_time,
	text_session_distance,
	text_calories,
	text_heartrate,
};

static char text_time_inc_big[10];
static char text_time_inc_small[10];
static char text_time_dec_small[10];
static char text_time_dec_big[10];
static char text_distance_inc_big[10];
static char text_distance_inc_small[10];
static char text_distance_dec_small[10];
static char text_distance_dec_big[10];

static const struct menu_button main_menu_buttons[] = {
	{
		.button_id = BTN_TILE1,
		.action_id = BTN_ACTION_GOTO_TIME_SELECTION,
		.image_id = IMG_TILE1_BLANK_LIGHT,
		.press_image_id = IMG_TILE1_FRAME_LIGHT,
		.text_field_id = SF_LABEL1_LIGHT,
		.localized_text = text_time,
		.top_layer = IMG_TILE1_TIME_ICON_LIGHT,

	},
	{
		.button_id = BTN_TILE2,
		.action_id = BTN_ACTION_GOTO_DISTANCE_SELECTION,
		.image_id = IMG_TILE2_BLANK_LIGHT,
		.press_image_id = IMG_TILE2_FRAME_LIGHT,
		.text_field_id = SF_LABEL2_LIGHT,
		.localized_text = text_distance,
		.top_layer = IMG_TILE2_DISTANCE_ICON_LIGHT,

	},
	{
		.button_id = BTN_TILE3,
		.action_id = BTN_ACTION_GOTO_FREE_SESSION,
		.image_id = IMG_TILE3_BLANK_LIGHT,
		.press_image_id = IMG_TILE3_FRAME_LIGHT,
		.text_field_id = SF_LABEL3_LIGHT,
		.localized_text = text_free,
		.top_layer = IMG_TILE3_FREE_ICON_LIGHT,

	},
	{
		.button_id = BTN_TILE4,
		.action_id = BTN_ACTION_GOTO_SETTINGS,
		.image_id = IMG_TILE4_BLANK_LIGHT,
		.press_image_id = IMG_TILE4_FRAME_LIGHT,
		.text_field_id = SF_LABEL4_LIGHT,
		.localized_text = text_settings,
		.top_layer = IMG_TILE4_SETTINGS_ICON_LIGHT,
		
	},
};

static const struct menu_button time_selection_buttons[] = {
	{
		.button_id = BTN_TILE1T,
		.action_id = BTN_ACTION_BIG_INCREASE_TIME,
		.image_id = IMG_HALFTILE10_BLANK_LIGHT,
		.press_image_id = IMG_HALFTILE10_FRAME_LIGHT,
		.text_field_id = SF_LABEL10_LIGHT,
		.text = text_time_inc_big,
	},
	{
		.button_id = BTN_TILE1B,
		.action_id = BTN_ACTION_INCREASE_TIME,
		.image_id = IMG_HALFTILE11_BLANK_LIGHT,
		.press_image_id = IMG_HALFTILE11_FRAME_LIGHT,
		.text_field_id = SF_LABEL11_LIGHT,
		.text = text_time_inc_small,
	},
	{
		.button_id = BTN_TILE3T,
		.action_id = BTN_ACTION_DECREASE_TIME,
		.image_id = IMG_HALFTILE30_BLANK_LIGHT,
		.press_image_id = IMG_HALFTILE30_FRAME_LIGHT,
		.text_field_id = SF_LABEL30_LIGHT,
		.text = text_time_dec_small,
	},
	{
		.button_id = BTN_TILE3B,
		.action_id = BTN_ACTION_BIG_DECREASE_TIME,
		.image_id = IMG_HALFTILE31_BLANK_LIGHT,
		.press_image_id = IMG_HALFTILE31_FRAME_LIGHT,
		.text_field_id = SF_LABEL31_LIGHT,
		.text = text_time_dec_big,
	},
	{
		.button_id = BTN_TILE4B,
		.action_id = BTN_ACTION_GOTO_MAIN_MENU,
		.image_id = IMG_HALFTILE41_BLANK_LIGHT,
		.press_image_id = IMG_HALFTILE41_FRAME_LIGHT,
		.top_layer = IMG_HALFTILE41_HOME_ICON_LIGHT,
	},
	{
		.button_id = BTN_TILE4T,
		.action_id = BTN_ACTION_START_TIME_SESSION,
		.press_image_id = IMG_HALFTILE40_FRAME_DARK,
		.text_field_id = SF_LABEL40_DARK,
		.localized_text = text_start,
	},
};

static const struct menu_button distance_selection_buttons[] = {
	{
		.button_id = BTN_TILE1T,
		.action_id = BTN_ACTION_BIG_INCREASE_DISTANCE,
		.image_id = IMG_HALFTILE10_BLANK_LIGHT,
		.press_image_id = IMG_HALFTILE10_FRAME_LIGHT,
		.text_field_id = SF_LABEL10_LIGHT,
		.text = text_distance_inc_big,
	},
	{
		.button_id = BTN_TILE1B,
		.action_id = BTN_ACTION_INCREASE_DISTANCE,
		.image_id = IMG_HALFTILE11_BLANK_LIGHT,
		.press_image_id = IMG_HALFTILE11_FRAME_LIGHT,
		.text_field_id = SF_LABEL11_LIGHT,
		.text = text_distance_inc_small,
	},
	{
		.button_id = BTN_TILE3T,
		.action_id = BTN_ACTION_DECREASE_DISTANCE,
		.image_id = IMG_HALFTILE30_BLANK_LIGHT,
		.press_image_id = IMG_HALFTILE30_FRAME_LIGHT,
		.text_field_id = SF_LABEL30_LIGHT,
		.text = text_distance_dec_small,
	},
	{
		.button_id = BTN_TILE3B,
		.action_id = BTN_ACTION_BIG_DECREASE_DISTANCE,
		.image_id = IMG_HALFTILE31_BLANK_LIGHT,
		.press_image_id = IMG_HALFTILE31_FRAME_LIGHT,
		.text_field_id = SF_LABEL31_LIGHT,
		.text = text_distance_dec_big,
	},
	{
		.button_id = BTN_TILE4B,
		.action_id = BTN_ACTION_GOTO_MAIN_MENU,
		.image_id = IMG_HALFTILE41_BLANK_LIGHT,
		.press_image_id = IMG_HALFTILE41_FRAME_LIGHT,
		.top_layer = IMG_HALFTILE41_HOME_ICON_LIGHT,
	},
	{
		.button_id = BTN_TILE4T,
		.action_id = BTN_ACTION_START_DISTANCE_SESSION,
		.press_image_id = IMG_HALFTILE40_FRAME_DARK,
		.text_field_id = SF_LABEL40_DARK,
		.localized_text = text_start,
	},
};

static const struct menu_button settings1_buttons[] = {
	{
		.button_id = BTN_TILE1,
		.action_id = BTN_ACTION_TOGGLE_LANGUAGE,
		.image_id = IMG_TILE1_BLANK_LIGHT,
		.press_image_id = IMG_TILE1_FRAME_LIGHT,
		.text_field_id = SF_LABEL1_LIGHT,
		.localized_text = text_language,
		.top_layer = IMG_TILE1_GERMAN_ICON,
	},
	{
		.button_id = BTN_TILE2,
		.action_id = BTN_ACTION_TOGGLE_LDMODE,
		.image_id = IMG_TILE2_BLANK_LIGHT,
		.press_image_id = IMG_TILE2_FRAME_LIGHT,
		.text_field_id = SF_LABEL2_LIGHT,
		.localized_text = text_light,
		.top_layer = IMG_TILE2_SUN_ICON_LIGHT,
	},
	{
		.button_id = BTN_TILE3,
		.action_id = BTN_ACTION_GOTO_HRCONNECT,
		.image_id = IMG_TILE3_BLANK_LIGHT,
		.press_image_id = IMG_TILE3_FRAME_LIGHT,
		.text_field_id = SF_LABEL3_LIGHT,
		.localized_text = text_hr,
		.top_layer = IMG_TILE3_HR_ICON_LIGHT,
	},
	{
		.button_id = BTN_TILE4,
		.action_id = BTN_ACTION_GOTO_SETTINGS2,
		.image_id = IMG_TILE4_BLANK_LIGHT,
		.press_image_id = IMG_TILE4_FRAME_LIGHT,
		.text_field_id = SF_LABEL4_LIGHT,
		.localized_text = text_next,
		.top_layer = IMG_TILE4_NEXT_ICON_LIGHT,
	},
};

static const struct menu_button settings2_buttons[] = {
	{
		.button_id = BTN_TILE0,
#ifdef CICLOTTE
		/* No resistance calibration to reset on the servo brake. */
		.action_id = BTN_ACTION_NONE,
#else
		.action_id = BTN_ACTION_TILE0,
#endif
		.text_field_id = SF_LABEL0_DARK,
		.localized_text = text_settings,
	},
	{
		.button_id = BTN_TILE1,
		.action_id = BTN_ACTION_GOTO_INFO,
		.image_id = IMG_TILE1_BLANK_LIGHT,
		.press_image_id = IMG_TILE1_FRAME_LIGHT,
		.text_field_id = SF_LABEL1_LIGHT,
		.localized_text = text_info,
		.top_layer = IMG_TILE1_INFO_LIGHT,
	},
#ifdef CICLOTTE
	{
		.button_id = BTN_TILE2,
		.action_id = BTN_ACTION_NONE,
		.image_id = IMG_TILE2_BLANK_LIGHT,
	},
#else
	{
		.button_id = BTN_TILE2,
		.action_id = BTN_ACTION_GOTO_CALIBRATE_UP,
		.image_id = IMG_TILE2_BLANK_LIGHT,
		.press_image_id = IMG_TILE2_FRAME_LIGHT,
		.text_field_id = SF_LABEL2_LIGHT,
		.localized_text = text_calibration,
		.top_layer = IMG_TILE2_CALIBRATE_UP_LIGHT,
	},
#endif
	{
		.button_id = BTN_TILE3,
		.action_id = BTN_ACTION_TOGGLE_BLUETOOTH,
		.image_id = IMG_TILE3_BLANK_LIGHT,
		.press_image_id = IMG_TILE3_FRAME_LIGHT,
		.text_field_id = SF_LABEL3_LIGHT,
		.localized_text = text_bt_off,
		.top_layer = IMG_TILE3_BT_ICON_OFF_LIGHT,
	},
	{
		.button_id = BTN_TILE4,
		.action_id = BTN_ACTION_GOTO_MAIN_MENU,
		.image_id = IMG_TILE4_BLANK_LIGHT,
		.press_image_id = IMG_TILE4_FRAME_LIGHT,
		.text_field_id = SF_LABEL4_LIGHT,
		.localized_text = text_home,
		.top_layer = IMG_TILE4_HOME_LIGHT,
	},
};

static const struct menu_button hr_connect_buttons[] = {
	{
		.button_id = BTN_TILE4T,
		.action_id = BTN_ACTION_REFRESH_HR,
		.image_id = IMG_HALFTILE40_BLANK_NEUTRAL,
		.press_image_id = IMG_HALFTILE40_FRAME_LIGHT,
		.top_layer = IMG_HALFTILE40_REFRESH_HR,
	},
	{
		.button_id = BTN_TILE3,
		.action_id = BTN_ACTION_TILE3,
		.image_id = IMG_TILE3_BLANK_LIGHT,
		.press_image_id = IMG_TILE3_FRAME_LIGHT,
		.text_field_id = SF_LABEL3_LIGHT,
		.localized_text = text_ble,
		.top_layer = IMG_TILE3_HR_ICON_LIGHT,
	},
	{
		.button_id = BTN_TILE4B,
		.action_id = BTN_ACTION_GOTO_SETTINGS,
		.image_id = IMG_HALFTILE41_BLANK_LIGHT,
		.press_image_id = IMG_HALFTILE41_FRAME_LIGHT,
		.top_layer = IMG_HALFTILE41_HOME_ICON_LIGHT,
	},
};

static const struct menu_button setup_buttons[] = {
	{
		.button_id = BTN_TILE1,
		.action_id = BTN_ACTION_NONE,
		.image_id = IMG_TILE1_BLANK_LIGHT,
		.press_image_id = IMG_TILE1_FRAME_LIGHT,
		.text_field_id = SF_LABEL1_LIGHT,
		.localized_text = text_battery,
		.top_layer = IMG_TILE1_BATTERY_ICON_LIGHT,
	},
	{
		.button_id = BTN_TILE2,
		.action_id = BTN_ACTION_NONE,
		.image_id = IMG_TILE2_BLANK_LIGHT,
		.press_image_id = IMG_TILE2_FRAME_LIGHT,
		.text_field_id = SF_LABEL2_LIGHT,
		.localized_text = text_hr,
		.top_layer = IMG_TILE3_HR_ICON_LIGHT,
	},
	{
		.button_id = BTN_TILE3,
		.action_id = BTN_ACTION_TOGGLE_BLUETOOTH,
		.image_id = IMG_TILE3_BLANK_LIGHT,
		.press_image_id = IMG_TILE3_FRAME_LIGHT,
		.text_field_id = SF_LABEL3_LIGHT,
		.localized_text = text_bt_off,
		.top_layer = IMG_TILE3_BT_ICON_OFF_LIGHT,
	},
	{
		.button_id = BTN_TILE4,
		.action_id = BTN_ACTION_GOTO_SETTINGS,
		.image_id = IMG_TILE4_BLANK_LIGHT,
		.press_image_id = IMG_TILE4_FRAME_LIGHT,
		.text_field_id = SF_LABEL4_LIGHT,
		.localized_text = text_back,
		.top_layer = IMG_TILE4_BACK_ICON_LIGHT,
	},
};

#ifndef CICLOTTE
static struct menu_button calibration_button = {
	.button_id = BTN_TILE4,
	.action_id = BTN_ACTION_GOTO_CALIBRATE_DOWN,
	.image_id = IMG_TILE4_BLANK_LIGHT,
	.press_image_id = IMG_TILE4_FRAME_LIGHT,
	.text_field_id = SF_LABEL4_LIGHT,
	.localized_text = text_next,
	.top_layer = IMG_TILE4_NEXT_ICON_LIGHT,
};
#endif

static const struct menu_button information_buttons[] = {
	{
		.button_id = BTN_TILE3,
		.action_id = BTN_ACTION_UNLOCK_DEV_MODE,
	},
	{
		.button_id = BTN_TILE4B,
		.action_id = BTN_ACTION_RETURN_FROM_INFO,
		.image_id = IMG_HALFTILE41_BLANK_LIGHT,
		.press_image_id = IMG_HALFTILE41_FRAME_LIGHT,
		.top_layer = IMG_HALFTILE41_HOME_ICON_LIGHT,
	},
};

static const struct menu_button developer_buttons[] = {
	{
		.button_id = BTN_TILE4B,
		.action_id = BTN_ACTION_GOTO_MAIN_MENU,
		.image_id = IMG_HALFTILE41_BLANK_LIGHT,
		.press_image_id = IMG_HALFTILE41_FRAME_LIGHT,
		.top_layer = IMG_HALFTILE41_HOME_ICON_LIGHT,
	},
};

static const struct menu_button image_browser_buttons[] = {
	{
		.button_id = BTN_TILE4,
		.action_id = BTN_ACTION_TILE4,
		.image_id = IMG_TILE4_BLANK_LIGHT,
		.press_image_id = IMG_TILE4_FRAME_LIGHT,
		.text_field_id = SF_LABEL4_LIGHT,
		.localized_text = text_next,
		.top_layer = IMG_TILE4_NEXT_ICON_LIGHT,
	},
};

static const struct menu_button session_buttons[] = {
#ifdef CICLOTTE
	/* As on the AVR Ciclotte: tile 4 sets the brake level, tapping the
	 * header (tile 0) ends the session. There is no pause. */
	{
		.button_id = BTN_TILE4T,
		.action_id = BTN_ACTION_INCREASE_ANGLE,
		.image_id = IMG_HALFTILE40_BLANK_LIGHT,
		.press_image_id = IMG_HALFTILE40_FRAME_LIGHT,
		.top_layer = IMG_HALFTILE40_PLUS_ICON_LIGHT,
	},
	{
		.button_id = BTN_TILE4B,
		.action_id = BTN_ACTION_DECREASE_ANGLE,
		.image_id = IMG_HALFTILE41_BLANK_LIGHT,
		.press_image_id = IMG_HALFTILE41_FRAME_LIGHT,
		.top_layer = IMG_HALFTILE41_MINUS_ICON_LIGHT,
	},
	{
		.button_id = BTN_TILE0,
		.action_id = BTN_ACTION_STOP_SESSION,
	},
#else
	{
		.button_id = BTN_TILE4T,
		.action_id = BTN_ACTION_PAUSE_SESSION,
		.image_id = IMG_HALFTILE40_BLANK_NEUTRAL,
		.press_image_id = IMG_HALFTILE40_FRAME_LIGHT,
		.top_layer = IMG_HALFTILE40_PAUSE_ICON_DARK,
	},
	{
		.button_id = BTN_TILE4B,
		.action_id = BTN_ACTION_STOP_SESSION,
		.image_id = IMG_HALFTILE41_BLANK_LIGHT,
		.press_image_id = IMG_HALFTILE41_FRAME_LIGHT,
		.top_layer = IMG_HALFTILE41_STOP_ICON_LIGHT,
	},
#endif
	{
		.button_id = BTN_TILE1L,
		.action_id = BTN_ACTION_LEFTCYCLE_SESSION_TILE1,
		.press_image_id = IMG_RINGTILE10_FRAME_LIGHT,
	},
	{
		.button_id = BTN_TILE1R,
		.action_id = BTN_ACTION_RIGHTCYCLE_SESSION_TILE1,
		.press_image_id = IMG_RINGTILE11_FRAME_LIGHT,
	},
	{
		.button_id = BTN_TILE2L,
		.action_id = BTN_ACTION_LEFTCYCLE_SESSION_TILE2,
		.press_image_id = IMG_RINGTILE20_FRAME_LIGHT,
	},
	{
		.button_id = BTN_TILE2R,
		.action_id = BTN_ACTION_RIGHTCYCLE_SESSION_TILE2,
		.press_image_id = IMG_RINGTILE21_FRAME_LIGHT,
	},
	{
		.button_id = BTN_TILE3L,
		.action_id = BTN_ACTION_LEFTCYCLE_SESSION_TILE3,
		.press_image_id = IMG_RINGTILE30_FRAME_LIGHT,
	},
	{
		.button_id = BTN_TILE3R,
		.action_id = BTN_ACTION_RIGHTCYCLE_SESSION_TILE3,
		.press_image_id = IMG_RINGTILE31_FRAME_LIGHT,
	},
};

static const struct menu_button bt_connection_buttons[] = {
#ifdef CICLOTTE
	{
		.button_id = BTN_TILE0,
		.action_id = BTN_ACTION_STOP_SESSION,
	},
	{
		.button_id = BTN_TILE4T,
		.action_id = BTN_ACTION_INCREASE_ANGLE,
		.image_id = IMG_HALFTILE40_BLANK_LIGHT,
		.press_image_id = IMG_HALFTILE40_FRAME_LIGHT,
		.top_layer = IMG_HALFTILE40_PLUS_ICON_LIGHT,
	},
	{
		.button_id = BTN_TILE4B,
		.action_id = BTN_ACTION_DECREASE_ANGLE,
		.image_id = IMG_HALFTILE41_BLANK_LIGHT,
		.press_image_id = IMG_HALFTILE41_FRAME_LIGHT,
		.top_layer = IMG_HALFTILE41_MINUS_ICON_LIGHT,
	},
#else
	{
		.button_id = BTN_TILE4B,
		.action_id = BTN_ACTION_STOP_SESSION,
		.image_id = IMG_HALFTILE41_BLANK_LIGHT,
		.press_image_id = IMG_HALFTILE41_FRAME_LIGHT,
		.text_field_id = SF_HALFTILE41_LABEL_LIGHT,
		.localized_text = text_end,
		.top_layer = IMG_HALFTILE41_DISCONBT_LIGHT,
	},
#endif
};

static const struct menu_button summary_buttons[] = {
	{
		.button_id = BTN_TILE4,
		.action_id = BTN_ACTION_GOTO_MAIN_MENU,
		.image_id = IMG_TILE4_BLANK_LIGHT,
		.press_image_id = IMG_TILE4_FRAME_LIGHT,
		.text_field_id = SF_LABEL4_LIGHT,
		.localized_text = text_home,
		.top_layer = IMG_TILE4_HOME_LIGHT,
	},
	{
		.button_id = BTN_TILE1L,
		.action_id = BTN_ACTION_LEFTCYCLE_SESSION_TILE1,
		.press_image_id = IMG_RINGTILE10_FRAME_LIGHT,
	},
	{
		.button_id = BTN_TILE1R,
		.action_id = BTN_ACTION_RIGHTCYCLE_SESSION_TILE1,
		.press_image_id = IMG_RINGTILE11_FRAME_LIGHT,
	},
	{
		.button_id = BTN_TILE2L,
		.action_id = BTN_ACTION_LEFTCYCLE_SESSION_TILE2,
		.press_image_id = IMG_RINGTILE20_FRAME_LIGHT,
	},
	{
		.button_id = BTN_TILE2R,
		.action_id = BTN_ACTION_RIGHTCYCLE_SESSION_TILE2,
		.press_image_id = IMG_RINGTILE21_FRAME_LIGHT,
	},
	{
		.button_id = BTN_TILE3L,
		.action_id = BTN_ACTION_LEFTCYCLE_SESSION_TILE3,
		.press_image_id = IMG_RINGTILE30_FRAME_LIGHT,
	},
	{
		.button_id = BTN_TILE3R,
		.action_id = BTN_ACTION_RIGHTCYCLE_SESSION_TILE3,
		.press_image_id = IMG_RINGTILE31_FRAME_LIGHT,
	},
};

static const struct menu_button confirm_stop_buttons[] = {
	{
		.button_id = BTN_TILE1,
		.action_id = BTN_ACTION_CONFIRM_CONFIRM,
		.image_id = IMG_TILE1_BLANK_LIGHT,
		.press_image_id = IMG_TILE1_FRAME_LIGHT,
		.text_field_id = SF_LABEL1_LIGHT,
		.localized_text = text_yes,
		.top_layer = IMG_TILE1_CHECK_ICON_LIGHT,
	},
	{
		.button_id = BTN_TILE2,
		.action_id = BTN_ACTION_CONFIRM_CANCEL,
		.image_id = IMG_TILE2_BLANK_LIGHT,
		.press_image_id = IMG_TILE2_FRAME_LIGHT,
		.text_field_id = SF_LABEL2_LIGHT,
		.localized_text = text_no,
		.top_layer = IMG_TILE2_CANCEL_ICON_LIGHT,
	},
};

static const struct menu_page pages[] = {
	[PAGE_MAIN_MENU] = {
		.page_id = PAGE_MAIN_MENU,
		.images = main_menu_images,
		.image_count = ARRAY_SIZE(main_menu_images),
		.buttons = main_menu_buttons,
		.button_count = ARRAY_SIZE(main_menu_buttons),
	},
	[PAGE_TIME_SELECTION] = {
		.page_id = PAGE_TIME_SELECTION,
		.images = time_selection_images,
		.image_count = ARRAY_SIZE(time_selection_images),
		.buttons = time_selection_buttons,
		.button_count = ARRAY_SIZE(time_selection_buttons),
	},
	[PAGE_DISTANCE_SELECTION] = {
		.page_id = PAGE_DISTANCE_SELECTION,
		.images = distance_selection_images,
		.image_count = ARRAY_SIZE(distance_selection_images),
		.buttons = distance_selection_buttons,
		.button_count = ARRAY_SIZE(distance_selection_buttons),
	},
	[PAGE_SETTINGS_1] = {
		.page_id = PAGE_SETTINGS_1,
		.images = settings1_images,
		.image_count = ARRAY_SIZE(settings1_images),
		.buttons = settings1_buttons,
		.button_count = ARRAY_SIZE(settings1_buttons),
	},
	[PAGE_SETTINGS_2] = {
		.page_id = PAGE_SETTINGS_2,
		.images = settings2_images,
		.image_count = ARRAY_SIZE(settings2_images),
		.buttons = settings2_buttons,
		.button_count = ARRAY_SIZE(settings2_buttons),
	},
	[PAGE_SETUP] = {
		.page_id = PAGE_SETUP,
		.images = setup_images,
		.image_count = ARRAY_SIZE(setup_images),
		.buttons = setup_buttons,
		.button_count = ARRAY_SIZE(setup_buttons),
	},
	[PAGE_HR_CONNECT] = {
		.page_id = PAGE_HR_CONNECT,
		.images = hr_connect_images,
		.image_count = ARRAY_SIZE(hr_connect_images),
		.buttons = hr_connect_buttons,
		.button_count = ARRAY_SIZE(hr_connect_buttons),
	},
#ifndef CICLOTTE
	[PAGE_CALIBRATION] = {
		.page_id = PAGE_CALIBRATION,
		.images = calibration_images,
		.image_count = ARRAY_SIZE(calibration_images),
		.buttons = &calibration_button,
		.button_count = 1,
	},
#endif
	[PAGE_INFORMATION] = {
		.page_id = PAGE_INFORMATION,
		.images = information_images,
		.image_count = ARRAY_SIZE(information_images),
		.buttons = information_buttons,
		.button_count = ARRAY_SIZE(information_buttons),
	},
	[PAGE_DEVELOPER] = {
		.page_id = PAGE_DEVELOPER,
		.images = developer_images,
		.image_count = ARRAY_SIZE(developer_images),
		.buttons = developer_buttons,
		.button_count = ARRAY_SIZE(developer_buttons),
	},
	[PAGE_IMAGE_BROWSER] = {
		.page_id = PAGE_IMAGE_BROWSER,
		.images = image_browser_images,
		.image_count = ARRAY_SIZE(image_browser_images),
		.buttons = image_browser_buttons,
		.button_count = ARRAY_SIZE(image_browser_buttons),
	},
	[PAGE_SESSION] = {
		.page_id = PAGE_SESSION,
		.images = session_images,
		.image_count = ARRAY_SIZE(session_images),
		.buttons = session_buttons,
		.button_count = ARRAY_SIZE(session_buttons),
	},
	[PAGE_BT_CONNECTION] = {
		.page_id = PAGE_BT_CONNECTION,
		.images = bt_connection_images,
		.image_count = ARRAY_SIZE(bt_connection_images),
		.buttons = bt_connection_buttons,
		.button_count = ARRAY_SIZE(bt_connection_buttons),
	},
	[PAGE_SUMMARY] = {
		.page_id = PAGE_SUMMARY,
		.images = summary_images,
		.image_count = ARRAY_SIZE(summary_images),
		.buttons = summary_buttons,
		.button_count = ARRAY_SIZE(summary_buttons),
	},
	[PAGE_CONFIRM_STOP] = {
		.page_id = PAGE_CONFIRM_STOP,
		.images = confirm_stop_images,
		.image_count = ARRAY_SIZE(confirm_stop_images),
		.buttons = confirm_stop_buttons,
		.button_count = ARRAY_SIZE(confirm_stop_buttons),
	},
};

static const struct menu_page *active_page;
static uint8_t button_pressed;
static struct session_state session;
static uint8_t dev_mode_unlock_counter;
struct image_browser_asset {
	uint8_t image_id;
	const char *name;
};

static const struct image_browser_asset image_browser_assets[] = {
	{0x00, "IMG_BG_DARK_HT40_NEUTRAL"},
	{0x01, "IMG_BG_GRID_LIGHT"},
	{0x02, "IMG_TILE0_NOHRD_LOGO"},
	{0x04, "IMG_TILE1_BLANK_LIGHT"},
	{0x05, "IMG_TILE2_BLANK_LIGHT"},
	{0x06, "IMG_TILE3_BLANK_LIGHT"},
	{0x07, "IMG_TILE4_BLANK_LIGHT"},
	{0x09, "IMG_TILE1_BLANK_DARK"},
	{0x0a, "IMG_TILE2_BLANK_DARK"},
	{0x0b, "IMG_TILE3_BLANK_DARK"},
	{0x0c, "IMG_TILE4_BLANK_DARK"},
	{0x0e, "IMG_TILE1_PRESSED_LIGHT"},
	{0x0f, "IMG_TILE2_PRESSED_LIGHT"},
	{0x10, "IMG_TILE3_PRESSED_LIGHT"},
	{0x11, "IMG_TILE4_PRESSED_LIGHT"},
	{0x13, "IMG_TILE1_PRESSED_DARK"},
	{0x14, "IMG_TILE2_PRESSED_DARK"},
	{0x15, "IMG_TILE3_PRESSED_DARK"},
	{0x16, "IMG_TILE4_PRESSED_DARK"},
	{0x17, "IMG_HALFTILE10_BLANK_LIGHT"},
	{0x18, "IMG_HALFTILE11_BLANK_LIGHT"},
	{0x19, "IMG_HALFTILE30_BLANK_LIGHT"},
	{0x1a, "IMG_HALFTILE31_BLANK_LIGHT"},
	{0x1b, "IMG_HALFTILE40_BLANK_LIGHT"},
	{0x1c, "IMG_HALFTILE41_BLANK_LIGHT"},
	{0x1d, "IMG_HALFTILE10_BLANK_DARK"},
	{0x1e, "IMG_HALFTILE11_BLANK_DARK"},
	{0x1f, "IMG_HALFTILE30_BLANK_DARK"},
	{0x20, "IMG_HALFTILE31_BLANK_DARK"},
	{0x21, "IMG_HALFTILE40_BLANK_DARK"},
	{0x22, "IMG_HALFTILE41_BLANK_DARK"},
	{0x23, "IMG_HALFTILE40_BLANK_NEUTRAL"},
	{0x24, "IMG_HALFTILE41_BLANK_NEUTRAL"},
	{0x25, "IMG_HALFTILE10_PRESSED_LIGHT"},
	{0x26, "IMG_HALFTILE11_PRESSED_LIGHT"},
	{0x27, "IMG_HALFTILE20_PRESSED_LIGHT"},
	{0x28, "IMG_HALFTILE21_PRESSED_LIGHT"},
	{0x29, "IMG_HALFTILE30_PRESSED_LIGHT"},
	{0x2a, "IMG_HALFTILE31_PRESSED_LIGHT"},
	{0x2b, "IMG_HALFTILE40_PRESSED_LIGHT"},
	{0x2c, "IMG_HALFTILE41_PRESSED_LIGHT"},
	{0x2d, "IMG_HALFTILE10_PRESSED_DARK"},
	{0x2e, "IMG_HALFTILE11_PRESSED_DARK"},
	{0x2f, "IMG_HALFTILE20_PRESSED_DARK"},
	{0x30, "IMG_HALFTILE21_PRESSED_DARK"},
	{0x31, "IMG_HALFTILE30_PRESSED_DARK"},
	{0x32, "IMG_HALFTILE31_PRESSED_DARK"},
	{0x33, "IMG_HALFTILE40_PRESSED_DARK"},
	{0x34, "IMG_HALFTILE41_PRESSED_DARK"},
	{0x36, "IMG_TILE1_FRAME_LIGHT"},
	{0x37, "IMG_TILE2_FRAME_LIGHT"},
	{0x38, "IMG_TILE3_FRAME_LIGHT"},
	{0x39, "IMG_TILE4_FRAME_LIGHT"},
	{0x3a, "IMG_HALFTILE10_FRAME_LIGHT"},
	{0x3b, "IMG_HALFTILE11_FRAME_LIGHT"},
	{0x3c, "IMG_HALFTILE20_FRAME_LIGHT"},
	{0x3d, "IMG_HALFTILE21_FRAME_LIGHT"},
	{0x3e, "IMG_HALFTILE30_FRAME_LIGHT"},
	{0x3f, "IMG_HALFTILE31_FRAME_LIGHT"},
	{0x40, "IMG_HALFTILE40_FRAME_LIGHT"},
	{0x41, "IMG_HALFTILE41_FRAME_LIGHT"},
	{0x43, "IMG_TILE1_FRAME_DARK"},
	{0x44, "IMG_TILE2_FRAME_DARK"},
	{0x45, "IMG_TILE3_FRAME_DARK"},
	{0x46, "IMG_TILE4_FRAME_DARK"},
	{0x47, "IMG_HALFTILE10_FRAME_DARK"},
	{0x48, "IMG_HALFTILE11_FRAME_DARK"},
	{0x49, "IMG_HALFTILE20_FRAME_DARK"},
	{0x4a, "IMG_HALFTILE21_FRAME_DARK"},
	{0x4b, "IMG_HALFTILE30_FRAME_DARK"},
	{0x4c, "IMG_HALFTILE31_FRAME_DARK"},
	{0x4d, "IMG_HALFTILE40_FRAME_DARK"},
	{0x4e, "IMG_HALFTILE41_FRAME_DARK"},
	{0x4f, "IMG_RINGTILE10_FRAME_LIGHT"},
	{0x50, "IMG_RINGTILE11_FRAME_LIGHT"},
	{0x51, "IMG_RINGTILE20_FRAME_LIGHT"},
	{0x52, "IMG_RINGTILE21_FRAME_LIGHT"},
	{0x53, "IMG_RINGTILE30_FRAME_LIGHT"},
	{0x54, "IMG_RINGTILE31_FRAME_LIGHT"},
	{0x55, "IMG_TILE1_FRAME2_LIGHT"},
	{0x56, "IMG_TILE2_FRAME2_LIGHT"},
	{0x57, "IMG_TILE3_FRAME2_LIGHT"},
	{0x58, "IMG_TILE1_FRAME2_DARK"},
	{0x59, "IMG_TILE2_FRAME2_DARK"},
	{0x5a, "IMG_TILE3_FRAME2_DARK"},
	{0x5b, "IMG_RINGTILE10_FRAME_DARK"},
	{0x5c, "IMG_RINGTILE11_FRAME_DARK"},
	{0x5d, "IMG_RINGTILE20_FRAME_DARK"},
	{0x5e, "IMG_RINGTILE21_FRAME_DARK"},
	{0x5f, "IMG_RINGTILE30_FRAME_DARK"},
	{0x60, "IMG_RINGTILE31_FRAME_DARK"},
	{0x61, "IMG_BG_GRID_LIGHT_TILE4_DARK"},
	{0x64, "IMG_TILE0_DISTANCE"},
	{0x65, "IMG_TILE0_FINISH"},
	{0x67, "IMG_TILE0_TIME"},
	{0x68, "IMG_TILE1_TIME_ICON_LIGHT"},
	{0x69, "IMG_TILE2_DISTANCE_ICON_LIGHT"},
	{0x6a, "IMG_TILE3_FREE_ICON_LIGHT"},
	{0x6b, "IMG_TILE4_SETTINGS_ICON_LIGHT"},
	{0x6c, "IMG_TILE1_TIME_ICON_DARK"},
	{0x6d, "IMG_TILE2_DISTANCE_ICON_DARK"},
	{0x6e, "IMG_TILE3_FREE_ICON_DARK"},
	{0x6f, "IMG_TILE4_SETTINGS_ICON_DARK"},
	{0x70, "IMG_TILE1_CHECK_ICON_LIGHT"},
	{0x71, "IMG_TILE2_CANCEL_ICON_LIGHT"},
	{0x72, "IMG_TILE1_CHECK_ICON_DARK"},
	{0x73, "IMG_TILE2_CANCEL_ICON_DARK"},
	{0x74, "IMG_HALFTILE41_LEFT_ARROW_ICON_LIGHT"},
	{0x75, "IMG_HALFTILE41_LEFT_ARROW_ICON_DARK"},
	{0x76, "IMG_HALFTILE41_STOP_ICON_LIGHT"},
	{0x77, "IMG_HALFTILE41_STOP_ICON_DARK"},
	{0x78, "IMG_HALFTILE40_PAUSE_ICON_LIGHT"},
	{0x79, "IMG_HALFTILE40_PAUSE_ICON_DARK"},
	{0x7a, "IMG_HALFTILE40_PLUS_ICON_LIGHT"},
	{0x7b, "IMG_HALFTILE41_MINUS_ICON_LIGHT"},
	{0x7c, "IMG_HALFTILE40_PLUS_ICON_DARK"},
	{0x7d, "IMG_HALFTILE41_MINUS_ICON_DARK"},
	{0x7e, "IMG_HALFTILE40_PLAY_ICON_LIGHT"},
	{0x7f, "IMG_HALFTILE40_PLAY_ICON_DARK"},
	{0x80, "IMG_TILE2_SUN_ICON_LIGHT"},
	{0x81, "IMG_TILE2_MOON_ICON_DARK"},
	{0x82, "IMG_TILE3_HR_ICON_LIGHT"},
	{0x83, "IMG_TILE3_HR_ICON_DARK"},
	{0x84, "IMG_TILE1_ENGLISH_ICON"},
	{0x85, "IMG_TILE1_GERMAN_ICON"},
	{0x86, "IMG_TILE1_FRENCH_ICON"},
	{0x87, "IMG_TILE1_SPANISH_ICON"},
	{0x88, "IMG_TILE1_US_ICON"},
	{0x89, "IMG_TILE0_SETTINGS"},
	{0x8a, "IMG_TILE4_HOME_LIGHT"},
	{0x8b, "IMG_TILE4_HOME_DARK"},
	{0x8c, "IMG_TILE4_NEXT_ICON_LIGHT"},
	{0x8d, "IMG_TILE4_NEXT_ICON_DARK"},
	{0x8e, "IMG_TILE3_BT_ICON_OFF_LIGHT"},
	{0x8f, "IMG_TILE3_BT_ICON_OFF_DARK"},
	{0x90, "IMG_TILE3_BT_ICON_ON"},
	{0x91, "IMG_TILE2_CALIBRATE_DOWN_LIGHT"},
	{0x92, "IMG_TILE2_CALIBRATE_DOWN_DARK"},
	{0x93, "IMG_TILE2_CALIBRATE_UP_LIGHT"},
	{0x94, "IMG_TILE2_CALIBRATE_UP_DARK"},
	{0x95, "IMG_TILE1_INFO_LIGHT"},
	{0x96, "IMG_TILE1_INFO_DARK"},
	{0x97, "IMG_HALFTILE41_HOME_ICON_LIGHT"},
	{0x98, "IMG_HALFTILE41_HOME_ICON_DARK"},
	{0x99, "IMG_TILE0_INFO_ICON"},
	{0x9b, "IMG_HALFTILE40_REFRESH_HR"},
	{0x9c, "IMG_HALFTILE40_WAIT_HR_1_3"},
	{0x9d, "IMG_HALFTILE40_WAIT_HR_2_3"},
	{0x9e, "IMG_HALFTILE40_WAIT_HR_3_3"},
	{0x9f, "IMG_HALFTILE41_PAUSED_ICON_DARK"},
	{0xa0, "IMG_HALFTILE41_PAUSED_ICON_LIGHT"},
	{0xa1, "IMG_TILE4_CHECK_ICON_DARK"},
	{0xa2, "IMG_TILE4_CHECK_ICON_LIGHT"},
	{0xa3, "IMG_BG_GRID_DARK"},
	{0xa4, "IMG_TILE1_RING_1_9"},
	{0xa5, "IMG_TILE1_RING_2_9"},
	{0xa6, "IMG_TILE1_RING_3_9"},
	{0xa7, "IMG_TILE1_RING_4_9"},
	{0xa8, "IMG_TILE1_RING_5_9"},
	{0xa9, "IMG_TILE1_RING_6_9"},
	{0xaa, "IMG_TILE1_RING_7_9"},
	{0xab, "IMG_TILE1_RING_8_9"},
	{0xac, "IMG_TILE1_RING_9_9"},
	{0xad, "IMG_TILE2_RING_1_9"},
	{0xae, "IMG_TILE2_RING_2_9"},
	{0xaf, "IMG_TILE2_RING_3_9"},
	{0xb0, "IMG_TILE2_RING_4_9"},
	{0xb1, "IMG_TILE2_RING_5_9"},
	{0xb2, "IMG_TILE2_RING_6_9"},
	{0xb3, "IMG_TILE2_RING_7_9"},
	{0xb4, "IMG_TILE2_RING_8_9"},
	{0xb5, "IMG_TILE2_RING_9_9"},
	{0xb6, "IMG_TILE3_RING_1_9"},
	{0xb7, "IMG_TILE3_RING_2_9"},
	{0xb8, "IMG_TILE3_RING_3_9"},
	{0xb9, "IMG_TILE3_RING_4_9"},
	{0xba, "IMG_TILE3_RING_5_9"},
	{0xbb, "IMG_TILE3_RING_6_9"},
	{0xbc, "IMG_TILE3_RING_7_9"},
	{0xbd, "IMG_TILE3_RING_8_9"},
	{0xbe, "IMG_TILE3_RING_9_9"},
	{0xbf, "IMG_HR_INDICATOR_ICON_OFF"},
	{0xc0, "IMG_HR_INDICATOR_ICON_ON"},
	{0xc1, "IMG_BG_GRID_DARK_TILE4_DARK"},
	{0xc2, "IMG_BG_LIGHT"},
	{0xc3, "IMG_BG_DARK"},
	{0xc4, "IMG_BG_BOOT"},
	{0xc5, "IMG_BG_HALFGRID_LIGHT"},
	{0xc6, "IMG_TILE3_QRCODE"},
	{0xc7, "IMG_TILE1_NOHRD_INFO"},
	{0xc8, "IMG_HALFTILE41_DISCONBT_LIGHT"},
	{0xc9, "IMG_HALFTILE41_DISCONBT_DARK"},
	{0xca, "IMG_TILE3_CONTRAST_ICON_LIGHT"},
	{0xcb, "IMG_TILE3_CONTRAST_ICON_DARK"},
	{0xcc, "IMG_HALFTILE40_VERSION_NUMBER"},
	{0xcd, "IMG_BG_HALFGRID_DARK"},
	{0xce, "IMG_BG_SS_1"},
	{0xcf, "IMG_BG_SS_2"},
	{0xd0, "IMG_BG_SS_3"},
	{0xd1, "IMG_BG_SS_4"},
	{0xd2, "IMG_BG_LIGHT_HT40_NEUTRAL"},
	{0xd3, "IMG_BG_CALIBRATION"},
	{0xd4, "IMG_BG_BLACK"},
	{0xd5, "IMG_TILE2_CUSTOMIZE_ICON_LIGHT"},
	{0xd6, "IMG_TILE1_CUSTOMIZE_ICON_DARK"},
	{0xd7, "IMG_TILE3_SETUP_ICON_LIGHT"},
	{0xd8, "IMG_TILE2_SETUP_ICON_DARK"},
	{0xd9, "IMG_TILE1_FULL_BRIGHTNESS_ICON_LIGHT"},
	{0xda, "IMG_TILE1_FULL_BRIGHTNESS_ICON_DARK"},
	{0xdb, "IMG_TILE2_HALF_BRIGHTNESS_ICON_LIGHT"},
	{0xdc, "IMG_TILE2_HALF_BRIGHTNESS_ICON_DARK"},
	{0xdd, "IMG_TILE3_LOW_BRIGHTNESS_ICON_LIGHT"},
	{0xde, "IMG_TILE3_LOW_BRIGHTNESS_ICON_DARK"},
	{0xdf, "IMG_TILE1_BATTERY_ICON_LIGHT"},
	{0xe0, "IMG_TILE1_BATTERY_ICON_DARK"},
	{0xe1, "IMG_TILE1_GRID_ICON_LIGHT"},
	{0xe2, "IMG_TILE1_GRID_ICON_DARK"},
	{0xe3, "IMG_TILE0_CUSTOM_LOGO"},
	{0xe4, "IMG_TILE0_SETUP_LOGO"},
	{0xe5, "IMG_TILE0_CONTRAST_LOGO"},
	{0xe6, "IMG_TILE4_BACK_ICON_LIGHT"},
	{0xe7, "IMG_TILE4_BACK_ICON_DARK"},
};

static struct session_ring session_rings[] = {
	{.idx = SESSION_METRIC_WATTS, .value_field = SF_VALUE1_LIGHT,
	 .label_field = SF_LABEL1_LIGHT, .first_image = IMG_TILE1_RING_1_9,
	 .blank_image = IMG_TILE1_BLANK_LIGHT},
	{.idx = SESSION_METRIC_CADENCE, .value_field = SF_VALUE2_LIGHT,
	 .label_field = SF_LABEL2_LIGHT, .first_image = IMG_TILE2_RING_1_9,
	 .blank_image = IMG_TILE2_BLANK_LIGHT},
	{.idx = SESSION_METRIC_RESISTANCE, .value_field = SF_VALUE3_LIGHT,
	 .label_field = SF_LABEL3_LIGHT, .first_image = IMG_TILE3_RING_1_9,
	 .blank_image = IMG_TILE3_BLANK_LIGHT},
};

static struct session_ring summary_rings[] = {
	{.idx = SUMMARY_METRIC_AVG_WATTS, .value_field = SF_VALUE1_LIGHT,
	 .label_field = SF_LABEL1_LIGHT, .first_image = IMG_TILE1_RING_1_9,
	 .blank_image = IMG_TILE1_BLANK_LIGHT},
	{.idx = SUMMARY_METRIC_AVG_HEARTRATE, .value_field = SF_VALUE2_LIGHT,
	 .label_field = SF_LABEL2_LIGHT, .first_image = IMG_TILE2_RING_1_9,
	 .blank_image = IMG_TILE2_BLANK_LIGHT},
	{.idx = SUMMARY_METRIC_MAX_HEARTRATE, .value_field = SF_VALUE3_LIGHT,
	 .label_field = SF_LABEL3_LIGHT, .first_image = IMG_TILE3_RING_1_9,
	 .blank_image = IMG_TILE3_BLANK_LIGHT},
};

static uint8_t session_ring_pref[ARRAY_SIZE(session_rings)] = {
	SESSION_METRIC_WATTS,
	SESSION_METRIC_CADENCE,
	SESSION_METRIC_RESISTANCE,
};

static void display_send(const uint8_t *data, size_t len)
{
	for (size_t i = 0; i < len; i++) {
		uart_poll_out(uart_dev, data[i]);
	}
}

static uint16_t rx_crc_ccitt(const uint8_t *data, size_t len)
{
	uint16_t crc = 0x0000;
	const uint16_t polynomial = 0x1021;

	while (len--) {
		crc ^= ((uint16_t)*data++) << 8;

		for (uint8_t bit = 0; bit < 8; bit++) {
			if ((crc & 0x8000) != 0) {
				crc = (crc << 1) ^ polynomial;
			} else {
				crc <<= 1;
			}
		}
	}

	return crc;
}

static void display_cmd(const uint8_t *payload, size_t len)
{
	uint8_t buffer[len + 7];
	uint16_t crc = rx_crc_ccitt(payload, len);
	size_t pos = 0;

	buffer[pos++] = 0xaa;

	for (size_t i = 0; i < len; i++) {
		buffer[pos++] = payload[i];
	}

	buffer[pos++] = (uint8_t)((crc >> 8) & 0xff);
	buffer[pos++] = (uint8_t)(crc & 0xff);
	buffer[pos++] = 0xe4;
	buffer[pos++] = 0x1b;
	buffer[pos++] = 0x11;
	buffer[pos++] = 0xee;

	display_send(buffer, pos);
}

static uint8_t darkmode_image_for(uint8_t image)
{
	static const uint8_t light_dark_images[][2] = {
		{IMG_BG_GRID_LIGHT, IMG_BG_GRID_DARK},
		{IMG_BG_HALFGRID_LIGHT, IMG_BG_DARK_HT40_NEUTRAL},
		{IMG_BG_LIGHT_HT40_NEUTRAL, IMG_BG_DARK_HT40_NEUTRAL},
		{IMG_TILE1_TIME_ICON_LIGHT, IMG_TILE1_TIME_ICON_DARK},
		{IMG_TILE2_DISTANCE_ICON_LIGHT, IMG_TILE2_DISTANCE_ICON_DARK},
		{IMG_TILE3_FREE_ICON_LIGHT, IMG_TILE3_FREE_ICON_DARK},
		{IMG_TILE4_SETTINGS_ICON_LIGHT, IMG_TILE4_SETTINGS_ICON_DARK},
		{IMG_HALFTILE41_MINUS_ICON_LIGHT, IMG_HALFTILE41_MINUS_ICON_DARK},
		{IMG_HALFTILE40_PLUS_ICON_LIGHT, IMG_HALFTILE40_PLUS_ICON_DARK},
		{IMG_HALFTILE40_BLANK_LIGHT, IMG_HALFTILE40_BLANK_DARK},
		{IMG_HALFTILE41_LEFT_ARROW_ICON_LIGHT, IMG_HALFTILE41_LEFT_ARROW_ICON_DARK},
		{IMG_TILE1_BLANK_LIGHT, IMG_TILE1_BLANK_DARK},
		{IMG_TILE2_BLANK_LIGHT, IMG_TILE2_BLANK_DARK},
		{IMG_TILE3_BLANK_LIGHT, IMG_TILE3_BLANK_DARK},
		{IMG_TILE4_BLANK_LIGHT, IMG_TILE4_BLANK_DARK},
		{IMG_TILE1_FRAME_LIGHT, IMG_TILE1_FRAME_DARK},
		{IMG_TILE2_FRAME_LIGHT, IMG_TILE2_FRAME_DARK},
		{IMG_TILE3_FRAME_LIGHT, IMG_TILE3_FRAME_DARK},
		{IMG_TILE4_FRAME_LIGHT, IMG_TILE4_FRAME_DARK},
		{IMG_HALFTILE10_FRAME_LIGHT, IMG_HALFTILE10_FRAME_DARK},
		{IMG_HALFTILE11_FRAME_LIGHT, IMG_HALFTILE11_FRAME_DARK},
		{IMG_HALFTILE20_FRAME_LIGHT, IMG_HALFTILE20_FRAME_DARK},
		{IMG_HALFTILE21_FRAME_LIGHT, IMG_HALFTILE21_FRAME_DARK},
		{IMG_HALFTILE30_FRAME_LIGHT, IMG_HALFTILE30_FRAME_DARK},
		{IMG_HALFTILE31_FRAME_LIGHT, IMG_HALFTILE31_FRAME_DARK},
		{IMG_HALFTILE40_FRAME_LIGHT, IMG_HALFTILE40_FRAME_DARK},
		{IMG_HALFTILE41_FRAME_LIGHT, IMG_HALFTILE41_FRAME_DARK},
		{IMG_HALFTILE10_BLANK_LIGHT, IMG_HALFTILE10_BLANK_DARK},
		{IMG_HALFTILE11_BLANK_LIGHT, IMG_HALFTILE11_BLANK_DARK},
		{IMG_HALFTILE30_BLANK_LIGHT, IMG_HALFTILE30_BLANK_DARK},
		{IMG_HALFTILE31_BLANK_LIGHT, IMG_HALFTILE31_BLANK_DARK},
		{IMG_HALFTILE41_BLANK_LIGHT, IMG_HALFTILE41_BLANK_DARK},
		{IMG_TILE2_SUN_ICON_LIGHT, IMG_TILE2_MOON_ICON_DARK},
		{IMG_TILE3_HR_ICON_LIGHT, IMG_TILE3_HR_ICON_DARK},
		{IMG_TILE4_NEXT_ICON_LIGHT, IMG_TILE4_NEXT_ICON_DARK},
		{IMG_TILE4_HOME_LIGHT, IMG_TILE4_HOME_DARK},
		{IMG_TILE1_INFO_LIGHT, IMG_TILE1_INFO_DARK},
		{IMG_TILE3_BT_ICON_OFF_LIGHT, IMG_TILE3_BT_ICON_OFF_DARK},
		{IMG_TILE2_CALIBRATE_UP_LIGHT, IMG_TILE2_CALIBRATE_UP_DARK},
		{IMG_TILE2_CALIBRATE_DOWN_LIGHT, IMG_TILE2_CALIBRATE_DOWN_DARK},
		{IMG_HALFTILE41_STOP_ICON_LIGHT, IMG_HALFTILE41_STOP_ICON_DARK},
		{IMG_HALFTILE40_PAUSE_ICON_LIGHT, IMG_HALFTILE40_PAUSE_ICON_DARK},
		{IMG_HALFTILE40_PLAY_ICON_LIGHT, IMG_HALFTILE40_PLAY_ICON_DARK},
		{IMG_HALFTILE41_HOME_ICON_LIGHT, IMG_HALFTILE41_HOME_ICON_DARK},
		{IMG_HALFTILE41_DISCONBT_LIGHT, IMG_HALFTILE41_DISCONBT_DARK},
		{IMG_TILE2_CANCEL_ICON_LIGHT, IMG_TILE2_CANCEL_ICON_DARK},
		{IMG_HALFTILE41_PAUSED_ICON_LIGHT, IMG_HALFTILE41_PAUSED_ICON_DARK},
		{IMG_RINGTILE10_FRAME_LIGHT, IMG_RINGTILE10_FRAME_DARK},
		{IMG_RINGTILE11_FRAME_LIGHT, IMG_RINGTILE11_FRAME_DARK},
		{IMG_RINGTILE20_FRAME_LIGHT, IMG_RINGTILE20_FRAME_DARK},
		{IMG_RINGTILE21_FRAME_LIGHT, IMG_RINGTILE21_FRAME_DARK},
		{IMG_RINGTILE30_FRAME_LIGHT, IMG_RINGTILE30_FRAME_DARK},
		{IMG_RINGTILE31_FRAME_LIGHT, IMG_RINGTILE31_FRAME_DARK},
		{IMG_TILE1_CHECK_ICON_LIGHT, IMG_TILE1_CHECK_ICON_DARK},
		{IMG_TILE3_CONTRAST_ICON_LIGHT, IMG_TILE3_CONTRAST_ICON_DARK},
		{IMG_TILE2_CUSTOMIZE_ICON_LIGHT, IMG_TILE1_CUSTOMIZE_ICON_DARK},
		{IMG_TILE3_SETUP_ICON_LIGHT, IMG_TILE2_SETUP_ICON_DARK},
		{IMG_TILE1_FULL_BRIGHTNESS_ICON_LIGHT, IMG_TILE1_FULL_BRIGHTNESS_ICON_DARK},
		{IMG_TILE2_HALF_BRIGHTNESS_ICON_LIGHT, IMG_TILE2_HALF_BRIGHTNESS_ICON_DARK},
		{IMG_TILE3_LOW_BRIGHTNESS_ICON_LIGHT, IMG_TILE3_LOW_BRIGHTNESS_ICON_DARK},
		{IMG_TILE1_BATTERY_ICON_LIGHT, IMG_TILE1_BATTERY_ICON_DARK},
		{IMG_TILE1_GRID_ICON_LIGHT, IMG_TILE1_GRID_ICON_DARK},
		{IMG_TILE4_BACK_ICON_LIGHT, IMG_TILE4_BACK_ICON_DARK},
		{0, 0},
	};

	if (!app_darkmode) {
		return image;
	}

	for (size_t i = 0; light_dark_images[i][0] != 0; i++) {
		if (light_dark_images[i][0] == image) {
			return light_dark_images[i][1];
		}
	}

	return image;
}

static uint8_t darkmode_text_field_for(uint8_t field)
{
	static const uint8_t light_dark_texts[][2] = {
		{SF_VALUE0_LIGHT, SF_VALUE0_DARK},
		{SF_LABEL0_LIGHT, SF_LABEL0_DARK},
		{SF_LABEL1_LIGHT, SF_LABEL1_DARK},
		{SF_LABEL2_LIGHT, SF_LABEL2_DARK},
		{SF_LABEL3_LIGHT, SF_LABEL3_DARK},
		{SF_LABEL4_LIGHT, SF_LABEL4_DARK},
		{SF_VALUE1_LIGHT, SF_VALUE1_DARK},
		{SF_VALUE2_LIGHT, SF_VALUE2_DARK},
		{SF_VALUE3_LIGHT, SF_VALUE3_DARK},
		{SF_VALUE4_LIGHT, SF_VALUE4_DARK},
		{SF_LABEL10_LIGHT, SF_LABEL10_DARK},
		{SF_LABEL31_LIGHT, SF_LABEL31_DARK},
		{SF_LABEL11_LIGHT, SF_LABEL11_DARK},
		{SF_LABEL30_LIGHT, SF_LABEL30_DARK},
		{SF_INFO_LINE0_LIGHT, SF_INFO_LINE0_DARK},
		{SF_INFO_LINE1_LIGHT, SF_INFO_LINE1_DARK},
		{SF_INFO_LINE2_LIGHT, SF_INFO_LINE2_DARK},
		{SF_INFO_LINE3_LIGHT, SF_INFO_LINE3_DARK},
		{SF_INFO_LINE4_LIGHT, SF_INFO_LINE4_DARK},
		{SF_HALFTILE41_LABEL_LIGHT, SF_HALFTILE41_LABEL_DARK},
		{0, 0},
	};

	if (!app_darkmode) {
		return field;
	}

	for (size_t i = 0; light_dark_texts[i][0] != 0; i++) {
		if (light_dark_texts[i][0] == field) {
			return light_dark_texts[i][1];
		}
	}

	return field;
}

static void display_show_image(uint8_t image)
{
	uint8_t payload[] = {0x80, darkmode_image_for(image)};

	display_cmd(payload, sizeof(payload));
}

static void display_show_image_raw(uint8_t image)
{
	uint8_t payload[] = {0x80, image};

	display_cmd(payload, sizeof(payload));
}

static void display_set_brightness(uint8_t brightness)
{
	uint8_t payload[] = {0xba, brightness};

	if (display_brightness == brightness) {
		return;
	}

	if (brightness == DISPLAY_BRIGHTNESS_OFF) {
		display_show_image_raw(IMG_BG_BLACK);
	}

	display_cmd(payload, sizeof(payload));
	display_brightness = brightness;
}

static void DISPLAY_UNUSED display_show_gif(uint8_t gif)
{
	uint8_t payload[] = {0x88, gif};

	display_cmd(payload, sizeof(payload));
}

static void DISPLAY_UNUSED display_remove_image(uint8_t image)
{
	uint8_t payload[] = {0x85, image};

	display_cmd(payload, sizeof(payload));
}

static void display_activate_button(uint8_t button)
{
	uint8_t payload[] = {0xa2, button};

	display_cmd(payload, sizeof(payload));
}

static void DISPLAY_UNUSED display_deactivate_button(uint8_t button)
{
	uint8_t payload[] = {0xa3, button};

	display_cmd(payload, sizeof(payload));
}

static void display_show_text(uint8_t field, const char *text, size_t len)
{
	uint8_t buffer[len + 2];

	buffer[0] = 0x9f;
	buffer[1] = darkmode_text_field_for(field);

	for (size_t i = 0; i < len; i++) {
		buffer[i + 2] = (uint8_t)text[i];
	}

	display_cmd(buffer, sizeof(buffer));
}

static void display_show_text_raw(uint8_t field, const char *text, size_t len)
{
	uint8_t buffer[len + 2];

	buffer[0] = 0x9f;
	buffer[1] = field;

	for (size_t i = 0; i < len; i++) {
		buffer[i + 2] = (uint8_t)text[i];
	}

	display_cmd(buffer, sizeof(buffer));
}

static void display_activate_swipe(void)
{
	uint8_t payload[] = {0xc9, 0x01};

	display_cmd(payload, sizeof(payload));
}

static const char *hr_search_mode_label(void)
{
	switch (hr_search_mode) {
	case HR_SEARCH_OFF:
		return "AUS";
	case HR_SEARCH_BLE:
	default:
		return "BLE";
	}
}

static bool hr_search_mode_has_ble(void)
{
	return hr_search_mode == HR_SEARCH_BLE;
}

static void cycle_hr_search_mode(void)
{
	hr_search_mode = (hr_search_mode + 1U) % HR_SEARCH_COUNT;
	if (hr_search_mode == HR_SEARCH_OFF) {
		(void)ble_hr_disconnect();
	}
}

static const struct menu_button *find_button(uint8_t button_id)
{
	if (active_page == NULL) {
		return NULL;
	}

	for (size_t i = 0; i < active_page->button_count; i++) {
		if (active_page->buttons[i].button_id == button_id) {
			return &active_page->buttons[i];
		}
	}

	return NULL;
}

static void show_button(const struct menu_button *button, bool pressed)
{
	const char *text;
	uint8_t top_layer;

	if (button == NULL) {
		return;
	}

	text = button->localized_text != NULL ?
	       button->localized_text[app_language] : button->text;
	top_layer = button->top_layer;

	if (button->action_id == BTN_ACTION_TOGGLE_LANGUAGE) {
		top_layer = IMG_TILE1_ENGLISH_ICON + app_language;
	}

	if (button->action_id == BTN_ACTION_TOGGLE_LDMODE) {
		text = app_darkmode ? text_dark[app_language] : text_light[app_language];
		top_layer = app_darkmode ? IMG_TILE2_MOON_ICON_DARK :
					   IMG_TILE2_SUN_ICON_LIGHT;
	}

	if (button->action_id == BTN_ACTION_TOGGLE_BLUETOOTH) {
		text = ble_ftms_is_enabled() ? text_bt_on[app_language] :
					       text_bt_off[app_language];
		top_layer = ble_ftms_is_enabled() ? IMG_TILE3_BT_ICON_ON :
						    IMG_TILE3_BT_ICON_OFF_LIGHT;
	}

	if (button->action_id == BTN_ACTION_TILE3) {
		text = hr_search_mode_label();
		top_layer = IMG_TILE3_HR_ICON_LIGHT;
	}

	if (button->action_id == BTN_ACTION_PAUSE_SESSION && session.paused) {
		top_layer = IMG_HALFTILE40_PLAY_ICON_LIGHT;
	}

	if (pressed) {
		if (button->press_image_id != 0) {
			display_show_image(button->press_image_id);
		}
	} else {
		if (button->image_id != 0) {
			display_show_image(button->image_id);
		}
	}

	if (button->text_field_id != 0 && text != NULL) {
		display_show_text(button->text_field_id, text, strlen(text));
	}

	if (top_layer != 0) {
		display_show_image(top_layer);
	}

	if (button->action_id != BTN_ACTION_NONE) {
		display_activate_button(button->button_id);
	}
}

static void construct_page(enum app_page_id page_id)
{
	const struct menu_page *page = &pages[page_id];

	for (size_t i = 0; i < page->image_count; i++) {
		display_show_image(page->images[i]);
	}

	for (size_t i = 0; i < page->button_count; i++) {
		show_button(&page->buttons[i], false);
	}

	active_page = page;
	if (page_id == PAGE_SETTINGS_1) {
		display_show_text(SF_LABEL0_DARK, text_settings[app_language],
				  strlen(text_settings[app_language]));
	}
	display_activate_swipe();
}

static void rearm_active_page_controls(void)
{
	if (active_page == NULL) {
		return;
	}

	for (size_t i = 0; i < active_page->button_count; i++) {
		const struct menu_button *button = &active_page->buttons[i];

		if (button->action_id != BTN_ACTION_NONE) {
			display_activate_button(button->button_id);
		}
	}

	display_activate_swipe();
}

static void deactivate_active_page_controls(void)
{
	if (active_page == NULL) {
		return;
	}

	for (size_t i = 0; i < active_page->button_count; i++) {
		const struct menu_button *button = &active_page->buttons[i];

		if (button->action_id != BTN_ACTION_NONE) {
			display_deactivate_button(button->button_id);
		}
	}
}

static void show_information_page(void)
{
	dev_mode_unlock_counter = 0;

	construct_page(PAGE_INFORMATION);
	display_show_text_raw(SF_HALFTILE40_LINE2_LIGHT, text_version_firmware,
			      strlen(text_version_firmware));
	display_show_text_raw(SF_HALFTILE40_LINE3_LIGHT, text_version_display_fw,
			      strlen(text_version_display_fw));
	display_show_text(SF_LABEL0_DARK, text_info[app_language],
			  strlen(text_info[app_language]));
	display_show_text(SF_INFO_LINE0_LIGHT, text_info_line0[app_language],
			  strlen(text_info_line0[app_language]));
	display_show_text(SF_INFO_LINE1_LIGHT, text_info_line1[app_language],
			  strlen(text_info_line1[app_language]));
	display_show_text(SF_INFO_LINE2_LIGHT, text_info_line2[app_language],
			  strlen(text_info_line2[app_language]));
	display_show_text(SF_INFO_LINE3_LIGHT, text_info_line3[app_language],
			  strlen(text_info_line3[app_language]));
	display_show_text(SF_INFO_LINE4_LIGHT, text_info_line4[app_language],
			  strlen(text_info_line4[app_language]));
}

static const char *run_external_flash_test(void)
{
	static char result[24];
	const struct device *ext_flash_dev = DEVICE_DT_GET(DT_NODELABEL(external_flash));
	static const uint8_t pattern[4] = {0xDE, 0xAD, 0xBE, 0xEF};
	uint8_t readback[sizeof(pattern)] = {0};
	int ret;

	if (!device_is_ready(ext_flash_dev)) {
		snprintf(result, sizeof(result), "Flash: not ready");
		return result;
	}

	ret = flash_erase(ext_flash_dev, DEV_PAGE_FLASH_TEST_OFFSET,
			  DEV_PAGE_FLASH_TEST_SIZE);
	if (ret == 0) {
		ret = flash_write(ext_flash_dev, DEV_PAGE_FLASH_TEST_OFFSET,
				  pattern, sizeof(pattern));
	}
	if (ret == 0) {
		ret = flash_read(ext_flash_dev, DEV_PAGE_FLASH_TEST_OFFSET,
				 readback, sizeof(readback));
	}

	if (ret == 0 && memcmp(pattern, readback, sizeof(pattern)) == 0) {
		snprintf(result, sizeof(result), "Flash: OK");
	} else {
		snprintf(result, sizeof(result), "Flash: FAIL (%d)", ret);
	}

	return result;
}

static void format_developer_resistance_line(char *buffer, size_t len)
{
#ifdef CICLOTTE
	snprintf(buffer, len, "Servo L%u %udeg %s", servo_level + 1U,
		 servo_level_angle_deg[servo_level],
		 servo_output_enabled ? "on" : "off");
#else
	snprintf(buffer, len, "Resistance raw: %u", resistance_raw);
#endif
}

static void show_developer_page(void)
{
	char line0[24];
	char line1[24];
	const char *flash_status;

	construct_page(PAGE_DEVELOPER);
	display_show_text(SF_LABEL0_DARK, "Developer", strlen("Developer"));

	format_developer_resistance_line(line0, sizeof(line0));
	display_show_text(SF_INFO_LINE0_LIGHT, line0, strlen(line0));

	snprintf(line1, sizeof(line1), "Cadence pin: %d",
		gpio_pin_get_dt(&tick_input));
	display_show_text(SF_INFO_LINE1_LIGHT, line1, strlen(line1));

	flash_status = run_external_flash_test();
	display_show_text(SF_INFO_LINE2_LIGHT, flash_status, strlen(flash_status));
	display_show_text(SF_INFO_LINE3_LIGHT, "", 0);
	display_show_text(SF_INFO_LINE4_LIGHT, "", 0);
}

static void update_developer_page(void)
{
	static uint32_t last_update_ms;
	uint32_t now_ms = k_uptime_get_32();
	char line0[24];
	char line1[24];

	if (active_page == NULL || active_page->page_id != PAGE_DEVELOPER) {
		return;
	}

	if ((uint32_t)(now_ms - last_update_ms) < 250U) {
		return;
	}
	last_update_ms = now_ms;

	format_developer_resistance_line(line0, sizeof(line0));
	display_show_text(SF_INFO_LINE0_LIGHT, line0, strlen(line0));

	snprintf(line1, sizeof(line1), "Cadence pin: %d",
		gpio_pin_get_dt(&tick_input));
	display_show_text(SF_INFO_LINE1_LIGHT, line1, strlen(line1));
}

static void format_hr_bpm(char *buffer, size_t len)
{
	uint8_t bpm = ble_hr_get_bpm();

	if (bpm == 0U) {
		(void)snprintf(buffer, len, "---");
	} else {
		(void)snprintf(buffer, len, "%u", bpm);
	}
}

static uint8_t hr_current_bpm(void)
{
	return ble_hr_get_bpm();
}

static void show_hr_search_control(enum ble_hr_state state)
{
	static bool was_scanning;
	static bool refresh_drawn;
	static uint8_t wait_idx;
	static uint32_t last_wait_ms;
	uint32_t now_ms;

	if (state != BLE_HR_SCANNING) {
		wait_idx = 0;
		last_wait_ms = 0;
		was_scanning = false;
		if (!refresh_drawn) {
			show_button(&hr_connect_buttons[0], false);
			refresh_drawn = true;
		} else {
			display_activate_button(BTN_TILE4T);
		}
		return;
	}

	now_ms = k_uptime_get_32();

	if (!was_scanning) {
		wait_idx = 0;
		last_wait_ms = now_ms;
		was_scanning = true;
		refresh_drawn = false;
		display_show_image(IMG_HALFTILE40_BLANK_NEUTRAL);
		display_show_image(IMG_HALFTILE40_WAIT_HR_1_3);
		display_activate_button(BTN_TILE4T);
		return;
	}

	if ((uint32_t)(now_ms - last_wait_ms) >= 800U) {
		wait_idx = (wait_idx + 1U) % 3U;
		last_wait_ms = now_ms;
		display_show_image(IMG_HALFTILE40_BLANK_NEUTRAL);
		display_show_image(IMG_HALFTILE40_WAIT_HR_1_3 + wait_idx);
	}

	display_activate_button(BTN_TILE4T);
}

static void show_hr_status_lines(void)
{
	char value[8];
	char rssi[8];
	const char *line0 = hr_search_mode_label();
	const char *line1 = "Bereit";
	const char *line2 = "Gurt nah";
	const char *line3 = "druecken";
	const char *line4 = "";
	const char *sensor = ble_hr_get_name();
	enum ble_hr_state state = ble_hr_get_state();

	(void)snprintf(rssi, sizeof(rssi), "---");

	if (hr_search_mode == HR_SEARCH_OFF) {
		line1 = "Puls";
		line2 = "aus";
		line3 = "";
		line4 = "";
		goto show_lines;
	}

	if (!hr_search_mode_has_ble()) {
		line1 = "Puls";
		line2 = "aus";
		line3 = "";
		line4 = "";
		goto show_lines;
	}

	switch (state) {
	case BLE_HR_SCANNING:
		hr_last_pairing_error = 0;
		line1 = "Suche...";
		line2 = "Gurt nah";
		line3 = "halten";
		line4 = "";
		break;
	case BLE_HR_CONNECTING:
		line1 = "Verbinde";
		line2 = sensor;
		line3 = "";
		line4 = "";
		break;
	case BLE_HR_CONNECTED:
		line1 = "Verbunden";
		line2 = sensor;
		(void)snprintf(rssi, sizeof(rssi), "%d dB", ble_hr_get_rssi());
		line3 = rssi;
		line4 = "";
		break;
	case BLE_HR_FAILED:
		line1 = "Kein Gurt";
		line2 = "Nochmal";
		line3 = "BLE";
		if (hr_last_pairing_error != 0) {
			(void)snprintf(rssi, sizeof(rssi), "E%d",
				       hr_last_pairing_error < 0 ?
				       -hr_last_pairing_error :
				       hr_last_pairing_error);
			line4 = rssi;
		} else {
			line4 = "druecken";
		}
		break;
	case BLE_HR_IDLE:
	default:
		break;
	}

show_lines:
	format_hr_bpm(value, sizeof(value));
	display_show_text(SF_LABEL0_DARK, text_heartrate[app_language],
			  strlen(text_heartrate[app_language]));
	display_show_text(SF_VALUE1_LIGHT, value, strlen(value));
	display_show_text(SF_LABEL1_LIGHT, "BPM", strlen("BPM"));
	display_show_text(SF_LABEL2_LIGHT, "", 0);
	display_show_text(SF_VALUE2_LIGHT, "", 0);
	display_show_text(SF_INFO_LINE0_LIGHT, line0, strlen(line0));
	display_show_text(SF_INFO_LINE1_LIGHT, line1, strlen(line1));
	display_show_text(SF_INFO_LINE2_LIGHT, line2, strlen(line2));
	display_show_text(SF_INFO_LINE3_LIGHT, line3, strlen(line3));
	display_show_text(SF_INFO_LINE4_LIGHT, line4, strlen(line4));

	display_show_text(SF_HALFTILE40_LINE1_LIGHT, "", 0);
	display_show_text(SF_HALFTILE40_LINE2_LIGHT, "", 0);
	display_show_text(SF_HALFTILE40_LINE3_LIGHT, "", 0);
	show_hr_search_control(state);
	display_show_image(ble_hr_is_connected() ?
			   IMG_HR_INDICATOR_ICON_ON :
			   IMG_HR_INDICATOR_ICON_OFF);
}

static void show_hr_connect_page(void)
{
	construct_page(PAGE_HR_CONNECT);
	show_hr_status_lines();
}

static void show_bt_error(int err)
{
	char value[8];
	unsigned int code;

	if (err < 0) {
		err = -err;
	}

	code = err > 999 ? 999U : (unsigned int)err;
	(void)snprintf(value, sizeof(value), "E%u", code);
	display_show_text(SF_VALUE3_LIGHT, value, strlen(value));
}

static void show_image_browser_asset(void)
{
	const struct image_browser_asset *asset;

	if (image_browser_index >= ARRAY_SIZE(image_browser_assets)) {
		image_browser_index = 0;
	}

	asset = &image_browser_assets[image_browser_index];
	display_show_image_raw(asset->image_id);
	display_show_text_raw(SF_LABEL1_LIGHT, asset->name, strlen(asset->name));
	display_show_text_raw(SF_LABEL2_DARK, asset->name, strlen(asset->name));
	display_activate_button(BTN_TILE4);
}

static void image_browser_next(void)
{
	image_browser_index = (image_browser_index + 1U) % ARRAY_SIZE(image_browser_assets);
	show_image_browser_asset();
}

static uint32_t clamp_u32(uint32_t value, uint32_t min, uint32_t max)
{
	if (value < min) {
		return min;
	}

	if (value > max) {
		return max;
	}

	return value;
}

static uint32_t current_time_small_step(void)
{
	return custom_time_sec >= SESSION_TIME_THRESH_SEC ?
	       SESSION_SMALL_TIME_STEP_HIGH_SEC :
	       SESSION_SMALL_TIME_STEP_LOW_SEC;
}

static uint32_t current_time_big_step(void)
{
	return custom_time_sec >= SESSION_TIME_THRESH_SEC ?
	       SESSION_BIG_TIME_STEP_HIGH_SEC :
	       SESSION_BIG_TIME_STEP_LOW_SEC;
}

static uint32_t current_distance_small_step(void)
{
	return custom_distance_m >= SESSION_DISTANCE_THRESH_M ?
	       SESSION_SMALL_DISTANCE_STEP_HIGH_M :
	       SESSION_SMALL_DISTANCE_STEP_LOW_M;
}

static uint32_t current_distance_big_step(void)
{
	return custom_distance_m >= SESSION_DISTANCE_THRESH_M ?
	       SESSION_BIG_DISTANCE_STEP_HIGH_M :
	       SESSION_BIG_DISTANCE_STEP_LOW_M;
}

static bool use_imperial_units(void)
{
	return app_language == APP_LANGUAGE_US;
}

static uint32_t meters_to_tenths_mile(uint32_t meters)
{
	return (uint32_t)((((uint64_t)meters * 10U) + (METERS_PER_MILE / 2U)) /
			 METERS_PER_MILE);
}

static uint32_t centi_kmh_to_mph(uint32_t centi_kmh)
{
	return (uint32_t)((((uint64_t)centi_kmh * 621U) + 50000U) / 100000U);
}

static const char *distance_label(void)
{
	return use_imperial_units() ? text_distance_mi :
	       text_session_distance[app_language];
}

static double low_pass(double previous, double input, double alpha)
{
	return previous + alpha * (input - previous);
}

static double bike_speed_from_watts(double power_watts)
{
	const double air_resistance = 0.226366;
	const double total_resistance = 3.871;
	const double transmission = 0.95;
	double velocity = 8.0;

	if (power_watts <= 0.0) {
		return 0.0;
	}

	for (uint8_t i = 0; i < 10; i++) {
		double f = velocity * (air_resistance * velocity * velocity +
				       total_resistance) -
			   transmission * power_watts;
		double fp = 3.0 * air_resistance * velocity * velocity +
			    total_resistance;
		double next = fp == 0.0 ? 0.0 : velocity - (f / fp);
		double diff = next > velocity ? next - velocity : velocity - next;

		if (next < 0.0) {
			next = 0.0;
		}

		velocity = next;
		if (diff < 0.05) {
			break;
		}
	}

	return velocity;
}

static uint16_t calculate_bike_power_watts(uint8_t resistance_percent,
					   uint8_t cadence_rpm)
{
	static bool filter_initialized;
	static double filtered_resistance;
	static double filtered_cadence;
	static double filtered_power;
	double resistance = (double)resistance_percent / 100.0;
	double cadence = (double)cadence_rpm / 60.0;
	double a1;
	double a2;
	double a3;
	double torque;
	double power;

	if (!filter_initialized) {
		filtered_resistance = resistance;
		filtered_cadence = cadence;
		filtered_power = 0.0;
		filter_initialized = true;
	} else {
		filtered_resistance = low_pass(filtered_resistance, resistance, 0.20);
		filtered_cadence = low_pass(filtered_cadence, cadence, 0.35);
	}

	if (cadence_rpm == 0U || filtered_cadence < 0.05) {
		filtered_power = low_pass(filtered_power, 0.0, 0.50);
		return (uint16_t)(filtered_power + 0.5);
	}

	if (filtered_resistance < 0.325) {
		a1 = 76.356 * filtered_resistance * filtered_resistance -
		     4.4448 * filtered_resistance + 4.4206;
		a2 = -1.4699 * filtered_resistance - 2.1172;
		a3 = -34.093 * filtered_resistance * filtered_resistance *
		     filtered_resistance +
		     10.033 * filtered_resistance * filtered_resistance -
		     0.3734 * filtered_resistance + 0.3677;
	} else {
		a1 = 98.857 * filtered_resistance - 22.315;
		a2 = -34.924 * filtered_resistance + 9.2642;
		a3 = 4.0951 * filtered_resistance - 1.174;
	}

	torque = a3 * filtered_cadence * filtered_cadence * filtered_cadence +
		 a2 * filtered_cadence * filtered_cadence +
		 a1 * filtered_cadence;
	if (torque < 0.0) {
		torque = 0.0;
	}

	power = 6.283185307179586 * torque * filtered_cadence;
	filtered_power = low_pass(filtered_power, power, 0.25);

	if (filtered_power > 2000.0) {
		return 2000U;
	}

	return (uint16_t)(filtered_power + 0.5);
}

static uint16_t calculate_bike_speed_centi_kmh(uint16_t power_watts)
{
	static bool filter_initialized;
	static double filtered_speed;
	double speed = bike_speed_from_watts((double)power_watts);

	if (!filter_initialized) {
		filtered_speed = speed;
		filter_initialized = true;
	} else {
		filtered_speed = low_pass(filtered_speed, speed, 0.25);
	}

	if (filtered_speed <= 0.0) {
		return 0U;
	}

	return (uint16_t)(filtered_speed * 360.0 + 0.5);
}

static void normalize_preselection_values(void)
{
	if (custom_time_sec >= SESSION_TIME_THRESH_SEC) {
		custom_time_sec = (custom_time_sec / SESSION_SMALL_TIME_STEP_HIGH_SEC) *
				  SESSION_SMALL_TIME_STEP_HIGH_SEC;
	}

	if (custom_distance_m >= SESSION_DISTANCE_THRESH_M) {
		custom_distance_m = (custom_distance_m / 1000U) * 1000U;
	}
}

static void format_step(char *buffer, size_t len, int32_t step)
{
	uint32_t magnitude = step < 0 ? (uint32_t)-step : (uint32_t)step;
	char sign = step < 0 ? '-' : '+';

	if (magnitude >= 1000U && (magnitude % 1000U) == 0U) {
		(void)snprintf(buffer, len, "%c%uk", sign,
			       (unsigned int)(magnitude / 1000U));
	} else if (magnitude >= 60U && (magnitude % 60U) == 0U) {
		(void)snprintf(buffer, len, "%c%u min", sign,
			       (unsigned int)(magnitude / 60U));
	} else {
		(void)snprintf(buffer, len, "%c%u m", sign,
			       (unsigned int)magnitude);
	}
}

static void format_distance_step(char *buffer, size_t len, int32_t step_m)
{
	uint32_t magnitude = step_m < 0 ? (uint32_t)-step_m : (uint32_t)step_m;
	char sign = step_m < 0 ? '-' : '+';
	uint32_t tenths_mile;

	if (!use_imperial_units()) {
		format_step(buffer, len, step_m);
		return;
	}

	tenths_mile = meters_to_tenths_mile(magnitude);
	if (tenths_mile >= 10U && (tenths_mile % 10U) == 0U) {
		(void)snprintf(buffer, len, "%c%u", sign,
			       (unsigned int)(tenths_mile / 10U));
	} else {
		(void)snprintf(buffer, len, "%c%u.%u", sign,
			       (unsigned int)(tenths_mile / 10U),
			       (unsigned int)(tenths_mile % 10U));
	}
}

static void update_preselection_button_texts(void)
{
	format_step(text_time_inc_big, sizeof(text_time_inc_big),
		    (int32_t)current_time_big_step());
	format_step(text_time_inc_small, sizeof(text_time_inc_small),
		    (int32_t)current_time_small_step());
	format_step(text_time_dec_small, sizeof(text_time_dec_small),
		    -(int32_t)current_time_small_step());
	format_step(text_time_dec_big, sizeof(text_time_dec_big),
		    -(int32_t)current_time_big_step());

	format_distance_step(text_distance_inc_big, sizeof(text_distance_inc_big),
			     (int32_t)current_distance_big_step());
	format_distance_step(text_distance_inc_small, sizeof(text_distance_inc_small),
			     (int32_t)current_distance_small_step());
	format_distance_step(text_distance_dec_small, sizeof(text_distance_dec_small),
			     -(int32_t)current_distance_small_step());
	format_distance_step(text_distance_dec_big, sizeof(text_distance_dec_big),
			     -(int32_t)current_distance_big_step());
}

static void format_preselection_time(char *buffer, size_t len)
{
	if (custom_time_sec >= 3600U) {
		(void)snprintf(buffer, len, "%02u:%02u",
			       (unsigned int)(custom_time_sec / 3600U),
			       (unsigned int)((custom_time_sec / 60U) % 60U));
	} else {
		(void)snprintf(buffer, len, "%u",
			       (unsigned int)(custom_time_sec / 60U));
	}
}

static void format_distance_value(char *buffer, size_t len, uint32_t distance_m)
{
	if (use_imperial_units()) {
		uint32_t hundredths_mile =
			(uint32_t)((((uint64_t)distance_m * 100U) +
				    (METERS_PER_MILE / 2U)) /
				   METERS_PER_MILE);

		if (hundredths_mile >= 10000U) {
			(void)snprintf(buffer, len, "%u",
				       (unsigned int)((hundredths_mile + 50U) / 100U));
		} else if (hundredths_mile >= 1000U) {
			(void)snprintf(buffer, len, "%u.%u",
				       (unsigned int)(hundredths_mile / 100U),
				       (unsigned int)((hundredths_mile / 10U) % 10U));
		} else {
			(void)snprintf(buffer, len, "%u.%02u",
				       (unsigned int)(hundredths_mile / 100U),
				       (unsigned int)(hundredths_mile % 100U));
		}
		return;
	}

	if (distance_m >= 100000U) {
		(void)snprintf(buffer, len, "%uk",
			       (unsigned int)((distance_m + 500U) / 1000U));
	} else if (distance_m >= 10000U) {
		(void)snprintf(buffer, len, "%u.%uk",
			       (unsigned int)(distance_m / 1000U),
			       (unsigned int)((distance_m % 1000U) / 100U));
	} else if (distance_m >= 1000U) {
		(void)snprintf(buffer, len, "%u.%02uk",
			       (unsigned int)(distance_m / 1000U),
			       (unsigned int)((distance_m % 1000U) / 10U));
	} else {
		(void)snprintf(buffer, len, "%um",
			       (unsigned int)distance_m);
	}
}

static void update_preselection_page(void)
{
	char value[12];
	const char *unit;

	if (active_page == NULL) {
		return;
	}

	normalize_preselection_values();
	update_preselection_button_texts();

	if (active_page->page_id == PAGE_TIME_SELECTION) {
		format_preselection_time(value, sizeof(value));
		unit = custom_time_sec >= 3600U ? text_session_time[app_language] :
		       text_minutes[app_language];
	} else if (active_page->page_id == PAGE_DISTANCE_SELECTION) {
		format_distance_value(value, sizeof(value), custom_distance_m);
		unit = use_imperial_units() ? distance_label() :
		       text_session_distance[app_language];
	} else {
		return;
	}

	display_show_text(SF_VALUE2_LIGHT, value, strlen(value));
	display_show_text(SF_LABEL2_LIGHT, unit, strlen(unit));

	for (size_t i = 0; i < active_page->button_count; i++) {
		show_button(&active_page->buttons[i], false);
	}
}

static void show_preselection_page(enum app_page_id page_id)
{
	update_preselection_button_texts();
	construct_page(page_id);
	update_preselection_page();
}

#ifdef CICLOTTE
static uint8_t servo_level_resistance_percent(void)
{
	/* Same mapping as the AVR firmware: level 1..10 -> 10..100 %. */
	return (uint8_t)((servo_level + 1U) * 10U);
}

static int servo_apply(void)
{
	uint32_t angle = servo_level_angle_deg[servo_level];
	uint32_t pulse_us = SERVO_PULSE_MIN_US +
			    (angle * (SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US)) /
			    SERVO_MAX_ANGLE_DEG;

	if (!pwm_is_ready_dt(&brake_servo)) {
		return -ENODEV;
	}

	/* A 0 ns pulse keeps the line low, so the servo stops holding. */
	return pwm_set_pulse_dt(&brake_servo,
				servo_output_enabled ? PWM_USEC(pulse_us) : 0U);
}

static void servo_set_enabled(bool enabled)
{
	servo_output_enabled = enabled;
	(void)servo_apply();
}

/* The servo brake has a fixed travel, so there is nothing to calibrate;
 * report "calibrated" to the NOHRD app and reject calibration writes. */
static bool custom_ble_resistance_calibrated(void)
{
	return true;
}

static int custom_ble_set_resistance_calibration(uint8_t value_id,
						 const uint8_t *data,
						 size_t len)
{
	ARG_UNUSED(value_id);
	ARG_UNUSED(data);
	ARG_UNUSED(len);

	return -ENOTSUP;
}
#else
static uint8_t normalize_resistance_percent(uint16_t raw)
{
	int32_t span;
	int32_t percent;

	if (!resistance_calibrated ||
	    resistance_cal_raw_100 == resistance_cal_raw_0) {
		return (uint8_t)((raw * 100U) / 4095U);
	}

	span = (int32_t)resistance_cal_raw_0 - resistance_cal_raw_100;
	percent = 100 - (((int32_t)raw - resistance_cal_raw_100) * 100) / span;

	if (percent < 0) {
		return 0;
	}

	if (percent > 100) {
		return 100;
	}

	return (uint8_t)percent;
}

static int save_resistance_calibration(void)
{
	struct resistance_cal_data data = {
		.magic = RESISTANCE_CAL_MAGIC,
		.raw_100 = resistance_cal_raw_100,
		.raw_0 = resistance_cal_raw_0,
	};

	if (!resistance_nvs_ready) {
		return -ENODEV;
	}

	return nvs_write(&resistance_nvs, RESISTANCE_CAL_NVS_ID,
			 &data, sizeof(data));
}

static int clear_resistance_calibration(void)
{
	int ret = 0;

	resistance_cal_raw_100 = 210;
	resistance_cal_raw_0 = 980;
	resistance_calibrated = false;
	measured_resistance_percent = 0;

	if (resistance_nvs_ready) {
		ret = nvs_delete(&resistance_nvs, RESISTANCE_CAL_NVS_ID);
		if (ret == -ENOENT) {
			ret = 0;
		}
	}

	return ret;
}

static bool custom_ble_resistance_calibrated(void)
{
	return resistance_calibrated;
}

static int custom_ble_set_resistance_calibration(uint8_t value_id,
						 const uint8_t *data,
						 size_t len)
{
	uint16_t raw;

	if (data == NULL || len < sizeof(raw)) {
		return -EINVAL;
	}

	raw = (uint16_t)data[0] | ((uint16_t)data[1] << 8);

	switch (value_id) {
	case 0x00:
	case 0x01:
		resistance_cal_raw_100 = raw;
		break;
	case 0x02:
		resistance_cal_raw_0 = raw;
		break;
	default:
		return -EINVAL;
	}

	resistance_calibrated = resistance_cal_raw_100 != resistance_cal_raw_0;
	return save_resistance_calibration();
}
#endif

static bool session_ring_preferences_valid(const struct session_ring_pref_data *data)
{
	if (data->magic != SESSION_RING_PREF_MAGIC) {
		return false;
	}

	for (size_t i = 0; i < ARRAY_SIZE(data->idx); i++) {
		if (data->idx[i] >= SESSION_METRIC_COUNT) {
			return false;
		}
	}

	return true;
}

static int save_session_ring_preferences(void)
{
	struct session_ring_pref_data data = {
		.magic = SESSION_RING_PREF_MAGIC,
		.idx = {
			session_rings[0].idx,
			session_rings[1].idx,
			session_rings[2].idx,
		},
	};

	if (!resistance_nvs_ready) {
		return -ENODEV;
	}

	return nvs_write(&resistance_nvs, SESSION_RING_PREF_NVS_ID,
			 &data, sizeof(data));
}

static bool ui_settings_valid(const struct ui_settings_data *data)
{
	return data->magic == UI_SETTINGS_MAGIC &&
	       data->language < ARRAY_SIZE(text_language) &&
	       data->darkmode <= 1U &&
	       data->bt_enabled <= 1U;
}

static int save_ui_settings(void)
{
	struct ui_settings_data data = {
		.magic = UI_SETTINGS_MAGIC,
		.language = app_language,
		.darkmode = app_darkmode ? 1U : 0U,
		.bt_enabled = ble_ftms_is_enabled() ? 1U : 0U,
		.hr_search_mode = (uint8_t)hr_search_mode,
	};

	if (!resistance_nvs_ready) {
		return -ENODEV;
	}

	return nvs_write(&resistance_nvs, UI_SETTINGS_NVS_ID,
			 &data, sizeof(data));
}

static void apply_session_ring_preferences(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(session_rings); i++) {
		session_rings[i].idx = session_ring_pref[i];
	}
}

static uint8_t summary_metric_for_session_metric(uint8_t metric)
{
	switch (metric) {
	case SESSION_METRIC_WATTS:
		return SUMMARY_METRIC_AVG_WATTS;
	case SESSION_METRIC_SPEED:
		return SUMMARY_METRIC_AVG_SPEED;
	case SESSION_METRIC_PACE:
		return SUMMARY_METRIC_AVG_PACE;
	case SESSION_METRIC_TIME:
		return SUMMARY_METRIC_TIME;
	case SESSION_METRIC_DISTANCE:
		return SUMMARY_METRIC_DISTANCE;
	case SESSION_METRIC_CALORIES:
		return SUMMARY_METRIC_CALORIES;
	case SESSION_METRIC_HEARTRATE:
		return SUMMARY_METRIC_HEARTRATE;
	case SESSION_METRIC_RESISTANCE:
		return SUMMARY_METRIC_MAX_HEARTRATE;
	case SESSION_METRIC_CADENCE:
	default:
		return SUMMARY_METRIC_AVG_HEARTRATE;
	}
}

#ifndef CICLOTTE
static void show_calibration_page(uint8_t step)
{
	const char *line0 = text_cal_line0[app_language];
	const char *line1 = text_cal_line1[app_language];
	const char *line2 = text_cal_line2[app_language];
	const char *line3 = text_cal_line3_max[app_language];
	const char *line4 = text_cal_line4_max[app_language];
	char value[8];

	if (step == 1U) {
		calibration_restore_darkmode = app_darkmode;
	}
	app_darkmode = false;
	calibration_step = step;

	calibration_button.image_id = IMG_TILE4_BLANK_LIGHT;
	calibration_button.press_image_id = IMG_TILE4_FRAME_LIGHT;
	calibration_button.text_field_id = SF_LABEL4_LIGHT;

	if (step == 1U) {
		calibration_button.action_id = BTN_ACTION_GOTO_CALIBRATE_DOWN;
		calibration_button.localized_text = text_next;
		calibration_button.top_layer = IMG_TILE4_NEXT_ICON_LIGHT;
	} else if (step == 2U) {
		line3 = text_cal_line3_min[app_language];
		line4 = text_cal_line4_min[app_language];
		calibration_button.action_id = BTN_ACTION_GOTO_CALIBRATE_END;
		calibration_button.localized_text = text_next;
		calibration_button.top_layer = IMG_TILE4_NEXT_ICON_LIGHT;
	} else {
		line0 = text_cal_line0_end[app_language];
		line1 = text_cal_line1_end[app_language];
		line2 = "";
		line3 = "";
		line4 = "";
		calibration_button.action_id = BTN_ACTION_RETURN_FROM_CALIBRATE;
		calibration_button.localized_text = text_home;
		calibration_button.top_layer = IMG_TILE4_HOME_LIGHT;
	}

	construct_page(PAGE_CALIBRATION);

	if (step == 1U) {
		display_show_gif(GIF_CALIBRATE_UP);
		(void)snprintf(value, sizeof(value), "1/2");
	} else if (step == 2U) {
		display_show_gif(GIF_CALIBRATE_DOWN);
		(void)snprintf(value, sizeof(value), "2/2");
	} else {
		(void)snprintf(value, sizeof(value), "OK");
	}

	display_show_text(SF_LABEL0_DARK, text_calibration[app_language],
			  strlen(text_calibration[app_language]));
	display_show_text(SF_INFO_LINE0_LIGHT, line0, strlen(line0));
	display_show_text(SF_INFO_LINE1_LIGHT, line1, strlen(line1));
	display_show_text(SF_INFO_LINE2_LIGHT, line2, strlen(line2));
	display_show_text(SF_INFO_LINE3_LIGHT, line3, strlen(line3));
	display_show_text(SF_INFO_LINE4_LIGHT, line4, strlen(line4));
	display_show_text(SF_LABEL3_LIGHT, text_step[app_language],
			  strlen(text_step[app_language]));
	display_show_text(SF_VALUE3_LIGHT, value, strlen(value));
}
#endif

static unsigned int resistance_display_value(void)
{
#ifdef CICLOTTE
	/* Shown as brake level 1..10, like the AVR Ciclotte. */
	return session.resistance_percent / 10U;
#else
	return session.resistance_percent;
#endif
}

static void format_duration(char *buffer, size_t len, uint32_t seconds)
{
	if (seconds >= 3600U) {
		(void)snprintf(buffer, len, "%uh%02um",
			       (unsigned int)(seconds / 3600U),
			       (unsigned int)((seconds / 60U) % 60U));
	} else {
		(void)snprintf(buffer, len, "%02u:%02u",
			       (unsigned int)(seconds / 60U),
			       (unsigned int)(seconds % 60U));
	}
}

static void format_pace_mmss(char *buffer, size_t len, uint32_t seconds)
{
	uint32_t minutes = seconds / 60U;
	uint32_t secs = seconds % 60U;

	if (seconds > MAX_DISPLAY_PACE_SEC) {
		(void)snprintf(buffer, len, "--:--");
		return;
	}

	if (minutes > 999U) {
		minutes = 999U;
		secs = 59U;
	}

	if (minutes >= 100U) {
		(void)snprintf(buffer, len, "%03u:%02u",
			       (unsigned int)minutes,
			       (unsigned int)secs);
	} else {
		(void)snprintf(buffer, len, "%02u:%02u",
			       (unsigned int)minutes,
			       (unsigned int)secs);
	}
}

static void format_session_metric(uint8_t metric, char *buffer, size_t len)
{
	uint32_t pace_seconds;
	uint32_t display_seconds;
	uint32_t display_distance;

	switch (metric) {
	case SESSION_METRIC_WATTS:
		(void)snprintf(buffer, len, "%u", session.watts);
		break;
	case SESSION_METRIC_CADENCE:
		(void)snprintf(buffer, len, "%u", session.cadence_rpm);
		break;
	case SESSION_METRIC_RESISTANCE:
		(void)snprintf(buffer, len, "%u", resistance_display_value());
		break;
	case SESSION_METRIC_SPEED:
		if (use_imperial_units()) {
			(void)snprintf(buffer, len, "%u",
				       (unsigned int)centi_kmh_to_mph(session.speed_centi_kmh));
		} else {
			(void)snprintf(buffer, len, "%u",
				       (unsigned int)((session.speed_centi_kmh + 50U) / 100U));
		}
		break;
	case SESSION_METRIC_PACE:
		if (session.speed_centi_kmh == 0U) {
			(void)snprintf(buffer, len, "--:--");
			break;
		}
		pace_seconds = use_imperial_units() ?
			       579364U / session.speed_centi_kmh :
			       360000U / session.speed_centi_kmh;
		format_pace_mmss(buffer, len, pace_seconds);
		break;
	case SESSION_METRIC_TIME:
		format_duration(buffer, len, session.elapsed_sec);
		break;
	case SESSION_METRIC_DISTANCE:
		format_distance_value(buffer, len, session.distance_m);
		break;
	case SESSION_METRIC_CALORIES:
		(void)snprintf(buffer, len, "%u", session.calories);
		break;
	case SESSION_METRIC_HEARTRATE:
		if (session.heartrate_bpm == 0U) {
			(void)snprintf(buffer, len, "---");
		} else {
			(void)snprintf(buffer, len, "%u", session.heartrate_bpm);
		}
		break;
	default:
		(void)snprintf(buffer, len, "---");
		break;
	}

	if (metric == SESSION_METRIC_TIME && session.mode == SESSION_TIME) {
		display_seconds = session.elapsed_sec >= custom_time_sec ?
				  0U : custom_time_sec - session.elapsed_sec;
		format_duration(buffer, len, display_seconds);
	}

	if (metric == SESSION_METRIC_DISTANCE && session.mode == SESSION_DISTANCE) {
		display_distance = session.distance_m >= custom_distance_m ?
				   0U : custom_distance_m - session.distance_m;
		format_distance_value(buffer, len, display_distance);
	}
}

static void format_summary_metric(uint8_t metric, char *buffer, size_t len)
{
	uint32_t avg_watts;
	uint32_t avg_heartrate;
	uint32_t avg_speed_centi_kmh;
	uint32_t pace_seconds;

	switch (metric) {
	case SUMMARY_METRIC_AVG_WATTS:
		avg_watts = session.elapsed_sec == 0U ? 0U :
			    session.watt_seconds / session.elapsed_sec;
		(void)snprintf(buffer, len, "%u", (unsigned int)avg_watts);
		break;
	case SUMMARY_METRIC_MAX_HEARTRATE:
		if (session.max_heartrate_bpm == 0U) {
			(void)snprintf(buffer, len, "---");
		} else {
			(void)snprintf(buffer, len, "%u", session.max_heartrate_bpm);
		}
		break;
	case SUMMARY_METRIC_AVG_HEARTRATE:
		if (session.heartrate_samples == 0U) {
			(void)snprintf(buffer, len, "---");
		} else {
			avg_heartrate = session.heartrate_sum /
					session.heartrate_samples;
			(void)snprintf(buffer, len, "%u", (unsigned int)avg_heartrate);
		}
		break;
	case SUMMARY_METRIC_AVG_SPEED:
		if (session.elapsed_sec == 0U) {
			(void)snprintf(buffer, len, "---");
		} else {
			avg_speed_centi_kmh = (uint32_t)(((uint64_t)session.distance_m *
							   360U) /
							  session.elapsed_sec);
			if (use_imperial_units()) {
				(void)snprintf(buffer, len, "%u",
					       (unsigned int)centi_kmh_to_mph(avg_speed_centi_kmh));
			} else {
				(void)snprintf(buffer, len, "%u",
					       (unsigned int)((avg_speed_centi_kmh + 50U) / 100U));
			}
		}
		break;
	case SUMMARY_METRIC_AVG_PACE:
		if (session.distance_m == 0U) {
			(void)snprintf(buffer, len, "--:--");
		} else {
			pace_seconds = use_imperial_units() ?
				       (session.elapsed_sec * METERS_PER_MILE) /
				       session.distance_m :
				       (session.elapsed_sec * 1000U) /
				       session.distance_m;
			format_pace_mmss(buffer, len, pace_seconds);
		}
		break;
	case SUMMARY_METRIC_TIME:
		format_duration(buffer, len, session.elapsed_sec);
		break;
	case SUMMARY_METRIC_DISTANCE:
		format_distance_value(buffer, len, session.distance_m);
		break;
	case SUMMARY_METRIC_CALORIES:
		(void)snprintf(buffer, len, "%u", session.calories);
		break;
	case SUMMARY_METRIC_HEARTRATE:
		if (session.heartrate_bpm == 0U) {
			(void)snprintf(buffer, len, "---");
		} else {
			(void)snprintf(buffer, len, "%u", session.heartrate_bpm);
		}
		break;
	default:
		(void)snprintf(buffer, len, "---");
		break;
	}
}

static void update_session_ring(const struct session_ring *ring, bool rebuild_tile)
{
	char value[12];
	const char *label;

	if (rebuild_tile) {
		display_show_image(ring->blank_image);
		display_show_image(ring->first_image + ring->idx);
		label = ring->idx == SESSION_METRIC_DISTANCE ?
			distance_label() :
			session_metric_labels[ring->idx][app_language];
		display_show_text(ring->label_field, label, strlen(label));
	}

	format_session_metric(ring->idx, value, sizeof(value));
	display_show_text(ring->value_field, value, strlen(value));
}

static void update_summary_ring(const struct session_ring *ring, bool rebuild_tile)
{
	char value[12];
	const char *label;

	if (rebuild_tile) {
		display_show_image(ring->blank_image);
		display_show_image(ring->first_image + ring->idx);
		label = ring->idx == SUMMARY_METRIC_DISTANCE ?
			distance_label() :
			summary_metric_labels[ring->idx][app_language];
		display_show_text(ring->label_field, label, strlen(label));
	}

	format_summary_metric(ring->idx, value, sizeof(value));
	display_show_text(ring->value_field, value, strlen(value));
}

static void update_session_header(void)
{
	char value[12];
	const char *label = text_session_time[app_language];

	if (session.mode == SESSION_DISTANCE) {
		label = distance_label();
	}

	display_show_text(SF_LABEL0_DARK, label, strlen(label));

	if (session.mode == SESSION_DISTANCE) {
		format_session_metric(SESSION_METRIC_DISTANCE, value, sizeof(value));
	} else {
		format_session_metric(SESSION_METRIC_TIME, value, sizeof(value));
	}

	display_show_text(SF_VALUE0_DARK, value, strlen(value));
}

static void update_session_values(void)
{
	if (active_page == NULL || active_page->page_id != PAGE_SESSION) {
		return;
	}

	update_session_header();

	for (size_t i = 0; i < ARRAY_SIZE(session_rings); i++) {
		update_session_ring(&session_rings[i], false);
	}
}

static void update_summary_values(void)
{
	if (active_page == NULL || active_page->page_id != PAGE_SUMMARY) {
		return;
	}

	for (size_t i = 0; i < ARRAY_SIZE(summary_rings); i++) {
		update_summary_ring(&summary_rings[i], false);
	}
}

static void render_session_page(void)
{
	construct_page(PAGE_SESSION);
	display_show_image(session.mode == SESSION_DISTANCE ? IMG_TILE0_DISTANCE : IMG_TILE0_TIME);
	for (size_t i = 0; i < ARRAY_SIZE(session_rings); i++) {
		update_session_ring(&session_rings[i], true);
	}
	update_session_values();
}

static void update_bt_connection_values(void)
{
	char value[12];

	if (active_page == NULL || active_page->page_id != PAGE_BT_CONNECTION) {
		return;
	}

	(void)snprintf(value, sizeof(value), "%u", session.watts);
	display_show_text(SF_VALUE1_LIGHT, value, strlen(value));

	(void)snprintf(value, sizeof(value), "%u", resistance_display_value());
	display_show_text(SF_VALUE2_LIGHT, value, strlen(value));

	(void)snprintf(value, sizeof(value), "%u", session.heartrate_bpm);
	display_show_text(SF_VALUE3_LIGHT, value, strlen(value));
}

static void render_bt_connection_page(void)
{
	construct_page(PAGE_BT_CONNECTION);
	display_show_text(SF_LABEL0_DARK, text_bt[app_language],
			  strlen(text_bt[app_language]));
	display_show_text(SF_LABEL1_LIGHT, text_watts[app_language],
			  strlen(text_watts[app_language]));
	display_show_text(SF_LABEL2_LIGHT, text_resistance[app_language],
			  strlen(text_resistance[app_language]));
	display_show_text(SF_LABEL3_LIGHT, text_heartrate[app_language],
			  strlen(text_heartrate[app_language]));
	update_bt_connection_values();
}

static void bt_disconnect_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	(void)ble_ftms_disconnect();
}

static void start_bt_connection_session(void)
{
	(void)k_work_cancel_delayable(&bt_disconnect_work);
	(void)ble_ftms_notify_session_started();

	bt_connection_session_active = true;
	countdown_active = false;
	session.mode = SESSION_FREE;
	session.active = true;
	session.paused = false;
	session.summary_active = false;
	session.elapsed_sec = 0;
	session.distance_m = 0;
	session.watt_seconds = 0;
	session.heartrate_sum = 0;
	session.heartrate_samples = 0;
	session.inactive_sec = 0;
	session.calories = 0;
	session.heartrate_bpm = 0;
	session.max_heartrate_bpm = 0;

#ifdef CICLOTTE
	servo_set_enabled(true);
#endif
	render_bt_connection_page();
}

static void end_bt_connection_session(void)
{
	(void)k_work_cancel_delayable(&bt_disconnect_work);

#ifdef CICLOTTE
	servo_set_enabled(false);
#endif
	bt_connection_session_active = false;
	session.active = false;
	session.paused = false;
	session.mode = SESSION_NONE;
	session.summary_active = false;
	construct_page(PAGE_MAIN_MENU);
}

static void handle_ftms_connection_state(void)
{
	bool connected = ble_ftms_is_connected();

	if (connected && !last_ftms_connected) {
		start_bt_connection_session();
	} else if (!connected && last_ftms_connected) {
		end_bt_connection_session();
	}

	last_ftms_connected = connected;
}

static void start_session(enum session_mode mode)
{
	session.mode = mode;
	session.active = true;
	session.paused = false;
	session.summary_active = false;
	session.elapsed_sec = 0;
	session.distance_m = 0;
	session.watt_seconds = 0;
	session.heartrate_sum = 0;
	session.heartrate_samples = 0;
	session.inactive_sec = 0;
	session.calories = 0;
	session.heartrate_bpm = 0;
	session.max_heartrate_bpm = 0;

	apply_session_ring_preferences();

#ifdef CICLOTTE
	servo_set_enabled(true);
#endif
	render_session_page();
}

static void show_summary(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(session_rings); i++) {
		session_ring_pref[i] = session_rings[i].idx;
	}
	(void)save_session_ring_preferences();

#ifdef CICLOTTE
	servo_set_enabled(false);
#endif

	session.active = false;
	session.paused = false;
	session.mode = SESSION_NONE;
	session.summary_active = true;

	for (size_t i = 0; i < ARRAY_SIZE(summary_rings); i++) {
		summary_rings[i].idx =
			summary_metric_for_session_metric(session_rings[i].idx);
	}

	construct_page(PAGE_SUMMARY);
	display_show_text(SF_LABEL0_DARK, text_summary[app_language],
			  strlen(text_summary[app_language]));
	for (size_t i = 0; i < ARRAY_SIZE(summary_rings); i++) {
		update_summary_ring(&summary_rings[i], true);
	}
}

static void stop_session(void)
{
	show_summary();
}

static void show_stop_confirmation(void)
{
	confirm_stop_was_paused = session.paused;
	session.paused = true;
	construct_page(PAGE_CONFIRM_STOP);
	display_show_text(SF_HALFTILE40_LINE2_DARK, text_quit[app_language],
			  strlen(text_quit[app_language]));
}

static void cancel_stop_confirmation(void)
{
	session.paused = confirm_stop_was_paused;
	render_session_page();
}

static void leave_summary(void)
{
	session.summary_active = false;
	construct_page(PAGE_MAIN_MENU);
}

static void rebuild_active_page_after_wake(enum app_page_id page_id)
{
	switch (page_id) {
	case PAGE_TIME_SELECTION:
	case PAGE_DISTANCE_SELECTION:
		show_preselection_page(page_id);
		break;
	case PAGE_HR_CONNECT:
		show_hr_connect_page();
		break;
	case PAGE_INFORMATION:
		show_information_page();
		break;
	case PAGE_DEVELOPER:
		show_developer_page();
		break;
	case PAGE_IMAGE_BROWSER:
		construct_page(PAGE_IMAGE_BROWSER);
		show_image_browser_asset();
		break;
	case PAGE_SESSION:
		render_session_page();
		break;
	case PAGE_BT_CONNECTION:
		render_bt_connection_page();
		break;
	case PAGE_SUMMARY:
		show_summary();
		break;
	case PAGE_CONFIRM_STOP:
		show_stop_confirmation();
		break;
	case PAGE_MAIN_MENU:
	case PAGE_SETTINGS_1:
	case PAGE_SETTINGS_2:
	case PAGE_SETUP:
	case PAGE_CALIBRATION:
	default:
		construct_page(page_id);
		break;
	}
}

static void display_show_wake_splash(void)
{
	display_show_image_raw(IMG_BG_SS_1 + wake_splash_index);
	wake_splash_index = (wake_splash_index + 1U) % 4U;
	display_set_brightness(DISPLAY_BRIGHTNESS_FULL);
	k_msleep(DISPLAY_WAKE_SPLASH_MS);
}

static void display_show_boot_splash(void)
{
	display_show_image_raw(IMG_BG_BOOT);
	display_set_brightness(DISPLAY_BRIGHTNESS_FULL);
	k_msleep(DISPLAY_WAKE_SPLASH_MS);
}

static void wake_display(void)
{
	enum app_page_id page_id = PAGE_MAIN_MENU;

	if (active_page != NULL) {
		page_id = active_page->page_id;
	}

	display_power_state = DISPLAY_POWER_ACTIVE;
	display_last_activity_ms = k_uptime_get_32();
	display_show_wake_splash();
	display_last_activity_ms = k_uptime_get_32();
	rebuild_active_page_after_wake(page_id);
}

static bool display_standby_allowed(void)
{
	return active_page != NULL &&
	       !countdown_active &&
	       !session.active &&
	       !session.summary_active &&
	       active_page->page_id != PAGE_BT_CONNECTION;
}

static void display_note_activity(void)
{
	display_last_activity_ms = k_uptime_get_32();

	if (display_power_state != DISPLAY_POWER_ACTIVE) {
		wake_display();
	}
}

static void display_power_update(void)
{
	uint32_t now_ms = k_uptime_get_32();
	uint32_t elapsed_ms;

	if (cadence_wake_pending) {
		cadence_wake_pending = false;
		if (display_power_state != DISPLAY_POWER_ACTIVE) {
			wake_display();
			return;
		}
		display_last_activity_ms = now_ms;
	}

	if (!display_standby_allowed()) {
		if (display_power_state != DISPLAY_POWER_ACTIVE) {
			wake_display();
			return;
		}
		display_last_activity_ms = now_ms;
		return;
	}

	elapsed_ms = now_ms - display_last_activity_ms;

	if (elapsed_ms >= DISPLAY_IDLE_OFF_MS &&
	    display_power_state != DISPLAY_POWER_OFF) {
		deactivate_active_page_controls();
		display_set_brightness(DISPLAY_BRIGHTNESS_OFF);
		display_activate_button(BTN_WAKEUP);
		display_power_state = DISPLAY_POWER_OFF;
#ifdef CICLOTTE
		/* AVR equivalent: PWM off when entering deep sleep. */
		servo_set_enabled(false);
#endif
		return;
	}

	if (elapsed_ms >= DISPLAY_IDLE_DIM_MS &&
	    display_power_state == DISPLAY_POWER_ACTIVE) {
		deactivate_active_page_controls();
		display_set_brightness(DISPLAY_BRIGHTNESS_DIM);
		display_activate_button(BTN_WAKEUP);
		display_power_state = DISPLAY_POWER_DIM;
	}
}

static void cycle_session_ring(uint8_t ring_index, int8_t direction)
{
	struct session_ring *ring;

	if (ring_index >= ARRAY_SIZE(session_rings)) {
		return;
	}

	ring = &session_rings[ring_index];
	if (direction < 0) {
		ring->idx = ring->idx == 0U ?
			    SESSION_METRIC_COUNT - 1U : ring->idx - 1U;
	} else {
		ring->idx = (ring->idx + 1U) % SESSION_METRIC_COUNT;
	}

	update_session_ring(ring, true);
}

static void cycle_summary_ring(uint8_t ring_index, int8_t direction)
{
	struct session_ring *ring;

	if (ring_index >= ARRAY_SIZE(summary_rings)) {
		return;
	}

	ring = &summary_rings[ring_index];
	if (direction < 0) {
		ring->idx = ring->idx == 0U ?
			    SUMMARY_METRIC_COUNT - 1U : ring->idx - 1U;
	} else {
		ring->idx = (ring->idx + 1U) % SUMMARY_METRIC_COUNT;
	}

	update_summary_ring(ring, true);
}

static void show_countdown_value(void)
{
	char value[4];

	(void)snprintf(value, sizeof(value), "%u", countdown_value);
	display_show_text(SF_LABEL2_LIGHT, text_start[app_language],
			  strlen(text_start[app_language]));
	display_show_text(SF_VALUE2_LIGHT, value, strlen(value));
}

static void start_session_countdown(enum session_mode mode)
{
	countdown_active = true;
	countdown_mode = (uint8_t)mode;
	countdown_value = 3;
	countdown_last_ms = k_uptime_get_32();

	display_show_image(IMG_BG_GRID_LIGHT);
	display_show_image(IMG_TILE2_BLANK_LIGHT);
	show_countdown_value();
}

static void countdown_update(void)
{
	uint32_t now_ms;

	if (!countdown_active) {
		return;
	}

	now_ms = k_uptime_get_32();
	if ((uint32_t)(now_ms - countdown_last_ms) < 1000U) {
		return;
	}

	countdown_last_ms = now_ms;

	if (countdown_value > 1U) {
		countdown_value--;
		show_countdown_value();
		return;
	}

	countdown_active = false;
	start_session((enum session_mode)countdown_mode);
}

static void update_hr_connect_page(void)
{
	static uint32_t last_update_ms;
	uint32_t now_ms = k_uptime_get_32();

	ble_hr_process();

	if (active_page == NULL || active_page->page_id != PAGE_HR_CONNECT) {
		return;
	}

	if ((uint32_t)(now_ms - last_update_ms) < 500U) {
		return;
	}

	last_update_ms = now_ms;
	show_hr_status_lines();
}

#ifdef CICLOTTE
static void servo_change_level(int8_t delta)
{
	if (delta > 0 && servo_level < SERVO_LEVEL_COUNT - 1U) {
		servo_level++;
	} else if (delta < 0 && servo_level > 0U) {
		servo_level--;
	}

	(void)servo_apply();
	measured_resistance_percent = servo_level_resistance_percent();
	session.resistance_percent = measured_resistance_percent;

	if (bt_connection_session_active) {
		update_bt_connection_values();
	} else {
		update_session_values();
	}
}
#endif

static void run_button_action(uint8_t action_id)
{
	int ret;
	enum ble_hr_state hr_state;

	switch (action_id) {
	case BTN_ACTION_TOGGLE_LANGUAGE:
		app_language = (app_language + 1) % ARRAY_SIZE(text_language);
		(void)save_ui_settings();
		construct_page(active_page->page_id);
		break;
	case BTN_ACTION_TOGGLE_LDMODE:
		app_darkmode = !app_darkmode;
		(void)save_ui_settings();
		construct_page(active_page->page_id);
		break;
	case BTN_ACTION_TOGGLE_BLUETOOTH:
		ret = ble_ftms_set_enabled(!ble_ftms_is_enabled());
		if (ret == 0) {
			app_bt_enabled = ble_ftms_is_enabled();
			(void)save_ui_settings();
		}
		construct_page(active_page->page_id);
		if (ret != 0 && active_page != NULL &&
		    active_page->page_id == PAGE_SETTINGS_2) {
			show_bt_error(ret);
		}
		break;
	case BTN_ACTION_GOTO_FREE_SESSION:
		start_session_countdown(SESSION_FREE);
		break;
	case BTN_ACTION_START_TIME_SESSION:
		start_session_countdown(SESSION_TIME);
		break;
	case BTN_ACTION_START_DISTANCE_SESSION:
		start_session_countdown(SESSION_DISTANCE);
		break;
	case BTN_ACTION_GOTO_TIME_SELECTION:
		show_preselection_page(PAGE_TIME_SELECTION);
		break;
	case BTN_ACTION_GOTO_DISTANCE_SELECTION:
		show_preselection_page(PAGE_DISTANCE_SELECTION);
		break;
	case BTN_ACTION_BIG_INCREASE_TIME:
		custom_time_sec = clamp_u32(custom_time_sec + current_time_big_step(),
					    SESSION_MIN_TIME_SEC, SESSION_MAX_TIME_SEC);
		update_preselection_page();
		break;
	case BTN_ACTION_INCREASE_TIME:
		custom_time_sec = clamp_u32(custom_time_sec + current_time_small_step(),
					    SESSION_MIN_TIME_SEC, SESSION_MAX_TIME_SEC);
		update_preselection_page();
		break;
	case BTN_ACTION_DECREASE_TIME:
		custom_time_sec = custom_time_sec <= current_time_small_step() ?
				  SESSION_MIN_TIME_SEC :
				  custom_time_sec - current_time_small_step();
		custom_time_sec = clamp_u32(custom_time_sec, SESSION_MIN_TIME_SEC,
					    SESSION_MAX_TIME_SEC);
		update_preselection_page();
		break;
	case BTN_ACTION_BIG_DECREASE_TIME:
		custom_time_sec = custom_time_sec <= current_time_big_step() ?
				  SESSION_MIN_TIME_SEC :
				  custom_time_sec - current_time_big_step();
		custom_time_sec = clamp_u32(custom_time_sec, SESSION_MIN_TIME_SEC,
					    SESSION_MAX_TIME_SEC);
		update_preselection_page();
		break;
	case BTN_ACTION_BIG_INCREASE_DISTANCE:
		custom_distance_m = clamp_u32(custom_distance_m + current_distance_big_step(),
					      SESSION_MIN_DISTANCE_M, SESSION_MAX_DISTANCE_M);
		update_preselection_page();
		break;
	case BTN_ACTION_INCREASE_DISTANCE:
		custom_distance_m = clamp_u32(custom_distance_m + current_distance_small_step(),
					      SESSION_MIN_DISTANCE_M, SESSION_MAX_DISTANCE_M);
		update_preselection_page();
		break;
	case BTN_ACTION_DECREASE_DISTANCE:
		custom_distance_m = custom_distance_m <= current_distance_small_step() ?
				    SESSION_MIN_DISTANCE_M :
				    custom_distance_m - current_distance_small_step();
		custom_distance_m = clamp_u32(custom_distance_m, SESSION_MIN_DISTANCE_M,
					      SESSION_MAX_DISTANCE_M);
		update_preselection_page();
		break;
	case BTN_ACTION_BIG_DECREASE_DISTANCE:
		custom_distance_m = custom_distance_m <= current_distance_big_step() ?
				    SESSION_MIN_DISTANCE_M :
				    custom_distance_m - current_distance_big_step();
		custom_distance_m = clamp_u32(custom_distance_m, SESSION_MIN_DISTANCE_M,
					      SESSION_MAX_DISTANCE_M);
		update_preselection_page();
		break;
	case BTN_ACTION_LEFTCYCLE_SESSION_TILE1:
		if (active_page != NULL && active_page->page_id == PAGE_SUMMARY) {
			cycle_summary_ring(0, -1);
		} else {
			cycle_session_ring(0, -1);
		}
		break;
	case BTN_ACTION_RIGHTCYCLE_SESSION_TILE1:
		if (active_page != NULL && active_page->page_id == PAGE_SUMMARY) {
			cycle_summary_ring(0, 1);
		} else {
			cycle_session_ring(0, 1);
		}
		break;
	case BTN_ACTION_LEFTCYCLE_SESSION_TILE2:
		if (active_page != NULL && active_page->page_id == PAGE_SUMMARY) {
			cycle_summary_ring(1, -1);
		} else {
			cycle_session_ring(1, -1);
		}
		break;
	case BTN_ACTION_RIGHTCYCLE_SESSION_TILE2:
		if (active_page != NULL && active_page->page_id == PAGE_SUMMARY) {
			cycle_summary_ring(1, 1);
		} else {
			cycle_session_ring(1, 1);
		}
		break;
	case BTN_ACTION_LEFTCYCLE_SESSION_TILE3:
		if (active_page != NULL && active_page->page_id == PAGE_SUMMARY) {
			cycle_summary_ring(2, -1);
		} else {
			cycle_session_ring(2, -1);
		}
		break;
	case BTN_ACTION_RIGHTCYCLE_SESSION_TILE3:
		if (active_page != NULL && active_page->page_id == PAGE_SUMMARY) {
			cycle_summary_ring(2, 1);
		} else {
			cycle_session_ring(2, 1);
		}
		break;
#ifdef CICLOTTE
	case BTN_ACTION_INCREASE_ANGLE:
		servo_change_level(1);
		break;
	case BTN_ACTION_DECREASE_ANGLE:
		servo_change_level(-1);
		break;
#endif
	case BTN_ACTION_PAUSE_SESSION:
		session.paused = !session.paused;
		if (active_page != NULL && active_page->page_id == PAGE_SESSION) {
			show_button(&session_buttons[0], false);
		}
		break;
	case BTN_ACTION_RESUME_SESSION:
		session.paused = false;
		if (active_page != NULL && active_page->page_id == PAGE_SESSION) {
			show_button(&session_buttons[0], false);
		}
		break;
	case BTN_ACTION_STOP_SESSION:
		if (active_page != NULL && active_page->page_id == PAGE_BT_CONNECTION) {
			(void)ble_ftms_notify_session_ended();
			(void)k_work_reschedule(&bt_disconnect_work,
						 K_MSEC(BT_SESSION_END_DISCONNECT_DELAY_MS));
			break;
		}
		if (session.active) {
			show_stop_confirmation();
		}
		break;
	case BTN_ACTION_CONFIRM_CONFIRM:
		if (active_page != NULL && active_page->page_id == PAGE_CONFIRM_STOP) {
			stop_session();
		}
		break;
	case BTN_ACTION_CONFIRM_CANCEL:
		if (active_page != NULL && active_page->page_id == PAGE_CONFIRM_STOP) {
			cancel_stop_confirmation();
		}
		break;
	case BTN_ACTION_GOTO_INFO:
		show_information_page();
		break;
	case BTN_ACTION_GOTO_HRCONNECT:
		show_hr_connect_page();
		break;
	case BTN_ACTION_REFRESH_HR:
		hr_state = ble_hr_get_state();
		if ((hr_state == BLE_HR_CONNECTED ||
		     hr_state == BLE_HR_CONNECTING ||
		     hr_state == BLE_HR_SCANNING)) {
			if (hr_search_mode_has_ble()) {
				(void)ble_hr_disconnect();
			}
		} else {
			if (!hr_search_mode_has_ble()) {
				hr_search_mode = HR_SEARCH_BLE;
				(void)save_ui_settings();
				show_button(&hr_connect_buttons[1], false);
			}
			if (hr_search_mode_has_ble()) {
				ret = ble_hr_start_pairing();
				hr_last_pairing_error = ret;
			}
		}
		show_hr_status_lines();
		break;
	case BTN_ACTION_UNLOCK_DEV_MODE:
		dev_mode_unlock_counter++;
		if (dev_mode_unlock_counter >= DEV_MODE_UNLOCK_TAPS) {
			dev_mode_unlock_counter = 0;
			show_developer_page();
		}
		break;
	case BTN_ACTION_RETURN_FROM_INFO:
		construct_page(PAGE_MAIN_MENU);
		break;
#ifndef CICLOTTE
	case BTN_ACTION_TILE0:
		if (active_page != NULL &&
		    active_page->page_id == PAGE_SETTINGS_2) {
			const char *status;

			ret = clear_resistance_calibration();
			status = ret == 0 ? text_cal_reset[app_language] :
					    text_reset_error[app_language];
			construct_page(PAGE_SETTINGS_2);
			display_show_text(SF_LABEL0_DARK, status, strlen(status));
		}
		break;
#endif
	case BTN_ACTION_TILE3:
		if (active_page != NULL &&
		    active_page->page_id == PAGE_HR_CONNECT) {
			cycle_hr_search_mode();
			(void)save_ui_settings();
			show_button(&hr_connect_buttons[1], false);
			show_hr_status_lines();
		}
		break;
	case BTN_ACTION_TILE4:
		if (active_page != NULL &&
		    active_page->page_id == PAGE_IMAGE_BROWSER) {
			image_browser_next();
		}
		break;
#ifndef CICLOTTE
	case BTN_ACTION_GOTO_CALIBRATE_UP:
		show_calibration_page(1);
		break;
	case BTN_ACTION_GOTO_CALIBRATE_DOWN:
		resistance_cal_raw_100 = resistance_raw;
		show_calibration_page(2);
		break;
	case BTN_ACTION_GOTO_CALIBRATE_END:
		resistance_cal_raw_0 = resistance_raw;
		resistance_calibrated = resistance_cal_raw_100 != resistance_cal_raw_0;
		measured_resistance_percent = normalize_resistance_percent(resistance_raw);
		(void)save_resistance_calibration();
		show_calibration_page(3);
		break;
	case BTN_ACTION_RETURN_FROM_CALIBRATE:
		app_darkmode = calibration_restore_darkmode;
		construct_page(PAGE_SETTINGS_1);
		break;
#endif
	case BTN_ACTION_GOTO_MAIN_MENU:
		if (active_page != NULL && active_page->page_id == PAGE_SUMMARY) {
			leave_summary();
		} else {
			construct_page(PAGE_MAIN_MENU);
		}
		break;
	case BTN_ACTION_GOTO_SETTINGS:
		construct_page(PAGE_SETTINGS_1);
		break;
	case BTN_ACTION_GOTO_SETTINGS2:
	case BTN_ACTION_CUSTOMIZE_SETTINGS:
		construct_page(PAGE_SETTINGS_2);
		break;
	case BTN_ACTION_SETUP_SETTINGS:
		construct_page(PAGE_SETUP);
		break;
	default:
		break;
	}
}

static void act_on_user_event(void)
{
	const struct menu_button *button;

	if (countdown_active) {
		last_user_state = user_state;
		return;
	}

	if (user_state == last_user_state) {
		return;
	}

	if (display_power_state != DISPLAY_POWER_ACTIVE) {
		wake_display();
		button_pressed = 0;
		last_user_state = user_state;
		return;
	}

	if (app_response_id == BTN_WAKEUP) {
		button_pressed = 0;
		last_user_state = user_state;
		return;
	}

	display_note_activity();

	switch (user_state) {
	case USER_PRESS:
		button_pressed = app_response_id;
		button = find_button(app_response_id);
		show_button(button, true);
		break;
	case USER_RELEASE:
		if (button_pressed != 0 && button_pressed == app_response_id) {
			button = find_button(app_response_id);
			show_button(button, false);
			if (button != NULL) {
				run_button_action(button->action_id);
			}
		}
		button_pressed = 0;
		break;
	case USER_SWIPE_LEFT:
	case USER_SWIPE_RIGHT:
	case USER_SWIPE_UP:
	case USER_SWIPE_DOWN:
		if (button_pressed != 0) {
			button = find_button(button_pressed);
			show_button(button, false);
			button_pressed = 0;
		}
		break;
	case USER_NONE:
	default:
		break;
	}

	last_user_state = user_state;
}

static void ring_buffer_init(struct ring_buffer *ring_buffer, uint16_t size)
{
	ring_buffer->size = size;
	ring_buffer->start = 0;
	ring_buffer->end = 0;
}

static bool ring_buffer_is_full(struct ring_buffer *ring_buffer)
{
	return (ring_buffer->end + 1) % ring_buffer->size == ring_buffer->start;
}

static bool ring_buffer_is_empty(struct ring_buffer *ring_buffer)
{
	return ring_buffer->start == ring_buffer->end;
}

static void ring_buffer_write(struct ring_buffer *ring_buffer, uint8_t element)
{
	if (ring_buffer_is_full(ring_buffer)) {
		ring_buffer->start = (ring_buffer->start + 1) % ring_buffer->size;
		uart_rx_overflow = true;
	}

	ring_buffer->buffer[ring_buffer->end] = element;
	ring_buffer->end = (ring_buffer->end + 1) % ring_buffer->size;
}

static uint8_t ring_buffer_read(struct ring_buffer *ring_buffer)
{
	uint8_t element;

	if (ring_buffer_is_empty(ring_buffer)) {
		return 0;
	}

	element = ring_buffer->buffer[ring_buffer->start];
	ring_buffer->start = (ring_buffer->start + 1) % ring_buffer->size;

	return element;
}

static void parser_init(struct parser_context *context)
{
	context->state = PENDING;
	context->payload_index = 0;
	context->current_message.complete = false;
	context->current_message.expected_length = 0;
}

static bool parse_display(struct ring_buffer *ring_buffer,
			  struct parser_context *context)
{
	if (context->state == FINISHED) {
		parser_init(context);
	}

	while (!ring_buffer_is_empty(ring_buffer)) {
		uint8_t byte = ring_buffer_read(ring_buffer);

		gpio_pin_toggle_dt(&led);

		switch (context->state) {
		case PENDING:
			if (byte == 0xaa) {
				context->current_message.prefix = byte;
				context->state = COLLECTING;
				context->payload_index = 0;
			}
			break;
		case COLLECTING:
			if (context->payload_index >= DISPLAY_RX_MAX) {
				context->state = ABORTED;
				break;
			}

			context->current_message.payload[context->payload_index++] = byte;

			if (context->payload_index > 4 &&
			    context->current_message.payload[context->payload_index - 4] == 0xe4 &&
			    context->current_message.payload[context->payload_index - 3] == 0x1b &&
			    context->current_message.payload[context->payload_index - 2] == 0x11 &&
			    context->current_message.payload[context->payload_index - 1] == 0xee) {
				context->state = FINISHED;
				context->current_message.payload_length =
					context->payload_index - 6;
				context->current_message.complete = true;
				return true;
			}
			break;
		case ABORTED:
			parser_init(context);
			return false;
		case FINISHED:
		case STARTED:
			break;
		}
	}

	return false;
}

static void uart_irq_handler(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	uart_irq_seen = true;

	if (!uart_irq_update(dev)) {
		return;
	}

	while (uart_irq_rx_ready(dev)) {
		uint8_t data[16];
		int rx_len = uart_fifo_read(dev, data, sizeof(data));

		if (rx_len <= 0) {
			break;
		}

		for (int i = 0; i < rx_len; i++) {
			ring_buffer_write(&display_rx_buffer, data[i]);
		}
	}
}

static void act_on_display_response(const uint8_t *payload, size_t len)
{
	if (len < 3) {
		return;
	}

	uint8_t command_id = payload[0];
	uint8_t target_id = payload[1];
	uint8_t response_code = payload[2];

	switch (command_id) {
	case CMD_BUTTON:
		switch (response_code) {
		case BTN_PRESS:
			user_state = USER_PRESS;
			app_response_id = target_id;
			break;
		case BTN_RELEASE:
			user_state = USER_RELEASE;
			app_response_id = target_id;
			break;
		default:
			break;
		}
		break;
	case CMD_SWIPE:
		switch (response_code) {
		case SWIPE_UP:
			user_state = USER_SWIPE_UP;
			break;
		case SWIPE_DOWN:
			user_state = USER_SWIPE_DOWN;
			break;
		case SWIPE_LEFT:
			user_state = USER_SWIPE_LEFT;
			break;
		case SWIPE_RIGHT:
			user_state = USER_SWIPE_RIGHT;
			break;
		default:
			break;
		}
		break;
	case 0x80:
		break;
	default:
		break;
	}
}

static void display_poll_rx(void)
{
	uint8_t byte;

	while (uart_poll_in(uart_dev, &byte) == 0) {
		ring_buffer_write(&display_rx_buffer, byte);
	}

	if (parse_display(&display_rx_buffer, &display_parser)) {
		act_on_display_response(display_parser.current_message.payload,
					display_parser.current_message.payload_length);
	}
}

static void tick_input_handler(const struct device *port,
			       struct gpio_callback *cb,
			       gpio_port_pins_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	cadence_edge_count++;
	cadence_wake_pending = true;
}

static int init_resistance_storage(void)
{
	const struct device *flash_dev = DEVICE_DT_GET(DT_MTD_FROM_FIXED_PARTITION(STORAGE_NODE));
	struct flash_pages_info page_info;
#ifndef CICLOTTE
	struct resistance_cal_data data;
#endif
	struct session_ring_pref_data pref_data;
	struct ui_settings_data ui_data;
	uint32_t storage_offset = DT_REG_ADDR(STORAGE_NODE);
	uint32_t storage_size = DT_REG_SIZE(STORAGE_NODE);
	int ret;

	if (!device_is_ready(flash_dev)) {
		return -ENODEV;
	}

	ret = flash_get_page_info_by_offs(flash_dev,
					  storage_offset,
					  &page_info);
	if (ret < 0) {
		return ret;
	}

	resistance_nvs.flash_device = flash_dev;
	resistance_nvs.offset = storage_offset;
	resistance_nvs.sector_size = page_info.size;
	resistance_nvs.sector_count = storage_size / page_info.size;

	ret = nvs_mount(&resistance_nvs);
	if (ret < 0) {
		return ret;
	}

	resistance_nvs_ready = true;

#ifndef CICLOTTE
	ret = nvs_read(&resistance_nvs, RESISTANCE_CAL_NVS_ID,
		       &data, sizeof(data));
	if (ret == sizeof(data) &&
	    data.magic == RESISTANCE_CAL_MAGIC &&
	    data.raw_100 != data.raw_0) {
		resistance_cal_raw_100 = data.raw_100;
		resistance_cal_raw_0 = data.raw_0;
		resistance_calibrated = true;
	}
#endif

	ret = nvs_read(&resistance_nvs, SESSION_RING_PREF_NVS_ID,
		       &pref_data, sizeof(pref_data));
	if (ret == sizeof(pref_data) &&
	    session_ring_preferences_valid(&pref_data)) {
		memcpy(session_ring_pref, pref_data.idx, sizeof(session_ring_pref));
	}

	ret = nvs_read(&resistance_nvs, UI_SETTINGS_NVS_ID,
		       &ui_data, sizeof(ui_data));
	if (ret == sizeof(ui_data) && ui_settings_valid(&ui_data)) {
		app_language = ui_data.language;
		app_darkmode = ui_data.darkmode != 0U;
		app_bt_enabled = true;
		hr_search_mode = ui_data.hr_search_mode == HR_SEARCH_OFF ?
				 HR_SEARCH_OFF : HR_SEARCH_BLE;
	}

	return 0;
}

static int init_training_inputs(void)
{
	int ret;

	if (!gpio_is_ready_dt(&tick_input)) {
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&tick_input, GPIO_INPUT);
	if (ret < 0) {
		return ret;
	}

	ret = gpio_pin_interrupt_configure_dt(&tick_input, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret < 0) {
		return ret;
	}

	gpio_init_callback(&tick_input_cb, tick_input_handler, BIT(tick_input.pin));
	ret = gpio_add_callback(tick_input.port, &tick_input_cb);
	if (ret < 0) {
		return ret;
	}

#ifdef CICLOTTE
	if (!pwm_is_ready_dt(&brake_servo)) {
		return -ENODEV;
	}

	return 0;
#else
	if (!adc_is_ready_dt(&resistance_adc)) {
		return -ENODEV;
	}

	return adc_channel_setup_dt(&resistance_adc);
#endif
}

#ifdef CICLOTTE
static bool update_resistance_input(void)
{
	/* No sensor: the brake position is whatever the GUI last commanded. */
	measured_resistance_percent = servo_level_resistance_percent();
	return true;
}
#else
static bool update_resistance_input(void)
{
	static uint32_t last_sample_ms;
	uint32_t now_ms = k_uptime_get_32();
	int16_t sample;
	int ret;
	struct adc_sequence sequence;

	if ((uint32_t)(now_ms - last_sample_ms) < RESISTANCE_SAMPLE_MS) {
		return false;
	}

	last_sample_ms = now_ms;
	sample = 0;

	ret = adc_sequence_init_dt(&resistance_adc, &sequence);
	if (ret < 0) {
		return false;
	}

	sequence.buffer = &sample;
	sequence.buffer_size = sizeof(sample);

	ret = adc_read_dt(&resistance_adc, &sequence);
	if (ret < 0) {
		return false;
	}

	if (sample < 0) {
		sample = 0;
	}

	resistance_raw = (uint16_t)sample;
	if (resistance_raw > 4095U) {
		resistance_raw = 4095U;
	}

	measured_resistance_percent = normalize_resistance_percent(resistance_raw);
	return true;
}
#endif

static uint32_t take_cadence_edges(void)
{
	uint32_t edges;
	unsigned int key = irq_lock();

	edges = cadence_edge_count;
	cadence_edge_count = 0;

	irq_unlock(key);

	return edges;
}

static void update_cadence_input(void)
{
	static uint32_t last_calc_ms;
	uint32_t now_ms = k_uptime_get_32();
	uint32_t elapsed_ms;
	uint32_t edges;
	uint32_t flywheel_rpm;
	uint32_t rpm;

	elapsed_ms = now_ms - last_calc_ms;
	if (elapsed_ms < SIMULATED_BIKE_NOTIFY_MS) {
		return;
	}

	last_calc_ms = now_ms;
	edges = take_cadence_edges();
	flywheel_rpm = (edges * 60U * 1000U) / elapsed_ms;
	rpm = ((flywheel_rpm * 100U) + (FLYWHEEL_TO_PEDAL_RATIO_X100 / 2U)) /
	      FLYWHEEL_TO_PEDAL_RATIO_X100;

	if (rpm > 255U) {
		rpm = 255U;
	}

	measured_cadence_rpm = (uint8_t)rpm;
}

static void simulated_bike_training_update(void)
{
	static uint32_t last_notify_ms;
	static uint32_t last_display_rearm_ms;
#if ENABLE_SIMULATED_BIKE_TRAINING
	static uint16_t phase;
#endif
	uint32_t now_ms = k_uptime_get_32();
	bool advance_session = session.active && !session.paused;
	uint16_t power_watt;
	uint16_t resistance_percent;
	uint16_t cadence_rpm;
	uint16_t speed_centi_kmh;
	bool finish_session = false;

	if ((uint32_t)(now_ms - last_notify_ms) < SIMULATED_BIKE_NOTIFY_MS) {
		return;
	}

	last_notify_ms = now_ms;

#if ENABLE_SIMULATED_BIKE_TRAINING
	power_watt = 100 + phase;
	resistance_percent = 20 + ((phase * 20) / 100);
	cadence_rpm = 50 + ((phase * 30) / 100);
	speed_centi_kmh = 2200 + ((phase * 1000) / 100);
#else
	(void)update_resistance_input();
	update_cadence_input();
	resistance_percent = measured_resistance_percent;
	cadence_rpm = measured_cadence_rpm;
#endif

#if USE_REAL_RESISTANCE_INPUT
	update_resistance_input();
	resistance_percent = measured_resistance_percent;
#endif

#if USE_REAL_CADENCE_INPUT
	update_cadence_input();
	cadence_rpm = measured_cadence_rpm;
#endif

#if !ENABLE_SIMULATED_BIKE_TRAINING || USE_REAL_RESISTANCE_INPUT || USE_REAL_CADENCE_INPUT
	power_watt = calculate_bike_power_watts(resistance_percent, cadence_rpm);
	speed_centi_kmh = calculate_bike_speed_centi_kmh(power_watt);
#endif

	session.watts = power_watt;
	session.resistance_percent = resistance_percent;
	session.cadence_rpm = cadence_rpm;
	session.speed_centi_kmh = speed_centi_kmh;
	session.heartrate_bpm = hr_current_bpm();

	if (advance_session) {
		session.elapsed_sec++;
		session.distance_m += speed_centi_kmh / 360U;
		session.watt_seconds += power_watt;
		session.calories = (uint16_t)((session.elapsed_sec * session.watts) / 4184U);
		if (bt_connection_session_active) {
			session.inactive_sec = 0;
		} else if (cadence_rpm > 0U || speed_centi_kmh > 0U || power_watt > 0U) {
			session.inactive_sec = 0;
		} else if (session.inactive_sec < SESSION_INACTIVITY_TIMEOUT_SEC) {
			session.inactive_sec++;
		}

		if (session.inactive_sec >= SESSION_INACTIVITY_TIMEOUT_SEC) {
			finish_session = true;
		}

		if (session.heartrate_bpm > 0U) {
			session.heartrate_sum += session.heartrate_bpm;
			session.heartrate_samples++;
			if (session.heartrate_bpm > session.max_heartrate_bpm) {
				session.max_heartrate_bpm = session.heartrate_bpm;
			}
		}

		if (session.mode == SESSION_TIME &&
		    session.elapsed_sec >= custom_time_sec) {
			session.elapsed_sec = custom_time_sec;
			finish_session = true;
		}

		if (session.mode == SESSION_DISTANCE &&
		    session.distance_m >= custom_distance_m) {
			session.distance_m = custom_distance_m;
			finish_session = true;
		}
	}

	(void)ble_ftms_notify_indoor_bike(speed_centi_kmh,
					  cadence_rpm * 2,
					  resistance_percent,
					  power_watt);

	if (finish_session) {
		show_summary();
	} else if (advance_session) {
		if ((active_page != NULL &&
		     (active_page->page_id == PAGE_SESSION ||
		      active_page->page_id == PAGE_BT_CONNECTION)) &&
		    (uint32_t)(now_ms - last_display_rearm_ms) >=
			    DISPLAY_REARM_INTERVAL_MS) {
			rearm_active_page_controls();
			last_display_rearm_ms = now_ms;
		}

		if (bt_connection_session_active) {
			update_bt_connection_values();
		} else {
			update_session_values();
		}
	} else if (session.summary_active) {
		update_summary_values();
	}

#if ENABLE_SIMULATED_BIKE_TRAINING
	if (!session.active || !session.paused) {
		phase = (phase + 5) % 101;
	}
#endif
}

int main(void)
{
	int ret;

	if (!device_is_ready(uart_dev)) {
		return 0;
	}

	if (!gpio_is_ready_dt(&led)) {
		return 0;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		return 0;
	}

	ring_buffer_init(&display_rx_buffer, UART_RX_RING_SIZE);
	parser_init(&display_parser);

	ret = uart_irq_callback_user_data_set(uart_dev, uart_irq_handler, NULL);
	if (ret < 0) {
		return 0;
	}

	uart_irq_rx_enable(uart_dev);

	bool boot_healthy = true;

	if (ble_ftms_init() != 0) {
		boot_healthy = false;
	}

	if (init_resistance_storage() != 0) {
		boot_healthy = false;
	}
	ble_ftms_set_calibration_handlers(custom_ble_resistance_calibrated,
					  custom_ble_set_resistance_calibration);

	if (app_bt_enabled) {
		(void)ble_ftms_set_enabled(true);
	}

	if (init_training_inputs() != 0) {
		boot_healthy = false;
	}
#ifdef CICLOTTE
	else {
		/* Like the AVR boot: drive the brake to the current level. */
		servo_set_enabled(true);
	}
#endif

	display_show_boot_splash();
	construct_page(PAGE_MAIN_MENU);
	display_last_activity_ms = k_uptime_get_32();

#ifdef CONFIG_BOOTLOADER_MCUBOOT
	/* Only confirm the running image once BLE/storage/input init
	 * succeeded; otherwise leave it unconfirmed so MCUboot reverts
	 * to the previous image on the next reset. */
	if (boot_healthy) {
		(void)boot_write_img_confirmed();
	}
#else
	(void)boot_healthy;
#endif

	while (1) {
		(void)update_resistance_input();
		display_poll_rx();
		act_on_user_event();
		countdown_update();
		handle_ftms_connection_state();
		update_hr_connect_page();
		update_developer_page();
		simulated_bike_training_update();
		display_power_update();

		k_msleep(DISPLAY_POLL_MS);
	}
	return 0;
}
