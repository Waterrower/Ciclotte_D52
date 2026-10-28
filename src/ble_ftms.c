#include "ble_ftms.h"

#include <errno.h>
#include <stddef.h>
#include <string.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#ifdef CONFIG_MCUMGR_TRANSPORT_BT
#include <zephyr/mgmt/mcumgr/transport/smp_bt.h>
#endif
#include <zephyr/sys/byteorder.h>

#define BT_UUID_FTMS_VAL 0x1826
#define BT_UUID_FTMS_FEATURE_VAL 0x2acc
#define BT_UUID_INDOOR_BIKE_DATA_VAL 0x2ad2
#define BT_UUID_SUPPORTED_INCLINATION_RANGE_VAL 0x2ad5
#define BT_UUID_SUPPORTED_RESISTANCE_RANGE_VAL 0x2ad6
#define BT_UUID_SUPPORTED_HEART_RATE_RANGE_VAL 0x2ad7
#define BT_UUID_SUPPORTED_POWER_RANGE_VAL 0x2ad8
#define BT_UUID_FITNESS_MACHINE_STATUS_VAL 0x2ada

#define FTMS_STATUS_STOPPED_OR_PAUSED_BY_USER 0x02
#define FTMS_STATUS_STARTED_OR_RESUMED_BY_USER 0x04

static struct bt_conn *current_conn;
static bool bt_ready;
static bool ftms_enabled;
static bool ftms_advertising;
static bool heart_rate_notify_enabled;
static bool indoor_bike_notify_enabled;
static bool ftms_status_notify_enabled;
static bool custom_nohrd_indicate_enabled;
static bool custom_nohrd_notify_enabled;
static int ftms_last_error;
static bool custom_nohrd_indication_pending;
static const uint8_t *custom_nohrd_queued_data;
static size_t custom_nohrd_queued_len;
static struct bt_gatt_indicate_params custom_nohrd_indicate_params;
static ble_ftms_calibration_status_cb_t calibration_status_cb;
static ble_ftms_calibration_set_cb_t calibration_set_cb;

static const struct bt_uuid_16 heart_rate_service_uuid = BT_UUID_INIT_16(BT_UUID_HRS_VAL);
static const struct bt_uuid_16 heart_rate_measurement_uuid =
	BT_UUID_INIT_16(BT_UUID_HRS_MEASUREMENT_VAL);
static const struct bt_uuid_16 ftms_uuid = BT_UUID_INIT_16(BT_UUID_FTMS_VAL);
static const struct bt_uuid_16 ftms_feature_uuid =
	BT_UUID_INIT_16(BT_UUID_FTMS_FEATURE_VAL);
static const struct bt_uuid_16 supported_inclination_range_uuid =
	BT_UUID_INIT_16(BT_UUID_SUPPORTED_INCLINATION_RANGE_VAL);
static const struct bt_uuid_16 supported_resistance_range_uuid =
	BT_UUID_INIT_16(BT_UUID_SUPPORTED_RESISTANCE_RANGE_VAL);
static const struct bt_uuid_16 supported_heart_rate_range_uuid =
	BT_UUID_INIT_16(BT_UUID_SUPPORTED_HEART_RATE_RANGE_VAL);
static const struct bt_uuid_16 supported_power_range_uuid =
	BT_UUID_INIT_16(BT_UUID_SUPPORTED_POWER_RANGE_VAL);
static const struct bt_uuid_16 indoor_bike_data_uuid =
	BT_UUID_INIT_16(BT_UUID_INDOOR_BIKE_DATA_VAL);
static const struct bt_uuid_16 fitness_machine_status_uuid =
	BT_UUID_INIT_16(BT_UUID_FITNESS_MACHINE_STATUS_VAL);
static const struct bt_uuid_128 custom_nohrd_bike_service_uuid =
	BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x00001001, 0xc042, 0x66ba,
					    0x1335, 0x90118f542c77));
static const struct bt_uuid_128 custom_nohrd_bike_write_uuid =
	BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x8ec92001, 0xf315, 0x4f60,
					    0x9fb8, 0x838830daea50));
static const struct bt_uuid_128 custom_nohrd_bike_notify_uuid =
	BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x8ec92002, 0xf315, 0x4f60,
					    0x9fb8, 0x838830daea50));

extern const struct bt_gatt_service_static ftms_service;
extern const struct bt_gatt_service_static custom_nohrd_bike_service;

static const uint8_t ftms_feature[8] = {
	0x04, 0x04, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
};
static const uint8_t supported_inclination_range[6] = {
	0x18, 0xfc, 0xe8, 0x03, 0x01, 0x00,
};
static const uint8_t supported_resistance_range[6] = {
	0x00, 0x00, 0xe8, 0x03, 0x01, 0x00,
};
static const uint8_t supported_heart_rate_range[3] = {
	0x00, 0xff, 0x01,
};
static const uint8_t supported_power_range[6] = {
	0x00, 0x00, 0xa0, 0x0f, 0x01, 0x00,
};

static const struct bt_data advertising_data[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL,
		      BT_UUID_16_ENCODE(BT_UUID_FTMS_VAL),
		      BT_UUID_16_ENCODE(BT_UUID_DIS_VAL)),
	BT_DATA_BYTES(BT_DATA_SVC_DATA16, BT_UUID_16_ENCODE(BT_UUID_FTMS_VAL),
		      0x01, 0x20, 0x00),
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
		sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

#ifdef CONFIG_MCUMGR_TRANSPORT_BT
static const struct bt_data scan_response_data[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, SMP_BT_SVC_UUID_VAL),
};
#endif

static ssize_t read_ftms_feature(struct bt_conn *conn,
				 const struct bt_gatt_attr *attr,
				 void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset,
				 ftms_feature, sizeof(ftms_feature));
}

static ssize_t read_supported_inclination_range(struct bt_conn *conn,
						const struct bt_gatt_attr *attr,
						void *buf, uint16_t len,
						uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset,
				 supported_inclination_range,
				 sizeof(supported_inclination_range));
}

static ssize_t read_supported_resistance_range(struct bt_conn *conn,
					       const struct bt_gatt_attr *attr,
					       void *buf, uint16_t len,
					       uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset,
				 supported_resistance_range,
				 sizeof(supported_resistance_range));
}

static ssize_t read_supported_heart_rate_range(struct bt_conn *conn,
					       const struct bt_gatt_attr *attr,
					       void *buf, uint16_t len,
					       uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset,
				 supported_heart_rate_range,
				 sizeof(supported_heart_rate_range));
}

static ssize_t read_supported_power_range(struct bt_conn *conn,
					  const struct bt_gatt_attr *attr,
					  void *buf, uint16_t len,
					  uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset,
				 supported_power_range,
				 sizeof(supported_power_range));
}

static void heart_rate_ccc_changed(const struct bt_gatt_attr *attr,
				   uint16_t value)
{
	ARG_UNUSED(attr);

	heart_rate_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
}

static void indoor_bike_ccc_changed(const struct bt_gatt_attr *attr,
				    uint16_t value)
{
	ARG_UNUSED(attr);

	indoor_bike_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
}

static void fitness_machine_status_ccc_changed(const struct bt_gatt_attr *attr,
					       uint16_t value)
{
	ARG_UNUSED(attr);

	ftms_status_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
}

static void custom_nohrd_ccc_changed(const struct bt_gatt_attr *attr,
				     uint16_t value)
{
	ARG_UNUSED(attr);

	custom_nohrd_indicate_enabled = (value == BT_GATT_CCC_INDICATE);
}

static void custom_nohrd_notify_ccc_changed(const struct bt_gatt_attr *attr,
					    uint16_t value)
{
	ARG_UNUSED(attr);

	custom_nohrd_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
}

static int custom_nohrd_start_indication(const uint8_t *data, size_t len);

static void custom_nohrd_indicate_cb(struct bt_conn *conn,
				     struct bt_gatt_indicate_params *params,
				     uint8_t err)
{
	const uint8_t *queued_data;
	size_t queued_len;

	ARG_UNUSED(conn);
	ARG_UNUSED(params);
	ARG_UNUSED(err);

	custom_nohrd_indication_pending = false;

	if (custom_nohrd_queued_data != NULL) {
		queued_data = custom_nohrd_queued_data;
		queued_len = custom_nohrd_queued_len;
		custom_nohrd_queued_data = NULL;
		custom_nohrd_queued_len = 0U;
		(void)custom_nohrd_start_indication(queued_data, queued_len);
	}
}

static int custom_nohrd_start_indication(const uint8_t *data, size_t len)
{
	int err;

	if (current_conn == NULL || !custom_nohrd_indicate_enabled) {
		return 0;
	}

	memset(&custom_nohrd_indicate_params, 0,
	       sizeof(custom_nohrd_indicate_params));
	custom_nohrd_indicate_params.attr = &custom_nohrd_bike_service.attrs[2];
	custom_nohrd_indicate_params.data = data;
	custom_nohrd_indicate_params.len = len;
	custom_nohrd_indicate_params.func = custom_nohrd_indicate_cb;
	custom_nohrd_indication_pending = true;

	err = bt_gatt_indicate(current_conn, &custom_nohrd_indicate_params);
	if (err != 0) {
		custom_nohrd_indication_pending = false;
	}

	return err;
}

static int custom_nohrd_send(const uint8_t *data, size_t len)
{
	if (current_conn == NULL) {
		return 0;
	}

	if (custom_nohrd_notify_enabled) {
		return bt_gatt_notify_uuid(current_conn,
					   &custom_nohrd_bike_notify_uuid.uuid,
					   custom_nohrd_bike_service.attrs,
					   data, len);
	}

	if (!custom_nohrd_indicate_enabled || custom_nohrd_indication_pending) {
		if (custom_nohrd_indicate_enabled &&
		    custom_nohrd_queued_data == NULL) {
			custom_nohrd_queued_data = data;
			custom_nohrd_queued_len = len;
		}
		return 0;
	}

	return custom_nohrd_start_indication(data, len);
}

static void custom_nohrd_send_response(bool success)
{
	static const uint8_t response_success[] = {
		0xe2, 0xb0, 0xfe, 0x01, 0x01, 0x01, 0x00, 0x00,
	};
	static const uint8_t response_fail[] = {
		0xe2, 0xb0, 0xfe, 0x01, 0x02, 0x01, 0x00, 0x00,
	};

	(void)custom_nohrd_send(success ? response_success : response_fail,
				success ? sizeof(response_success) :
					  sizeof(response_fail));
}

static void custom_nohrd_send_calibrated(bool calibrated)
{
	static const uint8_t calibrated_ack[] = {
		0xe2, 0xb0, 0xfe, 0x40, 0x01, 0x00, 0x00, 0x00,
	};
	static const uint8_t calibrated_nack[] = {
		0xe2, 0xb0, 0xfe, 0x40, 0x02, 0x00, 0x00, 0x00,
	};

	(void)custom_nohrd_send(calibrated ? calibrated_ack : calibrated_nack,
				calibrated ? sizeof(calibrated_ack) :
					     sizeof(calibrated_nack));
}

static void custom_nohrd_send_transition(uint8_t transition)
{
	static const uint8_t ready[] = {0xe2, 0xd2, 0x00};
	static const uint8_t in_use[] = {0xe2, 0xd2, 0x01};
	static const uint8_t finished[] = {0xe2, 0xd2, 0x02};
	const uint8_t *data = ready;

	if (transition == 0x01) {
		data = in_use;
	} else if (transition == 0x02) {
		data = finished;
	}

	(void)custom_nohrd_send(data, sizeof(ready));
}

static int ftms_notify_status(uint8_t status, uint8_t parameter)
{
	uint8_t data[] = {status, parameter};

	if (current_conn == NULL || !ftms_status_notify_enabled) {
		return 0;
	}

	return bt_gatt_notify_uuid(current_conn,
				   &fitness_machine_status_uuid.uuid,
				   ftms_service.attrs, data, sizeof(data));
}

static int custom_nohrd_protocol_offset(const uint8_t *data, size_t len)
{
	for (size_t i = 0; i < len; i++) {
		if (data[i] == 0xfe) {
			return (int)i;
		}
	}

	return -1;
}

static ssize_t write_custom_nohrd(struct bt_conn *conn,
				  const struct bt_gatt_attr *attr,
				  const void *buf, uint16_t len,
				  uint16_t offset, uint8_t flags)
{
	const uint8_t *data = buf;
	bool success = false;
	int protocol_offset;
	size_t cmd_offset;

	ARG_UNUSED(conn);
	ARG_UNUSED(attr);
	ARG_UNUSED(flags);

	if (offset != 0U) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	protocol_offset = custom_nohrd_protocol_offset(data, len);
	if (protocol_offset < 0 || (size_t)protocol_offset + 2U >= len) {
		custom_nohrd_send_response(false);
		return len;
	}

	cmd_offset = (size_t)protocol_offset + 1U;

	switch (data[cmd_offset]) {
	case 0x01:
		if (cmd_offset + 1U < len && calibration_set_cb != NULL) {
			success = calibration_set_cb(data[cmd_offset + 1U],
						    &data[cmd_offset + 2U],
						    len - (cmd_offset + 2U)) == 0;
		}
		custom_nohrd_send_response(success);
		break;
	case 0x02:
		success = cmd_offset + 1U < len &&
			  (data[cmd_offset + 1U] == 0x01 ||
			   data[cmd_offset + 1U] == 0x02);
		custom_nohrd_send_response(success);
		break;
	case 0x04:
		custom_nohrd_send_response(true);
		custom_nohrd_send_calibrated(calibration_status_cb != NULL &&
					     calibration_status_cb());
		break;
	default:
		custom_nohrd_send_response(false);
		break;
	}

	return len;
}

/* Registered dynamically (see ble_ftms_hrs_set_active) only once a BLE HR
 * sensor is actually connected, so idle FTMS-only clients never see an
 * advertised Heart Rate Service that has no data behind it. Not const:
 * bt_gatt_service_register() writes the assigned handles into the array. */
static struct bt_gatt_attr heart_rate_attrs[] = {
	BT_GATT_PRIMARY_SERVICE(&heart_rate_service_uuid.uuid),
	BT_GATT_CHARACTERISTIC(&heart_rate_measurement_uuid.uuid,
			       BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_NONE,
			       NULL, NULL, NULL),
	BT_GATT_CCC(heart_rate_ccc_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
};

static struct bt_gatt_service heart_rate_service = BT_GATT_SERVICE(heart_rate_attrs);
static bool heart_rate_service_registered;

BT_GATT_SERVICE_DEFINE(ftms_service,
	BT_GATT_PRIMARY_SERVICE(&ftms_uuid.uuid),
	BT_GATT_CHARACTERISTIC(&ftms_feature_uuid.uuid,
			       BT_GATT_CHRC_READ,
			       BT_GATT_PERM_READ,
			       read_ftms_feature, NULL, NULL),
	BT_GATT_CHARACTERISTIC(&supported_inclination_range_uuid.uuid,
			       BT_GATT_CHRC_READ,
			       BT_GATT_PERM_READ,
			       read_supported_inclination_range, NULL, NULL),
	BT_GATT_CHARACTERISTIC(&supported_resistance_range_uuid.uuid,
			       BT_GATT_CHRC_READ,
			       BT_GATT_PERM_READ,
			       read_supported_resistance_range, NULL, NULL),
	BT_GATT_CHARACTERISTIC(&supported_heart_rate_range_uuid.uuid,
			       BT_GATT_CHRC_READ,
			       BT_GATT_PERM_READ,
			       read_supported_heart_rate_range, NULL, NULL),
	BT_GATT_CHARACTERISTIC(&supported_power_range_uuid.uuid,
			       BT_GATT_CHRC_READ,
			       BT_GATT_PERM_READ,
			       read_supported_power_range, NULL, NULL),
	BT_GATT_CHARACTERISTIC(&indoor_bike_data_uuid.uuid,
			       BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_NONE,
			       NULL, NULL, NULL),
	BT_GATT_CCC(indoor_bike_ccc_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(&fitness_machine_status_uuid.uuid,
			       BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_NONE,
			       NULL, NULL, NULL),
	BT_GATT_CCC(fitness_machine_status_ccc_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

BT_GATT_SERVICE_DEFINE(custom_nohrd_bike_service,
	BT_GATT_PRIMARY_SERVICE(&custom_nohrd_bike_service_uuid.uuid),
	BT_GATT_CHARACTERISTIC(&custom_nohrd_bike_write_uuid.uuid,
			       BT_GATT_CHRC_WRITE |
			       BT_GATT_CHRC_INDICATE,
			       BT_GATT_PERM_WRITE,
			       NULL, write_custom_nohrd, NULL),
	BT_GATT_CCC(custom_nohrd_ccc_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(&custom_nohrd_bike_notify_uuid.uuid,
			       BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_NONE,
			       NULL, NULL, NULL),
	BT_GATT_CCC(custom_nohrd_notify_ccc_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

static int start_advertising(void)
{
	int err;

	if (!bt_ready || ftms_advertising || current_conn != NULL) {
		return 0;
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2,
			      advertising_data, ARRAY_SIZE(advertising_data),
#ifdef CONFIG_MCUMGR_TRANSPORT_BT
			      scan_response_data, ARRAY_SIZE(scan_response_data));
#else
			      NULL, 0);
#endif
	if (err == -EALREADY) {
		err = 0;
	}

	if (err == 0) {
		ftms_advertising = true;
	}

	return err;
}

static int stop_advertising(void)
{
	int err;

	if (!ftms_advertising) {
		return 0;
	}

	err = bt_le_adv_stop();
	if (err == -EALREADY || err == -EINVAL || err == -EACCES) {
		err = 0;
	}

	if (err == 0) {
		ftms_advertising = false;
	}

	return err;
}

int ble_ftms_hrs_set_active(bool active)
{
	int err;

	if (active == heart_rate_service_registered) {
		return 0;
	}

	if (active) {
		err = bt_gatt_service_register(&heart_rate_service);
	} else {
		heart_rate_notify_enabled = false;
		err = bt_gatt_service_unregister(&heart_rate_service);
	}

	if (err == 0) {
		heart_rate_service_registered = active;
	}

	return err;
}

int ble_ftms_notify_heart_rate(uint8_t bpm)
{
	uint8_t data[2] = { 0x00, bpm };

	if (current_conn == NULL || !heart_rate_service_registered ||
	    !heart_rate_notify_enabled) {
		return 0;
	}

	return bt_gatt_notify_uuid(current_conn, &heart_rate_measurement_uuid.uuid,
				   heart_rate_attrs, data, sizeof(data));
}

int ble_ftms_pause_advertising(void)
{
	return stop_advertising();
}

int ble_ftms_resume_advertising(void)
{
	if (!ftms_enabled) {
		return 0;
	}

	return start_advertising();
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	struct bt_conn_info info;

	if (bt_conn_get_info(conn, &info) != 0) {
		return;
	}

	if (info.role != BT_CONN_ROLE_PERIPHERAL) {
		return;
	}

	if (err != 0) {
		if (ftms_enabled) {
			(void)start_advertising();
		}
		return;
	}

	current_conn = bt_conn_ref(conn);
	ftms_advertising = false;
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	ARG_UNUSED(reason);

	if (current_conn != conn) {
		return;
	}

	bt_conn_unref(current_conn);
	current_conn = NULL;
	heart_rate_notify_enabled = false;
	indoor_bike_notify_enabled = false;
	ftms_status_notify_enabled = false;
	custom_nohrd_indicate_enabled = false;
	custom_nohrd_notify_enabled = false;
	custom_nohrd_indication_pending = false;
	custom_nohrd_queued_data = NULL;
	custom_nohrd_queued_len = 0U;

	if (ftms_enabled) {
		(void)start_advertising();
	}
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

int ble_ftms_init(void)
{
	int err;

	if (bt_ready) {
		return 0;
	}

	err = bt_enable(NULL);
	if (err == 0) {
		bt_ready = true;
	}
	ftms_last_error = err;

	return err;
}

int ble_ftms_set_enabled(bool enabled)
{
	int err;

	if (!bt_ready) {
		err = ble_ftms_init();
		if (err != 0) {
			ftms_last_error = err;
			return err;
		}
	}

	if (enabled) {
		err = start_advertising();
		ftms_last_error = err;
		if (err == 0) {
			ftms_enabled = true;
		}
		return err;
	}

	ftms_enabled = false;
	err = stop_advertising();

	if (current_conn != NULL) {
		int disconnect_err = bt_conn_disconnect(current_conn,
							BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		if (err == 0) {
			err = disconnect_err;
		}
	}

	ftms_last_error = err;
	return err;
}

int ble_ftms_last_error(void)
{
	return ftms_last_error;
}

bool ble_ftms_is_enabled(void)
{
	return ftms_enabled;
}

bool ble_ftms_is_connected(void)
{
	return current_conn != NULL;
}

int ble_ftms_disconnect(void)
{
	if (current_conn == NULL) {
		return 0;
	}

	return bt_conn_disconnect(current_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
}

void ble_ftms_set_calibration_handlers(ble_ftms_calibration_status_cb_t status_cb,
				       ble_ftms_calibration_set_cb_t set_cb)
{
	calibration_status_cb = status_cb;
	calibration_set_cb = set_cb;
}

int ble_ftms_notify_session_started(void)
{
	custom_nohrd_send_transition(0x01);

	return ftms_notify_status(FTMS_STATUS_STARTED_OR_RESUMED_BY_USER, 0x00);
}

int ble_ftms_notify_session_ended(void)
{
	int err;

	custom_nohrd_send_transition(0x02);
	custom_nohrd_send_transition(0x00);

	err = ftms_notify_status(FTMS_STATUS_STOPPED_OR_PAUSED_BY_USER, 0x01);
	if (err == -ENOTCONN) {
		err = 0;
	}

	return err;
}

int ble_ftms_notify_indoor_bike(uint16_t speed_centi_kmh, uint16_t cadence_half_rpm,
				int16_t resistance_percent, int16_t power_watt)
{
	uint8_t data[10];

	if (current_conn == NULL || !indoor_bike_notify_enabled) {
		return 0;
	}

	sys_put_le16(0x0064, &data[0]);
	sys_put_le16(speed_centi_kmh, &data[2]);
	sys_put_le16(cadence_half_rpm, &data[4]);
	sys_put_le16((uint16_t)(resistance_percent * 2), &data[6]);
	sys_put_le16((uint16_t)power_watt, &data[8]);

	return bt_gatt_notify_uuid(current_conn, &indoor_bike_data_uuid.uuid,
				   ftms_service.attrs, data, sizeof(data));
}
