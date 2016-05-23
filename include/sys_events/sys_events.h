#ifndef __SYSTEM_EVENTS_H__
#define __SYSTEM_EVENTS_H__

#include <stdint.h>
#include "bootlogic.h"

#ifndef __packed
#define __packed __attribute__((packed))
#endif

/**
 * @defgroup infra_system_events System Events
 *
 * Implements a circular buffer of system events. This is used to report system
 * events to cloud debug tool as well as reporting them to the companion ble app.
 *
 * @ingroup infra
 * @{
 */

#define SYSTEM_EVENT_TYPE_BOOT        0x1   /*!< Boot event identifier */
#define SYSTEM_EVENT_TYPE_PANIC       0x2   /*!< Panic event identifier */
#define SYSTEM_EVENT_TYPE_SHUTDOWN    0x3   /*!< Shutdown event identifier */
#define SYSTEM_EVENT_TYPE_UPTIME      0x4   /*!< Uptime event identifier */
#define SYSTEM_EVENT_TYPE_BATTERY     0x5   /*!< Battery event identifier */
#define SYSTEM_EVENT_TYPE_BLE_PAIRING 0x6   /*!< BLE pairing event identifier*/
#define SYSTEM_EVENT_TYPE_BLE_CONN    0x7   /*!< BLE connected event identifier*/
#define SYSTEM_EVENT_TYPE_WORN        0x8   /*!< Worn event identifier*/
#define SYSTEM_EVENT_TYPE_NFC         0x9   /*!< Nfc reader event identifier*/
#define SYSTEM_EVENT_TYPE_OTA         0xa   /*!< OTA event identifier */
#define SYSTEM_EVENT_TYPE_HW_CHARGING 0x10  /*!< Hardware charging event identifier*/
#define SYSTEM_EVENT_TYPE_FLASH       0x11  /*!< Flash event identifier*/

/** Start of the project-specific system event range */
#define SYSTEM_EVENT_USER_RANGE_START 0x100 /*!< Project specific events starts with id 0x100 */

/** Size of the event structure in flash */
#define SYSTEM_EVENT_SIZE             0x20

/** Version of the system event structure. */
#define SYSTEM_EVENT_VERSION          1

/**
 * Boot event data
 */
struct event_data_boot {
	uint32_t uptime;    /*!< Uptime in sec */
	uint8_t reason;     /*!< Boot reason */
	uint8_t target;     /*!< Boot target */
} __packed;

/**
 * Panic event data
 */
struct event_data_panic {
	uint8_t hash[4];     /*!< build hash for the panic*/
	uint32_t values[3];
#define SYSTEM_EVENT_PANIC_ARC   0x1
#define SYSTEM_EVENT_PANIC_QUARK 0x2
	uint8_t cpu;
} __packed;

/**
 * Shutdown event data
 */
struct event_data_shutdown {
	enum shutdown_type {
		SYSTEM_EVENT_SHUTDOWN_TYPE_SHUTDOWN = 1,
		SYSTEM_EVENT_SHUTDOWN_TYPE_REBOOT = 2,
		SYSTEM_EVENT_SHUTDOWN_TYPE_POWER_OFF = 3
	} __packed type;                 /*!< Shutdown type ( reboot / power off) */
	uint8_t reason;                  /*!< Shutdown reason */
} __packed;

/**
 * Uptime event data
 */
struct event_data_uptime {
	uint32_t time;
} __packed;

/**
 * Battery event data
 */
struct event_data_battery {
	enum battery_event_type {
		SYSTEM_EVENT_BATT_CHARGE = 1,      /*!< Event is a battery charge state */
		SYSTEM_EVENT_BATT_LEVEL  = 2       /*!< Event is a battery level change */
	} __packed type;                       /*!< Type of battery event */

	union {
		enum charge_state {
			SYSTEM_EVENT_BATT_CHARGING    = 1, /*!< Battery is charging */
			SYSTEM_EVENT_BATT_DISCHARGING = 2, /*!< Battery is discharging */
			SYSTEM_EVENT_BATT_CH_COMPLETE = 3  /*!< Battery is charged */
		} __packed state;                      /*!< if type is @ref SYSTEM_EVENT_BATT_CHARGE */
		uint8_t data;                          /*!< battery level in percent */
	} u;
} __packed;

/**
 * Hardware charging event data
 */
struct event_data_hw_charging {
	uint16_t level_mv;     /*!< Battery level in millivolt */
} __packed;

/**
 * Fota event data
 */
struct event_data_ota {
	enum ota_state {
		SYSTEM_EVENT_OTA_TYPE_STARTED = 1,
		SYSTEM_EVENT_OTA_TYPE_COMPLETE = 2,
	} __packed state;                /*!< Fota type ( success / failed ) */
	uint8_t error_code;              /*!< OTA error code */
} __packed;

/**
 * Flash event data
 */
struct event_data_flash {
	enum flash_state {
		SYSTEM_EVENT_FLASH_TYPE_STARTED = 1,
		SYSTEM_EVENT_FLASH_TYPE_COMPLETE = 2,
	} __packed state;                /*!< Fota type ( success / failed ) */
	uint8_t error_code;              /*!< OTA error code */
} __packed;

/**
 * Event header
 * This structure is common to all system events. The type field allows to know
 * the actual type of the data field
 */
struct system_event_header {
	uint8_t size;       /*!< size of the event structure. */
	uint8_t version;    /*!< version of the system event */
	uint16_t type;      /*!< event type */
	uint32_t timestamp; /*!< event generation timestamp */
	uint8_t hash[4];    /*!< build hash */
} __packed;

/**
 * Event
 * This structure contains the header and the data of the event, depending on its type.
 */
struct system_event {
	struct system_event_header h; /* !< Event header */
	union {
		struct event_data_boot boot;
		struct event_data_panic panic;
		struct event_data_shutdown shutdown;
		struct event_data_battery battery;
		struct event_data_uptime uptime;
		struct event_data_ota ota;
		struct event_data_hw_charging hw_charging;
		struct event_data_flash flash;
		char user;
	} event_data;          /* !< Event data */
} __packed;

/**
 * Initialize the system event manager.
 * This should be done only once, as soon as possible.
 */
void system_events_init(void);

/**
 * Push a system event
 *
 * @param evt the system event pointer to push to storage. The pointer is not
 *            freed by this function.
 */
void system_event_push(struct system_event *evt);

/**
 * Pop a system event
 *
 * This function retrieves the first available system event in the buffer.
 *
 * @return struct system_event an allocated pointer to the retrieved system
 *                             event. NULL if no event in the buffer.
 */
struct system_event *system_event_pop(void);

/**
 * Push a boot system event in the buffer.
 *
 * @param reset_reason the reset reason we want to push in the buffer
 * @param boot_target the boot target we want to push in the buffer
 */
void system_event_push_boot_event(enum reset_reasons reset_reason, enum boot_targets boot_target);

/**
 * Push a shutdown system event in the buffer.
 *
 * @param type the type of shutdown we want to push in the buffer, it could be:
 *         - PUPDR_POWER_OFF
 *         - PUPDR_SHUTDOWN
 *         - PUPDR_REBOOT
 * @param param the cause of the shutdown
 */
void system_event_push_shutdown(int type, int param);

/**
 * Push an uptime system event.
 */
void system_event_push_uptime(void);

/**
 * Push a battery system event.
 * @param type the type of battery event we want to push in the buffer, it could be:
 *         - SYSTEM_EVENT_BATT_LEVEL
 *         - SYSTEM_EVENT_BATT_CHARGE
 * @param data details to push in the buffer (depends on the type)
 */
void system_event_push_battery(uint8_t type, uint8_t data);

/**
 * Push an ota system event.
 *
 */
void system_event_push_ota(uint8_t state, uint8_t error_code);

/**
 * Push a hardware charging system event.
 *
 * @param level_mv millivolt battery level.
 */
void system_event_push_hw_charging(uint16_t level_mv);

/**
 * Push a flash system event.
 *
 */
void system_event_push_flash(uint8_t state, uint8_t error_code);
/** @} */
#endif
