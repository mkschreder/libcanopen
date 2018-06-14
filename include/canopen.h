/*
 * Copyright (C) 2017 Martin K. Schröder <mkschreder.uk@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/

#pragma once

#include <libfirmware/can.h>
#include <libfirmware/timestamp.h>
#include <libfirmware/mutex.h>
#include <libfirmware/sem.h>
#include <libfirmware/thread.h>
#include <libfirmware/atomic.h>
#include <libfirmware/work.h>
#include <libfirmware/vardir.h>

enum {
	CANOPEN_COB_NMT = 0,
	CANOPEN_COB_SYNC_OR_EMCY = 0x080,
	CANOPEN_COB_TIME	= 0x100,
	CANOPEN_COB_TXPDO_0 = 0x180,
	CANOPEN_COB_RXPDO_0 = 0x200,
	CANOPEN_COB_TXPDO_1 = 0x280,
	CANOPEN_COB_RXPDO_1 = 0x300,
	CANOPEN_COB_TXPDO_2 = 0x380,
	CANOPEN_COB_RXPDO_2 = 0x400,
	CANOPEN_COB_TXPDO_3 = 0x480,
	CANOPEN_COB_RXPDO_3 = 0x500,
	CANOPEN_COB_TXSDO	= 0x580,
	CANOPEN_COB_RXSDO	= 0x600,
	CANOPEN_COB_LSS		= 0x780,
};

enum {
	CANOPEN_LSS_RX = 0x64,
	CANOPEN_LSS_TX = 0x65
};

enum {
	CANOPEN_NMT_CMD_ENABLE = 0x01,
	CANOPEN_NMT_CMD_DISABLE = 0x02,
	CANOPEN_NMT_CMD_PREOP = 0x80,
	CANOPEN_NMT_CMD_RESET = 0x81,
	CANOPEN_NMT_CMD_RESET_COM = 0x82
};

enum {
	CANOPEN_SDO_CMD_WRITE1		= 0x2F,
	CANOPEN_SDO_CMD_WRITE2		= 0x2B,
	CANOPEN_SDO_CMD_WRITE3		= 0x27,
	CANOPEN_SDO_CMD_WRITE4		= 0x23,
	CANOPEN_SDO_CMD_WRITE		= 0x60,
	CANOPEN_SDO_CMD_READ		= 0x40,
	CANOPEN_SDO_CMD_READ1		= 0x4F,
	CANOPEN_SDO_CMD_READ2		= 0x4B,
	CANOPEN_SDO_CMD_READ3		= 0x47,
	CANOPEN_SDO_CMD_READ4		= 0x43,
	CANOPEN_SDO_CMD_ABORT		= 0x80
};

#define CANOPEN_SDO_ERR_TOGGLE_BIT			0x05030000
#define CANOPEN_SDO_ERR_CLIENT_SERVER		0x05040001
#define CANOPEN_SDO_ERR_UNSUPPORTED_ACCESS	0x06010000
#define CANOPEN_SDO_ERR_NO_EXIST			0x06020000
#define CANOPEN_SDO_ERR_PDO_NO_MAP			0x06040041
#define CANOPEN_SDO_ERR_PDO_MAP_TOO_BIG		0x06040042
#define CANOPEN_SDO_ERR_INCOMPATIBLE_PARAMS	0x06040043
#define CANOPEN_SDO_ERR_GENERAL_INTERNAL	0x06040047
#define CANOPEN_SDO_ERR_LEN_INVAL			0x06070010
#define CANOPEN_SDO_ERR_LEN_HIGH			0x06070012
#define CANOPEN_SDO_ERR_LEN_LOW				0x06070013
#define CANOPEN_SDO_ERR_SUBINDEX_INVAL		0x06090011
#define CANOPEN_SDO_ERR_VALUE_OUT_OF_RANGE	0x06090030
#define CANOPEN_SDO_ERR_VALUE_TOO_HIGH		0x06090031
#define CANOPEN_SDO_ERR_VALUE_TOO_LOW		0x06090032
#define CANOPEN_SDO_ERR_GENERAL				0x08000000
#define CANOPEN_SDO_ERR_NOSTORE				0x08000020
#define CANOPEN_SDO_ERR_NOSTORE_LOCAL		0x08000021
#define CANOPEN_SDO_ERR_NOSTORE_STATE		0x08000022

#define CANOPEN_REG_DEVICE_TYPE						0x100000
#define CANOPEN_REG_DEVICE_ERROR					0x100100
#define CANOPEN_REG_DEVICE_ERROR_GENERIC_BIT		(1 << 0)
#define CANOPEN_REG_DEVICE_ERROR_CURRENT_BIT		(1 << 1)
#define CANOPEN_REG_DEVICE_ERROR_VOLTAGE_BIT		(1 << 2)
#define CANOPEN_REG_DEVICE_ERROR_TEMP_BIT			(1 << 3)
#define CANOPEN_REG_DEVICE_ERROR_COMM_BIT			(1 << 4)

#define CANOPEN_REG_DEVICE_MFR_STATUS				0x100200
#define CANOPEN_REG_DEVICE_MFR_STATUS_INIT_DONE_BIT		(1 << 16)
#define CANOPEN_REG_DEVICE_MFR_STATUS_TRIG1_REACHED_BIT	(1 << 17)
#define CANOPEN_REG_DEVICE_MFR_STATUS_TRIG2_REACHED_BIT	(1 << 18)
#define CANOPEN_REG_DEVICE_MFR_STATUS_TRIG3_REACHED_BIT	(1 << 19)
#define CANOPEN_REG_DEVICE_MFR_STATUS_TRIG4_REACHED_BIT	(1 << 20)
#define CANOPEN_REG_DEVICE_MFR_STATUS_AUTORUN_BIT		(1 << 21)
#define CANOPEN_REG_DEVICE_MFR_STATUS_LIM_SW_POS_BIT		(1 << 22)
#define CANOPEN_REG_DEVICE_MFR_STATUS_LIM_SW_NEG_BIT		(1 << 23)
#define CANOPEN_REG_DEVICE_MFR_STATUS_CAPTURE_BIT			(1 << 24)
#define CANOPEN_REG_DEVICE_MFR_STATUS_CMD_REACHED_BIT		(1 << 25)
#define CANOPEN_REG_DEVICE_MFR_STATUS_MOTOR_I2T_BIT		(1 << 26)
#define CANOPEN_REG_DEVICE_MFR_STATUS_DRIVE_I2T_BIT		(1 << 27)
#define CANOPEN_REG_DEVICE_MFR_STATUS_FAULT_BIT			(uint32_t)(1 << 31)

#define CANOPEN_REG_DEVICE_NAME						0x100800
#define CANOPEN_REG_DEVICE_HW_VER					0x100900
#define CANOPEN_REG_DEVICE_SW_VER					0x100A00
#define CANOPEN_REG_DEVICE_SERIAL					0x101800
#define CANOPEN_REG_DEVICE_SYNC_COB_ID				0x100500
#define CANOPEN_REG_DEVICE_CYCLE_PERIOD				0x100600
#define CANOPEN_REG_RXPDO_BASE						0x140000
#define CANOPEN_REG_TXPDO_BASE						0x180000

#define CANOPEN_REG_MANUFACTURER_SEGMENT			0x200000
#define CANOPEN_REG_MOTION_ERROR					0x200000
#define CANOPEN_REG_MOTION_ERROR_CAN_ERROR_BIT		(1 << 0)
#define CANOPEN_REG_MOTION_ERROR_SHORT_BIT			(1 << 1)
#define CANOPEN_REG_MOTION_ERROR_INVALID_SETUP_BIT	(1 << 2)
#define CANOPEN_REG_MOTION_ERROR_CONTROL_ERR_BIT	(1 << 3)
#define CANOPEN_REG_MOTION_ERROR_COM_ERR_BIT		(1 << 4)
#define CANOPEN_REG_MOTION_ERROR_POS_WRAP_BIT		(1 << 5)
#define CANOPEN_REG_MOTION_ERROR_LIM_SW_POS_BIT		(1 << 6)
#define CANOPEN_REG_MOTION_ERROR_LIM_SW_NEG_BIT		(1 << 7)
#define CANOPEN_REG_MOTION_ERROR_OVCT_FAULT_BIT		(1 << 8)
#define CANOPEN_REG_MOTION_ERROR_I2T_FAULT_BIT		(1 << 9)
#define CANOPEN_REG_MOTION_ERROR_MOT_OVT_BIT		(1 << 10)
#define CANOPEN_REG_MOTION_ERROR_DRV_OVT_BIT		(1 << 11)
#define CANOPEN_REG_MOTION_ERROR_VHI_BIT			(1 << 12)
#define CANOPEN_REG_MOTION_ERROR_VLO_BIT			(1 << 13)
#define CANOPEN_REG_MOTION_ERROR_CMD_ERR_BIT		(1 << 14)
#define CANOPEN_REG_MOTION_ERROR_NO_EN_BIT			(1 << 15)

#define CANOPEN_REG_MOTION_ERROR_MASK				0x200100
#define CANOPEN_REG_DETAILED_ERROR					0x200200
#define CANOPEN_REG_DETAILED_ERROR2					0x200900
#define CANOPEN_REG_EXT_REFERENCE					0x201C00
#define CANOPEN_REG_EXT_REFERENCE_TYPE				0x201D00
#define CANOPEN_REG_EXT_REFERENCE_TYPE_ONLINE		1
#define CANOPEN_REG_EXT_REFERENCE_TYPE_ANALOG		2
#define CANOPEN_REG_TML_RUN							0x207700
#define CANOPEN_REG_TS_CURRENT_ACTUAL				0x207E00

#define CANOPEN_REG_DRIVE_CONN_FAULT_ACTION			0x600700
#define CANOPEN_REG_DRIVE_ERROR_CODE				0x603F00

#define CANOPEN_REG_DRIVE_CONTROL					0x604000
#define CANOPEN_REG_DRIVE_CONTROL_SW_ON_BIT			(1 << 0)
#define CANOPEN_REG_DRIVE_CONTROL_V_EN_BIT			(1 << 1)
#define CANOPEN_REG_DRIVE_CONTROL_RUN_BIT			(1 << 2)
#define CANOPEN_REG_DRIVE_CONTROL_OP_EN_BIT			(1 << 3)
#define CANOPEN_REG_DRIVE_CONTROL_MODE1_BIT			(1 << 4)
#define CANOPEN_REG_DRIVE_CONTROL_MODE2_BIT			(1 << 5)
#define CANOPEN_REG_DRIVE_CONTROL_FAULT_RST_BIT		(1 << 7)
#define CANOPEN_REG_DRIVE_CONTROL_HALT_BIT			(1 << 8)

#define CANOPEN_REG_DRIVE_STATUS					0x604100
#define CANOPEN_REG_DRIVE_STATUS_RDY_BIT			(1 << 0)
#define CANOPEN_REG_DRIVE_STATUS_SW_ON_BIT			(1 << 1)
#define CANOPEN_REG_DRIVE_STATUS_OP_EN_BIT			(1 << 2)
#define CANOPEN_REG_DRIVE_STATUS_FAULT_BIT			(1 << 3)
#define CANOPEN_REG_DRIVE_STATUS_VMOT_BIT			(1 << 4)
#define CANOPEN_REG_DRIVE_STATUS_QSTOP_BIT			(1 << 5)
#define CANOPEN_REG_DRIVE_STATUS_SW_ON_DIS_BIT		(1 << 6)
#define CANOPEN_REG_DRIVE_STATUS_WARNING_BIT		(1 << 7)
#define CANOPEN_REG_DRIVE_STATUS_HOMING_BIT			(1 << 8)
#define CANOPEN_REG_DRIVE_STATUS_REMOTE_BIT			(1 << 9)
#define CANOPEN_REG_DRIVE_STATUS_TARGET_REACHED_BIT	(1 << 10)
#define CANOPEN_REG_DRIVE_STATUS_INTERNAL_LIMIT_BIT	(1 << 11)
#define CANOPEN_REG_DRIVE_STATUS_EVENT_BIT			(1 << 14)
#define CANOPEN_REG_DRIVE_STATUS_AXIS_ON_BIT		(1 << 15)

#define CANOPEN_REG_DRIVE_REQUESTED_MODE			0x606000
#define CANOPEN_REG_DRIVE_CURRENT_MODE				0x606100
#define CANOPEN_REG_DRIVE_MODE_EXT_REF_TORQUE		(-5)
#define CANOPEN_REG_DRIVE_MODE_EXT_REF_SPEED		(-4)
#define CANOPEN_REG_DRIVE_MODE_EXT_REF_POS			(-3)
#define CANOPEN_REG_DRIVE_MODE_CAM_POS				(-2)
#define CANOPEN_REG_DRIVE_MODE_GEAR_POS				(-1)
#define CANOPEN_REG_DRIVE_MODE_PROFILE_POS			(1)
#define CANOPEN_REG_DRIVE_MODE_PROFILE_VEL			(3)
#define CANOPEN_REG_DRIVE_MODE_HOMING				(6)
#define CANOPEN_REG_DRIVE_MODE_INTERP_POS			(7)

#define CANOPEN_REG_DRIVE_FAULT_ACTION				0x605e00
#define CANOPEN_REG_DRIVE_POSITION_DEMAND			0x606200
#define CANOPEN_REG_DRIVE_POSITION_ACTUAL			0x606300
#define CANOPEN_REG_DRIVE_POSITION_MEASURED			0x606400
#define CANOPEN_REG_DRIVE_FOLLOWING_ERROR_WINDOW	0x606500
#define CANOPEN_REG_DRIVE_POSITION_WINDOW			0x606700
#define CANOPEN_REG_DRIVE_VELOCITY_DEMAND			0x606B00
#define CANOPEN_REG_DRIVE_VELOCITY_ACTUAL			0x606C00
#define CANOPEN_REG_DRIVE_VELOCITY_WINDOW			0x606D00
#define CANOPEN_REG_DRIVE_VELOCITY_THRESHOLD		0x606F00
#define CANOPEN_REG_DRIVE_VELOCITY_THRESHOLD_TIME	0x607000
#define CANOPEN_REG_DRIVE_TARGET_TORQUE				0x607100
#define CANOPEN_REG_DRIVE_MAX_CURRENT				0x607300
#define CANOPEN_REG_DRIVE_TORQUE_DEMAND				0x607400
#define CANOPEN_REG_DRIVE_CURRENT_RATED				0x607500
#define CANOPEN_REG_DRIVE_CURRENT_ACTUAL			0x607800
#define CANOPEN_REG_DRIVE_DC_VOLTAGE				0x607900
#define CANOPEN_REG_DRIVE_TARGET_POSITION			0x607A00
#define CANOPEN_REG_DRIVE_POLARITY					0x607E00
#define CANOPEN_REG_DRIVE_PROFILE_VELOCITY			0x608100
#define CANOPEN_REG_DRIVE_PROFILE_ACCELERATION		0x608300
#define CANOPEN_REG_DRIVE_PROFILE_DECELERATION		0x608400
#define CANOPEN_REG_DRIVE_FOLLOWING_ERROR			0x60F400
#define CANOPEN_REG_DRIVE_TARGET_VELOCITY			0x60FF00
#define CANOPEN_REG_DRIVE_SUPPORTED_MODES			0x650200

#define CANOPEN_PDO_TYPE_ACYCLIC	0
#define CANOPEN_PDO_TYPE_CYCLIC(n)	(((n) < 1)?1:(((n) > 240)?240:(n)))
#define CANOPEN_PDO_TYPE_SYNC_RTR	252
#define CANOPEN_PDO_TYPE_ASYNC_RTR	253
#define CANOPEN_PDO_TYPE_ASYNC		254
#define CANOPEN_PDO_TYPE_ASYNC_255	255

#define CANOPEN_DEFAULT_TXPDO_COUNT 8
#define CANOPEN_DEFAULT_RXPDO_COUNT 8
#define CANOPEN_DEFAULT_PDO_MAP_COUNT 8

#define CANOPEN_TXPDO_BASE (uint16_t)0x1800
#define CANOPEN_RXPDO_BASE (uint16_t)0x1400

enum {
	CANOPEN_DEVICE_TYPE_DRIVE = 402
};

enum {
	CANOPEN_DEVICE_ERR_GENERIC	= 1,
	CANOPEN_DEVICE_ERR_CURRENT	= 1 << 1,
	CANOPEN_DEVICE_ERR_VOLTAGE	= 1 << 2,
	CANOPEN_DEVICE_ERR_TEMP		= 1 << 3,
	CANOPEN_DEVICE_ERR_COMM		= 1 << 4,
	CANOPEN_DEVICE_ERR_PROFILE	= 1 << 5,
};

enum {
	CANOPEN_LSS_CMD_SWITCH_MODE = 0x04,
	CANOPEN_LSS_CMD_SET_ID		= 0x11,
	CANOPEN_LSS_CMD_SET_BAUD	= 0x13,
	CANOPEN_LSS_CMD_SAVE		= 0x17,
	CANOPEN_LSS_CMD_GET_ID		= 0x5e,
	CANOPEN_LSS_CMD_RESET		= 0x80,
	CANOPEN_LSS_CMD_FASTSCAN	= 0x81
};

typedef enum {
	CANOPEN_LSS_STATE_OFF,
	CANOPEN_LSS_STATE_SCAN_WAIT_CONFIRM,
	CANOPEN_LSS_STATE_ENABLE_WAIT_CONFIRM,
	CANOPEN_LSS_STATE_SET_ID_WAIT_CONFIRM,
} canopen_lss_state_t;

typedef enum {
	CANOPEN_MASTER,
	CANOPEN_SLAVE
} canopen_mode_t;

enum {
	CANOPEN_WRITE,
	CANOPEN_READ,
	CANOPEN_ERROR
};

struct canopen_request {
	uint8_t node_id;
	int type;
	char *data;
	uint32_t length;
	struct mutex mx;
	struct list_head list;
};

struct canopen_device_type {
	uint16_t profile;
	const char *name;
};

struct canopen_serial_number {
	uint32_t part[4];
};

struct canopen_pdo_entry {
	uint32_t cob_id;
	uint8_t type;
	uint16_t inhibit_time;
	uint16_t event_time;
	uint32_t map[CANOPEN_DEFAULT_PDO_MAP_COUNT];
	uint8_t map_entries; // number of entries as exposed on the can bus
	uint8_t last[8]; // last transmitted message
	bool transmit;
	uint8_t sync_cycles;
};

// this is used when user configure the pdo
struct canopen_pdo_config {
	uint32_t cob_id;
	uint8_t index;
	uint8_t type;
	uint16_t inhibit_time;
	uint16_t event_time;
	uint32_t map[8];
};

#define CANOPEN_PDO_MAP_ENTRY(id, size) (uint32_t)((uint32_t)(id) << 8 | (uint32_t)(size))
#define CANOPEN_PDO_SIZE_32 0x20
#define CANOPEN_PDO_SIZE_16 0x10
#define CANOPEN_PDO_SIZE_8 0x08
#define CANOPEN_PDO_DISABLED 0x80000000
#define CANOPEN_COB_DISABLED 0x80000000

struct canopen_communication_profile {
	/*
	 * 0x1000
	 * This object describes the type of the logical device and its
	 * functionality. It is comprised of a 16 bit field that describes the
	 * device profile, and a second 16 bit field that gives additional
	 * information about the specific functionality of the device.
	 */
	uint32_t device_type;
	/*
	 * 0x1001
	 * This object is an error register for the device. It is a field of 8 bits, each of which indicates a particular type of error. If a bit is set to 1, the specified error has occurred.
	 * The bits have the following meaning:
	 * 0: generic error
	 * 1: current
	 * 2: voltage
	 * 3: temperature
	 * 4: communication error (overrun, error state)
	 * 5: device profile specific
	 * 6: reserved
	 * 7: manufacturer specific
	 */
	uint8_t error_flags;
	/*
	 * 0x1003
	 * This object holds errors that have occurred on the device and have been signaled via the Emergency object. It is an error history.
	 * Writing to sub-index 0 deletes the entire error history
	 */
	uint32_t error_history[8];
	/*
	 * 0x1005
	 * This object defines the COB ID of the synchronization object (SYNC).
	 * The device generates a SYNC message if bit 30 is set.
	 * The meaning of other bits is the same as for other communication objects.
	 */
	uint32_t sync_cob_id;
	/*
	 * 0x1006
	 * This object defines the communication cycle period, in microseconds.
	 * Its value is 0 if it is not used.
	 */
	uint32_t cycle_period;
	/*
	 * 0x1007
	 * This object contains the length of the time window for synchronous messages, in microseconds.
	 * Its value is 0 if it is not used.
	 */
	uint32_t sync_window_length;
	/*
	 * 0x1008
	 * This object contains the name of the device as given by the manufacturer.
	 */
	//const char *device_name;
	/*
	 * 0x1009
	 * This object contains the manufacturer hardware version description.
	 */
	//const char *hardware_version;
	/*
	 * 0x100A
	 * This object contains the manufacturer software version description.
	 */
	//const char *software_version;
	/*
	 * 0x1010
	 * This object controls the saving of parameters in non-volatile memory.
	 * With read access, the device provides information about its save capabilities. Sub-indexes reference different groups of parameters.
	 * Sub-index 1: all parameters
	 * Parameters are saved when 0x65766173 (ASCII value of "SAVE") is written to the appropriate sub-index.
	 */
	//uint32_t save;
	/*
	 * 0x1011
	 * This object controls the restoring of default parameters.
	 * With read access, the device provides information about its restore capabilities. Sub-indexes reference different groups of parameters.
	 * Sub-index 1: all parameters
	 * Parameters are restored when 0x64616F6C (ASCII value of "LOAD") is written to the appropriate sub-index.
	 */
	//uint32_t load;
	/*
	 * 0x1013
	 * This object contains the drives internal time at a resolution of
	 * microseconds. It can be mapped into a PDO in order to define a high
	 * resolution time stamp.  It can be used to synchronize clocks of multiple
	 * drives over CANopen network as follows: map object 1013h to RPDO, a
	 * high-resolution time stamp producer transmits a time stamp over the
	 * CANopen network, and each drive adjusts its internal clock according to
	 * the value that the producer sent.
	 */
	//uint32_t timestamp;
	/*
	 * 0x1014
	 * This object defines the COB-ID used for the emergency message (EMCY).
	 */
	//uint32_t emergency_cob_id;
	/*
	 * 0x1015
	 * This object defines the inhibit time used for the emergency message.
	 * The time must be a multiple of 100 milliseconds.
	 */
	uint16_t emergency_inhibit_time;
	/*
	 * 0x1018
	 * This object contains general information about the device.
	 * Sub-index 1 contains a unique value allocated each manufacturer.
	 * Sub-index 2 defines the manufacturer specific product code (device version).
	 * Sub-index 3 defines the revision number.
	 *      Bit 31-16 is the major revision number
	 *           Bit 15-0 the minor revision number.
	 *           Sub-index 4 defines a manufacturer specific serial number.
	 */
	uint32_t identity[4];

	/* PDO Configuration */
	struct canopen_pdo_entry rxpdo[CANOPEN_DEFAULT_RXPDO_COUNT];
	struct canopen_pdo_entry txpdo[CANOPEN_DEFAULT_TXPDO_COUNT];
};

struct canopen_drive_profile {
	/*!
	 * 0x6007
	 * This object indicates the action to be performed when one of the following events occurs:
	 * CAN bus off
	 * Heartbeat lost
	 * Node guarding lost
	 * NMT stopped (stop remote node indication activated)
	 * Reset communication (reset communication indication activated)
	 * Reset application (reset node indication activated)
	 *
	 *  The following value definitions are valid:
	 *  0 = No action
	 *  1 = Fault signal
	 *  2 = Disable voltage command
	 *  3 = Quick Stop command
	 *  -x = Manufacturer-specific
	 */
	uint8_t abort_action;
	/*
	 * 0x603F
	 * This object indicates the error code of the last error that occurred in the drive device.
	 */
	uint16_t error;
	/*
	 * 0x6040
	 * This object controls the CiA-402 FSA, CiA-402 modes and manufacturer-specific entities.
	 * This object is organized bit-wise. The bits have the following meaning:
	 * bit 0: switch on
	 * bit 1: enable voltage
	 * bit 2: quick stop
	 * bit 3: enable operation
	 * bit 4-6: mode-specific
	 * bit 7: fault reset
	 * bit 8: halt
	 * bit 9: mode-specific
	 * bit 10: reserved
	 * bit 11: begin on time
	 * bit 12-15: manufacturer-specific
	 */
	uint16_t control;
	/*
	 * 0x6041
	 * This object indicates the current state of the FSA, the operation mode and manufacturer-specific entities.
	 * This object is organized bit-wise. The bits have the following meaning:
	 * bit 0: ready to switch on
	 * bit 1: switched on
	 * bit 2: operation enabled
	 * bit 3: fault
	 * bit 4: voltage enabled
	 * bit 5: quick stop
	 * bit 6: switch on disabled
	 * bit 7: warning
	 * bit 8: manufacturer-specific
	 * bit 9: remote
	 * bit 10: target reached
	 * bit 11: internal limit active
	 * bit 12-13: mode-specific
	 * bit 14-15: manufacturer-specific
	 */
	uint16_t status;
	/*
	 * 0x6060
	 * The object selects the operational mode. This object shows only the
	 * value of the requested operation mode. The actual operation mode of the
	 * PDS is reflected in the Modes of Operation Display object (6061h) The
	 * following value definitions are valid:
	 * 0 = no mode change / no mode assigned
	 * 1 = profile position mode
	 * 2 = velocity mode
	 * 3 = profile velocity mode
	 * 4 = profile torque mode
	 * 5 = reserved
	 * 6 = homing mode
	 * 7 = interpolated position mode
	 * 8 = cyclic synchronous position mode
	 * 9 = cyclic synchronous velocity mode
	 * 10 = cyclic synchronous torque mode
	 * -x = manufacturer-specific
	 */
	uint8_t requested_mode;
	/*
	 * 0x6061
	 * This object indicates the actual operation mode.
	 * The following value definitions are valid:
	 * 0 = no mode change / no mode assigned
	 * 1 = profile position mode
	 * 2 = velocity mode
	 * 3 = profile velocity mode
	 * 4 = profile torque mode
	 * 5 = reserved
	 * 6 = homing mode
	 * 7 = interpolated position mode
	 * 8 = cyclic synchronous position mode
	 * 9 = cyclic synchronous velocity mode
	 * 10 = cyclic synchronous torque mode
	 * -x = manufacturer-specific
	 */
	uint8_t current_mode;
	/*
	 * 0x6062
	 * This object indicates the demanded position value.
	 */
	int32_t position_demand;
	/*
	 * 0x6063
	 * This object indicates the predicted position value after kalman filtering.
	 * Note: if internal value is a float, this value is truncated to nearest integer. 
	 */
	int32_t position_actual;
	/*
	 * 0x6064
	 * This object indicates the actual value of the position measurement device.
	 */
	int32_t position_measured;
	/*
	 * 0x6065
	 * This object indicates the symmetrical range of tolerated position values
	 * relative to the target position. If the current position is out of range
	 * a following error occurs.  This object indicates the range of tolerated
	 * position values symmetrically to the position demand value (object
	 * 6062h). If the following error actual value (object 60F4h) is out of the
	 * following error window, a following error occurs. A following error may
	 * occur when a drive is blocked, or an unreachable profile velocity
	 * occurs, or due to incorrect closed-loop coefficients. If the value of
	 * the following error window is FFFFFFFFh, following control is disabled.
	 */
	uint32_t following_error_window;
	/*
	 * 0x6067
	 * This object indicates the symmetrical range of accepted positions
	 * relative to the target position. If the actual value of the position
	 * encoder is within the position window, the target position is regarded
	 * as reached. If the value of the position window is FFFFFFFFh, position
	 * window control is disabled.
	 */
	uint32_t position_window;
	/* 
	 * 0x606B
	 * This object indicates the output value of the trajectory generator.
	 */
	int32_t velocity_demand;
	/*
	 * 0x606C
	 * This object indicates the actual velocity value derived either from the
	 * velocity sensor or the position sensor.
	 */
	int32_t velocity_actual;
	/*
	 * 0x606D
	 * This object indicates the velocity window.
	 */
	uint16_t velocity_window;
	/*
	 * 0x606F
	 * This object indicates the velocity threshold.
	 */
	uint16_t velocity_threshold;
	/*
	 * 0x6070
	 * This object indicates the velocity threshold time in ms
	 */
	uint16_t velocity_threshold_time;
	/*
	 * 0x6071
	 * This object indicates the input value for the torque controller in profile torque mode in mNm
	 */
	int16_t target_torque;
	/*
	 * 0x6073
	 * This object indicates the maximum permissible torque creating current in the motor in mA
	 */
	uint16_t max_current;
	/*
	 * 0x6074
	 * This object provides the command value for the current loop in mA
	 */
	int16_t torque_demand;
	/*
	 * 0x6075
	 * This object provides the motor rated current in mA
	 */
	uint32_t motor_rated_current;
	/*
	 * 0x6078
	 * This object indicates the actual value of the current. It corresponds to
	 * the current in the motor.
	 */
	uint16_t motor_current;
	/*
	 * 0x6079
	 * This object indicates the instantaneous DC link current voltage at the drive device in mV
	 */
	uint32_t dc_voltage;
	/*
	 * 0x607A
	 * This object indicates the commanded position to which the drive will
	 * move in position profile mode or cyclic synchronous position mode. The
	 * value of this object can be interpreted as absolute or relative
	 * depending on bit 6 of the controlword.
	 */
	int32_t target_position;
	/*
	 * 0x607B
	 * This object indicates the maximum and minimum position range limits. It
	 * limits the numerical range of the input value. Upon reaching or
	 * exceeding these limits, the input value automatically wraps to the other
	 * end of the range. Wrap-around of the input value may be prevented by
	 * setting software position limits as defined in the software position
	 * limit object (607Dh).
	 */
	int32_t min_position_range_limit;
	int32_t max_position_range_limit;
	/*
	 * 0x607C
	 * This object indicates the difference between the zero position for the
	 * application and the machine home position. After the machine home
	 * position is found and homing is completed, the zero position is offset
	 * from the home position by adding the home offset value to the home
	 * position. All subsequent absolute moves are executed relative to this
	 * new zero position.  If this object is not implemented, home offset is
	 * considered to be 0. Negative values indicate the opposite direction.
	 */
	int32_t home_offset;
	/*
	 * 0x607D
	 * This object indicates the maximum and minimum software position limits.
	 * These parameters define the absolute position limits for the position
	 * demand value and the position actual value. Every new target position is
	 * checked against these limits. The limit positions are always relative to
	 * the machine home position. Before being compared to the target position,
	 * they are corrected internally by the home offset, as follows:
	 *
	 * Corrected min position limit = (min position limit - home offset)
	 * Corrected max position limit = (max position limit - home offset)
	 */
	int32_t min_software_pos_limit;
	int32_t max_software_pos_limit;
	/*
	 * 0x607E
	 * Position demand value and position actual value are multiplied by 1 or
	 * -1, depending on the value of the polarity flag.
	 */
	uint8_t polarity;
	/*
	 * 0x6081
	 * This object indicates the commanded velocity normally attained at the
	 * end of the acceleration ramp during a profiled motion. It is valid for
	 * both directions of motion.
	 *
	 * This object is used in profile position mode and interpolated position
	 * mode.
	 */
	int32_t profile_velocity;
	/*
	 * 0x6083
	 * This object indicates the commanded acceleration.
	 *
	 * This object is used in the profile position mode, profile velocity mode,
	 * and interpolated position mode.
	 */
	uint32_t profile_acceleration;
	/*
	 * 0x6084
	 * This object indicates the deceleration.
	 *
	 * This object is used in the profile position mode, profile velocity mode,
	 * and interpolated position mode.
	 */
	uint32_t profile_deceleration;
	/*
	 * 0x6085
	 * This object indicates the deceleration used to stop the motor when the
	 * quick stop function is activated and the quick stop option code is set
	 * to 2 or 6.  The quick stop deceleration is also used if the fault
	 * reaction option code is 2 and the halt option code is 2.
	 */
	uint32_t quick_stop_deceleration;
	/*
	 * 0x6086
	 * This object indicates the type of motion profile used to perform a
	 * profiled motion.  The following value definitions are valid:
	 * 0 = linear ramp (trapezoidal profile)
	 */
	int16_t motion_profile_type;
	/*
	 * 0x6089
	 * The position notation index is used to scale the objects for which it mandatory.
	 * Note: the value of this object is fixed to factor = 1.
	 */
	int8_t position_notation_index;
	/*
	 * 0x608A
	 * This object indicates position units.
	 * Note: the value of this object is fixed to steps.
	 */
	uint8_t position_dimension_index;
	/*
	 * 0x608B
	 * The velocity notation index is used to scale the objects for which it mandatory.
	 * Note: the value of this object is fixed at 0.01.
	 */
	int8_t velocity_notation_index;
	/*
	 * 0x608C
	 * This object indicates velocity units.
	 * Note: the value of this object is fixed at rpm.
	 */
	int8_t velocity_dimension_index;
	/*
	 * 0x608D
	 * The acceleration notation index is used to scale the objects for which it mandatory.
	 * Note: the value of this object is fixed tat 0.01.
	 */
	int8_t acceleration_notation_index;
	/*
	 * 0x608E
	 * This object indicates acceleration units.
	 * Note: the value of this object is fixed at rpm/second
	 */
	uint8_t acceleration_dimension_index;
	/*
	 * 0x608F
	 * This object indicates the configured encoder increments and number of
	 * motor revolutions. It is calculated by the following formula: position
	 * encoder resolution = (encoder increments/motor revolutions)
	 */
	uint32_t encoder_increments;
	uint32_t motor_revolutions;
	/*
	 * 0x6098
	 * This object indicates the homing method to be used.
	 * The following value definitions are valid:
	 * -4 = homing on hard stop in positive direction with Index
	 * -3 = homing on hard stop in negative direction with Index
	 * -2 = homing on hard stop in positive direction
	 * -1 = homing on hard stop in negative direction
	 * 0 = no homing method assigned
	 * 1 = homing method 1 to be used
	 * .
	 * .
	 * 36 = homing method 36 to used
	 */
	int8_t homing_method;
	/*
	 * 0x6099
	 * This object indicates the commanded speeds used during homing procedure.
	 */
	uint32_t fast_homing_speed;
	uint32_t slow_homing_speed;
	/*
	 * 0x609A
	 * This object indicates the acceleration and deceleration to be used during homing operation.
	 */
	uint32_t homing_acceleration;
	/*
	 * 0x60C5
	 * This object indicates the maximum acceleration. It is used to limit the
	 * acceleration to an acceptable value in order to prevent the motor and
	 * the moved mechanics from being damaged.
	 */
	uint32_t max_acceleration;
	/*
	 * 0x60C6
	 * This object indicates the maximum deceleration. It is used to limit the
	 * deceleration to an acceptable value in order to prevent the motor and
	 * the moved mechanics from being damaged.
	 */
	uint32_t max_deceleration;
	/*
	 * 0x60F4
	 * This object indicates the actual value of the following error.
	 */
	int32_t following_error_actual;
	/*
	 * 0x60FA
	 * This object indicates the control effort as the output of the position
	 * control loop. In the position control function, notation of the control
	 * effort is mode-dependent and therefore not specified.
	 */
	int32_t control_effort;
	/*
	 * 0x60FD
	 * This object provides digital inputs.
	 * This object is organized bit-wise. The bits have the following meaning:
	 * bit 0: negative limit switch
	 * bit 1: positive limit switch
	 * bit 2: home switch
	 * bit 3: reserved
	 * bit 16-31: manufacturer-specific
	 * The bit values have the following meaning:
	 * 0 = switch is off
	 * 1 = switch is on
	 */
	uint32_t digital_inputs;
	/*
	 * 0x60FF
	 * This object indicates the configured target velocity and is used as
	 * input for the trajectory generator.
	 */
	int32_t target_velocity;
	/*
	 * 0x6402
	 * This object indicates the type of motor attached to and driven by the drive device.
	 * The following value definition is valid:
	 *  
	 *  0008h = stepper motor
	 *  0009h = micro-step stepper motor
	 */
	uint16_t motor_type;
	/*
	 * 0x6502
	 * This object provides information about the supported drive modes.
	 * This object is organized bit-wise. The bits have the following meaning:
	 * bit 0: profile position mode
	 * bit 1: velocity mode
	 * bit 2: profile velocity mode
	 * bit 3: profile torque mode
	 * bit 4: reserved
	 * bit 5: homing mode
	 * bit 6: interpolated position mode
	 * bit 7: cyclic synchronous position mode
	 * bit 8: cyclic synchronous velocity mode
	 * bit 9: cyclic synchronous torque mode
	 * bit 10-15: reserved
	 * bit 16-31: manufacturer-specific
	 *
	 *  The bit values have the following meaning:
	 *  0 = mode is not supported
	 *  1 = mode is supported
	 */
	uint32_t supported_modes;
};

struct canopen_counters {
	atomic_t sync_in;
};

struct canopen_listener {
	struct list_head list;
	void (*callback)(struct canopen_listener *self, uint8_t node_id, struct can_message *msg);
};

struct canopen {
	can_port_t port;
	canopen_mode_t mode;
	thread_t task;

	//struct list_head requests;
	struct mutex mx;
	struct semaphore quit;

	uint8_t address;
	struct vardir *vardir;
	struct vardir_entry_ops *var_ops;
	struct can_listener listener;
	struct canopen_counters cnt;

	struct list_head emcy_listeners;

	struct {
		struct mutex mx;
		struct canopen_lss_request {
			uint8_t cmd;
			uint8_t node_id;
			uint32_t serial[4];
			struct semaphore done;
			bool running;
			int result;
		} req;
		timestamp_t timeout;
		//uint32_t current[4]; // lssid of currently scanned device
		uint32_t id; // current part being checked
		uint8_t part;
		int bit, retry;
		int bits_per_test;
		bool confirmed; // whether last bit was confirmed
		bool mute; // this flag is set after lss is disabled on a node where it was enabled previously (after being unlocked) so that other nodes can be scanned
		bool mute_fastscan;
		bool enabled; // specifies whether the device will respond to lss messages
		bool unlocked; // specifies whether we will respond to lss enable command
		canopen_lss_state_t state;
		//struct work timeout_handler;
	} lss;

	struct {
		struct mutex mx;
		struct canopen_sdo_request {
			uint8_t cmd;
			uint8_t node_id;
			uint8_t *output;
			size_t output_len;
			uint32_t id;
			uint8_t len;
			struct semaphore done;
			bool running;
			int result;
			timestamp_t timeout;
		} req;
	} sdo;

	struct canopen_communication_profile profile;

	struct mutex lock;

	timestamp_t sync_timeout;
};

void canopen_init(struct canopen *self, struct vardir *directory, can_port_t can, canopen_mode_t mode);
void canopen_destroy(struct canopen *self);

void canopen_set_identity(struct canopen *self, uint32_t uuid[4]);
void canopen_set_node_id(struct canopen *self, uint8_t id);
void canopen_set_sync_period(struct canopen *self, uint32_t period_us);

int canopen_lss_reset(struct canopen *self);
int canopen_lss_find_node(struct canopen *self, struct canopen_serial_number *serial);
int canopen_lss_mode(struct canopen *self, uint8_t mode);
int canopen_lss_set_node_id(struct canopen *self, struct canopen_serial_number *serial, uint8_t node_id);
int canopen_lss_enable(struct canopen *self, struct canopen_serial_number *node_serial);

int canopen_sdo_read(struct canopen *self, uint8_t node_id, uint32_t dict, void *data, size_t size);
int canopen_sdo_read_u32(struct canopen *self, uint8_t node_id, uint32_t dic, uint32_t *value);
int canopen_sdo_read_i32(struct canopen *self, uint8_t node_id, uint32_t dic, int32_t *value);
int canopen_sdo_read_u16(struct canopen *self, uint8_t node_id, uint32_t dic, uint16_t *value);
int canopen_sdo_read_i16(struct canopen *self, uint8_t node_id, uint32_t dic, int16_t *value);
int canopen_sdo_read_u8(struct canopen *self, uint8_t node_id, uint32_t dic, uint8_t *value);
int canopen_sdo_read_i8(struct canopen *self, uint8_t node_id, uint32_t dic, int8_t *value);

int canopen_sdo_write(struct canopen *self, uint8_t node_id, uint32_t dict, const uint8_t *data, size_t size);
int canopen_sdo_write_u32(struct canopen *self, uint8_t node_id, uint32_t dic, uint32_t value);
int canopen_sdo_write_i32(struct canopen *self, uint8_t node_id, uint32_t dic, int32_t value);
int canopen_sdo_write_u16(struct canopen *self, uint8_t node_id, uint32_t dic, uint16_t value);
int canopen_sdo_write_i16(struct canopen *self, uint8_t node_id, uint32_t dic, int16_t value);
int canopen_sdo_write_u8(struct canopen *self, uint8_t node_id, uint32_t dic, uint8_t value);
int canopen_sdo_write_i8(struct canopen *self, uint8_t node_id, uint32_t dic, int8_t value);

int canopen_sdo_read_device_type(struct canopen *self, uint8_t node_id, struct canopen_device_type *type);
int canopen_sdo_read_serial(struct canopen *self, uint8_t node_id, struct canopen_serial_number *serial);

int canopen_pdo_rx(struct canopen *self, uint8_t node_id, const struct canopen_pdo_config *conf);
int canopen_pdo_tx(struct canopen *self, uint8_t node_id, const struct canopen_pdo_config *conf);

// used for locally sending or receiving pdos
int canopen_pdo_tx_local(struct canopen *self, const struct canopen_pdo_config *conf);
int canopen_pdo_rx_local(struct canopen *self, const struct canopen_pdo_config *conf);

int canopen_pdo_transmit(struct canopen *self, uint16_t cob_id, const uint8_t data[8]);
int canopen_send_sync(struct canopen *self);

int canopen_nmt_enable(struct canopen *self, uint8_t node_id);
int canopen_nmt_reset(struct canopen *self, uint8_t node_id);

void canopen_listener_init(struct canopen_listener *self, void (*callback)(struct canopen_listener *self, uint8_t node_id, struct can_message *msg));

void canopen_register_listener(struct canopen *self, struct canopen_listener *l);

const char *canopen_strerror(int32_t err);
