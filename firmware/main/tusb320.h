#pragma once

/********************************************
 * TUSB320 CSR Register Definitions
 * (Datasheet: TUSB320, TUSB320I, SLLSEN9F – MAY 2015 – REVISED MARCH 2022)
 ********************************************/

/***************************************
 * Register: DEVICE_ID (Addresses 0x00 – 0x07)
 * Description: Returns an 8-byte ASCII string for the device ID.
 * For TUSB320, expected string: "TUSB320" (plus a leading 0x00)
 * Access: Read-only
 ***************************************/
#define REG_DEVICE_ID 0x00

typedef struct {
    uint8_t id[8];
} device_id_t;


/***************************************
 * Register: 0x08 – CURRENT_MODE & Accessory State
 ***************************************/
#define REG_08 0x08

// Bit masks for register 0x08
#define CURRENT_MODE_ADVERTISE_MASK 0xC0  // Bits 7:6
#define CURRENT_MODE_DETECT_MASK    0x30  // Bits 5:4
#define ACCESSORY_CONNECTED_MASK    0x0E  // Bits 3:1
#define ACTIVE_CABLE_DETECTION_MASK 0x01  // Bit 0

// Enum definitions for CURRENT_MODE_ADVERTISE (bits 7:6)
typedef enum {
    CURRENT_MODE_ADVERTISE_DEFAULT = 0,   // 00: Default (500 mA / 900 mA initial value at startup)
    CURRENT_MODE_ADVERTISE_MEDIUM,        // 01: Medium (1.5 A)
    CURRENT_MODE_ADVERTISE_HIGH,          // 10: High (3 A)
    CURRENT_MODE_ADVERTISE_RESERVED       // 11: Reserved
} current_mode_advertise_t; // Use mask CURRENT_MODE_ADVERTISE_MASK

// Enum definitions for CURRENT_MODE_DETECT (bits 5:4)
typedef enum {
    CURRENT_MODE_DETECT_DEFAULT = 0,      // 00: Default (startup value)
    CURRENT_MODE_DETECT_MEDIUM,           // 01: Medium
    CURRENT_MODE_DETECT_CHARGE_ACC,       // 10: Charge through accessory – 500 mA
    CURRENT_MODE_DETECT_HIGH              // 11: High
} current_mode_detect_t; // Use mask CURRENT_MODE_DETECT_MASK

// Enum definitions for ACCESSORY_CONNECTED (bits 3:1)
typedef enum {
    ACCESSORY_NONE = 0,           // 000: No accessory attached (default)
    ACCESSORY_RESERVED1,          // 001: Reserved
    ACCESSORY_RESERVED2,          // 010: Reserved
    ACCESSORY_RESERVED3,          // 011: Reserved
    ACCESSORY_AUDIO,              // 100: Audio accessory
    ACCESSORY_AUDIO_CHARGED,      // 101: Audio charged thru accessory
    ACCESSORY_DEBUG,              // 110: Debug accessory
    ACCESSORY_RESERVED4           // 111: Reserved
} accessory_connected_t; // Use mask ACCESSORY_CONNECTED_MASK

// Enum definitions for ACTIVE_CABLE_DETECTION (bit 0)
typedef enum {
    ACTIVE_CABLE_NOT_DETECTED = 0, // 0: Not detected
    ACTIVE_CABLE_DETECTED = 1      // 1: Detected
} active_cable_detection_t; // Use mask ACTIVE_CABLE_DETECTION_MASK

// Structure for register 0x08 state
typedef struct {
    current_mode_advertise_t current_mode_advertise; // Bits 7:6
    current_mode_detect_t    current_mode_detect;      // Bits 5:4
    accessory_connected_t    accessory_connected;      // Bits 3:1
    active_cable_detection_t active_cable_detection;   // Bit 0
} reg08_state_t;


/***************************************
 * Register: 0x09 – Attachment, Cable & Interrupt State
 ***************************************/
#define REG_09 0x09

// Bit masks for register 0x09
#define ATTACHED_STATE_MASK   0xC0  // Bits 7:6
#define CABLE_DIR_MASK        0x20  // Bit 5
#define INTERRUPT_STATUS_MASK 0x10  // Bit 4
#define DRP_DUTY_CYCLE_MASK   0x06  // Bits 2:1
// Bit 0 is Reserved and not needed

// Enum definitions for ATTACHED_STATE (bits 7:6)
typedef enum {
    ATTACHED_NOT_ATTACHED = 0,  // 00: Not attached (default)
    ATTACHED_SRC          = 1,  // 01: Attached.SRC (DFP)
    ATTACHED_SNK          = 2,  // 10: Attached.SNK (UFP)
    ATTACHED_ACCESSORY    = 3   // 11: Attached to an accessory
} attached_state_t; // Use mask ATTACHED_STATE_MASK

// Enum definitions for CABLE_DIR (bit 5)
typedef enum {
    CABLE_DIR_CC1 = 0,  // 0: CC1
    CABLE_DIR_CC2 = 1   // 1: CC2 (default)
} cable_dir_t; // Use mask CABLE_DIR_MASK

// Enum definitions for INTERRUPT_STATUS (bit 4)
typedef enum {
    INTERRUPT_CLEAR = 0,        // 0: Clear
    INTERRUPT_OCCURRED = 1      // 1: Interrupt (when INT_N is pulled low)
} interrupt_status_t; // Use mask INTERRUPT_STATUS_MASK

// Enum definitions for DRP_DUTY_CYCLE (bits 2:1)
typedef enum {
    DRP_DUTY_30 = 0,  // 00: 30% (default)
    DRP_DUTY_40 = 1,  // 01: 40%
    DRP_DUTY_50 = 2,  // 10: 50%
    DRP_DUTY_60 = 3   // 11: 60%
} drp_duty_cycle_t; // Use mask DRP_DUTY_CYCLE_MASK

// Structure for register 0x09 state (without reserved bit 0)
typedef struct {
    attached_state_t   attached_state;     // Bits 7:6
    cable_dir_t        cable_dir;          // Bit 5
    interrupt_status_t interrupt_status;   // Bit 4
    drp_duty_cycle_t   drp_duty_cycle;     // Bits 2:1
} reg09_state_t;


/***************************************
 * Register: 0x0A – Debounce, Mode Select & I2C Soft Reset
 ***************************************/
#define REG_0A 0x0A

// Bit masks for register 0x0A
#define DEBOUNCE_MASK       0xC0  // Bits 7:6
#define MODE_SELECT_MASK    0x30  // Bits 5:4
#define I2C_SOFT_RESET_MASK 0x08  // Bit 3
// Bits 2:0 are reserved

// Enum definitions for DEBOUNCE (bits 7:6)
typedef enum {
    DEBOUNCE_133MS = 0, // 00: 133 ms (default)
    DEBOUNCE_116MS = 1, // 01: 116 ms
    DEBOUNCE_151MS = 2, // 10: 151 ms
    DEBOUNCE_168MS = 3  // 11: 168 ms
} debounce_t; // Use mask DEBOUNCE_MASK

// Enum definitions for MODE_SELECT (bits 5:4)
typedef enum {
    MODE_SELECT_PORT_DEFAULT = 0, // 00: Maintain mode according to PORT pin selection (default)
    MODE_SELECT_UFP          = 1, // 01: UFP mode (unattached.SNK)
    MODE_SELECT_DFP          = 2, // 10: DFP mode (unattached.SRC)
    MODE_SELECT_DRP          = 3  // 11: DRP mode (start from unattached.SNK)
} mode_select_t; // Use mask MODE_SELECT_MASK

// Enum definitions for I2C_SOFT_RESET (bit 3)
typedef enum {
    I2C_SOFT_RESET_INACTIVE = 0, // 0: Normal operation
    I2C_SOFT_RESET_ACTIVE   = 1  // 1: Reset initiated (self-clearing)
} i2c_soft_reset_t; // Use mask I2C_SOFT_RESET_MASK

// Structure for register 0x0A state
typedef struct {
    debounce_t       debounce;       // Bits 7:6
    mode_select_t    mode_select;    // Bits 5:4
    i2c_soft_reset_t i2c_soft_reset; // Bit 3
    // Bits 2:0 are reserved
} reg0A_state_t;


/***************************************
 * Register: 0x45 – Disable Rd/Rp & Reserved
 ***************************************/
#define REG_45 0x45

// Bit masks for register 0x45
#define DISABLE_RD_RP_MASK 0x04  // Bit 2
// Bits 7:3 and 1:0 are reserved

// Enum definitions for DISABLE_RD_RP (bit 2)
typedef enum {
    RD_RP_NORMAL   = 0, // 0: Normal operation (default)
    RD_RP_DISABLED = 1  // 1: Disable Rd and Rp
} rd_rp_t; // Use mask DISABLE_RD_RP_MASK

// Structure for register 0x45 state
typedef struct {
    rd_rp_t disable_rd_rp; // Bit 2
    // Other bits are reserved
} reg45_state_t;
