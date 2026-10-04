/*
 * DaMiao (DM) motor CAN protocol for Zephyr.
 *
 * Implements the protocol used by DaMiao joint motors and the DM3520-1EC
 * driver (DM-S3519-1EC gearmotor):
 *
 *   MIT mode        CAN ID 0x000 + ESC_ID   pos16 vel12 kp12 kd12 tor12
 *   pos/vel mode    CAN ID 0x100 + ESC_ID   float pos, float vel
 *   velocity mode   CAN ID 0x200 + ESC_ID   float vel
 *   force-position  CAN ID 0x300 + ESC_ID   float pos, u16 vel*100, u16 cur*10000
 *   register access CAN ID 0x7FF            read 0x33 / write 0x55 / save 0xAA
 *   feedback        CAN ID MST_ID           id|state, pos16, vel12, tor12, Tmos, Tcoil
 *
 * plus the DM3519 "one-to-four" current frame (RoboMaster C620 style):
 *   0x200 (motors 1-4) / 0x1FF (5-8): four int16 currents, 16384 LSB = 20 A
 *   feedback 0x200 + motor number: pos16 (8192/rev), rpm16, cur16, Tcoil, Tmos
 *
 * Reference: DaMiao SDK, SDK/电机控制例程/stm32例程/dm_ctrl(DM3519 一拖四)/User.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef DM_MOTOR_H_
#define DM_MOTOR_H_

#include <stdbool.h>
#include <stdint.h>

#ifndef DM_HOST_TEST
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* CAN ID offsets per control mode (added to the motor's ESC_ID). */
#define DM_ID_MIT      0x000
#define DM_ID_POS_VEL  0x100
#define DM_ID_VEL      0x200
#define DM_ID_PSI      0x300  /* force-position hybrid */
#define DM_ID_REG      0x7FF  /* register read/write/save */

/* MIT-mode gain ranges (fixed by the protocol). */
#define DM_KP_MIN 0.0f
#define DM_KP_MAX 500.0f
#define DM_KD_MIN 0.0f
#define DM_KD_MAX 5.0f

/* Values of the CMODE register (RID 10). */
enum dm_ctrl_mode {
	DM_MODE_MIT = 1,
	DM_MODE_POS_VEL = 2,
	DM_MODE_VEL = 3,
	DM_MODE_PSI = 4,
};

/* Register IDs for dm_motor_read_reg()/dm_motor_write_reg(). */
enum dm_reg {
	DM_RID_UV_VALUE = 0,  /* float  under-voltage threshold */
	DM_RID_KT_VALUE = 1,  /* float  torque constant */
	DM_RID_OT_VALUE = 2,  /* float  over-temperature threshold */
	DM_RID_OC_VALUE = 3,  /* float  over-current threshold */
	DM_RID_ACC = 4,       /* float  acceleration */
	DM_RID_DEC = 5,       /* float  deceleration */
	DM_RID_MAX_SPD = 6,   /* float  max speed */
	DM_RID_MST_ID = 7,    /* u32    master (feedback) CAN ID */
	DM_RID_ESC_ID = 8,    /* u32    motor CAN ID */
	DM_RID_TIMEOUT = 9,   /* u32    command timeout, ms */
	DM_RID_CMODE = 10,    /* u32    control mode, enum dm_ctrl_mode */
	DM_RID_DAMP = 11,     /* float  motor damping */
	DM_RID_INERTIA = 12,  /* float  motor inertia */
	DM_RID_HW_VER = 13,   /* u32 */
	DM_RID_SW_VER = 14,   /* u32 */
	DM_RID_SN = 15,       /* u32 */
	DM_RID_NPP = 16,      /* u32    pole pairs */
	DM_RID_RS = 17,       /* float  phase resistance */
	DM_RID_LS = 18,       /* float  phase inductance */
	DM_RID_FLUX = 19,     /* float */
	DM_RID_GR = 20,       /* float  gear ratio */
	DM_RID_PMAX = 21,     /* float  position mapping range, rad */
	DM_RID_VMAX = 22,     /* float  velocity mapping range, rad/s */
	DM_RID_TMAX = 23,     /* float  torque mapping range, N.m */
	DM_RID_I_BW = 24,     /* float  current loop bandwidth */
	DM_RID_KP_ASR = 25,   /* float  speed loop Kp */
	DM_RID_KI_ASR = 26,   /* float  speed loop Ki */
	DM_RID_KP_APR = 27,   /* float  position loop Kp */
	DM_RID_KI_APR = 28,   /* float  position loop Ki */
	DM_RID_OV_VALUE = 29, /* float  over-voltage threshold */
	DM_RID_GREF = 30,     /* float  gear efficiency */
	DM_RID_DETA = 31,     /* float  speed loop damping */
	DM_RID_V_BW = 32,     /* float  speed loop filter bandwidth */
	DM_RID_IQ_CL = 33,    /* float  current loop gain boost */
	DM_RID_VL_CL = 34,    /* float  speed loop gain boost */
	DM_RID_CAN_BR = 35,   /* u32    CAN bitrate code, enum dm_can_br */
	DM_RID_SUB_VER = 36,  /* u32 */
	DM_RID_U_OFF = 50,    /* float */
	DM_RID_V_OFF = 51,    /* float */
	DM_RID_K1 = 52,       /* float */
	DM_RID_K2 = 53,       /* float */
	DM_RID_M_OFF = 54,    /* float  angle offset */
	DM_RID_DIR = 55,      /* float  direction */
	DM_RID_P_M = 80,      /* float  motor position */
	DM_RID_X_OUT = 81,    /* float  output shaft position */
};

/* CAN_BR register codes; >1M means CAN FD with bit-rate switching. */
enum dm_can_br {
	DM_CAN_BR_125K = 0,
	DM_CAN_BR_200K = 1,
	DM_CAN_BR_250K = 2,
	DM_CAN_BR_500K = 3,
	DM_CAN_BR_1M = 4,
	DM_CAN_BR_2M = 5,
	DM_CAN_BR_2M5 = 6,
	DM_CAN_BR_3M2 = 7,
	DM_CAN_BR_4M = 8,
	DM_CAN_BR_5M = 9,
};

/* Error/state nibble in feedback byte 0 (DaMiao manual). */
enum dm_state {
	DM_STATE_DISABLED = 0,
	DM_STATE_ENABLED = 1,
	DM_STATE_OVER_VOLTAGE = 8,
	DM_STATE_UNDER_VOLTAGE = 9,
	DM_STATE_OVER_CURRENT = 0xA,
	DM_STATE_MOS_OVER_TEMP = 0xB,
	DM_STATE_COIL_OVER_TEMP = 0xC,
	DM_STATE_COMM_LOST = 0xD,
	DM_STATE_OVERLOAD = 0xE,
};

/* Decoded feedback (standard DaMiao frame on MST_ID). */
struct dm_feedback {
	uint8_t id;      /* low nibble of byte 0: ESC_ID & 0xF */
	uint8_t state;   /* enum dm_state */
	uint16_t p_int;  /* raw 16-bit position */
	uint16_t v_int;  /* raw 12-bit velocity */
	uint16_t t_int;  /* raw 12-bit torque */
	float pos;       /* rad, [-PMAX, PMAX] */
	float vel;       /* rad/s, [-VMAX, VMAX] */
	float tor;       /* N.m, [-TMAX, TMAX] */
	float t_mos;     /* degC */
	float t_coil;    /* degC */
	int64_t stamp;   /* k_uptime_get() at reception */
	bool valid;
};

/* Decoded DM3519 one-to-four feedback (0x201..0x208). */
struct dm3519_feedback {
	float pos_deg;   /* 0..360, mechanical angle from the 8192-count encoder */
	int16_t rpm;
	float cur;       /* A */
	float t_coil;
	float t_mos;
	int64_t stamp;
	bool valid;
};

#ifndef DM_HOST_TEST
struct dm_motor {
	const struct device *can;
	uint16_t esc_id;  /* motor CAN ID (RID 8); DaMiao examples use 0x01 */
	uint16_t mst_id;  /* feedback CAN ID (RID 7); factory default 0x00 */
	bool fd;          /* send CAN FD frames with BRS (driver CAN_BR code > 1M) */
	float p_max;      /* must match the motor's PMAX register */
	float v_max;      /* VMAX */
	float t_max;      /* TMAX */
	struct dm_feedback fb;
	struct dm3519_feedback fb3519;
	/*
	 * Register replies (0x33 read / 0x55 write / 0xAA save) arrive on MST_ID
	 * like feedback frames; they are only interpreted while a request is
	 * pending.
	 */
	volatile bool reg_pending;
	uint8_t reg_cmd;
	uint8_t last_rid;
	uint32_t last_reg_raw;
	struct k_sem reg_sem;
	int rx_filter_fb;
	int rx_filter_3519;
};

/* Official defaults for the DM-S3519-1EC (2026 DaMiao selection table). */
#define DM_S3519_PMAX 12.5f
#define DM_S3519_VMAX 200.0f
#define DM_S3519_TMAX 10.0f
/* DM-J4310-2EC, for comparison. */
#define DM_J4310_PMAX 12.5f
#define DM_J4310_VMAX 30.0f
#define DM_J4310_TMAX 10.0f

/* --- setup ------------------------------------------------------------- */

/**
 * Initialise a motor object and install RX filters for its feedback ID, the
 * 0x7FF register-reply ID and (if esc_id is 1..8) the DM3519 one-to-four
 * feedback ID 0x200 + esc_id.  The CAN device must already be started.
 */
int dm_motor_init(struct dm_motor *m, const struct device *can, uint16_t esc_id, uint16_t mst_id,
		  float p_max, float v_max, float t_max);

/**
 * Read PMAX/VMAX/TMAX (RIDs 0x15-0x17) from the driver and adopt them for the
 * MIT/feedback mapping.  Returns 0 if all three were read, else the error.
 */
int dm_motor_sync_limits(struct dm_motor *m, k_timeout_t timeout);

/** Put the CAN controller in normal (or loopback) mode, optionally with FD. */
int dm_bus_configure(const struct device *can, bool loopback, bool fd);

/* --- mode commands ----------------------------------------------------- */

int dm_motor_enable(const struct dm_motor *m, uint16_t mode_id);   /* FF..FC */
int dm_motor_disable(const struct dm_motor *m, uint16_t mode_id);  /* FF..FD */
int dm_motor_set_zero(const struct dm_motor *m, uint16_t mode_id); /* FF..FE */
int dm_motor_clear_error(const struct dm_motor *m, uint16_t mode_id); /* FF..FB */

/* --- control frames ---------------------------------------------------- */

int dm_motor_mit(const struct dm_motor *m, float pos, float vel, float kp, float kd, float tor);
int dm_motor_pos_vel(const struct dm_motor *m, float pos, float vel);
int dm_motor_vel(const struct dm_motor *m, float vel);
int dm_motor_psi(const struct dm_motor *m, float pos, float vel, float cur);

/**
 * DM3519 one-to-four current command: currents in ampere for the four motors
 * addressed by the frame (ID 0x200 for motors 1-4, 0x1FF for motors 5-8).
 */
int dm3519_set_current(const struct device *can, uint16_t frame_id, const float cur[4]);

/* --- registers --------------------------------------------------------- */

/** Read a register; blocks up to timeout for the reply. Raw 32-bit value out. */
int dm_motor_read_reg(struct dm_motor *m, enum dm_reg rid, uint32_t *raw, k_timeout_t timeout);
int dm_motor_read_reg_float(struct dm_motor *m, enum dm_reg rid, float *val, k_timeout_t timeout);
int dm_motor_write_reg(struct dm_motor *m, enum dm_reg rid, uint32_t raw, k_timeout_t timeout);
int dm_motor_write_reg_float(struct dm_motor *m, enum dm_reg rid, float val, k_timeout_t timeout);
/** Save registers to flash (0xAA). The motor must be disabled first. */
int dm_motor_save_regs(struct dm_motor *m, k_timeout_t timeout);
/** Ask the motor to send one feedback frame without a control command. */
int dm_motor_request_feedback(const struct dm_motor *m);

#endif /* DM_HOST_TEST */

/* --- pure packing helpers (host-testable, no Zephyr dependency) -------- */

uint32_t dm_float_to_uint(float x, float x_min, float x_max, unsigned int bits);
float dm_uint_to_float(uint32_t x_int, float x_min, float x_max, unsigned int bits);
void dm_pack_mit(uint8_t data[8], float pos, float vel, float kp, float kd, float tor,
		 float p_max, float v_max, float t_max);
void dm_unpack_feedback(const uint8_t data[8], float p_max, float v_max, float t_max,
			struct dm_feedback *fb);
void dm3519_pack_current(uint8_t data[8], const float cur[4]);
void dm3519_unpack_feedback(const uint8_t data[8], struct dm3519_feedback *fb);

#ifdef __cplusplus
}
#endif

#endif /* DM_MOTOR_H_ */
