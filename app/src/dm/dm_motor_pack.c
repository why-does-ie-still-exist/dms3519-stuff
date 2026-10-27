/*
 * DaMiao motor frame packing helpers.  No Zephyr dependencies so the same
 * code is unit-tested on the host (see tests/host/test_dm_motor.c).
 * SPDX-License-Identifier: Apache-2.0
 */
#include "dm_motor.h"

/* ------------------------------------------------------------------------
 * Pure helpers
 * ---------------------------------------------------------------------- */

void dm_vel_pi_reset(struct dm_vel_pi *pi)
{
	pi->integ = 0.0f;
	pi->v_filt = 0.0f;
	pi->stuck_ms = 0.0f;
	pi->kicking = false;
}

float dm_vel_pi_step(struct dm_vel_pi *pi, float vel_cmd, float vel_meas, float dt)
{
	/* The clamp is always active: a cap of 0 means no torque, never
	 * "unlimited".  Set the cap to TMAX to effectively remove the limit.
	 */
	const float lim = pi->tor_max > 0.0f ? pi->tor_max : 0.0f;
	const float ilim = (pi->i_max > 0.0f && pi->i_max < lim) ? pi->i_max : lim;
	const float db = pi->deadband > 0.0f ? pi->deadband : 0.0f;
	/*
	 * Shrink the error towards zero by the deadband rather than clipping
	 * it, so there is no step in the response at the edge of the band.
	 * Quantisation error then contributes nothing to either term.
	 */
	/* Filter the measurement for the error; stiction detection below
	 * uses the raw value, where "zero" is unambiguous.
	 */
	if (pi->tau_ms > 0.0f && dt > 0.0f) {
		float a = dt * 1000.0f / pi->tau_ms;

		if (a > 1.0f) {
			a = 1.0f;
		}
		pi->v_filt += (vel_meas - pi->v_filt) * a;
	} else {
		pi->v_filt = vel_meas;
	}

	float err = vel_cmd - pi->v_filt;
	const float prev = pi->integ;
	float integ = prev;
	float tor;

	if (err > db) {
		err -= db;
	} else if (err < -db) {
		err += db;
	} else {
		err = 0.0f;
	}

	/* Friction feed-forward follows the command, not the error, so it is
	 * present from the first tick of a move and vanishes at rest.  g is the
	 * taper gain in [-1, 1].
	 */
	float g = 0.0f;

	if (vel_cmd != 0.0f) {
		if (pi->ff_taper > 0.0f) {
			g = vel_cmd / pi->ff_taper;
			if (g > 1.0f) {
				g = 1.0f;
			} else if (g < -1.0f) {
				g = -1.0f;
			}
		} else {
			g = vel_cmd > 0.0f ? 1.0f : -1.0f;
		}
	}

	/* Stiction: a move is asked for but the raw measurement reads zero.
	 * "Zero" is anything inside the deadband (the feedback cannot express
	 * 0 exactly, it reads +-half a count).
	 */
	bool moving_cmd = g > 0.5f || g < -0.5f;
	float sv = pi->stick_v > 0.0f ? pi->stick_v : db;
	float speed = vel_meas < 0.0f ? -vel_meas : vel_meas;
	float cmd_abs = vel_cmd < 0.0f ? -vel_cmd : vel_cmd;
	float release = 2.0f * sv > 0.3f * cmd_abs ? 2.0f * sv : 0.3f * cmd_abs;
	bool kick = false;

	if (pi->ff_brk > 0.0f && moving_cmd) {
		if (pi->kicking) {
			/* Hysteresis: hold the kick until the shaft is really moving. */
			kick = speed < release;
			if (!kick) {
				pi->stuck_ms = 0.0f;
			}
		} else if (speed < sv) {
			pi->stuck_ms += dt * 1000.0f;
			kick = pi->stuck_ms >= pi->stick_ms;
		} else {
			pi->stuck_ms = 0.0f;
		}
	} else {
		pi->stuck_ms = 0.0f;
	}
	pi->kicking = kick;

	/* No integration while kicking: the kick is doing the integrator's old
	 * job, and whatever it accumulated here would be the lurch.
	 */
	if (dt > 0.0f && !kick) {
		integ += pi->ki * err * dt;
	}
	/* Keep the integrator itself inside its clamp as a backstop. */
	if (integ > ilim) {
		integ = ilim;
	} else if (integ < -ilim) {
		integ = -ilim;
	}

	float ff = g * (pi->ff > 0.0f ? pi->ff : 0.0f);

	if (kick) {
		ff += g * pi->ff_brk;
	}

	tor = ff + pi->kp * err + integ;

	if (tor > lim) {
		tor = lim;
		/* Saturated high: let the integrator fall, never climb further. */
		if (integ > prev) {
			integ = prev;
		}
	} else if (tor < -lim) {
		tor = -lim;
		if (integ < prev) {
			integ = prev;
		}
	}
	pi->integ = integ;
	return tor;
}

uint32_t dm_float_to_uint(float x, float x_min, float x_max, unsigned int bits)
{
	float span = x_max - x_min;
	float max_int = (float)((1u << bits) - 1u);

	if (x < x_min) {
		x = x_min;
	} else if (x > x_max) {
		x = x_max;
	}
	return (uint32_t)((x - x_min) * max_int / span);
}

float dm_uint_to_float(uint32_t x_int, float x_min, float x_max, unsigned int bits)
{
	float span = x_max - x_min;

	return (float)x_int * span / (float)((1u << bits) - 1u) + x_min;
}

void dm_pack_mit(uint8_t data[8], float pos, float vel, float kp, float kd, float tor,
		 float p_max, float v_max, float t_max)
{
	uint32_t p = dm_float_to_uint(pos, -p_max, p_max, 16);
	uint32_t v = dm_float_to_uint(vel, -v_max, v_max, 12);
	uint32_t t = dm_float_to_uint(tor, -t_max, t_max, 12);
	uint32_t kp_i = dm_float_to_uint(kp, DM_KP_MIN, DM_KP_MAX, 12);
	uint32_t kd_i = dm_float_to_uint(kd, DM_KD_MIN, DM_KD_MAX, 12);

	data[0] = p >> 8;
	data[1] = p;
	data[2] = v >> 4;
	data[3] = ((v & 0xF) << 4) | (kp_i >> 8);
	data[4] = kp_i;
	data[5] = kd_i >> 4;
	data[6] = ((kd_i & 0xF) << 4) | (t >> 8);
	data[7] = t;
}

void dm_unpack_feedback(const uint8_t data[8], float p_max, float v_max, float t_max,
			struct dm_feedback *fb)
{
	fb->id = data[0] & 0x0F;
	fb->state = data[0] >> 4;
	fb->p_int = ((uint16_t)data[1] << 8) | data[2];
	fb->v_int = ((uint16_t)data[3] << 4) | (data[4] >> 4);
	fb->t_int = ((uint16_t)(data[4] & 0x0F) << 8) | data[5];
	fb->pos = dm_uint_to_float(fb->p_int, -p_max, p_max, 16);
	fb->vel = dm_uint_to_float(fb->v_int, -v_max, v_max, 12);
	fb->tor = dm_uint_to_float(fb->t_int, -t_max, t_max, 12);
	fb->t_mos = (float)data[6];
	fb->t_coil = (float)data[7];
	fb->valid = true;
}

void dm3519_pack_current(uint8_t data[8], const float cur[4])
{
	for (int i = 0; i < 4; i++) {
		float a = cur[i];

		if (a > 20.0f) {
			a = 20.0f;
		} else if (a < -20.0f) {
			a = -20.0f;
		}
		int16_t raw = (int16_t)(a * (16384.0f / 20.0f));

		data[2 * i] = (uint8_t)(raw >> 8);
		data[2 * i + 1] = (uint8_t)raw;
	}
}

void dm3519_unpack_feedback(const uint8_t data[8], struct dm3519_feedback *fb)
{
	int16_t pos = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
	int16_t rpm = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
	int16_t cur = (int16_t)(((uint16_t)data[4] << 8) | data[5]);

	fb->pos_deg = (float)pos / 8192.0f * 360.0f;
	fb->rpm = rpm;
	fb->cur = (float)cur / (16384.0f / 20.0f);
	fb->t_coil = (float)data[6];
	fb->t_mos = (float)data[7];
	fb->valid = true;
}

