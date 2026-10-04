/*
 * DaMiao motor frame packing helpers.  No Zephyr dependencies so the same
 * code is unit-tested on the host (see tests/host/test_dm_motor.c).
 * SPDX-License-Identifier: Apache-2.0
 */
#include "dm_motor.h"

/* ------------------------------------------------------------------------
 * Pure helpers
 * ---------------------------------------------------------------------- */

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

