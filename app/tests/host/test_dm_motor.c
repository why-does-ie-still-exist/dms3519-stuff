/*
 * Host-side unit test for the DaMiao frame packing helpers.
 *
 *   cc -std=c11 -Wall -Wextra -I../../src/dm -DDM_HOST_TEST \
 *      test_dm_motor.c ../../src/dm/dm_motor_pack.c -o test_dm_motor && ./test_dm_motor
 *
 * Reference values are taken from the DaMiao STM32 example (float_to_uint /
 * uint_to_float with PMAX 12.5, VMAX 30, TMAX 10) and the DM3519 current
 * frame scaling (16384 LSB = 20 A).
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "dm_motor.h"

static int failures;

#define CHECK(cond, ...)                                                                     \
	do {                                                                                 \
		if (!(cond)) {                                                               \
			failures++;                                                          \
			printf("FAIL %s:%d: ", __FILE__, __LINE__);                          \
			printf(__VA_ARGS__);                                                 \
			printf("\n");                                                        \
		}                                                                            \
	} while (0)

static void test_float_uint_roundtrip(void)
{
	/* 16-bit position over +-12.5 rad: resolution 25/65535 = 0.38 mrad */
	for (float x = -12.5f; x <= 12.5f; x += 0.7f) {
		uint32_t u = dm_float_to_uint(x, -12.5f, 12.5f, 16);
		float y = dm_uint_to_float(u, -12.5f, 12.5f, 16);

		CHECK(fabsf(x - y) < 0.0005f, "pos roundtrip %f -> %u -> %f", (double)x, u, (double)y);
	}
	/* 12-bit torque over +-10 N.m: resolution 20/4095 = 4.9 mN.m */
	for (float x = -10.0f; x <= 10.0f; x += 0.33f) {
		uint32_t u = dm_float_to_uint(x, -10.0f, 10.0f, 12);
		float y = dm_uint_to_float(u, -10.0f, 10.0f, 12);

		CHECK(fabsf(x - y) < 0.005f, "tor roundtrip %f -> %u -> %f", (double)x, u, (double)y);
	}
	/* clamping */
	CHECK(dm_float_to_uint(100.0f, -12.5f, 12.5f, 16) == 65535, "clamp high");
	CHECK(dm_float_to_uint(-100.0f, -12.5f, 12.5f, 16) == 0, "clamp low");
	/* midpoint: 0 rad -> 32767 (as in the DaMiao example) */
	CHECK(dm_float_to_uint(0.0f, -12.5f, 12.5f, 16) == 32767, "zero -> %u",
	      dm_float_to_uint(0.0f, -12.5f, 12.5f, 16));
}

static void test_mit_pack_layout(void)
{
	uint8_t d[8];

	/* All zero: pos 0 -> 0x7FFF, vel 0 -> 0x7FF, kp 0 -> 0, kd 0 -> 0, tor 0 -> 0x7FF */
	dm_pack_mit(d, 0, 0, 0, 0, 0, 12.5f, 30.0f, 10.0f);
	CHECK(d[0] == 0x7F && d[1] == 0xFF, "pos bytes %02x %02x", d[0], d[1]);
	CHECK(d[2] == 0x7F && (d[3] & 0xF0) == 0xF0, "vel bits %02x %02x", d[2], d[3]);
	CHECK((d[3] & 0x0F) == 0 && d[4] == 0, "kp bits %02x %02x", d[3], d[4]);
	CHECK(d[5] == 0 && (d[6] & 0xF0) == 0, "kd bits %02x %02x", d[5], d[6]);
	CHECK((d[6] & 0x0F) == 0x07 && d[7] == 0xFF, "tor bits %02x %02x", d[6], d[7]);

	/* Max everything: 0xFFFF, 0xFFF, 0xFFF, 0xFFF, 0xFFF -> all 0xFF */
	dm_pack_mit(d, 12.5f, 30.0f, 500.0f, 5.0f, 10.0f, 12.5f, 30.0f, 10.0f);
	for (int i = 0; i < 8; i++) {
		CHECK(d[i] == 0xFF, "max byte %d = %02x", i, d[i]);
	}

	/* A specific known encoding: kp = 250 -> 2047 = 0x7FF; kd = 2.5 -> 0x7FF */
	dm_pack_mit(d, 0, 0, 250.0f, 2.5f, 0, 12.5f, 30.0f, 10.0f);
	CHECK(((d[3] & 0x0F) << 8 | d[4]) == 0x7FF, "kp mid %03x", ((d[3] & 0x0F) << 8 | d[4]));
	CHECK((d[5] << 4 | d[6] >> 4) == 0x7FF, "kd mid %03x", (d[5] << 4 | d[6] >> 4));
}

static void test_feedback_unpack(void)
{
	struct dm_feedback fb;
	/* id 1, state enabled(1): byte0 = 0x11; pos 0x7FFF; vel 0x7FF; tor 0x7FF; Tmos 40; Tcoil 45 */
	uint8_t d[8] = {0x11, 0x7F, 0xFF, 0x7F, 0xF7, 0xFF, 40, 45};

	dm_unpack_feedback(d, 12.5f, 30.0f, 10.0f, &fb);
	CHECK(fb.id == 1 && fb.state == DM_STATE_ENABLED, "id/state %u/%u", fb.id, fb.state);
	CHECK(fb.p_int == 0x7FFF && fb.v_int == 0x7FF && fb.t_int == 0x7FF, "raw %x %x %x", fb.p_int,
	      fb.v_int, fb.t_int);
	CHECK(fabsf(fb.pos) < 0.001f && fabsf(fb.vel) < 0.01f && fabsf(fb.tor) < 0.003f,
	      "near zero %f %f %f", (double)fb.pos, (double)fb.vel, (double)fb.tor);
	CHECK(fb.t_mos == 40.0f && fb.t_coil == 45.0f, "temps");

	/* Pack then unpack a MIT frame's position bits through the feedback layout
	 * (the position field uses the same 16-bit mapping).
	 */
	uint8_t m[8];

	dm_pack_mit(m, 3.14159f, 0, 0, 0, 0, 12.5f, 30.0f, 10.0f);
	uint8_t f[8] = {0x01, m[0], m[1], 0x7F, 0xF7, 0xFF, 0, 0};

	dm_unpack_feedback(f, 12.5f, 30.0f, 10.0f, &fb);
	CHECK(fabsf(fb.pos - 3.14159f) < 0.001f, "pi roundtrip %f", (double)fb.pos);
}

static void test_dm3519(void)
{
	uint8_t d[8];
	float cur[4] = {1.1f, -20.0f, 0.0f, 10.0f};

	dm3519_pack_current(d, cur);
	/* 1.1 A * 819.2 = 901 = 0x0385 */
	CHECK(d[0] == 0x03 && d[1] == 0x85, "m1 %02x%02x", d[0], d[1]);
	/* -20 A -> -16384 = 0xC000 */
	CHECK(d[2] == 0xC0 && d[3] == 0x00, "m2 %02x%02x", d[2], d[3]);
	CHECK(d[4] == 0 && d[5] == 0, "m3");
	/* 10 A -> 8192 = 0x2000 */
	CHECK(d[6] == 0x20 && d[7] == 0x00, "m4 %02x%02x", d[6], d[7]);

	struct dm3519_feedback fb;
	/* pos 4096 -> 180 deg, rpm -1500, cur 0x0385 -> 1.1 A, Tcoil 50, Tmos 60 */
	uint8_t r[8] = {0x10, 0x00, 0xFA, 0x24, 0x03, 0x85, 50, 60};

	dm3519_unpack_feedback(r, &fb);
	CHECK(fabsf(fb.pos_deg - 180.0f) < 0.01f, "pos %f", (double)fb.pos_deg);
	CHECK(fb.rpm == -1500, "rpm %d", fb.rpm);
	CHECK(fabsf(fb.cur - 1.1f) < 0.002f, "cur %f", (double)fb.cur);
	CHECK(fb.t_coil == 50.0f && fb.t_mos == 60.0f, "temps");
}

int main(void)
{
	test_float_uint_roundtrip();
	test_mit_pack_layout();
	test_feedback_unpack();
	test_dm3519();
	if (failures == 0) {
		printf("dm_motor host tests: all passed\n");
		return 0;
	}
	printf("dm_motor host tests: %d failure(s)\n", failures);
	return 1;
}
