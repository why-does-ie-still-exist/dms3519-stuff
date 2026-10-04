/*
 * Shell commands for DaMiao motors:  dm ...
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdlib.h>
#include <string.h>

#include <zephyr/shell/shell.h>

#include "dm_motor.h"

static struct dm_motor motor;
static bool motor_ready;
static uint16_t cur_mode = DM_ID_MIT;

/* --- velocity streamer ---------------------------------------------------
 * The driver stops the motor (state D, comm loss) when it sees no command
 * within its TIMEOUT register, so velocity control must be sent continuously.
 */
enum stream_kind { STREAM_VEL, STREAM_TORQUE };

static struct k_work_delayable stream_work;
static uint32_t stream_period_ms;
static enum stream_kind stream_kind = STREAM_VEL;
static float vel_setpoint;
static float vel_ramp = 2.0f; /* rad/s per second, 0 = no ramp (default keeps the speed loop quiet) */
static float vel_current;     /* value actually being sent */
static float tor_setpoint;    /* N.m, MIT frame with Kp = Kd = 0 */
static float tor_current;     /* value actually being sent */
static float tor_ramp = 1.0f; /* N.m per second, 0 = no ramp */
static float kt_nm_per_a = 0.4537f; /* DaMiao selection table, output shaft */
static float speed_cap = 20.0f;     /* rad/s at the output; torque is cut above this */
static bool speed_tripped;
static uint32_t stream_count;
static uint8_t last_state = 0xFF;

static void stream_fn(struct k_work *work)
{
	ARG_UNUSED(work);
	if (stream_period_ms == 0 || !motor_ready) {
		return;
	}
	if (vel_ramp > 0.0f) {
		float step = vel_ramp * (float)stream_period_ms / 1000.0f;

		if (vel_current < vel_setpoint - step) {
			vel_current += step;
		} else if (vel_current > vel_setpoint + step) {
			vel_current -= step;
		} else {
			vel_current = vel_setpoint;
		}
	} else {
		vel_current = vel_setpoint;
	}

	if (stream_kind == STREAM_TORQUE) {
		if (tor_ramp > 0.0f) {
			float step = tor_ramp * (float)stream_period_ms / 1000.0f;

			if (tor_current < tor_setpoint - step) {
				tor_current += step;
			} else if (tor_current > tor_setpoint + step) {
				tor_current -= step;
			} else {
				tor_current = tor_setpoint;
			}
		} else {
			tor_current = tor_setpoint;
		}
		float tor = tor_current;

		/* Software runaway guard: torque with no load accelerates until
		 * the driver's limit, so cut it above speed_cap (output rad/s).
		 */
		if (motor.fb.valid && speed_cap > 0.0f &&
		    (motor.fb.vel > speed_cap || motor.fb.vel < -speed_cap)) {
			if (!speed_tripped) {
				printk("dm: speed cap %.1f rad/s exceeded (%.2f), torque cut\n",
				       (double)speed_cap, (double)motor.fb.vel);
			}
			speed_tripped = true;
			tor_setpoint = 0.0f;
			tor_current = 0.0f;
			tor = 0.0f;
		}
		(void)dm_motor_mit(&motor, 0.0f, 0.0f, 0.0f, 0.0f, tor);
	} else {
		(void)dm_motor_vel(&motor, vel_current);
	}
	stream_count++;

	if (motor.fb.valid && motor.fb.state != last_state) {
		last_state = motor.fb.state;
		if (last_state > DM_STATE_ENABLED) {
			printk("dm: motor state 0x%x (error) at %.2f rad/s cmd\n", last_state,
			       (double)vel_current);
		}
	}
	k_work_schedule(&stream_work, K_MSEC(stream_period_ms));
}

static void stream_start(uint32_t period_ms)
{
	static bool inited;

	if (!inited) {
		k_work_init_delayable(&stream_work, stream_fn);
		inited = true;
	}
	stream_period_ms = period_ms;
	if (period_ms) {
		k_work_schedule(&stream_work, K_NO_WAIT);
	} else {
		k_work_cancel_delayable(&stream_work);
	}
}

static const struct device *const can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));

static int cmd_fb(const struct shell *sh, size_t argc, char **argv);

static int need_motor(const struct shell *sh)
{
	if (!motor_ready) {
		shell_error(sh, "run: dm init <esc_id> <mst_id> [pmax vmax tmax]");
		return -ENODEV;
	}
	return 0;
}

static float argf(const char *s)
{
	return strtof(s, NULL);
}

static uint32_t argu(const char *s)
{
	return strtoul(s, NULL, 0);
}

static int cmd_init(const struct shell *sh, size_t argc, char **argv)
{
	uint16_t esc = argc > 1 ? argu(argv[1]) : 0x01;
	uint16_t mst = argc > 2 ? argu(argv[2]) : 0x00;
	float pmax = argc > 3 ? argf(argv[3]) : DM_S3519_PMAX;
	float vmax = argc > 4 ? argf(argv[4]) : DM_S3519_VMAX;
	float tmax = argc > 5 ? argf(argv[5]) : DM_S3519_TMAX;
	int ret = dm_motor_init(&motor, can_dev, esc, mst, pmax, vmax, tmax);

	if (ret) {
		shell_error(sh, "init failed: %d", ret);
		return ret;
	}
	motor_ready = true;
	/* Prefer the mapping ranges stored in the driver, if it answers. */
	if (dm_motor_sync_limits(&motor, K_MSEC(100)) == 0) {
		shell_print(sh, "limits read from driver");
	} else {
		shell_print(sh, "driver did not answer register reads; using given/default limits");
	}
	shell_print(sh, "motor esc 0x%03x mst 0x%03x PMAX %.2f VMAX %.2f TMAX %.2f fd %s", esc, mst,
		    (double)motor.p_max, (double)motor.v_max, (double)motor.t_max,
		    motor.fd ? "on" : "off");
	return 0;
}

static int parse_mode(const char *s, uint16_t *mode)
{
	if (!strcmp(s, "mit")) {
		*mode = DM_ID_MIT;
	} else if (!strcmp(s, "pos")) {
		*mode = DM_ID_POS_VEL;
	} else if (!strcmp(s, "vel")) {
		*mode = DM_ID_VEL;
	} else if (!strcmp(s, "psi")) {
		*mode = DM_ID_PSI;
	} else {
		return -EINVAL;
	}
	return 0;
}

static int cmd_mode(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_print(sh, "frame mode: 0x%03x (mit=0x000 pos=0x100 vel=0x200 psi=0x300)", cur_mode);
		return 0;
	}
	if (parse_mode(argv[1], &cur_mode)) {
		shell_error(sh, "mode is mit|pos|vel|psi");
		return -EINVAL;
	}
	shell_print(sh, "commands now use ID 0x%03x + esc_id", cur_mode);
	return 0;
}

#define SIMPLE_CMD(name, fn)                                                                 \
	static int cmd_##name(const struct shell *sh, size_t argc, char **argv)               \
	{                                                                                    \
		ARG_UNUSED(argc);                                                            \
		ARG_UNUSED(argv);                                                            \
		if (need_motor(sh)) {                                                        \
			return -ENODEV;                                                      \
		}                                                                            \
		int ret = fn(&motor, cur_mode);                                              \
		shell_print(sh, #name ": %d", ret);                                          \
		return ret;                                                                  \
	}

/*
 * Enable once and check the feedback state; retry only if the driver did not
 * confirm (DaMiao's SDKs blindly send it three times, which makes the driver
 * chirp three times).
 */
static int enable_repeat(const struct dm_motor *m, uint16_t mode)
{
	int ret = 0;

	for (int i = 0; i < 3; i++) {
		int64_t before = m->fb.stamp;

		ret = dm_motor_enable(m, mode);
		if (ret) {
			return ret;
		}
		k_sleep(K_MSEC(5));
		if (m->fb.valid && m->fb.stamp != before && m->fb.state == DM_STATE_ENABLED) {
			return 0;
		}
	}
	return ret;
}

SIMPLE_CMD(enable, enable_repeat)
SIMPLE_CMD(disable, dm_motor_disable)
SIMPLE_CMD(zero, dm_motor_set_zero)
SIMPLE_CMD(clearerr, dm_motor_clear_error)

static int cmd_mit(const struct shell *sh, size_t argc, char **argv)
{
	if (need_motor(sh)) {
		return -ENODEV;
	}
	if (argc != 6) {
		shell_error(sh, "usage: dm mit <pos rad> <vel rad/s> <kp> <kd> <torque Nm>");
		return -EINVAL;
	}
	return dm_motor_mit(&motor, argf(argv[1]), argf(argv[2]), argf(argv[3]), argf(argv[4]),
			    argf(argv[5]));
}

static int cmd_pos(const struct shell *sh, size_t argc, char **argv)
{
	if (need_motor(sh)) {
		return -ENODEV;
	}
	if (argc != 3) {
		shell_error(sh, "usage: dm pos <pos rad> <vel rad/s>");
		return -EINVAL;
	}
	return dm_motor_pos_vel(&motor, argf(argv[1]), argf(argv[2]));
}

static int cmd_vel(const struct shell *sh, size_t argc, char **argv)
{
	if (need_motor(sh)) {
		return -ENODEV;
	}
	if (argc != 2) {
		shell_error(sh, "usage: dm vel <vel rad/s>   (streamed continuously if 'dm stream' is on)");
		return -EINVAL;
	}
	vel_setpoint = argf(argv[1]);
	if (stream_period_ms) {
		shell_print(sh, "setpoint %.3f rad/s (streaming every %u ms)", (double)vel_setpoint,
			    stream_period_ms);
		return 0;
	}
	vel_current = vel_setpoint;
	return dm_motor_vel(&motor, vel_setpoint);
}

static int cmd_stop(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (need_motor(sh)) {
		return -ENODEV;
	}
	int ret;

	vel_setpoint = 0.0f;
	vel_current = 0.0f;
	tor_setpoint = 0.0f;
	tor_current = 0.0f;
	if (stream_kind == STREAM_TORQUE) {
		ret = dm_motor_mit(&motor, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
		shell_print(sh, "torque setpoint 0: %d", ret);
	} else {
		ret = dm_motor_vel(&motor, 0.0f);
		shell_print(sh, "velocity setpoint 0: %d", ret);
	}
	return ret;
}

static int cmd_stream(const struct shell *sh, size_t argc, char **argv)
{
	if (need_motor(sh)) {
		return -ENODEV;
	}
	if (argc < 2) {
		shell_print(sh, "streaming: %s (%u ms), sent %u, setpoint %.3f, ramp %.1f rad/s^2",
			    stream_period_ms ? "on" : "off", stream_period_ms, stream_count,
			    (double)vel_setpoint, (double)vel_ramp);
		return 0;
	}
	uint32_t hz = argu(argv[1]);

	if (hz > 1000) {
		shell_error(sh, "max 1000 Hz");
		return -EINVAL;
	}
	if (argc > 2) {
		vel_ramp = argf(argv[2]);
	}
	stream_start(hz ? 1000 / hz : 0);
	shell_print(sh, "streaming %s", hz ? "on" : "off");
	return 0;
}

/*
 * One-shot velocity-mode bring-up: disable, CTRL_MODE = 3, optionally save to
 * flash, switch the enable/disable frame ID to the 0x200 offset, enable, and
 * start streaming a 0 rad/s setpoint.
 */
static int cmd_velmode(const struct shell *sh, size_t argc, char **argv)
{
	bool save = argc > 1 && !strcmp(argv[1], "save");
	uint32_t hz = argc > 2 ? argu(argv[2]) : 100;
	int ret;

	if (need_motor(sh)) {
		return -ENODEV;
	}
	stream_start(0);
	vel_setpoint = 0.0f;
	vel_current = 0.0f;

	(void)dm_motor_disable(&motor, cur_mode);
	k_sleep(K_MSEC(10));

	ret = dm_motor_write_reg(&motor, DM_RID_CMODE, DM_MODE_VEL, K_MSEC(200));
	if (ret == -ETIMEDOUT) {
		shell_warn(sh, "no reply to CTRL_MODE write (motor off or IDs wrong?), continuing");
	} else if (ret) {
		shell_error(sh, "CTRL_MODE write failed: %d", ret);
		return ret;
	}
	if (save) {
		k_sleep(K_MSEC(10));
		ret = dm_motor_save_regs(&motor, K_MSEC(300));
		shell_print(sh, "save: %d", ret);
	}
	cur_mode = DM_ID_VEL;
	ret = enable_repeat(&motor, DM_ID_VEL);
	if (ret) {
		shell_error(sh, "enable failed: %d", ret);
		return ret;
	}
	stream_kind = STREAM_VEL;
	stream_start(hz ? 1000 / hz : 0);
	shell_print(sh, "velocity mode: enabled on ID 0x%03x, streaming at %u Hz; use 'dm vel <rad/s>'",
		    DM_ID_VEL + motor.esc_id, hz);
	return 0;
}

/*
 * Torque (current) mode bring-up: MIT mode with Kp = Kd = 0, streaming the
 * torque feed-forward term.
 */
static int cmd_torquemode(const struct shell *sh, size_t argc, char **argv)
{
	bool save = argc > 1 && !strcmp(argv[1], "save");
	uint32_t hz = argc > 2 ? argu(argv[2]) : 100;
	int ret;

	if (need_motor(sh)) {
		return -ENODEV;
	}
	stream_start(0);
	tor_setpoint = 0.0f;
	tor_current = 0.0f;
	speed_tripped = false;

	(void)dm_motor_disable(&motor, cur_mode);
	k_sleep(K_MSEC(10));

	ret = dm_motor_write_reg(&motor, DM_RID_CMODE, DM_MODE_MIT, K_MSEC(200));
	if (ret == -ETIMEDOUT) {
		shell_warn(sh, "no reply to CTRL_MODE write (motor off or IDs wrong?), continuing");
	} else if (ret) {
		shell_error(sh, "CTRL_MODE write failed: %d", ret);
		return ret;
	}
	if (save) {
		k_sleep(K_MSEC(10));
		ret = dm_motor_save_regs(&motor, K_MSEC(300));
		shell_print(sh, "save: %d", ret);
	}
	cur_mode = DM_ID_MIT;
	ret = enable_repeat(&motor, DM_ID_MIT);
	if (ret) {
		shell_error(sh, "enable failed: %d", ret);
		return ret;
	}
	stream_kind = STREAM_TORQUE;
	stream_start(hz ? 1000 / hz : 0);
	shell_print(sh, "torque mode: MIT on ID 0x%03x, Kp=Kd=0, streaming at %u Hz, ramp %.1f N.m/s, "
		    "speed cap %.1f rad/s; use 'dm torque <Nm>' or 'dm current <A>'",
		    DM_ID_MIT + motor.esc_id, hz, (double)tor_ramp, (double)speed_cap);
	return 0;
}

static int set_torque(const struct shell *sh, float tor)
{
	if (tor > motor.t_max) {
		tor = motor.t_max;
	} else if (tor < -motor.t_max) {
		tor = -motor.t_max;
	}
	tor_setpoint = tor;
	speed_tripped = false;
	if (stream_period_ms && stream_kind == STREAM_TORQUE) {
		shell_print(sh, "torque setpoint %.3f N.m (~%.2f A), streaming every %u ms", (double)tor,
			    (double)(tor / kt_nm_per_a), stream_period_ms);
		return 0;
	}
	shell_print(sh, "torque %.3f N.m (~%.2f A), single frame (run 'dm torquemode' to stream)",
		    (double)tor, (double)(tor / kt_nm_per_a));
	return dm_motor_mit(&motor, 0.0f, 0.0f, 0.0f, 0.0f, tor);
}

static int cmd_torque(const struct shell *sh, size_t argc, char **argv)
{
	if (need_motor(sh)) {
		return -ENODEV;
	}
	if (argc != 2) {
		shell_error(sh, "usage: dm torque <N.m at the output shaft>");
		return -EINVAL;
	}
	return set_torque(sh, argf(argv[1]));
}

static int cmd_current(const struct shell *sh, size_t argc, char **argv)
{
	if (need_motor(sh)) {
		return -ENODEV;
	}
	if (argc != 2) {
		shell_error(sh, "usage: dm current <A>   (converted with Kt, see 'dm kt')");
		return -EINVAL;
	}
	return set_torque(sh, argf(argv[1]) * kt_nm_per_a);
}

static int cmd_kt(const struct shell *sh, size_t argc, char **argv)
{
	if (argc > 1) {
		kt_nm_per_a = argf(argv[1]);
	}
	shell_print(sh, "Kt = %.4f N.m/A at the output shaft (used by 'dm current')",
		    (double)kt_nm_per_a);
	return 0;
}

static int cmd_torqueramp(const struct shell *sh, size_t argc, char **argv)
{
	if (argc > 1) {
		tor_ramp = argf(argv[1]);
	}
	shell_print(sh, "torque ramp = %.2f N.m/s (0 = step)", (double)tor_ramp);
	return 0;
}

static int cmd_speedcap(const struct shell *sh, size_t argc, char **argv)
{
	if (argc > 1) {
		speed_cap = argf(argv[1]);
		speed_tripped = false;
	}
	shell_print(sh, "speed cap = %.1f rad/s (0 = off); torque is cut above it%s",
		    (double)speed_cap, speed_tripped ? " [TRIPPED, set a new torque to clear]" : "");
	return 0;
}

static int cmd_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (need_motor(sh)) {
		return -ENODEV;
	}
	shell_print(sh, "esc 0x%03x mst 0x%03x frame-mode 0x%03x fd %s", motor.esc_id, motor.mst_id,
		    cur_mode, motor.fd ? "on" : "off");
	if (stream_kind == STREAM_TORQUE) {
		shell_print(sh, "stream %s %u ms (torque), sent %u, torque %.3f N.m (~%.2f A), sending %.3f, ramp %.1f N.m/s, cap %.1f rad/s%s",
			    stream_period_ms ? "on" : "off", stream_period_ms, stream_count,
			    (double)tor_setpoint, (double)(tor_setpoint / kt_nm_per_a), (double)tor_current,
			    (double)tor_ramp, (double)speed_cap, speed_tripped ? " TRIPPED" : "");
	} else {
		shell_print(sh, "stream %s %u ms (velocity), sent %u, setpoint %.3f, sending %.3f rad/s",
			    stream_period_ms ? "on" : "off", stream_period_ms, stream_count,
			    (double)vel_setpoint, (double)vel_current);
	}
	return cmd_fb(sh, 0, NULL);
}

static int cmd_psi(const struct shell *sh, size_t argc, char **argv)
{
	if (need_motor(sh)) {
		return -ENODEV;
	}
	if (argc != 4) {
		shell_error(sh, "usage: dm psi <pos rad> <vel rad/s> <current A>");
		return -EINVAL;
	}
	return dm_motor_psi(&motor, argf(argv[1]), argf(argv[2]), argf(argv[3]));
}

static int cmd_cur3519(const struct shell *sh, size_t argc, char **argv)
{
	float cur[4] = {0};

	if (argc < 2 || argc > 6) {
		shell_error(sh, "usage: dm cur3519 <m1 A> [m2] [m3] [m4] (frame 0x200)");
		return -EINVAL;
	}
	for (size_t i = 1; i < argc && i <= 4; i++) {
		cur[i - 1] = argf(argv[i]);
	}
	return dm3519_set_current(can_dev, 0x200, cur);
}

static int cmd_fb(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (need_motor(sh)) {
		return -ENODEV;
	}
	if (motor.fb.valid) {
		shell_print(sh, "std fb (%lld ms ago): id %u state %u pos %.4f rad vel %.3f rad/s "
			    "tor %.3f Nm Tmos %.0f Tcoil %.0f",
			    (long long)(k_uptime_get() - motor.fb.stamp), motor.fb.id,
			    motor.fb.state, (double)motor.fb.pos, (double)motor.fb.vel,
			    (double)motor.fb.tor, (double)motor.fb.t_mos, (double)motor.fb.t_coil);
	} else {
		shell_print(sh, "no standard feedback received yet on id 0x%03x", motor.mst_id);
	}
	if (motor.fb3519.valid) {
		shell_print(sh, "dm3519 fb (%lld ms ago): pos %.2f deg rpm %d cur %.3f A Tcoil %.0f "
			    "Tmos %.0f",
			    (long long)(k_uptime_get() - motor.fb3519.stamp),
			    (double)motor.fb3519.pos_deg, motor.fb3519.rpm, (double)motor.fb3519.cur,
			    (double)motor.fb3519.t_coil, (double)motor.fb3519.t_mos);
	}
	return 0;
}

static int cmd_request(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (need_motor(sh)) {
		return -ENODEV;
	}
	int ret = dm_motor_request_feedback(&motor);

	k_sleep(K_MSEC(20));
	if (ret == 0) {
		return cmd_fb(sh, 0, NULL);
	}
	return ret;
}

static bool reg_is_float(enum dm_reg rid)
{
	switch (rid) {
	case DM_RID_MST_ID:
	case DM_RID_ESC_ID:
	case DM_RID_TIMEOUT:
	case DM_RID_CMODE:
	case DM_RID_HW_VER:
	case DM_RID_SW_VER:
	case DM_RID_SN:
	case DM_RID_NPP:
	case DM_RID_CAN_BR:
	case DM_RID_SUB_VER:
		return false;
	default:
		return true;
	}
}

static int cmd_read(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t raw;
	int ret;

	if (need_motor(sh)) {
		return -ENODEV;
	}
	if (argc != 2) {
		shell_error(sh, "usage: dm read <rid>   (e.g. 21 PMAX, 22 VMAX, 23 TMAX, 10 CMODE, 35 CAN_BR)");
		return -EINVAL;
	}
	enum dm_reg rid = argu(argv[1]);

	ret = dm_motor_read_reg(&motor, rid, &raw, K_MSEC(200));
	if (ret) {
		shell_error(sh, "read rid %u failed: %d", rid, ret);
		return ret;
	}
	if (reg_is_float(rid)) {
		float f;

		memcpy(&f, &raw, 4);
		shell_print(sh, "rid %u = %f (raw 0x%08x)", rid, (double)f, raw);
	} else {
		shell_print(sh, "rid %u = %u (0x%08x)", rid, raw, raw);
	}
	return 0;
}

static int cmd_write(const struct shell *sh, size_t argc, char **argv)
{
	if (need_motor(sh)) {
		return -ENODEV;
	}
	if (argc != 3) {
		shell_error(sh, "usage: dm write <rid> <value>  (float or integer per register type)");
		return -EINVAL;
	}
	enum dm_reg rid = argu(argv[1]);

	int ret;

	if (reg_is_float(rid)) {
		ret = dm_motor_write_reg_float(&motor, rid, argf(argv[2]), K_MSEC(200));
	} else {
		ret = dm_motor_write_reg(&motor, rid, argu(argv[2]), K_MSEC(200));
	}
	shell_print(sh, "write rid %u: %d%s", rid, ret, ret == -ERANGE ? " (rejected by driver)" : "");
	return ret;
}

static int cmd_save(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (need_motor(sh)) {
		return -ENODEV;
	}
	int ret = dm_motor_save_regs(&motor, K_MSEC(300));

	shell_print(sh, "save: %d (motor must be disabled)", ret);
	return ret;
}

static int cmd_bus(const struct shell *sh, size_t argc, char **argv)
{
	bool loopback = false;
	bool fd = false;

	if (argc < 2) {
		shell_error(sh, "usage: dm bus <normal|loopback> [fd]");
		return -EINVAL;
	}
	if (!strcmp(argv[1], "loopback")) {
		loopback = true;
	} else if (strcmp(argv[1], "normal")) {
		shell_error(sh, "usage: dm bus <normal|loopback> [fd]");
		return -EINVAL;
	}
	if (argc > 2 && !strcmp(argv[2], "fd")) {
		fd = true;
	}
	int ret = dm_bus_configure(can_dev, loopback, fd);

	motor.fd = fd;
	shell_print(sh, "bus %s%s: %d", loopback ? "loopback" : "normal", fd ? " + FD/BRS" : "", ret);
	return ret;
}

static int cmd_info(const struct shell *sh, size_t argc, char **argv)
{
	static const struct {
		enum dm_reg rid;
		const char *name;
	} regs[] = {
		{DM_RID_ESC_ID, "ESC_ID"}, {DM_RID_MST_ID, "MST_ID"}, {DM_RID_CMODE, "CMODE"},
		{DM_RID_CAN_BR, "CAN_BR"}, {DM_RID_PMAX, "PMAX"},     {DM_RID_VMAX, "VMAX"},
		{DM_RID_TMAX, "TMAX"},     {DM_RID_GR, "GR"},         {DM_RID_KT_VALUE, "KT"},
		{DM_RID_MAX_SPD, "MAX_SPD"}, {DM_RID_TIMEOUT, "TIMEOUT"}, {DM_RID_HW_VER, "HW_VER"},
		{DM_RID_SW_VER, "SW_VER"}, {DM_RID_SUB_VER, "SUB_VER"},
	};

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (need_motor(sh)) {
		return -ENODEV;
	}
	for (size_t i = 0; i < ARRAY_SIZE(regs); i++) {
		uint32_t raw;
		int ret = dm_motor_read_reg(&motor, regs[i].rid, &raw, K_MSEC(200));

		if (ret) {
			shell_print(sh, "%-8s (rid %2u): no reply (%d)", regs[i].name, regs[i].rid, ret);
			continue;
		}
		if (reg_is_float(regs[i].rid)) {
			float f;

			memcpy(&f, &raw, 4);
			shell_print(sh, "%-8s (rid %2u): %f", regs[i].name, regs[i].rid, (double)f);
		} else {
			shell_print(sh, "%-8s (rid %2u): %u", regs[i].name, regs[i].rid, raw);
		}
	}
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	dm_cmds,
	SHELL_CMD_ARG(init, NULL, "<esc_id> <mst_id> [pmax vmax tmax]: set up a motor", cmd_init, 1, 5),
	SHELL_CMD_ARG(mode, NULL, "[mit|pos|vel|psi]: frame-ID mode used by enable/disable/zero", cmd_mode, 1, 1),
	SHELL_CMD(enable, NULL, "enable motor (FF..FC)", cmd_enable),
	SHELL_CMD(disable, NULL, "disable motor (FF..FD)", cmd_disable),
	SHELL_CMD(zero, NULL, "save current position as zero (FF..FE)", cmd_zero),
	SHELL_CMD(clearerr, NULL, "clear error (FF..FB)", cmd_clearerr),
	SHELL_CMD_ARG(mit, NULL, "<pos> <vel> <kp> <kd> <tor>: MIT frame", cmd_mit, 6, 0),
	SHELL_CMD_ARG(pos, NULL, "<pos> <vel>: position-velocity frame", cmd_pos, 3, 0),
	SHELL_CMD_ARG(vel, NULL, "<rad/s>: velocity setpoint (one frame, or continuous with stream)", cmd_vel, 2, 0),
	SHELL_CMD(stop, NULL, "velocity/torque setpoint 0", cmd_stop),
	SHELL_CMD_ARG(torquemode, NULL, "[save] [hz]: MIT mode with Kp=Kd=0, stream the torque term", cmd_torquemode, 1, 2),
	SHELL_CMD_ARG(torque, NULL, "<N.m>: torque (current) setpoint at the output shaft", cmd_torque, 2, 0),
	SHELL_CMD_ARG(current, NULL, "<A>: current setpoint, converted to torque with Kt", cmd_current, 2, 0),
	SHELL_CMD_ARG(kt, NULL, "[N.m/A]: show/set the Kt used by 'dm current'", cmd_kt, 1, 1),
	SHELL_CMD_ARG(torqueramp, NULL, "[N.m/s]: show/set the torque setpoint ramp (0 = step)", cmd_torqueramp, 1, 1),
	SHELL_CMD_ARG(speedcap, NULL, "[rad/s]: runaway guard for torque mode (0 = off)", cmd_speedcap, 1, 1),
	SHELL_CMD_ARG(stream, NULL, "[hz [ramp rad/s^2]]: stream the velocity setpoint (0 = off)", cmd_stream, 1, 2),
	SHELL_CMD_ARG(velmode, NULL, "[save] [hz]: disable, CTRL_MODE=3, enable on 0x200+ID, stream", cmd_velmode, 1, 2),
	SHELL_CMD(status, NULL, "motor, streamer and feedback status", cmd_status),
	SHELL_CMD_ARG(psi, NULL, "<pos> <vel> <cur>: force-position frame", cmd_psi, 4, 0),
	SHELL_CMD_ARG(cur3519, NULL, "<m1> [m2] [m3] [m4]: DM3519 one-to-four current frame (A)", cmd_cur3519, 2, 3),
	SHELL_CMD(fb, NULL, "show last feedback", cmd_fb),
	SHELL_CMD(request, NULL, "request a feedback frame (0x7FF/0xCC) and show it", cmd_request),
	SHELL_CMD_ARG(read, NULL, "<rid>: read register", cmd_read, 2, 0),
	SHELL_CMD_ARG(write, NULL, "<rid> <value>: write register (RAM)", cmd_write, 3, 0),
	SHELL_CMD(save, NULL, "save registers to flash (0x7FF/0xAA)", cmd_save),
	SHELL_CMD(info, NULL, "read the common registers", cmd_info),
	SHELL_CMD_ARG(bus, NULL, "<normal|loopback> [fd]: set CAN controller mode (fd = send FD+BRS frames)", cmd_bus, 2, 1),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(dm, &dm_cmds, "DaMiao motor commands", NULL);
