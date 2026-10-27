/*
 * Shell commands for DaMiao motors:  dm ...
 *
 * Several motors can be driven at once.  Each one lives in a slot with its own
 * dm_motor object, control-loop state and work item, so the loops run
 * independently on the system work queue; 'dm init <esc>' selects the slot the
 * other commands act on (creating it the first time) and leaves the rest
 * running untouched.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdlib.h>
#include <string.h>

#include <zephyr/shell/shell.h>

#include "dm_motor.h"

/* --- per-motor state -----------------------------------------------------
 * The driver stops the motor (state D, comm loss) when it sees no command
 * within its TIMEOUT register, so velocity control must be sent continuously.
 * Each slot has a delayable work item that re-schedules itself every period.
 */
enum stream_kind { STREAM_VEL, STREAM_TORQUE, STREAM_SAFEVEL };

/* Feedback older than this leaves the safevel loop open; it then commands 0. */
#define DM_FB_STALE_MS 100

#define DM_SLOTS 8
#define VEL_WIN_MAX 32

struct dm_slot {
	struct dm_motor m;
	bool ready;
	uint16_t cur_mode;             /* frame-ID offset used by enable/disable/zero */

	struct k_work_delayable stream_work;
	uint32_t stream_period_ms;
	k_ticks_t stream_next;         /* absolute tick of the next run, so the period does not drift */
	int64_t stream_t0_us;          /* start of the stream, for the trace clock */
	enum stream_kind stream_kind;
	uint32_t stream_count;
	uint8_t last_state;

	float vel_setpoint;
	float vel_ramp;                /* rad/s per second, 0 = no ramp */
	float vel_current;             /* value actually being sent */

	float tor_setpoint;            /* N.m, MIT frame with Kp = Kd = 0 */
	float tor_current;
	float tor_ramp;                /* N.m per second, 0 = no ramp */
	float speed_cap;               /* rad/s at the output; torque is cut above this */
	bool speed_tripped;

	/*
	 * Safe-velocity loop: velocity control whose torque saturates at a cap
	 * instead of tripping.  Gains are in N.m per rad/s and N.m per rad at
	 * the output shaft; the defaults are deliberately soft.
	 */
	struct dm_vel_pi vel_pi;
	bool svel_stale_warned;
	/*
	 * Idle means "commanding zero torque", as distinct from "regulating to
	 * zero rad/s".  'dm stop' idles; any 'dm vel' re-engages the loop.
	 */
	bool svel_idle;
	bool svel_deadband_user;       /* set once 'dm veldeadband' is used */
	/*
	 * Velocity used by the safevel loop.  The driver's own estimate jitters
	 * by +-0.15 rad/s at 1 kHz whatever VMAX is, so by default the loop
	 * differences the rotor position over vel_win ticks and divides by the
	 * gear ratio instead ('dm velsrc').
	 */
	bool vel_from_pos;
	uint8_t vel_win_ms;            /* difference window in ms (time-based, not ticks) */
	float pos_hist[32];            /* unwrapped rotor position per tick */
	uint32_t cyc_hist[32];         /* and when that sample was taken */
	uint8_t hist_idx;
	uint8_t hist_fill;
	float vel_est;                 /* output rad/s from the position difference */
	bool vel_valid;                /* false until the window holds two fresh feedback frames */
	bool svel_runaway;             /* independent guard tripped; cleared by a mode command */
	bool energised;                /* we have enabled this driver and not disabled it since */

	/*
	 * Host heartbeat watchdog: if armed, 'dm hb' must arrive at least every
	 * hb_timeout_ms or this slot zeroes its command, disables its driver and
	 * stops streaming.  Checked in stream_fn, so it runs on the MCU.
	 */
	uint32_t hb_timeout_ms;        /* 0 = watchdog off */
	bool hb_tripped;
};

static struct dm_slot slots[DM_SLOTS];
static struct dm_slot *cur;        /* slot the shell commands act on; NULL until 'dm init' */
static float kt_nm_per_a = 0.4537f; /* DaMiao selection table, output shaft; shared */
/* One heartbeat clock for all slots: the host is either alive or not. */
static uint32_t hb_last_ms;
/*
 * Single-motor lockout.  With several motors fed through one thin supply
 * lead, only one may be energised at a time: a command that would enable or
 * drive a motor is refused while another slot is energised or streaming.
 * 'dm lockout off' lifts it once the wiring can take the current.
 */
static bool lockout = true;

static const struct device *const can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));

static void stream_fn(struct k_work *work);
static const char *kind_name(enum stream_kind k);
static uint32_t argu(const char *s);

static void slot_defaults(struct dm_slot *s)
{
	s->cur_mode = DM_ID_MIT;
	s->stream_kind = STREAM_VEL;
	s->vel_ramp = 2.0f;
	s->tor_ramp = 1.0f;
	s->speed_cap = 20.0f;
	/* Friction numbers measured on an unloaded DM-S3519-1EC at the output
	 * shaft (tools/vel_step.py): 0.34 N.m to break away, 0.14 N.m to keep
	 * turning at 0.5 rad/s.  The feed-forward sits just under kinetic so
	 * the integrator always works upward from a small positive residual.
	 */
	s->vel_pi = (struct dm_vel_pi){
		.kp = 0.30f,
		.ki = 2.00f,
		.tor_max = 0.50f, /* plenty for an unloaded shaft (0.34 N.m breakaway); raise under load */
		.deadband = 0.01f,
		.ff = 0.14f,
		.ff_taper = 0.20f,
		.i_max = 0.15f,
		.ff_brk = 0.34f,
		.stick_ms = 20.0f,
		.stick_v = 0.05f,
		.tau_ms = 3.0f,
	};
	s->svel_idle = true;
	s->last_state = 0xFF;
	s->vel_from_pos = true;
	s->vel_win_ms = 10;
	k_work_init_delayable(&s->stream_work, stream_fn);
}

/*
 * Output-shaft velocity from the rotor position history.  Called once per
 * tick before the loop runs.  The window is a TIME (vel_win_ms), found by
 * walking back through the per-tick history using the feedback's own
 * timestamps, so it means the same thing at any loop rate and jitter in when
 * replies arrive does not alias into the estimate.  Sets vel_valid; while
 * false the loop must not trust the estimate (it runs open, 0 N.m).
 */
static float vel_from_position(struct dm_slot *s)
{
	struct dm_feedback *fb = &s->m.fb;
	uint32_t now = k_cycle_get_32();
	uint32_t hz = sys_clock_hw_cycles_per_sec();
	uint32_t win = (uint32_t)((uint64_t)s->vel_win_ms * hz / 1000U);

	s->pos_hist[s->hist_idx] = fb->pos_unw;
	s->cyc_hist[s->hist_idx] = fb->stamp_cyc;
	uint8_t newest = s->hist_idx;

	s->hist_idx = (s->hist_idx + 1) % VEL_WIN_MAX;
	if (s->hist_fill < VEL_WIN_MAX) {
		s->hist_fill++;
	}
	s->vel_valid = false;
	/* The newest feedback must itself be recent (within two windows). */
	if (!fb->valid || (uint32_t)(now - fb->stamp_cyc) > 2U * win) {
		return 0.0f;
	}
	/* Walk back to the first sample at least one window older. */
	for (uint8_t k = 1; k < s->hist_fill; k++) {
		uint8_t idx = (newest + VEL_WIN_MAX - k) % VEL_WIN_MAX;
		uint32_t dcyc = s->cyc_hist[newest] - s->cyc_hist[idx];

		if (dcyc >= win) {
			if (dcyc > 4U * win) {
				return 0.0f; /* gap in feedback: no trustworthy estimate */
			}
			float dt = (float)dcyc / (float)hz;
			float gr = s->m.gr > 0.0f ? s->m.gr : 1.0f;

			s->vel_valid = true;
			return (s->pos_hist[newest] - s->pos_hist[idx]) / dt / gr;
		}
	}
	return 0.0f;
}

static inline bool have_motor(void)
{
	return cur != NULL && cur->ready;
}

static struct dm_slot *slot_by_esc(uint16_t esc)
{
	for (int i = 0; i < DM_SLOTS; i++) {
		if (slots[i].ready && slots[i].m.esc_id == esc) {
			return &slots[i];
		}
	}
	return NULL;
}

static bool any_streaming(void)
{
	for (int i = 0; i < DM_SLOTS; i++) {
		if (slots[i].ready && slots[i].stream_period_ms) {
			return true;
		}
	}
	return false;
}

static void stream_start(struct dm_slot *s, uint32_t period_ms)
{
	s->stream_period_ms = period_ms;
	if (period_ms) {
		s->stream_next = k_uptime_ticks();
		s->stream_t0_us = k_ticks_to_us_floor64(s->stream_next);
		k_work_schedule(&s->stream_work, K_NO_WAIT);
	} else {
		k_work_cancel_delayable(&s->stream_work);
	}
}

/* Period in ms from a Hz argument; 1 kHz is the fastest the ms clock allows. */
static uint32_t hz_to_period(uint32_t hz)
{
	if (hz == 0) {
		return 0;
	}
	if (hz > 1000) {
		hz = 1000;
	}
	return 1000 / hz;
}

#define DM_DEFAULT_HZ 1000

/* Mode commands take "save" and/or a rate in Hz, in either order. */
static void parse_mode_args(size_t argc, char **argv, bool *save, uint32_t *hz)
{
	*save = false;
	*hz = DM_DEFAULT_HZ;
	for (size_t i = 1; i < argc; i++) {
		if (!strcmp(argv[i], "save")) {
			*save = true;
		} else {
			*hz = argu(argv[i]);
		}
	}
}

/* Any mode bring-up re-arms: the operator is clearly back in control. */
static void hb_rearm(struct dm_slot *s)
{
	hb_last_ms = k_uptime_get_32();
	s->hb_tripped = false;
}

/* Zero the command, disable the driver, stop the stream.  Used by the
 * watchdog trip and by 'dm release'.
 */
static void slot_halt(struct dm_slot *s)
{
	s->stream_period_ms = 0; /* stream_fn returns without rescheduling */
	k_work_cancel_delayable(&s->stream_work);
	s->vel_setpoint = 0.0f;
	s->vel_current = 0.0f;
	s->tor_setpoint = 0.0f;
	s->tor_current = 0.0f;
	s->svel_idle = true;
	dm_vel_pi_reset(&s->vel_pi);
	(void)dm_motor_mit(&s->m, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
	(void)dm_motor_disable(&s->m, s->cur_mode);
	s->energised = false;
}

static void hb_trip(struct dm_slot *s)
{
	slot_halt(s);
	s->hb_tripped = true;
	printk("dm: no host heartbeat for %u ms, motor 0x%03x disabled\n", s->hb_timeout_ms,
	       s->m.esc_id);
}

/* --- loop trace -----------------------------------------------------------
 * A RAM capture of one slot's loop at full rate: commanded (ramped) velocity,
 * measured velocity, torque sent and integrator state, per tick.  Armed with
 * 'dm trace <n>', it starts on the first tick where the loop is actually
 * working (safevel not idle, or any tick for other kinds) and stops after n
 * samples; 'dm trace dump' prints CSV.  Capturing to RAM keeps the loop timing
 * independent of the console.
 */
#define TRACE_MAX 2000

struct trace_sample {
	uint32_t t_us;
	float cmd;
	float meas;   /* velocity the loop used */
	float fbv;    /* the driver's own velocity estimate */
	float pos;    /* unwrapped rotor position, rad */
	uint32_t fb_cyc; /* cycle stamp of the feedback used (shows its update rate) */
	float tor;
	float integ;
	uint8_t state;
};

/*
 * In its own section at the end of .noinit (app/linker/dm_trace.ld): the CAN
 * message RAM must stay in the first 64 KiB of SRAM, and a 60 KiB buffer
 * linked ahead of it pushes it out and silently kills CAN.
 */
static struct trace_sample trace_buf[TRACE_MAX] Z_GENERIC_SECTION(.dm_trace);
static struct dm_slot *trace_slot;
static volatile uint32_t trace_n;      /* samples captured */
static volatile uint32_t trace_want;   /* 0 = not armed */
static volatile bool trace_running;

static void trace_tick(struct dm_slot *s)
{
	if (trace_want == 0 || s != trace_slot) {
		return;
	}
	if (!trace_running) {
		if (s->stream_kind == STREAM_SAFEVEL && s->svel_idle) {
			return; /* wait for the move to start */
		}
		trace_running = true;
		trace_n = 0;
	}
	if (trace_n >= trace_want) {
		trace_want = 0;
		trace_running = false;
		return;
	}
	struct trace_sample *t = &trace_buf[trace_n];

	t->t_us = (uint32_t)(k_ticks_to_us_floor64(k_uptime_ticks()) - s->stream_t0_us);
	t->cmd = s->stream_kind == STREAM_TORQUE ? s->tor_setpoint : s->vel_current;
	t->meas = (s->stream_kind == STREAM_SAFEVEL && s->vel_from_pos) ? s->vel_est : s->m.fb.vel;
	t->fbv = s->m.fb.vel;
	t->pos = s->m.fb.pos_unw;
	t->fb_cyc = s->m.fb.stamp_cyc;
	t->tor = s->stream_kind == STREAM_VEL ? 0.0f : s->tor_current;
	t->integ = s->vel_pi.integ;
	t->state = (s->m.fb.valid ? s->m.fb.state : 0x7F) | (s->vel_pi.kicking ? 0x80 : 0);
	trace_n++;
}

static void stream_fn(struct k_work *work)
{
	struct k_work_delayable *dw = k_work_delayable_from_work(work);
	struct dm_slot *s = CONTAINER_OF(dw, struct dm_slot, stream_work);
	struct dm_motor *m = &s->m;

	if (s->stream_period_ms == 0 || !s->ready) {
		return;
	}
	if (s->hb_timeout_ms && (uint32_t)(k_uptime_get_32() - hb_last_ms) > s->hb_timeout_ms) {
		hb_trip(s);
		return;
	}
	s->vel_est = vel_from_position(s);
	if (s->vel_ramp > 0.0f) {
		float step = s->vel_ramp * (float)s->stream_period_ms / 1000.0f;

		if (s->vel_current < s->vel_setpoint - step) {
			s->vel_current += step;
		} else if (s->vel_current > s->vel_setpoint + step) {
			s->vel_current -= step;
		} else {
			s->vel_current = s->vel_setpoint;
		}
	} else {
		s->vel_current = s->vel_setpoint;
	}

	if (s->stream_kind == STREAM_TORQUE) {
		if (s->tor_ramp > 0.0f) {
			float step = s->tor_ramp * (float)s->stream_period_ms / 1000.0f;

			if (s->tor_current < s->tor_setpoint - step) {
				s->tor_current += step;
			} else if (s->tor_current > s->tor_setpoint + step) {
				s->tor_current -= step;
			} else {
				s->tor_current = s->tor_setpoint;
			}
		} else {
			s->tor_current = s->tor_setpoint;
		}
		float tor = s->tor_current;

		/* Software runaway guard: torque with no load accelerates until
		 * the driver's limit, so cut it above speed_cap (output rad/s).
		 */
		if (m->fb.valid && s->speed_cap > 0.0f &&
		    (m->fb.vel > s->speed_cap || m->fb.vel < -s->speed_cap)) {
			if (!s->speed_tripped) {
				printk("dm: 0x%03x speed cap %.1f rad/s exceeded (%.2f), torque cut\n",
				       m->esc_id, (double)s->speed_cap, (double)m->fb.vel);
			}
			s->speed_tripped = true;
			s->tor_setpoint = 0.0f;
			s->tor_current = 0.0f;
			tor = 0.0f;
		}
		(void)dm_motor_mit(m, 0.0f, 0.0f, 0.0f, 0.0f, tor);
	} else if (s->stream_kind == STREAM_SAFEVEL) {
		/*
		 * Velocity loop closed here rather than in the driver, so the
		 * torque it asks for can be clamped.  vel_current is the ramped
		 * setpoint computed above.
		 */
		bool fresh = m->fb.valid && (k_uptime_get() - m->fb.stamp) < DM_FB_STALE_MS &&
			     (!s->vel_from_pos || s->vel_valid);
		float meas = s->vel_from_pos ? s->vel_est : m->fb.vel;
		/*
		 * Independent runaway guard on the DRIVER's velocity, which does
		 * not depend on our estimator: well above the setpoint means the
		 * loop has lost the plot, so idle at 0 N.m and say so.
		 */
		float sp = s->vel_setpoint < 0.0f ? -s->vel_setpoint : s->vel_setpoint;
		float drv = m->fb.vel < 0.0f ? -m->fb.vel : m->fb.vel;

		if (!s->svel_idle && m->fb.valid && drv > 1.5f * sp + 1.0f) {
			s->svel_idle = true;
			s->svel_runaway = true;
			s->vel_setpoint = 0.0f;
			s->vel_current = 0.0f;
			printk("dm: 0x%03x RUNAWAY: driver reports %.2f rad/s for a %.2f setpoint; "
			       "safevel idled at 0 N.m\n", m->esc_id, (double)m->fb.vel, (double)sp);
		}

		if (s->svel_idle) {
			/* Stopped: keep the frames flowing so the driver does
			 * not declare comm loss, but ask for no torque at all.
			 */
			dm_vel_pi_reset(&s->vel_pi);
			s->tor_current = 0.0f;
		} else if (fresh) {
			s->tor_current = dm_vel_pi_step(&s->vel_pi, s->vel_current, meas,
							(float)s->stream_period_ms / 1000.0f);
			s->svel_stale_warned = false;
		} else {
			/* Open loop: command nothing rather than guess. */
			dm_vel_pi_reset(&s->vel_pi);
			s->tor_current = 0.0f;
			if (!s->svel_stale_warned) {
				printk("dm: 0x%03x no fresh feedback, safevel commanding 0 N.m\n",
				       m->esc_id);
				s->svel_stale_warned = true;
			}
		}
		(void)dm_motor_mit(m, 0.0f, 0.0f, 0.0f, 0.0f, s->tor_current);
	} else {
		(void)dm_motor_vel(m, s->vel_current);
	}
	s->stream_count++;

	if (m->fb.valid && m->fb.state != s->last_state) {
		s->last_state = m->fb.state;
		if (s->last_state > DM_STATE_ENABLED) {
			printk("dm: 0x%03x state 0x%x (error) at %.2f rad/s cmd\n", m->esc_id,
			       s->last_state, (double)s->vel_current);
		}
	}
	trace_tick(s);

	s->stream_next += k_ms_to_ticks_ceil32(s->stream_period_ms);
	k_work_schedule(&s->stream_work, K_TIMEOUT_ABS_TICKS(s->stream_next));
}


static int cmd_fb(const struct shell *sh, size_t argc, char **argv);

static int need_motor(const struct shell *sh)
{
	if (!have_motor()) {
		shell_error(sh, "run: dm init <esc_id> [mst_id] [pmax vmax tmax]");
		return -ENODEV;
	}
	return 0;
}

/*
 * Enforce the single-motor lockout for a command that would energise the
 * selected motor.  Returns 0 if allowed; otherwise prints which motor is in
 * the way and returns -EBUSY.
 */
static int lockout_check(const struct shell *sh)
{
	if (!lockout) {
		return 0;
	}
	for (int i = 0; i < DM_SLOTS; i++) {
		struct dm_slot *s = &slots[i];

		if (s == cur || !s->ready) {
			continue;
		}
		if (s->energised || s->stream_period_ms) {
			shell_error(sh, "lockout: motor esc 0x%03x is %s; one motor at a time "
				    "('dm init 0x%x', 'dm disable', 'dm stream 0' first, or 'dm lockout off')",
				    s->m.esc_id, s->stream_period_ms ? "streaming" : "enabled",
				    s->m.esc_id);
			return -EBUSY;
		}
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


/*
 * 'dm init <esc> [mst] [pmax vmax tmax]': select the slot for this motor,
 * creating it if there is none.  Re-selecting a running motor does not
 * disturb it (its stream keeps going), so you can hop between motors.
 */
static int cmd_init(const struct shell *sh, size_t argc, char **argv)
{
	uint16_t esc = argc > 1 ? argu(argv[1]) : 0x01;
	/* MST_ID defaults to ESC_ID + 0x10, the DaMiao convention set_can_id.py uses. */
	uint16_t mst = argc > 2 ? argu(argv[2]) : esc + 0x10;
	float pmax = argc > 3 ? argf(argv[3]) : DM_S3519_PMAX;
	float vmax = argc > 4 ? argf(argv[4]) : DM_S3519_VMAX;
	float tmax = argc > 5 ? argf(argv[5]) : DM_S3519_TMAX;
	struct dm_slot *s = slot_by_esc(esc);
	int ret;

	if (s != NULL && s->m.mst_id == mst && argc <= 3) {
		cur = s;
		shell_print(sh, "selected motor esc 0x%03x mst 0x%03x (slot %d%s)", esc, mst,
			    (int)(s - slots), s->stream_period_ms ? ", streaming" : "");
		return 0;
	}
	if (s == NULL) {
		for (int i = 0; i < DM_SLOTS; i++) {
			if (!slots[i].ready) {
				s = &slots[i];
				break;
			}
		}
		if (s == NULL) {
			shell_error(sh, "all %d motor slots in use; 'dm release' one first", DM_SLOTS);
			return -ENOMEM;
		}
		slot_defaults(s);
	} else {
		/* Same ESC, different MST or limits: re-create in place. */
		if (s->stream_period_ms) {
			slot_halt(s);
			shell_warn(sh, "stream on esc 0x%03x stopped for re-init", esc);
		}
		s->ready = false;
		dm_motor_release(&s->m);
	}
	uint32_t timeout_raw;
	bool fd = s->m.fd;

	ret = dm_motor_init(&s->m, can_dev, esc, mst, pmax, vmax, tmax);
	if (ret) {
		shell_error(sh, "init failed: %d", ret);
		return ret;
	}
	s->m.fd = fd;
	s->ready = true;
	cur = s;
	/* Prefer the mapping ranges stored in the driver, if it answers. */
	if (dm_motor_sync_limits(&s->m, K_MSEC(100)) == 0) {
		shell_print(sh, "limits read from driver");
		if (dm_motor_read_reg(&s->m, DM_RID_TIMEOUT, &timeout_raw, K_MSEC(100)) == 0 &&
		    timeout_raw == 0) {
			shell_warn(sh, "driver TIMEOUT (rid 9) is 0: it will hold its last command "
				   "for ever if frames stop; 'dm write 9 <ms>' + 'dm save' to fix");
		}
	} else {
		shell_print(sh, "driver did not answer register reads; using given/default limits");
	}
	shell_print(sh, "motor esc 0x%03x mst 0x%03x PMAX %.2f VMAX %.2f TMAX %.2f fd %s (slot %d)",
		    esc, mst, (double)s->m.p_max, (double)s->m.v_max, (double)s->m.t_max,
		    s->m.fd ? "on" : "off", (int)(s - slots));
	return 0;
}

/* 'dm release [esc]': stop, disable and forget a motor, freeing its slot. */
static int cmd_release(const struct shell *sh, size_t argc, char **argv)
{
	struct dm_slot *s = argc > 1 ? slot_by_esc(argu(argv[1])) : cur;

	if (s == NULL || !s->ready) {
		shell_error(sh, "no such motor");
		return -ENODEV;
	}
	slot_halt(s);
	s->ready = false;
	dm_motor_release(&s->m);
	shell_print(sh, "released esc 0x%03x (stopped and disabled)", s->m.esc_id);
	if (cur == s) {
		cur = NULL;
	}
	return 0;
}

static const char *kind_name(enum stream_kind k)
{
	switch (k) {
	case STREAM_TORQUE:
		return "torque";
	case STREAM_SAFEVEL:
		return "safevel";
	default:
		return "velocity";
	}
}

/* 'dm motors': one line per slot in use. */
static int cmd_motors(const struct shell *sh, size_t argc, char **argv)
{
	int n = 0;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	for (int i = 0; i < DM_SLOTS; i++) {
		struct dm_slot *s = &slots[i];

		if (!s->ready) {
			continue;
		}
		n++;
		if (s->stream_period_ms) {
			shell_print(sh, "%c esc 0x%03x mst 0x%03x  %s %u Hz%s  set %.3f  sending %.3f  "
				    "fb %s state %u vel %.3f tor %.3f%s",
				    s == cur ? '*' : ' ', s->m.esc_id, s->m.mst_id,
				    kind_name(s->stream_kind), 1000U / s->stream_period_ms,
				    (s->stream_kind == STREAM_SAFEVEL && s->svel_idle) ? " IDLE" : "",
				    (double)(s->stream_kind == STREAM_TORQUE ? s->tor_setpoint
									     : s->vel_setpoint),
				    (double)(s->stream_kind == STREAM_VEL ? s->vel_current
									  : s->tor_current),
				    s->m.fb.valid ? "ok" : "none", s->m.fb.state, (double)s->m.fb.vel,
				    (double)s->m.fb.tor, s->hb_tripped ? " [WD TRIPPED]" : "");
		} else {
			shell_print(sh, "%c esc 0x%03x mst 0x%03x  stream off%s  fb %s state %u vel %.3f",
				    s == cur ? '*' : ' ', s->m.esc_id, s->m.mst_id,
				    s->energised ? ", ENABLED" : "", s->m.fb.valid ? "ok" : "none",
				    s->m.fb.state, (double)s->m.fb.vel);
		}
	}
	if (n == 0) {
		shell_print(sh, "no motors; 'dm init <esc>'");
	}
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
	if (need_motor(sh)) {
		return -ENODEV;
	}
	if (argc < 2) {
		shell_print(sh, "frame mode: 0x%03x (mit=0x000 pos=0x100 vel=0x200 psi=0x300)", cur->cur_mode);
		return 0;
	}
	if (parse_mode(argv[1], &cur->cur_mode)) {
		shell_error(sh, "mode is mit|pos|vel|psi");
		return -EINVAL;
	}
	shell_print(sh, "commands now use ID 0x%03x + esc_id", cur->cur_mode);
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
		int ret = fn(&cur->m, cur->cur_mode);                                              \
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

	/* A latched fault (comm-lost after the TIMEOUT register fired, over-
	 * voltage, ...) makes the driver ignore enable; clear it first.
	 */
	(void)dm_motor_clear_error(m, mode);
	k_sleep(K_MSEC(5));
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

static int cmd_enable(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (need_motor(sh) || lockout_check(sh)) {
		return -ENODEV;
	}
	int ret = enable_repeat(&cur->m, cur->cur_mode);

	cur->energised = (ret == 0);
	shell_print(sh, "enable: %d", ret);
	return ret;
}
static int cmd_disable(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (need_motor(sh)) {
		return -ENODEV;
	}
	int ret = dm_motor_disable(&cur->m, cur->cur_mode);

	cur->energised = false;
	shell_print(sh, "disable: %d", ret);
	if (cur->stream_period_ms) {
		shell_print(sh, "(its %s stream is still running; 'dm stream 0' stops it, or a mode "
			    "command re-enables)", kind_name(cur->stream_kind));
	}
	return ret;
}
SIMPLE_CMD(zero, dm_motor_set_zero)
SIMPLE_CMD(clearerr, dm_motor_clear_error)

static int cmd_mit(const struct shell *sh, size_t argc, char **argv)
{
	if (need_motor(sh) || lockout_check(sh)) {
		return -ENODEV;
	}
	if (argc != 6) {
		shell_error(sh, "usage: dm mit <pos rad> <vel rad/s> <kp> <kd> <torque Nm>");
		return -EINVAL;
	}
	return dm_motor_mit(&cur->m, argf(argv[1]), argf(argv[2]), argf(argv[3]), argf(argv[4]),
			    argf(argv[5]));
}

static int cmd_pos(const struct shell *sh, size_t argc, char **argv)
{
	if (need_motor(sh) || lockout_check(sh)) {
		return -ENODEV;
	}
	if (argc != 3) {
		shell_error(sh, "usage: dm pos <pos rad> <vel rad/s>");
		return -EINVAL;
	}
	return dm_motor_pos_vel(&cur->m, argf(argv[1]), argf(argv[2]));
}

static int cmd_vel(const struct shell *sh, size_t argc, char **argv)
{
	if (need_motor(sh) || lockout_check(sh)) {
		return -ENODEV;
	}
	if (argc != 2) {
		shell_error(sh, "usage: dm vel <vel rad/s>   (streamed continuously if 'dm stream' is on)");
		return -EINVAL;
	}
	cur->vel_setpoint = argf(argv[1]);
	if (cur->stream_kind == STREAM_SAFEVEL && cur->svel_idle) {
		/* Re-engage the loop after a stop, from a clean integrator. */
		cur->svel_idle = false;
		dm_vel_pi_reset(&cur->vel_pi);
	}
	if (cur->stream_period_ms) {
		if (cur->stream_kind == STREAM_SAFEVEL) {
			shell_print(sh, "setpoint %.3f rad/s, torque capped at %.2f N.m (every %u ms)",
				    (double)cur->vel_setpoint, (double)cur->vel_pi.tor_max, cur->stream_period_ms);
		} else {
			shell_print(sh, "setpoint %.3f rad/s (streaming every %u ms)",
				    (double)cur->vel_setpoint, cur->stream_period_ms);
		}
		return 0;
	}
	if (cur->stream_kind == STREAM_SAFEVEL) {
		/* The loop only exists while streaming; a raw 0x200 frame would
		 * also be wrong here because the driver is in MIT mode.
		 */
		shell_error(sh, "safevel needs the streamer: run 'dm safevelmode'");
		return -EINVAL;
	}
	cur->vel_current = cur->vel_setpoint;
	int ret = dm_motor_vel(&cur->m, cur->vel_setpoint);

	shell_print(sh, "one velocity frame on 0x%03x (%d); nothing is streaming on this motor, so "
		    "it only moves if the driver is in CTRL_MODE 3 and enabled. Run 'dm velmode' "
		    "or 'dm safevelmode' first.", DM_ID_VEL + cur->m.esc_id, ret);
	return ret;
}

static int cmd_stop(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (need_motor(sh)) {
		return -ENODEV;
	}
	int ret;

	cur->vel_setpoint = 0.0f;
	cur->vel_current = 0.0f;
	cur->tor_setpoint = 0.0f;
	cur->tor_current = 0.0f;
	if (cur->stream_kind == STREAM_SAFEVEL) {
		/*
		 * Idle rather than regulate to 0 rad/s: holding a zero-speed
		 * setpoint leaves the loop chasing feedback quantisation, which
		 * shows up as jitter at standstill.  'dm vel 0' still asks for
		 * an actively braked zero if that is what you want.
		 */
		cur->svel_idle = true;
		dm_vel_pi_reset(&cur->vel_pi);
		ret = dm_motor_mit(&cur->m, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
		shell_print(sh, "safevel stopped: 0 N.m, loop idle (%d); 'dm vel <rad/s>' resumes",
			    ret);
	} else if (cur->stream_kind == STREAM_TORQUE) {
		ret = dm_motor_mit(&cur->m, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
		shell_print(sh, "torque setpoint 0: %d", ret);
	} else {
		ret = dm_motor_vel(&cur->m, 0.0f);
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
			    cur->stream_period_ms ? "on" : "off", cur->stream_period_ms, cur->stream_count,
			    (double)cur->vel_setpoint, (double)cur->vel_ramp);
		return 0;
	}
	uint32_t hz = argu(argv[1]);

	if (hz > 1000) {
		shell_error(sh, "max 1000 Hz");
		return -EINVAL;
	}
	if (argc > 2) {
		cur->vel_ramp = argf(argv[2]);
	}
	stream_start(cur, hz_to_period(hz));
	shell_print(sh, "streaming %s%s", hz ? "on" : "off", hz > 1000 ? " (capped at 1000 Hz)" : "");
	return 0;
}

/*
 * One-shot velocity-mode bring-up: disable, CTRL_MODE = 3, optionally save to
 * flash, switch the enable/disable frame ID to the 0x200 offset, enable, and
 * start streaming a 0 rad/s setpoint.
 */
static int cmd_velmode(const struct shell *sh, size_t argc, char **argv)
{
	bool save;
	uint32_t hz;
	int ret;

	parse_mode_args(argc, argv, &save, &hz);

	if (need_motor(sh) || lockout_check(sh)) {
		return -ENODEV;
	}
	stream_start(cur, 0);
	cur->vel_setpoint = 0.0f;
	cur->vel_current = 0.0f;

	(void)dm_motor_disable(&cur->m, cur->cur_mode);
	k_sleep(K_MSEC(10));

	ret = dm_motor_write_reg(&cur->m, DM_RID_CMODE, DM_MODE_VEL, K_MSEC(200));
	if (ret == -ETIMEDOUT) {
		shell_warn(sh, "no reply to CTRL_MODE write (motor off or IDs wrong?), continuing");
	} else if (ret) {
		shell_error(sh, "CTRL_MODE write failed: %d", ret);
		return ret;
	}
	if (save) {
		k_sleep(K_MSEC(10));
		ret = dm_motor_save_regs(&cur->m, K_MSEC(300));
		shell_print(sh, "save: %d", ret);
	}
	cur->cur_mode = DM_ID_VEL;
	ret = enable_repeat(&cur->m, DM_ID_VEL);
	if (ret) {
		shell_error(sh, "enable failed: %d", ret);
		return ret;
	}
	cur->energised = true;
	cur->stream_kind = STREAM_VEL;
	hb_rearm(cur);
	stream_start(cur, hz_to_period(hz));
	if (cur->m.fb.valid && cur->m.fb.state != DM_STATE_ENABLED) {
		shell_warn(sh, "driver reports state 0x%x, not enabled (0xD = comm lost; 'dm clearerr' "
			   "then re-run, or check power)", cur->m.fb.state);
	}
	shell_print(sh, "velocity mode: enabled on ID 0x%03x, streaming at %u Hz; use 'dm vel <rad/s>'",
		    DM_ID_VEL + cur->m.esc_id, 1000U / cur->stream_period_ms);
	return 0;
}

/*
 * Torque (current) mode bring-up: MIT mode with Kp = Kd = 0, streaming the
 * torque feed-forward term.
 */
static int cmd_torquemode(const struct shell *sh, size_t argc, char **argv)
{
	bool save;
	uint32_t hz;
	int ret;

	parse_mode_args(argc, argv, &save, &hz);

	if (need_motor(sh) || lockout_check(sh)) {
		return -ENODEV;
	}
	stream_start(cur, 0);
	cur->tor_setpoint = 0.0f;
	cur->tor_current = 0.0f;
	cur->speed_tripped = false;

	(void)dm_motor_disable(&cur->m, cur->cur_mode);
	k_sleep(K_MSEC(10));

	ret = dm_motor_write_reg(&cur->m, DM_RID_CMODE, DM_MODE_MIT, K_MSEC(200));
	if (ret == -ETIMEDOUT) {
		shell_warn(sh, "no reply to CTRL_MODE write (motor off or IDs wrong?), continuing");
	} else if (ret) {
		shell_error(sh, "CTRL_MODE write failed: %d", ret);
		return ret;
	}
	if (save) {
		k_sleep(K_MSEC(10));
		ret = dm_motor_save_regs(&cur->m, K_MSEC(300));
		shell_print(sh, "save: %d", ret);
	}
	cur->cur_mode = DM_ID_MIT;
	ret = enable_repeat(&cur->m, DM_ID_MIT);
	if (ret) {
		shell_error(sh, "enable failed: %d", ret);
		return ret;
	}
	cur->energised = true;
	cur->stream_kind = STREAM_TORQUE;
	hb_rearm(cur);
	stream_start(cur, hz_to_period(hz));
	if (cur->m.fb.valid && cur->m.fb.state != DM_STATE_ENABLED) {
		shell_warn(sh, "driver reports state 0x%x, not enabled (0xD = comm lost; 'dm clearerr' "
			   "then re-run, or check power)", cur->m.fb.state);
	}
	shell_print(sh, "torque mode: MIT on ID 0x%03x, Kp=Kd=0, streaming at %u Hz, ramp %.1f N.m/s, "
		    "speed cap %.1f rad/s; use 'dm torque <Nm>' or 'dm current <A>'",
		    DM_ID_MIT + cur->m.esc_id, 1000U / cur->stream_period_ms, (double)cur->tor_ramp,
		    (double)cur->speed_cap);
	return 0;
}

/*
 * "Safe velocity" bring-up: velocity control with a saturating torque cap.
 *
 * The driver's own velocity mode (CTRL_MODE 3) will spend whatever torque it
 * needs up to TMAX to hold the setpoint, so a jam is only ever caught by the
 * driver's protections.  Here the velocity loop runs on the MCU instead: MIT
 * frames with Kp = Kd = 0 carry a torque computed from the velocity error and
 * clamped to 'dm torquecap'.
 *
 * Reaching the cap is a normal operating point, not a fault.  A stalled motor
 * pushes at exactly the cap and keeps pushing; when the load is removed it
 * accelerates and goes back to tracking the setpoint with no intervention.
 */
static int cmd_safevelmode(const struct shell *sh, size_t argc, char **argv)
{
	bool save;
	uint32_t hz;
	int ret;

	parse_mode_args(argc, argv, &save, &hz);
	if (hz < 200) {
		shell_error(sh, "safevel needs at least 200 Hz (velocity is differenced over %u ms of "
			    "feedback)", cur ? cur->vel_win_ms : 10);
		return -EINVAL;
	}
	if (cur != NULL) {
		cur->svel_runaway = false;
	}

	if (need_motor(sh) || lockout_check(sh)) {
		return -ENODEV;
	}
	stream_start(cur, 0);
	cur->vel_setpoint = 0.0f;
	cur->vel_current = 0.0f;
	cur->tor_setpoint = 0.0f;
	cur->tor_current = 0.0f;
	dm_vel_pi_reset(&cur->vel_pi);
	cur->svel_stale_warned = false;
	/* Enter idle: entering the mode should not start regulating anything. */
	cur->svel_idle = true;

	(void)dm_motor_disable(&cur->m, cur->cur_mode);
	k_sleep(K_MSEC(10));

	/* The torque term is carried by a MIT frame, so the driver runs MIT. */
	ret = dm_motor_write_reg(&cur->m, DM_RID_CMODE, DM_MODE_MIT, K_MSEC(200));
	if (ret == -ETIMEDOUT) {
		shell_warn(sh, "no reply to CTRL_MODE write (motor off or IDs wrong?), continuing");
	} else if (ret) {
		shell_error(sh, "CTRL_MODE write failed: %d", ret);
		return ret;
	}
	if (save) {
		k_sleep(K_MSEC(10));
		ret = dm_motor_save_regs(&cur->m, K_MSEC(300));
		shell_print(sh, "save: %d", ret);
	}
	cur->cur_mode = DM_ID_MIT;
	ret = enable_repeat(&cur->m, DM_ID_MIT);
	if (ret) {
		shell_error(sh, "enable failed: %d", ret);
		return ret;
	}
	cur->energised = true;
	if (cur->vel_pi.tor_max > cur->m.t_max) {
		cur->vel_pi.tor_max = cur->m.t_max;
	}
	/*
	 * Size the deadband to the resolution the driver actually reports:
	 * VMAX may differ from the default, and it sets the quantisation step.
	 */
	if (!cur->svel_deadband_user && !cur->vel_from_pos) {
		cur->vel_pi.deadband = DM_VEL_DEADBAND(cur->m.v_max);
	}
	cur->stream_kind = STREAM_SAFEVEL;
	hb_rearm(cur);
	stream_start(cur, hz_to_period(hz));
	if (cur->m.fb.valid && cur->m.fb.state != DM_STATE_ENABLED) {
		shell_warn(sh, "driver reports state 0x%x, not enabled (0xD = comm lost; 'dm clearerr' "
			   "then re-run, or check power)", cur->m.fb.state);
	}
	shell_print(sh, "safe velocity mode: MIT on ID 0x%03x, streaming at %u Hz, ramp %.1f rad/s^2",
		    DM_ID_MIT + cur->m.esc_id, 1000U / cur->stream_period_ms, (double)cur->vel_ramp);
	shell_print(sh, "torque cap %.2f N.m (~%.2f A), gains kp %.3f ki %.3f (imax %.2f), "
		    "deadband %.3f rad/s", (double)cur->vel_pi.tor_max,
		    (double)(cur->vel_pi.tor_max / kt_nm_per_a), (double)cur->vel_pi.kp,
		    (double)cur->vel_pi.ki, (double)cur->vel_pi.i_max, (double)cur->vel_pi.deadband);
	shell_print(sh, "friction ff %.3f N.m, breakaway kick %.3f N.m after %.0f ms stuck, "
		    "velocity filter %.1f ms, velocity from %s", (double)cur->vel_pi.ff,
		    (double)cur->vel_pi.ff_brk, (double)cur->vel_pi.stick_ms,
		    (double)cur->vel_pi.tau_ms,
		    cur->vel_from_pos ? "rotor position" : "driver estimate");
	shell_print(sh, "stalling is held at the cap, not tripped; speed returns when the load clears");
	shell_print(sh, "idle until 'dm vel <rad/s>'; 'dm stop' returns to idle (0 N.m)");
	return 0;
}

static int cmd_torquecap(const struct shell *sh, size_t argc, char **argv)
{
	if (need_motor(sh)) {
		return -ENODEV;
	}
	if (argc > 1) {
		float cap = argf(argv[1]);

		if (cap < 0.0f) {
			cap = 0.0f;
		}
		if (have_motor() && cap > cur->m.t_max) {
			shell_warn(sh, "clamped to TMAX %.2f N.m", (double)cur->m.t_max);
			cap = cur->m.t_max;
		}
		cur->vel_pi.tor_max = cap;
		dm_vel_pi_reset(&cur->vel_pi);
	}
	shell_print(sh, "safevel torque cap = %.3f N.m (~%.2f A); the loop saturates here rather "
		    "than cutting out", (double)cur->vel_pi.tor_max,
		    (double)(cur->vel_pi.tor_max / kt_nm_per_a));
	return 0;
}

static int cmd_velgains(const struct shell *sh, size_t argc, char **argv)
{
	if (need_motor(sh)) {
		return -ENODEV;
	}
	if (argc > 1) {
		cur->vel_pi.kp = argf(argv[1]);
	}
	if (argc > 2) {
		cur->vel_pi.ki = argf(argv[2]);
	}
	if (argc > 1) {
		dm_vel_pi_reset(&cur->vel_pi);
	}
	shell_print(sh, "safevel gains: kp %.4f N.m per rad/s, ki %.4f N.m per rad",
		    (double)cur->vel_pi.kp, (double)cur->vel_pi.ki);
	return 0;
}

static int cmd_velff(const struct shell *sh, size_t argc, char **argv)
{
	if (need_motor(sh)) {
		return -ENODEV;
	}
	if (argc > 1) {
		cur->vel_pi.ff = argf(argv[1]) < 0.0f ? 0.0f : argf(argv[1]);
	}
	if (argc > 2) {
		cur->vel_pi.ff_brk = argf(argv[2]) < 0.0f ? 0.0f : argf(argv[2]);
	}
	if (argc > 3) {
		cur->vel_pi.i_max = argf(argv[3]) < 0.0f ? 0.0f : argf(argv[3]);
	}
	if (argc > 1) {
		dm_vel_pi_reset(&cur->vel_pi);
	}
	shell_print(sh, "safevel feed-forward: kinetic %.3f N.m, breakaway kick %.3f N.m, "
		    "integrator clamp %.2f N.m (0 = torque cap)", (double)cur->vel_pi.ff,
		    (double)cur->vel_pi.ff_brk, (double)cur->vel_pi.i_max);
	return 0;
}

static int cmd_velfilt(const struct shell *sh, size_t argc, char **argv)
{
	if (need_motor(sh)) {
		return -ENODEV;
	}
	if (argc > 1) {
		cur->vel_pi.tau_ms = argf(argv[1]) < 0.0f ? 0.0f : argf(argv[1]);
	}
	if (argc > 2) {
		cur->vel_pi.stick_ms = argf(argv[2]) < 0.0f ? 0.0f : argf(argv[2]);
	}
	if (argc > 3) {
		cur->vel_pi.ff_taper = argf(argv[3]) < 0.0f ? 0.0f : argf(argv[3]);
	}
	if (argc > 4) {
		cur->vel_pi.stick_v = argf(argv[4]) < 0.0f ? 0.0f : argf(argv[4]);
	}
	if (argc > 1) {
		dm_vel_pi_reset(&cur->vel_pi);
	}
	shell_print(sh, "safevel filter: velocity low-pass %.1f ms, kick after %.0f ms under %.3f rad/s, "
		    "feed-forward tapered over %.2f rad/s", (double)cur->vel_pi.tau_ms,
		    (double)cur->vel_pi.stick_ms, (double)cur->vel_pi.stick_v,
		    (double)cur->vel_pi.ff_taper);
	return 0;
}

static int cmd_velsrc(const struct shell *sh, size_t argc, char **argv)
{
	if (need_motor(sh)) {
		return -ENODEV;
	}
	if (argc > 1) {
		if (!strcmp(argv[1], "pos")) {
			cur->vel_from_pos = true;
		} else if (!strcmp(argv[1], "fb")) {
			cur->vel_from_pos = false;
		} else {
			shell_error(sh, "usage: dm velsrc pos|fb [window ticks]");
			return -EINVAL;
		}
	}
	if (argc > 2) {
		uint32_t w = argu(argv[2]);

		cur->vel_win_ms = w < 2 ? 2 : (w > 30 ? 30 : w);
	}
	shell_print(sh, "safevel velocity source: %s (gear ratio %.3f); now %.4f rad/s%s vs driver %.4f",
		    cur->vel_from_pos ? "rotor position differenced over time" : "driver estimate",
		    (double)cur->m.gr, (double)cur->vel_est, cur->vel_valid ? "" : " (invalid)",
		    (double)cur->m.fb.vel);
	if (cur->vel_from_pos) {
		shell_print(sh, "window %u ms (needs feedback at least every %u ms)", cur->vel_win_ms,
			    cur->vel_win_ms);
	}
	return 0;
}

static int cmd_lockout(const struct shell *sh, size_t argc, char **argv)
{
	if (argc > 1) {
		if (!strcmp(argv[1], "on")) {
			lockout = true;
		} else if (!strcmp(argv[1], "off")) {
			lockout = false;
		} else {
			shell_error(sh, "usage: dm lockout [on|off]");
			return -EINVAL;
		}
	}
	int n = 0;

	for (int i = 0; i < DM_SLOTS; i++) {
		if (slots[i].ready && (slots[i].energised || slots[i].stream_period_ms)) {
			n++;
		}
	}
	shell_print(sh, "single-motor lockout %s; %d motor(s) currently energised or streaming%s",
		    lockout ? "ON" : "off", n,
		    (lockout && n > 1) ? " (more than one: disable all but one)" : "");
	return 0;
}

static int cmd_trace(const struct shell *sh, size_t argc, char **argv)
{
	if (argc > 1 && !strcmp(argv[1], "dump")) {
		uint32_t n = trace_running ? 0 : trace_n;

		shell_print(sh, "trace: %u samples%s", n, trace_want ? " (still armed)" : "");
		shell_print(sh, "t_us,cmd,meas,fbv,tor,integ,state,pos,fb_cyc");
		for (uint32_t i = 0; i < n; i++) {
			const struct trace_sample *t = &trace_buf[i];

			shell_print(sh, "%u,%.4f,%.4f,%.4f,%.4f,%.4f,%u,%.5f,%u", t->t_us, (double)t->cmd,
				    (double)t->meas, (double)t->fbv, (double)t->tor, (double)t->integ,
				    t->state, (double)t->pos, t->fb_cyc);
		}
		shell_print(sh, "trace: end");
		return 0;
	}
	if (argc > 1) {
		uint32_t n = argu(argv[1]);

		if (need_motor(sh)) {
			return -ENODEV;
		}
		if (n == 0 || n > TRACE_MAX) {
			shell_error(sh, "1..%d samples", TRACE_MAX);
			return -EINVAL;
		}
		trace_running = false;
		trace_n = 0;
		trace_slot = cur;
		trace_want = n;
		shell_print(sh, "trace armed on esc 0x%03x: %u samples at the loop rate, starts with "
			    "the next move ('dm vel'); 'dm trace dump' to read",
			    cur->m.esc_id, n);
		return 0;
	}
	shell_print(sh, "trace: %s, %u/%u samples%s", trace_want ? "armed" : "idle", trace_n,
		    trace_want ? trace_want : trace_n, trace_running ? " (capturing)" : "");
	return 0;
}

static int cmd_watchdog(const struct shell *sh, size_t argc, char **argv)
{
	if (need_motor(sh)) {
		return -ENODEV;
	}
	if (argc > 1) {
		cur->hb_timeout_ms = argu(argv[1]);
		/* Restart the clock so arming does not trip at once, but leave a
		 * trip in place: only a mode command restarts the stream and
		 * re-enables the driver, so clearing the flag here would report
		 * "ok" while the motor sits disabled.
		 */
		hb_last_ms = k_uptime_get_32();
	}
	if (cur->hb_timeout_ms) {
		shell_print(sh, "watchdog %u ms: 'dm hb' must arrive this often or the motor is "
			    "disabled%s", cur->hb_timeout_ms, cur->hb_tripped ? " [TRIPPED]" : "");
	} else {
		shell_print(sh, "watchdog off");
	}
	return 0;
}

/* Sent several times a second by the host, so keep the reply short. */
static int cmd_hb(const struct shell *sh, size_t argc, char **argv)
{
	bool tripped = false;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	hb_last_ms = k_uptime_get_32();
	/* A late heartbeat does not undo a trip: the motor stays disabled
	 * until a mode command re-enables it.
	 */
	for (int i = 0; i < DM_SLOTS; i++) {
		tripped |= slots[i].ready && slots[i].hb_tripped;
	}
	shell_print(sh, tripped ? "hb TRIPPED" : "hb ok");
	return 0;
}

/*
 * The feedback cannot express zero velocity (12 bits over +-VMAX puts zero at
 * code 2047.5), so a stationary motor reports +-VMAX/4095 for ever.  Without a
 * deadband that bias ratchets the integrator until it breaks stiction and the
 * motor jitters.  Shrinking VMAX (RID 22) is the real cure; this bounds the
 * damage at whatever VMAX is in use.
 */
static int cmd_veldeadband(const struct shell *sh, size_t argc, char **argv)
{
	float lsb = DM_VEL_LSB(have_motor() ? cur->m.v_max : DM_S3519_VMAX);

	if (need_motor(sh)) {
		return -ENODEV;
	}

	if (argc > 1) {
		float db = argf(argv[1]);

		if (db < 0.0f) {
			db = 0.0f;
		}
		cur->vel_pi.deadband = db;
		cur->svel_deadband_user = true;
		dm_vel_pi_reset(&cur->vel_pi);
	}
	shell_print(sh, "safevel deadband = %.4f rad/s (%.2f counts of %.4f); "
		    "velocity error inside it is ignored", (double)cur->vel_pi.deadband,
		    (double)(cur->vel_pi.deadband / lsb), (double)lsb);
	if (cur->vel_pi.deadband < lsb) {
		shell_warn(sh, "below one count: standstill bias will still wind the integrator");
	}
	return 0;
}

static int set_torque(const struct shell *sh, float tor)
{
	if (tor > cur->m.t_max) {
		tor = cur->m.t_max;
	} else if (tor < -cur->m.t_max) {
		tor = -cur->m.t_max;
	}
	cur->tor_setpoint = tor;
	cur->speed_tripped = false;
	if (cur->stream_period_ms && cur->stream_kind == STREAM_TORQUE) {
		shell_print(sh, "torque setpoint %.3f N.m (~%.2f A), streaming every %u ms", (double)tor,
			    (double)(tor / kt_nm_per_a), cur->stream_period_ms);
		return 0;
	}
	shell_print(sh, "torque %.3f N.m (~%.2f A), single frame (run 'dm torquemode' to stream)",
		    (double)tor, (double)(tor / kt_nm_per_a));
	return dm_motor_mit(&cur->m, 0.0f, 0.0f, 0.0f, 0.0f, tor);
}

static int cmd_torque(const struct shell *sh, size_t argc, char **argv)
{
	if (need_motor(sh) || lockout_check(sh)) {
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
	if (need_motor(sh) || lockout_check(sh)) {
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
	if (need_motor(sh)) {
		return -ENODEV;
	}
	if (argc > 1) {
		cur->tor_ramp = argf(argv[1]);
	}
	shell_print(sh, "torque ramp = %.2f N.m/s (0 = step)", (double)cur->tor_ramp);
	return 0;
}

static int cmd_speedcap(const struct shell *sh, size_t argc, char **argv)
{
	if (need_motor(sh)) {
		return -ENODEV;
	}
	if (argc > 1) {
		cur->speed_cap = argf(argv[1]);
		cur->speed_tripped = false;
	}
	shell_print(sh, "speed cap = %.1f rad/s (0 = off); torque is cut above it%s",
		    (double)cur->speed_cap, cur->speed_tripped ? " [TRIPPED, set a new torque to clear]" : "");
	return 0;
}

static int cmd_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (need_motor(sh)) {
		return -ENODEV;
	}
	shell_print(sh, "slot %d: esc 0x%03x mst 0x%03x frame-mode 0x%03x fd %s", (int)(cur - slots),
		    cur->m.esc_id, cur->m.mst_id, cur->cur_mode, cur->m.fd ? "on" : "off");
	if (cur->stream_kind == STREAM_SAFEVEL) {
		bool sat = cur->vel_pi.tor_max > 0.0f &&
			   (cur->tor_current >= cur->vel_pi.tor_max - 1e-4f ||
			    cur->tor_current <= -cur->vel_pi.tor_max + 1e-4f);

		shell_print(sh, "stream %s %u ms (safevel%s), sent %u, setpoint %.3f, sending %.3f rad/s",
			    cur->stream_period_ms ? "on" : "off", cur->stream_period_ms,
			    cur->svel_runaway ? ", RUNAWAY GUARD TRIPPED" : (cur->svel_idle ? ", IDLE" : ""), cur->stream_count,
			    (double)cur->vel_setpoint, (double)cur->vel_current);
		shell_print(sh, "torque %.3f N.m (~%.2f A) of %.2f cap%s, integ %.3f, kp %.3f ki %.3f, "
			    "deadband %.3f rad/s", (double)cur->tor_current,
			    (double)(cur->tor_current / kt_nm_per_a), (double)cur->vel_pi.tor_max,
			    sat ? " [AT CAP]" : "", (double)cur->vel_pi.integ, (double)cur->vel_pi.kp,
			    (double)cur->vel_pi.ki, (double)cur->vel_pi.deadband);
	} else if (cur->stream_kind == STREAM_TORQUE) {
		shell_print(sh, "stream %s %u ms (torque), sent %u, torque %.3f N.m (~%.2f A), sending %.3f, ramp %.1f N.m/s, cap %.1f rad/s%s",
			    cur->stream_period_ms ? "on" : "off", cur->stream_period_ms, cur->stream_count,
			    (double)cur->tor_setpoint, (double)(cur->tor_setpoint / kt_nm_per_a), (double)cur->tor_current,
			    (double)cur->tor_ramp, (double)cur->speed_cap, cur->speed_tripped ? " TRIPPED" : "");
	} else {
		shell_print(sh, "stream %s %u ms (velocity), sent %u, setpoint %.3f, sending %.3f rad/s",
			    cur->stream_period_ms ? "on" : "off", cur->stream_period_ms, cur->stream_count,
			    (double)cur->vel_setpoint, (double)cur->vel_current);
	}
	if (cur->hb_timeout_ms) {
		shell_print(sh, "watchdog %u ms, last hb %u ms ago%s", cur->hb_timeout_ms,
			    (uint32_t)(k_uptime_get_32() - hb_last_ms),
			    cur->hb_tripped ? " [TRIPPED]" : "");
	}
	return cmd_fb(sh, 0, NULL);
}

static int cmd_psi(const struct shell *sh, size_t argc, char **argv)
{
	if (need_motor(sh) || lockout_check(sh)) {
		return -ENODEV;
	}
	if (argc != 4) {
		shell_error(sh, "usage: dm psi <pos rad> <vel rad/s> <current A>");
		return -EINVAL;
	}
	return dm_motor_psi(&cur->m, argf(argv[1]), argf(argv[2]), argf(argv[3]));
}

static int cmd_cur3519(const struct shell *sh, size_t argc, char **argv)
{
	if (lockout) {
		shell_error(sh, "lockout: the one-to-four frame drives up to four motors at once; "
			    "'dm lockout off' first");
		return -EBUSY;
	}
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
	if (cur->m.fb.valid) {
		shell_print(sh, "std fb (%lld ms ago): id %u state %u pos %.4f rad vel %.3f rad/s "
			    "tor %.3f Nm Tmos %.0f Tcoil %.0f",
			    (long long)(k_uptime_get() - cur->m.fb.stamp), cur->m.fb.id,
			    cur->m.fb.state, (double)cur->m.fb.pos, (double)cur->m.fb.vel,
			    (double)cur->m.fb.tor, (double)cur->m.fb.t_mos, (double)cur->m.fb.t_coil);
	} else {
		shell_print(sh, "no standard feedback received yet on id 0x%03x", cur->m.mst_id);
	}
	if (cur->m.fb3519.valid) {
		shell_print(sh, "dm3519 fb (%lld ms ago): pos %.2f deg rpm %d cur %.3f A Tcoil %.0f "
			    "Tmos %.0f",
			    (long long)(k_uptime_get() - cur->m.fb3519.stamp),
			    (double)cur->m.fb3519.pos_deg, cur->m.fb3519.rpm, (double)cur->m.fb3519.cur,
			    (double)cur->m.fb3519.t_coil, (double)cur->m.fb3519.t_mos);
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
	int ret = dm_motor_request_feedback(&cur->m);

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

	ret = dm_motor_read_reg(&cur->m, rid, &raw, K_MSEC(200));
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
		ret = dm_motor_write_reg_float(&cur->m, rid, argf(argv[2]), K_MSEC(200));
	} else {
		ret = dm_motor_write_reg(&cur->m, rid, argu(argv[2]), K_MSEC(200));
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
	int ret = dm_motor_save_regs(&cur->m, K_MSEC(300));

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

	for (int i = 0; i < DM_SLOTS; i++) {
		slots[i].m.fd = fd;
	}
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
		int ret = dm_motor_read_reg(&cur->m, regs[i].rid, &raw, K_MSEC(200));

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


/* --- bus scan ------------------------------------------------------------
 * There is no broadcast on the DaMiao register channel: a 0x7FF read is only
 * answered by the motor whose ESC_ID is in the payload, and the answer comes
 * back on that motor's MST_ID, which we do not know either.  So probe every
 * ESC_ID with a read of RID 8 through a catch-all RX filter; each reply gives
 * ESC_ID (payload bytes 0-1) and MST_ID (the reply's CAN ID).  This is what
 * the DaMiao debug tool and the open-source scanners do too.
 *
 * Two motors with identical ESC_ID *and* MST_ID answer with identical frames
 * at the same time, which CAN merges into one; the host tool catches that case
 * afterwards by reading the serial number, which differs.
 */
#define SCAN_MAX 16

struct scan_hit {
	uint16_t esc;
	uint16_t mst;
	uint32_t count;
};

static struct scan_hit scan_hits[SCAN_MAX];
static volatile int scan_nhits;
static volatile bool scan_overflow;
static volatile uint32_t scan_other_frames;
static volatile uint16_t scan_other_id;
static K_SEM_DEFINE(scan_tx_sem, 0, 1);
static volatile int scan_tx_err;

static void scan_tx_done(const struct device *dev, int error, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);
	scan_tx_err = error;
	k_sem_give(&scan_tx_sem);
}

static void scan_rx(const struct device *dev, struct can_frame *frame, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);
	if (frame->flags & (CAN_FRAME_IDE | CAN_FRAME_RTR)) {
		return;
	}
	if (can_dlc_to_bytes(frame->dlc) >= 8 && frame->data[2] == 0x33 &&
	    frame->data[3] == DM_RID_ESC_ID) {
		uint16_t esc = frame->data[0] | ((frame->data[1] & 0x07) << 8);
		uint16_t mst = frame->id;

		for (int i = 0; i < scan_nhits; i++) {
			if (scan_hits[i].esc == esc && scan_hits[i].mst == mst) {
				scan_hits[i].count++;
				return;
			}
		}
		if (scan_nhits < SCAN_MAX) {
			scan_hits[scan_nhits].esc = esc;
			scan_hits[scan_nhits].mst = mst;
			scan_hits[scan_nhits].count = 1;
			scan_nhits++;
		} else {
			scan_overflow = true;
		}
		return;
	}
	scan_other_frames++;
	scan_other_id = frame->id;
}


static int cmd_scan(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t first = argc > 1 ? argu(argv[1]) : 0x000;
	uint32_t last = argc > 2 ? argu(argv[2]) : 0x7FE;
	uint32_t gap_us = argc > 3 ? argu(argv[3]) : 1000;
	struct can_filter all = {.id = 0, .mask = 0, .flags = 0};
	struct can_bus_err_cnt e0, e1;
	enum can_state s0, s1;
	int filter, tx_fail = 0, consecutive = 0;
	bool aborted = false;
	bool fd = have_motor() ? cur->m.fd : false;

	if (last > 0x7FE || first > last) {
		shell_error(sh, "usage: dm scan [first last] [gap_us]   (ids 0x000..0x7FE)");
		return -EINVAL;
	}
	if (any_streaming()) {
		shell_error(sh, "scan: a motor is still streaming; 'dm stop' and 'dm stream 0' it "
			    "(see 'dm motors')");
		return -EBUSY;
	}
	/*
	 * The MCAN acts on the first filter element that matches, so a reply on
	 * an initialised motor's MST_ID would go to its filter and never reach
	 * the catch-all.  Drop every slot's filters for the scan, reinstall after.
	 */
	for (int i = 0; i < DM_SLOTS; i++) {
		if (slots[i].ready) {
			dm_motor_release(&slots[i].m);
		}
	}
	scan_nhits = 0;
	scan_overflow = false;
	scan_other_frames = 0;
	filter = can_add_rx_filter(can_dev, scan_rx, NULL, &all);
	if (filter < 0) {
		shell_error(sh, "scan: catch-all filter failed: %d", filter);
	}
	if (filter >= 0) {
		can_get_state(can_dev, &s0, &e0);
		for (uint32_t id = first; id <= last; id++) {
			struct can_frame f = {
				.id = DM_ID_REG,
				.dlc = 4,
				.flags = fd ? (CAN_FRAME_FDF | CAN_FRAME_BRS) : 0,
				.data = {id & 0xFF, (id >> 8) & 0x07, 0x33, DM_RID_ESC_ID},
			};
			/*
			 * A blocking can_send() never returns while nothing on the
			 * bus acknowledges (the timeout only covers getting a
			 * mailbox), which would wedge the shell thread.  Use the
			 * completion callback and give up after a while instead.
			 */
			k_sem_reset(&scan_tx_sem);
			int ret = can_send(can_dev, &f, K_MSEC(20), scan_tx_done, NULL);

			if (ret == 0) {
				ret = k_sem_take(&scan_tx_sem, K_MSEC(100)) ? -ETIMEDOUT : scan_tx_err;
			}
			if (ret) {
				tx_fail++;
				if (ret == -ETIMEDOUT || ++consecutive >= 8) {
					shell_error(sh, "scan: aborted at esc 0x%03x, bus not "
						    "acknowledging (%d): no powered node, wrong "
						    "bitrate, or wiring", id, ret);
					/* Flush the frame still retrying in the TX FIFO. */
					(void)dm_bus_configure(can_dev, false, fd);
					aborted = true;
					break;
				}
			} else {
				consecutive = 0;
			}
			k_usleep(gap_us);
		}
		k_sleep(K_MSEC(50));
		can_remove_rx_filter(can_dev, filter);
		can_get_state(can_dev, &s1, &e1);
	}
	/* Reinstall every slot's filters, keeping its settings. */
	for (int i = 0; i < DM_SLOTS; i++) {
		struct dm_slot *s = &slots[i];

		if (!s->ready) {
			continue;
		}
		uint16_t esc = s->m.esc_id, mst = s->m.mst_id;
		float pmax = s->m.p_max, vmax = s->m.v_max, tmax = s->m.t_max;
		bool sfd = s->m.fd;

		if (dm_motor_init(&s->m, can_dev, esc, mst, pmax, vmax, tmax) != 0) {
			shell_error(sh, "scan: could not reinstall esc 0x%03x, slot dropped", esc);
			s->ready = false;
			if (cur == s) {
				cur = NULL;
			}
		}
		s->m.fd = sfd;
	}
	if (filter < 0) {
		return filter;
	}

	for (int i = 0; i < scan_nhits; i++) {
		shell_print(sh, "scan: found esc 0x%03x mst 0x%03x (%u replies)", scan_hits[i].esc,
			    scan_hits[i].mst, scan_hits[i].count);
	}
	if (scan_overflow) {
		shell_print(sh, "scan: more than %d responders, list truncated", SCAN_MAX);
	}
	if (scan_other_frames) {
		shell_print(sh, "scan: %u other frames seen (last id 0x%03x)", scan_other_frames,
			    scan_other_id);
	}
	if (tx_fail) {
		shell_print(sh, "scan: %d probe frames failed to send", tx_fail);
	}
	/* Counters falling or the state recovering (after an earlier empty-bus
	 * scan) is good news; only a rise hints at two nodes talking at once.
	 */
	if (!aborted && (e1.tx_err_cnt > e0.tx_err_cnt || e1.rx_err_cnt > e0.rx_err_cnt ||
			 s1 > s0)) {
		shell_print(sh, "scan: bus errors rose (tx %u->%u, rx %u->%u, state %d->%d); "
			    "duplicate IDs on the bus can cause this", e0.tx_err_cnt, e1.tx_err_cnt,
			    e0.rx_err_cnt, e1.rx_err_cnt, s0, s1);
	}
	shell_print(sh, "scan: %d motor(s) answered in esc 0x%03x..0x%03x", scan_nhits, first, last);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	dm_cmds,
	SHELL_CMD_ARG(init, NULL, "<esc_id> [mst_id = esc+0x10] [pmax vmax tmax]: select a motor (set it up the first time)", cmd_init, 1, 5),
	SHELL_CMD(motors, NULL, "list the motors in use and what each is doing", cmd_motors),
	SHELL_CMD_ARG(lockout, NULL, "[on|off]: allow only one motor to be energised at a time (default on)", cmd_lockout, 1, 1),
	SHELL_CMD_ARG(release, NULL, "[esc_id]: stop, disable and forget a motor", cmd_release, 1, 1),
	SHELL_CMD_ARG(mode, NULL, "[mit|pos|vel|psi]: frame-ID mode used by enable/disable/zero", cmd_mode, 1, 1),
	SHELL_CMD(enable, NULL, "enable motor (FF..FC)", cmd_enable),
	SHELL_CMD(disable, NULL, "disable motor (FF..FD)", cmd_disable),
	SHELL_CMD(zero, NULL, "save current position as zero (FF..FE)", cmd_zero),
	SHELL_CMD(clearerr, NULL, "clear error (FF..FB)", cmd_clearerr),
	SHELL_CMD_ARG(mit, NULL, "<pos> <vel> <kp> <kd> <tor>: MIT frame", cmd_mit, 6, 0),
	SHELL_CMD_ARG(pos, NULL, "<pos> <vel>: position-velocity frame", cmd_pos, 3, 0),
	SHELL_CMD_ARG(vel, NULL, "<rad/s>: velocity setpoint (one frame, or continuous with stream)", cmd_vel, 2, 0),
	SHELL_CMD(stop, NULL, "setpoint 0 (safevel: idle at 0 N.m until the next 'dm vel')", cmd_stop),
	SHELL_CMD_ARG(torquemode, NULL, "[save] [hz=1000]: MIT mode with Kp=Kd=0, stream the torque term", cmd_torquemode, 1, 2),
	SHELL_CMD_ARG(torque, NULL, "<N.m>: torque (current) setpoint at the output shaft", cmd_torque, 2, 0),
	SHELL_CMD_ARG(current, NULL, "<A>: current setpoint, converted to torque with Kt", cmd_current, 2, 0),
	SHELL_CMD_ARG(kt, NULL, "[N.m/A]: show/set the Kt used by 'dm current'", cmd_kt, 1, 1),
	SHELL_CMD_ARG(torqueramp, NULL, "[N.m/s]: show/set the torque setpoint ramp (0 = step)", cmd_torqueramp, 1, 1),
	SHELL_CMD_ARG(speedcap, NULL, "[rad/s]: runaway guard for torque mode (0 = off)", cmd_speedcap, 1, 1),
	SHELL_CMD_ARG(stream, NULL, "[hz [ramp rad/s^2]]: stream the velocity setpoint (0 = off)", cmd_stream, 1, 2),
	SHELL_CMD_ARG(velmode, NULL, "[save] [hz=1000]: disable, CTRL_MODE=3, enable on 0x200+ID, stream", cmd_velmode, 1, 2),
	SHELL_CMD_ARG(safevelmode, NULL, "[save] [hz=1000]: velocity control with a saturating torque cap", cmd_safevelmode, 1, 2),
	SHELL_CMD_ARG(torquecap, NULL, "[N.m]: show/set the safevel torque cap (saturates, no trip)", cmd_torquecap, 1, 1),
	SHELL_CMD_ARG(velgains, NULL, "[kp] [ki]: show/set the safevel velocity-loop gains", cmd_velgains, 1, 2),
	SHELL_CMD_ARG(velff, NULL, "[kinetic N.m] [breakaway N.m] [imax N.m]: safevel friction feed-forward, kick, integrator clamp", cmd_velff, 1, 3),
	SHELL_CMD_ARG(velfilt, NULL, "[tau ms] [stick ms] [taper rad/s] [stick rad/s]: safevel velocity filter, kick delay, ff taper, stuck threshold", cmd_velfilt, 1, 4),
	SHELL_CMD_ARG(velsrc, NULL, "[pos|fb] [window ms]: velocity the safevel loop uses (default: rotor position over 10 ms)", cmd_velsrc, 1, 2),
	SHELL_CMD_ARG(trace, NULL, "<n> | dump: capture n loop ticks of the selected motor to RAM, or print them as CSV", cmd_trace, 1, 1),
	SHELL_CMD_ARG(veldeadband, NULL, "[rad/s]: velocity error the safevel loop ignores (quantisation)", cmd_veldeadband, 1, 1),
	SHELL_CMD_ARG(watchdog, NULL, "[ms]: disable the motor if no 'dm hb' arrives within ms (0 = off)", cmd_watchdog, 1, 1),
	SHELL_CMD(hb, NULL, "host heartbeat for the watchdog", cmd_hb),
	SHELL_CMD(status, NULL, "motor, streamer and feedback status", cmd_status),
	SHELL_CMD_ARG(psi, NULL, "<pos> <vel> <cur>: force-position frame", cmd_psi, 4, 0),
	SHELL_CMD_ARG(cur3519, NULL, "<m1> [m2] [m3] [m4]: DM3519 one-to-four current frame (A)", cmd_cur3519, 2, 3),
	SHELL_CMD(fb, NULL, "show last feedback", cmd_fb),
	SHELL_CMD(request, NULL, "request a feedback frame (0x7FF/0xCC) and show it", cmd_request),
	SHELL_CMD_ARG(read, NULL, "<rid>: read register", cmd_read, 2, 0),
	SHELL_CMD_ARG(write, NULL, "<rid> <value>: write register (RAM)", cmd_write, 3, 0),
	SHELL_CMD(save, NULL, "save registers to flash (0x7FF/0xAA)", cmd_save),
	SHELL_CMD(info, NULL, "read the common registers", cmd_info),
	SHELL_CMD_ARG(scan, NULL, "[first last] [gap_us]: probe every ESC_ID, list motors (esc, mst)", cmd_scan, 1, 3),
	SHELL_CMD_ARG(bus, NULL, "<normal|loopback> [fd]: set CAN controller mode (fd = send FD+BRS frames)", cmd_bus, 2, 1),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(dm, &dm_cmds, "DaMiao motor commands", NULL);
