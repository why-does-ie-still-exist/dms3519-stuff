/*
 * DaMiao motor CAN protocol implementation on the Zephyr CAN API.
 * SPDX-License-Identifier: Apache-2.0
 */
#include "dm_motor.h"

#include <string.h>

#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

LOG_MODULE_REGISTER(dm_motor, CONFIG_DM_MOTOR_LOG_LEVEL);

/* ------------------------------------------------------------------------
 * Zephyr CAN glue
 * ---------------------------------------------------------------------- */

static int send_frame(const struct device *can, bool fd, uint16_t id, const uint8_t *data, uint8_t len)
{
	struct can_frame frame = {
		.id = id,
		.dlc = len,
		.flags = fd ? (CAN_FRAME_FDF | CAN_FRAME_BRS) : 0,
	};

	memcpy(frame.data, data, len);
	int ret = can_send(can, &frame, K_MSEC(20), NULL, NULL);

	if (ret != 0) {
		LOG_WRN("send id 0x%03x failed: %d", id, ret);
	}
	return ret;
}

#define send_std(m, id, data, len) send_frame((m)->can, (m)->fd, (id), (data), (len))

static int send_cmd(const struct dm_motor *m, uint16_t mode_id, uint8_t last)
{
	uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, last};

	return send_std(m, m->esc_id + mode_id, data, 8);
}

int dm_motor_enable(const struct dm_motor *m, uint16_t mode_id)
{
	return send_cmd(m, mode_id, 0xFC);
}

int dm_motor_disable(const struct dm_motor *m, uint16_t mode_id)
{
	return send_cmd(m, mode_id, 0xFD);
}

int dm_motor_set_zero(const struct dm_motor *m, uint16_t mode_id)
{
	return send_cmd(m, mode_id, 0xFE);
}

int dm_motor_clear_error(const struct dm_motor *m, uint16_t mode_id)
{
	return send_cmd(m, mode_id, 0xFB);
}

int dm_motor_mit(const struct dm_motor *m, float pos, float vel, float kp, float kd, float tor)
{
	uint8_t data[8];

	dm_pack_mit(data, pos, vel, kp, kd, tor, m->p_max, m->v_max, m->t_max);
	return send_std(m, m->esc_id + DM_ID_MIT, data, 8);
}

int dm_motor_pos_vel(const struct dm_motor *m, float pos, float vel)
{
	uint8_t data[8];

	memcpy(&data[0], &pos, 4); /* little-endian IEEE754, as the motor expects */
	memcpy(&data[4], &vel, 4);
	return send_std(m, m->esc_id + DM_ID_POS_VEL, data, 8);
}

int dm_motor_vel(const struct dm_motor *m, float vel)
{
	uint8_t data[4];

	memcpy(data, &vel, 4);
	return send_std(m, m->esc_id + DM_ID_VEL, data, 4);
}

int dm_motor_psi(const struct dm_motor *m, float pos, float vel, float cur)
{
	uint8_t data[8];
	uint16_t v = (uint16_t)(vel * 100.0f);
	uint16_t c = (uint16_t)(cur * 10000.0f);

	memcpy(&data[0], &pos, 4);
	sys_put_le16(v, &data[4]);
	sys_put_le16(c, &data[6]);
	return send_std(m, m->esc_id + DM_ID_PSI, data, 8);
}

int dm3519_set_current(const struct device *can, uint16_t frame_id, const float cur[4])
{
	uint8_t data[8];

	dm3519_pack_current(data, cur);
	return send_frame(can, false, frame_id, data, 8);
}

/* --- registers (0x7FF) --------------------------------------------------- */

static void reg_header(const struct dm_motor *m, uint8_t *data)
{
	data[0] = m->esc_id & 0xFF;
	data[1] = (m->esc_id >> 8) & 0x07;
}

/* Send a 0x7FF request and wait for the matching reply on MST_ID. */
static int reg_transact(struct dm_motor *m, uint8_t cmd, uint8_t rid, const uint8_t *val,
			k_timeout_t timeout)
{
	uint8_t data[8];
	uint8_t len = 4;
	int ret;

	reg_header(m, data);
	data[2] = cmd;
	data[3] = rid;
	if (val != NULL) {
		memcpy(&data[4], val, 4);
		len = 8;
	}

	k_sem_reset(&m->reg_sem);
	m->last_rid = 0xFF;
	m->reg_cmd = cmd;
	m->reg_pending = true;

	ret = send_std(m, DM_ID_REG, data, len);
	if (ret != 0) {
		m->reg_pending = false;
		return ret;
	}

	ret = k_sem_take(&m->reg_sem, timeout);
	m->reg_pending = false;
	if (ret != 0) {
		return -ETIMEDOUT;
	}
	if (cmd != 0xAA && m->last_rid != rid) {
		return -EIO;
	}
	return 0;
}

int dm_motor_read_reg(struct dm_motor *m, enum dm_reg rid, uint32_t *raw, k_timeout_t timeout)
{
	int ret = reg_transact(m, 0x33, rid, NULL, timeout);

	if (ret == 0) {
		*raw = m->last_reg_raw;
	}
	return ret;
}

int dm_motor_read_reg_float(struct dm_motor *m, enum dm_reg rid, float *val, k_timeout_t timeout)
{
	uint32_t raw;
	int ret = dm_motor_read_reg(m, rid, &raw, timeout);

	if (ret == 0) {
		memcpy(val, &raw, 4);
	}
	return ret;
}

int dm_motor_write_reg(struct dm_motor *m, enum dm_reg rid, uint32_t raw, k_timeout_t timeout)
{
	uint8_t val[4];
	int ret;

	sys_put_le32(raw, val);
	ret = reg_transact(m, 0x55, rid, val, timeout);
	if (ret == 0 && m->last_reg_raw != raw) {
		/* Driver range-checked the value and kept the old one. */
		LOG_WRN("rid %u write rejected, driver kept 0x%08x", rid, m->last_reg_raw);
		return -ERANGE;
	}
	return ret;
}

int dm_motor_write_reg_float(struct dm_motor *m, enum dm_reg rid, float val, k_timeout_t timeout)
{
	uint32_t raw;

	memcpy(&raw, &val, 4);
	return dm_motor_write_reg(m, rid, raw, timeout);
}

int dm_motor_save_regs(struct dm_motor *m, k_timeout_t timeout)
{
	return reg_transact(m, 0xAA, 0x01, NULL, timeout);
}

int dm_motor_request_feedback(const struct dm_motor *m)
{
	uint8_t data[4];

	reg_header(m, data);
	data[2] = 0xCC;
	data[3] = 0x00;
	return send_std(m, DM_ID_REG, data, 4);
}

int dm_motor_sync_limits(struct dm_motor *m, k_timeout_t timeout)
{
	float p, v, t;
	int ret;

	ret = dm_motor_read_reg_float(m, DM_RID_PMAX, &p, timeout);
	if (ret) {
		return ret;
	}
	ret = dm_motor_read_reg_float(m, DM_RID_VMAX, &v, timeout);
	if (ret) {
		return ret;
	}
	ret = dm_motor_read_reg_float(m, DM_RID_TMAX, &t, timeout);
	if (ret) {
		return ret;
	}
	if (p > 0.0f && v > 0.0f && t > 0.0f) {
		m->p_max = p;
		m->v_max = v;
		m->t_max = t;
		LOG_INF("limits from driver: PMAX %.2f VMAX %.2f TMAX %.2f", (double)p, (double)v,
			(double)t);
		return 0;
	}
	return -EINVAL;
}

int dm_bus_configure(const struct device *can, bool loopback, bool fd)
{
	can_mode_t mode = (loopback ? CAN_MODE_LOOPBACK : CAN_MODE_NORMAL) | (fd ? CAN_MODE_FD : 0);
	int ret;

	(void)can_stop(can);
	ret = can_set_mode(can, mode);
	if (ret) {
		LOG_ERR("can_set_mode 0x%x: %d", mode, ret);
		return ret;
	}
	return can_start(can);
}

/* --- reception ------------------------------------------------------------ */

static void rx_feedback(const struct device *dev, struct can_frame *frame, void *user_data)
{
	struct dm_motor *m = user_data;

	ARG_UNUSED(dev);
	if (can_dlc_to_bytes(frame->dlc) < 4) {
		return;
	}

	/* Register reply: [esc_id_l, esc_id_h, 0x33|0x55|0xAA, rid, value(4)] */
	if (m->reg_pending && frame->data[0] == (m->esc_id & 0xFF) &&
	    frame->data[1] == ((m->esc_id >> 8) & 0x07) &&
	    (frame->data[2] == 0x33 || frame->data[2] == 0x55 || frame->data[2] == 0xAA)) {
		m->last_rid = frame->data[3];
		m->last_reg_raw = can_dlc_to_bytes(frame->dlc) >= 8 ? sys_get_le32(&frame->data[4]) : 0;
		k_sem_give(&m->reg_sem);
		return;
	}

	if (can_dlc_to_bytes(frame->dlc) < 8) {
		return;
	}
	dm_unpack_feedback(frame->data, m->p_max, m->v_max, m->t_max, &m->fb);
	m->fb.stamp = k_uptime_get();
}

static void rx_3519(const struct device *dev, struct can_frame *frame, void *user_data)
{
	struct dm_motor *m = user_data;

	ARG_UNUSED(dev);
	if (can_dlc_to_bytes(frame->dlc) < 8) {
		return;
	}
	dm3519_unpack_feedback(frame->data, &m->fb3519);
	m->fb3519.stamp = k_uptime_get();
}

int dm_motor_init(struct dm_motor *m, const struct device *can, uint16_t esc_id, uint16_t mst_id,
		  float p_max, float v_max, float t_max)
{
	struct can_filter filter = {
		.mask = CAN_STD_ID_MASK,
		.flags = 0,
	};
	int ret;

	memset(m, 0, sizeof(*m));
	m->can = can;
	m->esc_id = esc_id;
	m->mst_id = mst_id;
	m->p_max = p_max;
	m->v_max = v_max;
	m->t_max = t_max;
	k_sem_init(&m->reg_sem, 0, 1);

	filter.id = mst_id;
	ret = can_add_rx_filter(can, rx_feedback, m, &filter);
	if (ret < 0) {
		LOG_ERR("feedback filter: %d", ret);
		return ret;
	}
	m->rx_filter_fb = ret;

	m->rx_filter_3519 = -1;
	if (esc_id >= 1 && esc_id <= 8) {
		filter.id = 0x200 + esc_id;
		ret = can_add_rx_filter(can, rx_3519, m, &filter);
		if (ret < 0) {
			LOG_WRN("dm3519 feedback filter: %d", ret);
		} else {
			m->rx_filter_3519 = ret;
		}
	}
	LOG_INF("motor esc_id 0x%03x mst_id 0x%03x PMAX %.1f VMAX %.1f TMAX %.1f", esc_id, mst_id,
		(double)p_max, (double)v_max, (double)t_max);
	return 0;
}
