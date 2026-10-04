/*
 * CANBed M4 CAN loopback self-test.
 *
 * Puts CAN0 into internal loopback (frames are transmitted, received by the
 * same controller, and also driven onto the bus pins), sends a classic
 * frame and a 64-byte CAN FD frame with bit-rate switching, and checks that
 * both come back intact.  Results go to the USB CDC console, where the
 * Zephyr shell (`can ...` commands) stays available afterwards.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/shell/shell.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/version.h>
#include <zephyr/sys/byteorder.h>

#include "dm_motor.h"

LOG_MODULE_REGISTER(loopback, LOG_LEVEL_INF);

static const struct device *const can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

CAN_MSGQ_DEFINE(rx_msgq, 8);

#define TEST_ID_CLASSIC 0x123
#define TEST_ID_FD      0x1ABCDEF

static int add_filters(void)
{
	const struct can_filter classic = {
		.id = TEST_ID_CLASSIC,
		.mask = CAN_STD_ID_MASK,
		.flags = 0,
	};
	const struct can_filter fd = {
		.id = TEST_ID_FD,
		.mask = CAN_EXT_ID_MASK,
		.flags = CAN_FILTER_IDE,
	};
	int ret;

	ret = can_add_rx_filter_msgq(can_dev, &rx_msgq, &classic);
	if (ret < 0) {
		LOG_ERR("standard filter: %d", ret);
		return ret;
	}
	ret = can_add_rx_filter_msgq(can_dev, &rx_msgq, &fd);
	if (ret < 0) {
		LOG_ERR("extended filter: %d", ret);
		return ret;
	}
	return 0;
}

static void tx_done(const struct device *dev, int error, void *user_data)
{
	if (error != 0) {
		LOG_ERR("tx callback error %d", error);
	}
}

/* Send one frame and wait for it to come back through the loopback. */
static int roundtrip(const struct can_frame *tx, const char *name)
{
	struct can_frame rx;
	int ret;

	k_msgq_purge(&rx_msgq);

	ret = can_send(can_dev, tx, K_MSEC(200), tx_done, NULL);
	if (ret != 0) {
		LOG_ERR("%s: can_send failed: %d", name, ret);
		return ret;
	}

	ret = k_msgq_get(&rx_msgq, &rx, K_MSEC(200));
	if (ret != 0) {
		LOG_ERR("%s: nothing received (%d)", name, ret);
		return -ETIMEDOUT;
	}

	size_t len = can_dlc_to_bytes(tx->dlc);

	if (rx.id != tx->id || rx.dlc != tx->dlc ||
	    (rx.flags & (CAN_FRAME_IDE | CAN_FRAME_FDF | CAN_FRAME_BRS)) !=
		    (tx->flags & (CAN_FRAME_IDE | CAN_FRAME_FDF | CAN_FRAME_BRS)) ||
	    memcmp(rx.data, tx->data, len) != 0) {
		LOG_ERR("%s: mismatch: id 0x%x/0x%x dlc %u/%u flags 0x%x/0x%x", name,
			rx.id, tx->id, rx.dlc, tx->dlc, rx.flags, tx->flags);
		LOG_HEXDUMP_ERR(rx.data, len, "rx");
		return -EIO;
	}

	LOG_INF("%s: OK (id 0x%x, %u bytes%s)", name, rx.id, (unsigned int)len,
		(rx.flags & CAN_FRAME_BRS) ? ", FD+BRS" : "");
	return 0;
}

static int run_selftest(void)
{
	struct can_frame frame;
	int failures = 0;

	/* Classic CAN 2.0A, 8 bytes */
	memset(&frame, 0, sizeof(frame));
	frame.id = TEST_ID_CLASSIC;
	frame.dlc = 8;
	for (int i = 0; i < 8; i++) {
		frame.data[i] = 0xA0 + i;
	}
	failures += (roundtrip(&frame, "classic 2.0A") != 0);

	/* CAN FD, extended ID, 64 bytes, bit-rate switch to the data bitrate */
	memset(&frame, 0, sizeof(frame));
	frame.id = TEST_ID_FD;
	frame.flags = CAN_FRAME_IDE | CAN_FRAME_FDF | CAN_FRAME_BRS;
	frame.dlc = can_bytes_to_dlc(64);
	for (int i = 0; i < 64; i++) {
		frame.data[i] = (uint8_t)(i * 3);
	}
	failures += (roundtrip(&frame, "CAN FD 64B") != 0);

	return failures;
}

static int cmd_loopback(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	int failures = run_selftest();

	shell_print(sh, "CAN loopback self-test: %s", failures ? "FAILED" : "PASSED");
	return failures ? -EIO : 0;
}
SHELL_CMD_REGISTER(loopback, NULL, "Run the CAN loopback self-test again", cmd_loopback);

/* Give the host a moment to open the CDC ACM console so the boot-time test
 * output is not lost; continue regardless after the timeout.
 */
static void wait_for_console(void)
{
	const struct device *console = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	uint32_t dtr = 0;

	for (int i = 0; i < 150 && dtr == 0; i++) {
		if (uart_line_ctrl_get(console, UART_LINE_CTRL_DTR, &dtr) != 0) {
			break;
		}
		k_sleep(K_MSEC(100));
	}
	k_sleep(K_MSEC(200));
}

int main(void)
{
	struct can_bus_err_cnt err;
	enum can_state state;
	can_mode_t caps;
	int ret;

	if (gpio_is_ready_dt(&led)) {
		gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
	}

	wait_for_console();
	printk("\nCANBed M4 (ATSAME51G19A) Zephyr %s, board %s\n", KERNEL_VERSION_STRING, CONFIG_BOARD);

	if (!device_is_ready(can_dev)) {
		LOG_ERR("CAN device %s not ready", can_dev->name);
		return 0;
	}

	ret = can_get_capabilities(can_dev, &caps);
	LOG_INF("%s capabilities: 0x%x (loopback %s, FD %s)", can_dev->name, caps,
		(caps & CAN_MODE_LOOPBACK) ? "yes" : "no", (caps & CAN_MODE_FD) ? "yes" : "no");

	ret = add_filters();
	if (ret < 0) {
		return 0;
	}

	ret = can_set_mode(can_dev, CAN_MODE_LOOPBACK | CAN_MODE_FD);
	if (ret != 0) {
		LOG_ERR("can_set_mode: %d", ret);
		return 0;
	}

	ret = can_start(can_dev);
	if (ret != 0) {
		LOG_ERR("can_start: %d", ret);
		return 0;
	}

	uint32_t rate = 0, rate_data = 0;

	can_get_core_clock(can_dev, &rate);
	LOG_INF("CAN core clock %u Hz, mode loopback+FD, %u kbit/s nominal", rate, 1000U);
	ARG_UNUSED(rate_data);

	int failures = run_selftest();

	can_get_state(can_dev, &state, &err);
	LOG_INF("bus state %d, tx errors %u, rx errors %u", state, err.tx_err_cnt, err.rx_err_cnt);

	if (failures == 0) {
		printk("\n*** CAN LOOPBACK SELF-TEST PASSED ***\n\n");
	} else {
		printk("\n*** CAN LOOPBACK SELF-TEST FAILED (%d) ***\n\n", failures);
	}

	/* Leave loopback: normal CAN 2.0 mode for the motor bus (dm bus ... changes it). */
	ret = dm_bus_configure(can_dev, false, false);
	if (ret) {
		LOG_ERR("switching to normal mode failed: %d", ret);
	} else {
		LOG_INF("CAN0 in normal mode, 1 Mbit/s; use 'dm' commands for the motor");
	}

	/* Heartbeat LED: fast blink on pass, slow on fail. */
	while (1) {
		gpio_pin_toggle_dt(&led);
		k_sleep(K_MSEC(failures ? 1000 : 200));
	}
	return 0;
}
