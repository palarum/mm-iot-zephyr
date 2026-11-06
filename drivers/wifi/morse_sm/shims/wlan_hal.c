/*
 * Copyright 2024 Morse Micro
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log_ctrl.h>
#include <zephyr/sys/reboot.h>

#include "mmhal.h"
#include "mmosal.h"
#include "mmdrv.h"   /* morselib internal: mmdrv_host_hw_restart_required(), TX pause */

#include "morse.h"

#include "morse_log.h"
LOG_MODULE_DECLARE(LOG_MODULE_NAME);

/** 10x8bit training seq */
#define BYTE_TRAIN 16

static mmhal_irq_handler_t spi_irq_handler = NULL;
static mmhal_irq_handler_t busy_irq_handler = NULL;

static const uint8_t spi_ones[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
				   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

void mmhal_wlan_hard_reset(void)
{
	const struct morse_config *cfg = morse_config0;
	const struct gpio_dt_spec *gpio_dt = &cfg->resetn;
	int ret = 0;

	if ((ret = gpio_pin_set_dt(gpio_dt, 1)) < 0) {
		LOG_ERR("Unhandled exception %d in %s\n", ret, __func__);
	}
	mmosal_task_sleep(5);
	if ((ret = gpio_pin_set_dt(gpio_dt, 0)) < 0) {
		LOG_ERR("Unhandled exception %d in %s\n", ret, __func__);
	}
	mmosal_task_sleep(20);
}

/* CME 2026-10-08: recover the mm8108 after a run of SPI failures.
 * CME 2026-11-05: was a bare reset pulse, which left the chip with no firmware. Now:
 *  1. After MORSE_SPI_ERR_RESET_THRESHOLD errors within MORSE_SPI_ERR_WINDOW_MS, ask morselib for its full
 *     hardware restart: TX paused, then mmdrv_deinit + mmdrv_init on the event-loop task
 *     (reset pulse, firmware reload, re-init, reconnect). This is what morselib's own health
 *     check does on failure, but that check runs at low priority and starves while the
 *     drv task loops on SPI errors, so it never got to run. Both calls only queue work, so
 *     they are safe here in the drv task holding the bus lock.
 *  2. If errors come back to the threshold again after the restart has had
 *     MORSE_RESTART_GRACE_MS to run, but within MORSE_RESTART_WINDOW_MS of it, the restart
 *     did not stick (mmdrv_init failure is silent: morselib is built with MMOSAL_NOASSERT),
 *     so reboot the badge. A reboot always brings HaLow back.
 */
#define MORSE_SPI_ERR_RESET_THRESHOLD 10
/* CME 2026-11-05: errors are counted within this window, not only back to back. In a real
 * storm the failing multi-byte DMA writes alternate with single-byte transfers that still
 * succeed (they bypass DMA), and those successes kept resetting a "consecutive" count, so
 * recovery never triggered. An occasional isolated error never reaches the threshold here. */
#define MORSE_SPI_ERR_WINDOW_MS       5000
#define MORSE_RESTART_GRACE_MS        15000   /* the restart itself: deinit, reset, firmware, init */
#define MORSE_RESTART_WINDOW_MS       120000  /* another storm this soon after a restart: reboot */
static uint32_t morse_spi_err_count;
static int64_t morse_spi_err_window_start_ms;
static int64_t morse_restart_requested_at_ms;  /* 0 = no restart requested yet */

static void morse_spi_recover(void)
{
	int64_t now_ms = k_uptime_get();
	int64_t since_restart_ms = now_ms - morse_restart_requested_at_ms;

	/* CME 2026-11-05: only once the chip has booted at least once (morse.c sets the status
	 * past INTERFACE_DISABLED after mmwlan_boot succeeds). Errors during the first boot run
	 * inside morse_iface_init at SYS_INIT, before morselib's event loop exists: a restart
	 * request can't run there, and the reboot that followed turned a HaLow fault into an
	 * endless boot loop. Let that boot fail instead; the connection manager then reports
	 * "HaLow failed" and the badge runs without HaLow. */
	if (morse_data0.status == WIFI_STATE_INTERFACE_DISABLED) {
		return;
	}

	if (morse_restart_requested_at_ms != 0 && since_restart_ms < MORSE_RESTART_GRACE_MS) {
		return;   /* the restart is still running: its own errors don't count */
	}
	if (morse_restart_requested_at_ms != 0 && since_restart_ms < MORSE_RESTART_WINDOW_MS) {
		LOG_ERR("mm8108 still failing %lld ms after a full restart - rebooting the badge\n",
			since_restart_ms);
		LOG_PANIC();
		sys_reboot(SYS_REBOOT_COLD);
	}

	LOG_WRN("mm8108 SPI failed %u times in %d ms - requesting a full HaLow restart"
		" (reset + firmware reload)\n", MORSE_SPI_ERR_RESET_THRESHOLD + 1, MORSE_SPI_ERR_WINDOW_MS);
	morse_restart_requested_at_ms = now_ms;
	mmdrv_host_set_tx_paused(MMDRV_PAUSE_SOURCE_MASK_HW_RESTART, true);
	mmdrv_host_hw_restart_required();
}

static void morse_spi_track_result(int ret, const char *op)
{
	if (ret < 0) {
		LOG_ERR("Unhandled error %d in %s\n", ret, op);
		int64_t now_ms = k_uptime_get();
		if (now_ms - morse_spi_err_window_start_ms > MORSE_SPI_ERR_WINDOW_MS) {
			morse_spi_err_window_start_ms = now_ms;   /* start a new counting window */
			morse_spi_err_count = 0;
		}
		if (++morse_spi_err_count > MORSE_SPI_ERR_RESET_THRESHOLD) {
			morse_spi_err_count = 0;
			morse_spi_recover();
		}
	}
}

#if defined(CONFIG_WIFI_MORSE_EXT_XTAL_INIT) && CONFIG_WIFI_MORSE_EXT_XTAL_INIT
bool mmhal_wlan_ext_xtal_init_is_required(void)
{
	return true;
}
#endif

void mmhal_wlan_spi_cs_assert(void)
{
}

void mmhal_wlan_spi_cs_deassert(void)
{
}

uint8_t mmhal_wlan_spi_rw(uint8_t data)
{
	const struct morse_config *cfg = morse_config0;
	const struct device *spi = cfg->spi.bus;
	const struct spi_config *spi_cfg = &cfg->spi.config;
	int ret = 0;
	uint8_t read_val = 0;

	struct spi_buf tx_bufs[] = {{.buf = &data, .len = 1}};

	const struct spi_buf_set tx = {
		.buffers = tx_bufs,
		.count = 1,
	};

	struct spi_buf rx_bufs[] = {{.buf = &read_val, .len = 1}};

	const struct spi_buf_set rx = {
		.buffers = rx_bufs,
		.count = 1,
	};

	ret = spi_transceive(spi, spi_cfg, &tx, &rx);
	morse_spi_track_result(ret, "spi_transceive");

	return read_val;
}

void mmhal_wlan_spi_read_buf(uint8_t *buf, unsigned len)
{
	const struct morse_config *cfg = morse_config0;
	const struct device *spi = cfg->spi.bus;
	const struct spi_config *spi_cfg = &cfg->spi.config;
	int ret = 0;

	struct spi_buf rx_bufs[] = {{.buf = buf, .len = len}};

	const struct spi_buf_set rx = {
		.buffers = rx_bufs,
		.count = 1,
	};

	ret = spi_read(spi, spi_cfg, &rx);
	morse_spi_track_result(ret, "spi_read()");
}

void mmhal_wlan_spi_write_buf(const uint8_t *buf, unsigned len)
{
	const struct morse_config *cfg = morse_config0;
	const struct device *spi = cfg->spi.bus;
	const struct spi_config *spi_cfg = &cfg->spi.config;
	int ret = 0;

	struct spi_buf tx_bufs[] = {{.buf = (void *)buf, .len = len}};

	const struct spi_buf_set tx = {
		.buffers = tx_bufs,
		.count = 1,
	};

	ret = spi_write(spi, spi_cfg, &tx);
	morse_spi_track_result(ret, "spi_write()");
}

void mmhal_wlan_send_training_seq(void)
{
	const struct morse_config *cfg = morse_config0;
	const struct device *spi = cfg->spi.bus;
	const struct gpio_dt_spec *cs_gpio = &cfg->spi.config.cs.gpio;
	struct spi_config spi_cfg = cfg->spi.config;
	gpio_flags_t flags = GPIO_OUTPUT_INACTIVE;
	int ret = 0;

	struct spi_buf tx_bufs = {.buf = (uint8_t *)spi_ones, .len = sizeof(spi_ones)};

	const struct spi_buf_set tx = {
		.buffers = &tx_bufs,
		.count = 1,
	};

	ret = gpio_pin_get_config_dt(cs_gpio, &flags);
	if (ret == -ENOSYS) {
		LOG_DBG("Platform does not implement gpio_pin_get_config(), using default flags\n");
	} else if (ret < 0) {
		LOG_ERR("Unhandled error %d in gpio_pin_get_config_dt()\n", ret);
		return;
	}

	ret = gpio_pin_configure(cs_gpio->port, cs_gpio->pin, flags & ~(GPIO_ACTIVE_LOW));
	if (ret != 0) {
		LOG_ERR("Unhandled error %d in gpio_pin_configure()\n", ret);
		return;
	}

	ret = spi_transceive(spi, &spi_cfg, &tx, NULL);
	if (ret != 0) {
		LOG_ERR("Unhandled error %d in spi_transceive()\n", ret);
		return;
	}
	/* Release lock on SPI bus */
	ret = spi_release(spi, &spi_cfg);

	ret = gpio_pin_configure(cs_gpio->port, cs_gpio->pin, flags | GPIO_ACTIVE_LOW);
	if (ret != 0) {
		LOG_ERR("Unhandled error %d in gpio_pin_configure()\n", ret);
		return;
	}
}

void mmhal_wlan_register_spi_irq_handler(mmhal_irq_handler_t handler)
{
	spi_irq_handler = handler;
}

bool mmhal_wlan_spi_irq_is_asserted(void)
{
	const struct morse_config *cfg = morse_config0;
	const struct gpio_dt_spec *gpio_dt = &cfg->spi_irq;
	int ret = 0;
	if ((ret = gpio_pin_get_dt(gpio_dt)) < 0) {
		LOG_ERR("Unhandled exception %d in %s\n", ret, __func__);
		return false;
	}
	return !!ret;
}

void mmhal_wlan_set_spi_irq_enabled(bool enabled)
{
	const struct morse_config *cfg = morse_config0;

	if (enabled) {
		/* The transiver will hold the IRQ line low if there is additional information
		 * to be retrived. Ideally the interrupt pin would be configured as a low level
		 * interrupt.
		 */
		if (mmhal_wlan_spi_irq_is_asserted()) {
			if (spi_irq_handler != NULL) {
				spi_irq_handler();
			}
		}
		/* CME 2026-10-13: GPIO_INT_WAKEUP registers the PINT line as an RW612
		 * PM2 wake source; without it the edge can't wake the host from PM2
		 * and the mm8108 handshake desyncs (TX failures, drv task livelock).
		 */
		gpio_pin_interrupt_configure_dt(&cfg->spi_irq,
						GPIO_INT_EDGE_TO_ACTIVE | GPIO_INT_WAKEUP);
	} else {
		gpio_pin_interrupt_configure_dt(&cfg->spi_irq, GPIO_INT_DISABLE);
	}
}

void mmhal_wlan_init(void)
{
	const struct morse_config *cfg = morse_config0;
	const struct gpio_dt_spec *gpio_dt = &cfg->resetn;
	int ret = 0;
	if ((ret = gpio_pin_set_dt(gpio_dt, 1)) < 0) {
		LOG_ERR("Unhandled exception %d in %s\n", ret, __func__);
	}
}

void mmhal_wlan_deinit(void)
{
	const struct morse_config *cfg = morse_config0;
	const struct gpio_dt_spec *gpio_dt = &cfg->resetn;
	int ret = 0;
	if ((ret = gpio_pin_set_dt(gpio_dt, 0)) < 0) {
		LOG_ERR("Unhandled exception %d in %s\n", ret, __func__);
	}
}

void mmhal_wlan_wake_assert(void)
{
	const struct morse_config *cfg = morse_config0;
	const struct gpio_dt_spec *gpio_dt = &cfg->wakeup;
	int ret = 0;
	if ((ret = gpio_pin_set_dt(gpio_dt, 1)) < 0) {
		LOG_ERR("Unhandled exception %d in %s\n", ret, __func__);
	}
}

void mmhal_wlan_wake_deassert(void)
{
	const struct morse_config *cfg = morse_config0;
	const struct gpio_dt_spec *gpio_dt = &cfg->wakeup;
	int ret = 0;
	if ((ret = gpio_pin_set_dt(gpio_dt, 0)) < 0) {
		LOG_ERR("Unhandled exception %d in %s\n", ret, __func__);
	}
}

bool mmhal_wlan_busy_is_asserted(void)
{
	const struct morse_config *cfg = morse_config0;
	const struct gpio_dt_spec *gpio_dt = &cfg->busy;
	int ret = 0;
	if ((ret = gpio_pin_get_dt(gpio_dt)) < 0) {
		LOG_ERR("Unhandled exception %d in %s\n", ret, __func__);
		return false;
	}
	return !!ret;
}

void mmhal_wlan_register_busy_irq_handler(mmhal_irq_handler_t handler)
{
	busy_irq_handler = handler;
}

void mmhal_wlan_set_busy_irq_enabled(bool enabled)
{
	const struct morse_config *cfg = morse_config0;

	if (enabled) {
		/* CME 2026-10-13: PM2 wake source, see mmhal_wlan_set_spi_irq_enabled */
		gpio_pin_interrupt_configure_dt(&cfg->busy,
						GPIO_INT_EDGE_TO_ACTIVE | GPIO_INT_WAKEUP);
	} else {
		gpio_pin_interrupt_configure_dt(&cfg->busy, GPIO_INT_DISABLE);
	}
}

/**
 * @brief This function handles BUSY interrupt.
 */
void morse_busy_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	if (busy_irq_handler != NULL) {
		busy_irq_handler();
	}
}

/**
 * @brief This function handles SPI IRQ interrupts.
 */
void morse_spi_irq_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	if (spi_irq_handler != NULL) {
		spi_irq_handler();
	}
}
