/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * GigaDevice GD32F10x USB device controller driver.
 *
 * The USB peripheral of the GD32F10x uses a 512 byte dedicated packet RAM.
 * A buffer descriptor table (BDT) of 16 bit entries at the start of the RAM
 * holds, per endpoint and direction, the buffer address and the byte count.
 * All addresses in the BDT are in 16 bit words: the CPU byte address of a
 * packet buffer is (bdt value * 2) + RAM base. Packet payload is copied by
 * 16 bit accesses, one USB word per location, matching the BDT addressing.
 *
 * The controller is used in single buffer mode; only the USBD_LP interrupt
 * line (IRQ 20) is serviced. An internal thread forwards endpoint transfer
 * completions and setup packets to the USB device stack in thread context.
 */

#define DT_DRV_COMPAT gd_gd32_usbd

#include "udc_common.h"

#include <string.h>

#include <zephyr/arch/common/ffs.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/usb/udc.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(udc_gd32, CONFIG_UDC_DRIVER_LOG_LEVEL);

/* USBD register offsets */
#define UDC_GD32_CTL_OFF        0x40U
#define UDC_GD32_INTF_OFF       0x44U
#define UDC_GD32_STAT_OFF       0x48U
#define UDC_GD32_DADDR_OFF      0x4CU
#define UDC_GD32_BADDR_OFF      0x50U
#define UDC_GD32_LPMCS_OFF      0x54U

/* USBD_CTL bits */
#define UDC_GD32_CTL_STIE       BIT(15)
#define UDC_GD32_CTL_ERRIE      BIT(13)
#define UDC_GD32_CTL_WKUPIE     BIT(12)
#define UDC_GD32_CTL_SPSIE      BIT(11)
#define UDC_GD32_CTL_RSTIE      BIT(10)
#define UDC_GD32_CTL_SOFIE      BIT(9)
#define UDC_GD32_CTL_RSREQ      BIT(4)
#define UDC_GD32_CTL_SETRST     BIT(0)

#define UDC_GD32_INTEN          (UDC_GD32_CTL_STIE | UDC_GD32_CTL_ERRIE | \
				 UDC_GD32_CTL_WKUPIE | UDC_GD32_CTL_SPSIE | \
				 UDC_GD32_CTL_RSTIE)

/* USBD_INTF bits */
#define UDC_GD32_INTF_STIF      BIT(15)
#define UDC_GD32_INTF_ERRIF     BIT(13)
#define UDC_GD32_INTF_WKUPIF    BIT(12)
#define UDC_GD32_INTF_SPSIF     BIT(11)
#define UDC_GD32_INTF_RSTIF     BIT(10)
#define UDC_GD32_INTF_SOFIF     BIT(9)
#define UDC_GD32_INTF_DIR       BIT(4)
#define UDC_GD32_INTF_EPNUM     BITS(0, 3)

/* USBD_DADDR bits */
#define UDC_GD32_DADDR_USBEN    BIT(7)
#define UDC_GD32_DADDR_ADDR     BITS(0, 6)

/* USBD_EPxCS bits */
#define UDC_GD32_EPX_RX_ST      BIT(15)
#define UDC_GD32_EPX_RX_DTG     BIT(14)
#define UDC_GD32_EPX_RX_STA     BITS(12, 13)
#define UDC_GD32_EPX_SETUP      BIT(11)
#define UDC_GD32_EPX_CTL        BITS(9, 10)
#define UDC_GD32_EPX_KCTL       BIT(8)
#define UDC_GD32_EPX_TX_ST      BIT(7)
#define UDC_GD32_EPX_TX_DTG     BIT(6)
#define UDC_GD32_EPX_TX_STA     BITS(4, 5)
#define UDC_GD32_EPX_AR         BITS(0, 3)

/* EPxCS state values */
#define UDC_GD32_EPX_TX_DISABLED   0U
#define UDC_GD32_EPX_TX_STALL      (1U << 4)
#define UDC_GD32_EPX_TX_NAK        (2U << 4)
#define UDC_GD32_EPX_TX_VALID      (3U << 4)
#define UDC_GD32_EPX_RX_DISABLED   0U
#define UDC_GD32_EPX_RX_STALL      (1U << 12)
#define UDC_GD32_EPX_RX_NAK        (2U << 12)
#define UDC_GD32_EPX_RX_VALID      (3U << 12)

#define UDC_GD32_EPX_TYPE_CONTROL  (1U << 9)
#define UDC_GD32_EPX_TYPE_ISO      (2U << 9)
#define UDC_GD32_EPX_TYPE_BULK     (0U)
#define UDC_GD32_EPX_TYPE_INT      (3U << 9)

/* EPxCS mask preserving reserved, kind and connection flags */
#define UDC_GD32_EPX_CS_MASK       (UDC_GD32_EPX_RX_ST | UDC_GD32_EPX_SETUP | \
				    UDC_GD32_EPX_CTL | UDC_GD32_EPX_KCTL | \
				    UDC_GD32_EPX_TX_ST | UDC_GD32_EPX_AR)

/* received byte count field of the BDT RX count word */
#define UDC_GD32_CNT_MASK          BITS(0, 9)

/* Packet RAM buffer layout (byte offsets), 512 bytes total */
#define UDC_GD32_RAM_SIZE          512U
#define UDC_GD32_EP0_TX_ADDR       0x40U
#define UDC_GD32_EP0_RX_ADDR       0x80U
#define UDC_GD32_DATA_BUF_ADDR     0xC0U

/* 16 bit word size of the packet RAM */
#define UDC_GD32_RAM_WORD          2U

/*
 * Number of bidirectional endpoints. This matches the num-bidir-endpoints
 * property of the controller in devicetree and sizes the per-endpoint
 * packet-buffer address table (one entry per direction) and the IN state
 * table. Endpoint index 0 is reserved for the control endpoint.
 */
#define UDC_GD32_NUM_EPS           8U

enum {
	UDC_GD32_EVT_XFER_FINISHED,
	UDC_GD32_EVT_XFER_NEW,
	UDC_GD32_EVT_SETUP,
};

struct gd32_usbd_bdt {
	volatile uint16_t tx_addr;
	volatile uint16_t tx_count;
	volatile uint16_t rx_addr;
	volatile uint16_t rx_count;
};

struct udc_gd32_config {
	uintptr_t base;
	uintptr_t ram;
	uint8_t num_of_eps;
	struct udc_ep_config *ep_cfg_out;
	struct udc_ep_config *ep_cfg_in;
	const struct device *clk_dev;
	uint16_t clk_id;
	struct reset_dt_spec reset;
	struct gpio_dt_spec disconnect_gpio;
	void (*irq_enable_func)(const struct device *dev);
	void (*irq_disable_func)(const struct device *dev);
	void (*make_thread)(const struct device *dev);
};

struct gd32_in_state {
	uint16_t off;
	uint16_t len;
	uint16_t mps;
	uint8_t zlp;
};

struct udc_gd32_data {
	struct k_thread thread_data;
	struct k_event events;
	atomic_t transfer_finished;
	atomic_t transfer_new;
	uint8_t setup[8];
	uint16_t ram_cur;
	uint16_t buf_addr[2U * UDC_GD32_NUM_EPS];
	struct gd32_in_state tx[UDC_GD32_NUM_EPS];
};

static inline int gd32_ep_to_bnum(const uint8_t ep)
{
	if (USB_EP_DIR_IS_IN(ep)) {
		return UDC_GD32_NUM_EPS + USB_EP_GET_IDX(ep);
	}

	return USB_EP_GET_IDX(ep);
}

static inline uint8_t gd32_pull_ep_from_bmsk(uint32_t *const bitmap)
{
	unsigned int bit;

	__ASSERT_NO_MSG(bitmap && *bitmap);

	bit = find_lsb_set(*bitmap) - 1;
	*bitmap &= ~BIT(bit);

	if (bit >= UDC_GD32_NUM_EPS) {
		return USB_EP_DIR_IN | (bit - UDC_GD32_NUM_EPS);
	} else {
		return USB_EP_DIR_OUT | bit;
	}
}

static inline uint16_t gd32_epcs_rd(const struct udc_gd32_config *config,
				    uint8_t ep)
{
	return *(volatile uint32_t *)(config->base + (uint32_t)ep * 4U) & 0xFFFFU;
}

static inline void gd32_epcs_wr(const struct udc_gd32_config *config, uint8_t ep,
				uint16_t val)
{
	*(volatile uint32_t *)(config->base + (uint32_t)ep * 4U) = val;
}

static struct gd32_usbd_bdt *gd32_bdt(const struct udc_gd32_config *config)
{
	return (struct gd32_usbd_bdt *)(uintptr_t)config->ram;
}

/*
 * Packet RAM copy helpers. Reads and writes use 16 bit accesses only,
 * referencing one USB word per location. The current GD32 firmware library
 * iterates over packet payload with 32 bit accesses which interleaves garbage
 * between data words under this (BDT consistent) addressing scheme; both
 * access patterns are isolated here in case hardware measurements require a
 * different mapping.
 */
static void gd32_ram_copy_from(struct net_buf *buf, uintptr_t ram,
			       uint16_t ram_off, uint16_t len)
{
	const volatile uint16_t *src =
		(const volatile uint16_t *)(ram + (uintptr_t)ram_off);
	uint8_t *dst = net_buf_tail(buf);
	uint16_t i;

	for (i = 0U; i < (len + 1U) / 2U; i++) {
		uint16_t word = src[i];

		dst[2U * i] = word & 0xFFU;
		if (2U * i + 1U < len) {
			dst[2U * i + 1U] = word >> 8;
		}
	}

	net_buf_add(buf, len);
}

static void gd32_ram_copy_to(const struct net_buf *buf, uintptr_t ram,
			     uint16_t ram_off, uint16_t to_off, uint16_t len)
{
	volatile uint16_t *dst = (volatile uint16_t *)(ram + (uintptr_t)ram_off);
	const uint8_t *src = buf->data + to_off;
	uint16_t i;

	for (i = 0U; i < (len + 1U) / 2U; i++) {
		uint16_t word = src[2U * i];

		if (2U * i + 1U < len) {
			word |= (uint16_t)src[2U * i + 1U] << 8;
		}
		dst[i] = word;
	}
}

static void gd32_ram_get_setup(const struct device *dev, uint8_t *setup)
{
	const struct udc_gd32_config *config = dev->config;
	const volatile uint16_t *src =
		(const volatile uint16_t *)(config->ram +
					    UDC_GD32_EP0_RX_ADDR);
	uint16_t i;

	for (i = 0U; i < 4U; i++) {
		uint16_t word = src[i];

		setup[2U * i] = word & 0xFFU;
		setup[2U * i + 1U] = word >> 8;
	}
}

static uint16_t gd32_rx_count(uint16_t max_len)
{
	if (max_len > 62U) {
		if ((max_len & 0x1FU) != 0U) {
			return (uint16_t)(((max_len >> 5) << 10) | 0x8000U);
		}
		return (uint16_t)((((max_len >> 5) - 1U) << 10) | 0x8000U);
	}

	return (uint16_t)(((max_len + 1U) & ~1U) << 9);
}

static void gd32_epcs_tx_stat_set(const struct udc_gd32_config *config,
				  uint8_t ep, uint16_t stat)
{
	uint16_t val = (uint16_t)((gd32_epcs_rd(config, ep) &
				   (UDC_GD32_EPX_TX_STA |
				    UDC_GD32_EPX_CS_MASK)) ^
				  stat);

	gd32_epcs_wr(config, ep, val | UDC_GD32_EPX_RX_ST | UDC_GD32_EPX_TX_ST);
}

static void gd32_epcs_rx_stat_set(const struct udc_gd32_config *config,
				  uint8_t ep, uint16_t stat)
{
	uint16_t val = (uint16_t)((gd32_epcs_rd(config, ep) &
				   (UDC_GD32_EPX_RX_STA |
				    UDC_GD32_EPX_CS_MASK)) ^
				  stat);

	gd32_epcs_wr(config, ep, val | UDC_GD32_EPX_RX_ST | UDC_GD32_EPX_TX_ST);
}

static void gd32_epcs_tx_st_clear(const struct udc_gd32_config *config, uint8_t ep)
{
	uint16_t val = (uint16_t)(gd32_epcs_rd(config, ep) &
				  (~UDC_GD32_EPX_TX_ST & UDC_GD32_EPX_CS_MASK));

	gd32_epcs_wr(config, ep, val | UDC_GD32_EPX_RX_ST);
}

static void gd32_epcs_rx_st_clear(const struct udc_gd32_config *config, uint8_t ep)
{
	uint16_t val = (uint16_t)(gd32_epcs_rd(config, ep) &
				  (~UDC_GD32_EPX_RX_ST & UDC_GD32_EPX_CS_MASK));

	gd32_epcs_wr(config, ep, val | UDC_GD32_EPX_TX_ST);
}

static void gd32_ep_dtg_clear(const struct udc_gd32_config *config, uint8_t ep,
			      bool tx_dir)
{
	uint16_t dtg = tx_dir ? UDC_GD32_EPX_TX_DTG : UDC_GD32_EPX_RX_DTG;

	if ((gd32_epcs_rd(config, ep) & dtg) == 0U) {
		return;
	}

	gd32_epcs_wr(config, ep,
		     dtg | (gd32_epcs_rd(config, ep) & UDC_GD32_EPX_CS_MASK) |
		     UDC_GD32_EPX_RX_ST | UDC_GD32_EPX_TX_ST);
}

static uint16_t gd32_buf_alloc(struct udc_gd32_data *priv, uint16_t size)
{
	uint16_t addr;

	if ((priv->ram_cur & 0x1U) != 0U) {
		priv->ram_cur++;
	}

	if (priv->ram_cur + size > UDC_GD32_RAM_SIZE) {
		return 0U;
	}

	addr = priv->ram_cur;
	priv->ram_cur += size;

	return addr;
}

static void gd32_ep0_arm(const struct udc_gd32_config *config)
{
	struct gd32_usbd_bdt *bdt = gd32_bdt(config);

	bdt[0].tx_addr = UDC_GD32_EP0_TX_ADDR / UDC_GD32_RAM_WORD;
	bdt[0].tx_count = 0U;
	bdt[0].rx_addr = UDC_GD32_EP0_RX_ADDR / UDC_GD32_RAM_WORD;
	bdt[0].rx_count = gd32_rx_count(64U);

	gd32_epcs_wr(config, 0U, UDC_GD32_EPX_TYPE_CONTROL |
		     UDC_GD32_EPX_RX_VALID | UDC_GD32_EPX_TX_NAK);
}

static void gd32_ep0_rx_arm(const struct udc_gd32_config *config)
{
	struct gd32_usbd_bdt *bdt = gd32_bdt(config);

	bdt[0].rx_count = gd32_rx_count(64U);
	gd32_epcs_rx_stat_set(config, 0U, UDC_GD32_EPX_RX_VALID);
}

static void gd32_set_device_address(const struct udc_gd32_config *config,
				    uint8_t addr)
{
	*(volatile uint32_t *)(config->base + UDC_GD32_DADDR_OFF) =
		UDC_GD32_DADDR_USBEN | (addr & UDC_GD32_DADDR_ADDR);
}

static int gd32_prep_out(const struct device *dev, struct udc_ep_config *const ep_cfg,
			 struct net_buf *const buf)
{
	const struct udc_gd32_config *config = dev->config;
	struct udc_gd32_data *priv = udc_get_private(dev);
	struct gd32_usbd_bdt *bdt = gd32_bdt(config);
	uint8_t idx = USB_EP_GET_IDX(ep_cfg->addr);
	uint16_t mps = udc_mps_ep_size(ep_cfg);
	uint16_t size = MIN(mps, net_buf_tailroom(buf));
	uint16_t addr;

	if (size == 0U) {
		LOG_WRN("No buffer space for OUT endpoint 0x%02x", ep_cfg->addr);
		return -EINVAL;
	}

	if (idx == 0U) {
		addr = UDC_GD32_EP0_RX_ADDR;
	} else {
		addr = priv->buf_addr[gd32_ep_to_bnum(ep_cfg->addr)];
		if (addr == 0U) {
			LOG_ERR("No packet buffer for endpoint 0x%02x", ep_cfg->addr);
			return -ENODEV;
		}
	}

	bdt[idx].rx_addr = addr / UDC_GD32_RAM_WORD;
	bdt[idx].rx_count = gd32_rx_count(size);
	gd32_epcs_rx_stat_set(config, idx, UDC_GD32_EPX_RX_VALID);

	LOG_DBG("Prepare OUT ep 0x%02x size %u", ep_cfg->addr, size);

	return 0;
}

static int gd32_prep_in(const struct device *dev, struct udc_ep_config *const ep_cfg,
			struct net_buf *const buf)
{
	const struct udc_gd32_config *config = dev->config;
	struct udc_gd32_data *priv = udc_get_private(dev);
	struct gd32_usbd_bdt *bdt = gd32_bdt(config);
	struct gd32_in_state *state;
	uint8_t idx = USB_EP_GET_IDX(ep_cfg->addr);
	uint16_t mps = udc_mps_ep_size(ep_cfg);
	struct udc_buf_info *bi = udc_get_buf_info(buf);
	uint16_t addr;
	uint16_t chunk;

	if (idx == 0U) {
		addr = UDC_GD32_EP0_TX_ADDR;
	} else {
		addr = priv->buf_addr[gd32_ep_to_bnum(ep_cfg->addr)];
		if (addr == 0U) {
			LOG_ERR("No packet buffer for endpoint 0x%02x", ep_cfg->addr);
			return -ENODEV;
		}
	}

	state = &priv->tx[idx];
	state->off = 0U;
	state->len = buf->len;
	state->mps = mps;
	state->zlp = (bi->zlp != 0U && (buf->len % mps) == 0U) ? 1U : 0U;

	if (state->len == 0U) {
		if (state->zlp != 0U) {
			state->zlp = 0U;
			bdt[idx].tx_count = 0U;
			gd32_epcs_tx_stat_set(config, idx, UDC_GD32_EPX_TX_VALID);
		} else {
			atomic_set_bit(&priv->transfer_finished,
				       gd32_ep_to_bnum(ep_cfg->addr));
			k_event_post(&priv->events, BIT(UDC_GD32_EVT_XFER_FINISHED));
		}
		return 0;
	}

	chunk = MIN(mps, state->len);
	gd32_ram_copy_to(buf, config->ram, addr, state->off, chunk);
	bdt[idx].tx_count = chunk;
	state->off = chunk;
	gd32_epcs_tx_stat_set(config, idx, UDC_GD32_EPX_TX_VALID);

	LOG_DBG("Prepare IN ep 0x%02x length %u", ep_cfg->addr, chunk);

	return 0;
}

static void gd32_handle_xfer_next(const struct device *dev,
				  struct udc_ep_config *const ep_cfg)
{
	struct net_buf *buf;
	int err;

	if (udc_ep_is_busy(ep_cfg)) {
		return;
	}

	buf = udc_buf_peek(ep_cfg);
	if (buf == NULL) {
		return;
	}

	if (ep_cfg->addr == USB_CONTROL_EP_OUT) {
		struct udc_buf_info *bi = udc_get_buf_info(buf);

		if (bi->setup) {
			/* setup packets are captured by the armed control OUT
			 * endpoint, no further preparation is required
			 */
			gd32_ep0_rx_arm(dev->config);
			return;
		}
	}

	if (USB_EP_DIR_IS_OUT(ep_cfg->addr)) {
		err = gd32_prep_out(dev, ep_cfg, buf);
	} else {
		err = gd32_prep_in(dev, ep_cfg, buf);
	}

	if (err != 0) {
		buf = udc_buf_get(ep_cfg);
		udc_submit_ep_event(dev, buf, -ECONNREFUSED);
	} else {
		udc_ep_set_busy(ep_cfg, true);
	}
}

static void gd32_handle_out_isr(const struct device *dev, uint8_t ep)
{
	const struct udc_gd32_config *config = dev->config;
	struct udc_gd32_data *priv = udc_get_private(dev);
	struct gd32_usbd_bdt *bdt = gd32_bdt(config);
	struct udc_ep_config *ep_cfg = udc_get_ep_cfg(dev, ep);
	struct net_buf *buf;
	uint16_t count;
	uint16_t addr;

	buf = udc_buf_peek(ep_cfg);
	if (buf == NULL) {
		LOG_ERR("No buffer for ep 0x%02x", ep);
		udc_submit_event(dev, UDC_EVT_ERROR, -ENOBUFS);
		return;
	}

	count = bdt[ep].rx_count & UDC_GD32_CNT_MASK;
	addr = (ep == 0U) ? UDC_GD32_EP0_RX_ADDR :
		priv->buf_addr[gd32_ep_to_bnum(ep_cfg->addr)];

	LOG_DBG("ISR ep 0x%02x byte_count %u room %u", ep, count,
		net_buf_tailroom(buf));

	count = MIN(count, net_buf_tailroom(buf));
	gd32_ram_copy_from(buf, config->ram, addr, count);

	if (net_buf_tailroom(buf) != 0U && count == udc_mps_ep_size(ep_cfg)) {
		if (gd32_prep_out(dev, ep_cfg, buf) != 0) {
			LOG_ERR("Failed to re-arm OUT endpoint 0x%02x", ep);
		}
	} else {
		atomic_set_bit(&priv->transfer_finished,
			       gd32_ep_to_bnum(ep_cfg->addr));
		k_event_post(&priv->events, BIT(UDC_GD32_EVT_XFER_FINISHED));
	}
}

static void gd32_handle_in_isr(const struct device *dev, uint8_t ep)
{
	const struct udc_gd32_config *config = dev->config;
	struct udc_gd32_data *priv = udc_get_private(dev);
	struct gd32_usbd_bdt *bdt = gd32_bdt(config);
	struct udc_ep_config *ep_cfg = udc_get_ep_cfg(dev, USB_EP_DIR_IN | ep);
	struct gd32_in_state *state = &priv->tx[ep];
	uint16_t addr;

	if (state->off < state->len) {
		struct net_buf *buf = udc_buf_peek(ep_cfg);
		uint16_t chunk = MIN(state->mps, state->len - state->off);

		if (buf != NULL) {
			addr = (ep == 0U) ? UDC_GD32_EP0_TX_ADDR :
				priv->buf_addr[gd32_ep_to_bnum(USB_EP_DIR_IN | ep)];
			gd32_ram_copy_to(buf, config->ram, addr, state->off, chunk);
			bdt[ep].tx_count = chunk;
			state->off += chunk;
			gd32_epcs_tx_stat_set(config, ep, UDC_GD32_EPX_TX_VALID);
		}
	} else if (state->zlp != 0U) {
		state->zlp = 0U;
		bdt[ep].tx_count = 0U;
		gd32_epcs_tx_stat_set(config, ep, UDC_GD32_EPX_TX_VALID);
	} else {
		atomic_set_bit(&priv->transfer_finished,
			       gd32_ep_to_bnum(USB_EP_DIR_IN | ep));
		k_event_post(&priv->events, BIT(UDC_GD32_EVT_XFER_FINISHED));
	}
}

static void gd32_isr_handler(const struct device *dev)
{
	const struct udc_gd32_config *config = dev->config;
	struct udc_gd32_data *priv = udc_get_private(dev);
	uint32_t intf;

	/* service successful transfers until no more events are pending */
	while (((intf = *(volatile uint32_t *)(config->base + UDC_GD32_INTF_OFF)) &
		UDC_GD32_INTF_STIF) != 0U) {
		uint8_t ep = intf & UDC_GD32_INTF_EPNUM;

		if ((intf & UDC_GD32_INTF_DIR) == 0U) {
			/* IN direction transaction */
			if ((gd32_epcs_rd(config, ep) & UDC_GD32_EPX_TX_ST) != 0U) {
				gd32_epcs_tx_st_clear(config, ep);
				gd32_handle_in_isr(dev, ep);
			}
		} else {
			/* OUT direction transaction */
			if ((gd32_epcs_rd(config, ep) & UDC_GD32_EPX_RX_ST) != 0U) {
				gd32_epcs_rx_st_clear(config, ep);

				if ((gd32_epcs_rd(config, ep) &
				     UDC_GD32_EPX_SETUP) != 0U) {
					if (ep == 0U) {
						gd32_ram_get_setup(dev, priv->setup);
						k_event_post(&priv->events,
							     BIT(UDC_GD32_EVT_SETUP));
					}
				} else {
					gd32_handle_out_isr(dev, ep);
				}
			}
		}
	}

	intf = *(volatile uint32_t *)(config->base + UDC_GD32_INTF_OFF);
	if (intf != 0U) {
		uint32_t clear = 0U;

		if ((intf & UDC_GD32_INTF_ERRIF) != 0U) {
			clear |= UDC_GD32_INTF_ERRIF;
			udc_submit_event(dev, UDC_EVT_ERROR, -EIO);
		}
		if ((intf & UDC_GD32_INTF_WKUPIF) != 0U) {
			clear |= UDC_GD32_INTF_WKUPIF;
			if (udc_is_suspended(dev)) {
				udc_set_suspended(dev, false);
				udc_submit_event(dev, UDC_EVT_RESUME, 0);
			}
		}
		if ((intf & UDC_GD32_INTF_SPSIF) != 0U) {
			clear |= UDC_GD32_INTF_SPSIF;
			if (!udc_is_suspended(dev)) {
				udc_set_suspended(dev, true);
				udc_submit_event(dev, UDC_EVT_SUSPEND, 0);
			}
		}
		if (IS_ENABLED(CONFIG_UDC_ENABLE_SOF) &&
		    (intf & UDC_GD32_INTF_SOFIF) != 0U) {
			clear |= UDC_GD32_INTF_SOFIF;
			udc_submit_event(dev, UDC_EVT_SOF, 0);
		}
		if ((intf & UDC_GD32_INTF_RSTIF) != 0U) {
			clear |= UDC_GD32_INTF_RSTIF;

			/* re-arm control endpoint and flushed buffers */
			priv->ram_cur = UDC_GD32_DATA_BUF_ADDR;
			memset(priv->buf_addr, 0, sizeof(priv->buf_addr));
			memset(priv->tx, 0, sizeof(priv->tx));
			gd32_ep0_arm(config);
			gd32_set_device_address(config, 0U);

			udc_submit_event(dev, UDC_EVT_RESET, 0);
		}

		/* clear bits by writing 0, preserving any new flags */
		*(volatile uint32_t *)(config->base + UDC_GD32_INTF_OFF) = ~clear;
	}
}

static void gd32_thread_handler(const struct device *const dev)
{
	struct udc_gd32_data *priv = udc_get_private(dev);
	struct udc_ep_config *ep_cfg;
	uint32_t evt;
	uint32_t eps;
	uint8_t ep;
	int err;

	evt = k_event_wait(&priv->events, UINT32_MAX, false, K_FOREVER);
	udc_lock_internal(dev, K_FOREVER);

	if (evt & BIT(UDC_GD32_EVT_XFER_FINISHED)) {
		k_event_clear(&priv->events, BIT(UDC_GD32_EVT_XFER_FINISHED));

		eps = atomic_clear(&priv->transfer_finished);

		while (eps) {
			struct net_buf *buf;

			ep = gd32_pull_ep_from_bmsk(&eps);
			ep_cfg = udc_get_ep_cfg(dev, ep);
			LOG_DBG("Finished event ep 0x%02x", ep);

			buf = udc_buf_get(ep_cfg);
			if (buf == NULL) {
				LOG_ERR("No buffer for finished ep 0x%02x", ep);
				continue;
			}

			udc_ep_set_busy(ep_cfg, false);

			err = udc_submit_ep_event(dev, buf, 0);
			if (err != 0) {
				udc_submit_event(dev, UDC_EVT_ERROR, err);
			}

			gd32_handle_xfer_next(dev, ep_cfg);
		}
	}

	if (evt & BIT(UDC_GD32_EVT_XFER_NEW)) {
		k_event_clear(&priv->events, BIT(UDC_GD32_EVT_XFER_NEW));

		eps = atomic_clear(&priv->transfer_new);

		while (eps) {
			ep = gd32_pull_ep_from_bmsk(&eps);
			ep_cfg = udc_get_ep_cfg(dev, ep);
			LOG_DBG("New transfer ep 0x%02x in the queue", ep);

			gd32_handle_xfer_next(dev, ep_cfg);
		}
	}

	if (evt & BIT(UDC_GD32_EVT_SETUP)) {
		k_event_clear(&priv->events, BIT(UDC_GD32_EVT_SETUP));

		udc_setup_received(dev, priv->setup);
	}

	udc_unlock_internal(dev);
}

static int gd32_ep_enable(const struct device *dev, struct udc_ep_config *const ep_cfg)
{
	const struct udc_gd32_config *config = dev->config;
	struct udc_gd32_data *priv = udc_get_private(dev);
	struct gd32_usbd_bdt *bdt = gd32_bdt(config);
	uint8_t idx = USB_EP_GET_IDX(ep_cfg->addr);
	uint16_t mps = udc_mps_ep_size(ep_cfg);
	uint16_t type;
	uint16_t addr = 0U;

	switch (ep_cfg->attributes & USB_EP_TRANSFER_TYPE_MASK) {
	case USB_EP_TYPE_CONTROL:
		type = UDC_GD32_EPX_TYPE_CONTROL;
		break;
	case USB_EP_TYPE_ISO:
		type = UDC_GD32_EPX_TYPE_ISO;
		break;
	case USB_EP_TYPE_BULK:
		type = UDC_GD32_EPX_TYPE_BULK;
		break;
	case USB_EP_TYPE_INTERRUPT:
		type = UDC_GD32_EPX_TYPE_INT;
		break;
	default:
		return -EINVAL;
	}

	if (idx == 0U) {
		gd32_epcs_wr(config, 0U, UDC_GD32_EPX_TYPE_CONTROL |
			     UDC_GD32_EPX_RX_VALID | UDC_GD32_EPX_TX_NAK);
		return 0;
	}

	if (priv->buf_addr[gd32_ep_to_bnum(ep_cfg->addr)] == 0U) {
		addr = gd32_buf_alloc(priv, ROUND_UP(mps, UDC_GD32_RAM_WORD));
		if (addr == 0U) {
			LOG_ERR("No packet RAM for endpoint 0x%02x", ep_cfg->addr);
			return -ENOBUFS;
		}
		priv->buf_addr[gd32_ep_to_bnum(ep_cfg->addr)] = addr;
	} else {
		addr = priv->buf_addr[gd32_ep_to_bnum(ep_cfg->addr)];
	}

	if (USB_EP_DIR_IS_IN(ep_cfg->addr)) {
		bdt[idx].tx_addr = addr / UDC_GD32_RAM_WORD;
		bdt[idx].tx_count = 0U;
		gd32_epcs_wr(config, idx, type | idx);
		gd32_epcs_tx_stat_set(config, idx, UDC_GD32_EPX_TX_NAK);
	} else {
		bdt[idx].rx_addr = addr / UDC_GD32_RAM_WORD;
		bdt[idx].rx_count = gd32_rx_count(mps);
		gd32_epcs_wr(config, idx, type | idx);
		gd32_epcs_rx_stat_set(config, idx, UDC_GD32_EPX_RX_NAK);
	}

	LOG_DBG("Enable ep 0x%02x mps %u", ep_cfg->addr, mps);

	return 0;
}

static int gd32_ep_disable(const struct device *dev, struct udc_ep_config *const ep_cfg)
{
	const struct udc_gd32_config *config = dev->config;
	uint8_t idx = USB_EP_GET_IDX(ep_cfg->addr);

	if (USB_EP_DIR_IS_IN(ep_cfg->addr)) {
		gd32_ep_dtg_clear(config, idx, true);
		gd32_epcs_tx_stat_set(config, idx, UDC_GD32_EPX_TX_DISABLED);
	} else {
		gd32_ep_dtg_clear(config, idx, false);
		gd32_epcs_rx_stat_set(config, idx, UDC_GD32_EPX_RX_DISABLED);
	}

	LOG_DBG("Disable ep 0x%02x", ep_cfg->addr);

	return 0;
}

static int gd32_ep_set_halt(const struct device *dev, struct udc_ep_config *const ep_cfg)
{
	const struct udc_gd32_config *config = dev->config;
	uint8_t idx = USB_EP_GET_IDX(ep_cfg->addr);

	if (idx == 0U) {
		gd32_epcs_tx_stat_set(config, 0U, UDC_GD32_EPX_TX_STALL);
		gd32_epcs_rx_stat_set(config, 0U, UDC_GD32_EPX_RX_STALL);
	} else if (USB_EP_DIR_IS_IN(ep_cfg->addr)) {
		gd32_epcs_tx_stat_set(config, idx, UDC_GD32_EPX_TX_STALL);
	} else {
		gd32_epcs_rx_stat_set(config, idx, UDC_GD32_EPX_RX_STALL);
	}

	if (idx != 0U) {
		ep_cfg->stat.halted = true;
	}

	LOG_DBG("Set halt ep 0x%02x", ep_cfg->addr);

	return 0;
}

static int gd32_ep_clear_halt(const struct device *dev,
			      struct udc_ep_config *const ep_cfg)
{
	const struct udc_gd32_config *config = dev->config;
	struct udc_gd32_data *priv = udc_get_private(dev);
	uint8_t idx = USB_EP_GET_IDX(ep_cfg->addr);

	if (idx == 0U) {
		return 0;
	}

	if (USB_EP_DIR_IS_IN(ep_cfg->addr)) {
		gd32_ep_dtg_clear(config, idx, true);
		gd32_epcs_tx_stat_set(config, idx, UDC_GD32_EPX_TX_NAK);
	} else {
		gd32_ep_dtg_clear(config, idx, false);
		gd32_epcs_rx_stat_set(config, idx, UDC_GD32_EPX_RX_NAK);
	}

	ep_cfg->stat.halted = false;

	if (!udc_ep_is_busy(ep_cfg) && udc_buf_peek(ep_cfg) != NULL) {
		atomic_set_bit(&priv->transfer_new, gd32_ep_to_bnum(ep_cfg->addr));
		k_event_post(&priv->events, BIT(UDC_GD32_EVT_XFER_NEW));
	}

	LOG_DBG("Clear halt ep 0x%02x", ep_cfg->addr);

	return 0;
}

static int gd32_ep_enqueue(const struct device *dev, struct udc_ep_config *const ep_cfg,
			   struct net_buf *buf)
{
	struct udc_gd32_data *priv = udc_get_private(dev);

	LOG_DBG("%s enqueue 0x%02x %p", dev->name, ep_cfg->addr, (void *)buf);
	udc_buf_put(ep_cfg, buf);

	if (!ep_cfg->stat.halted && !udc_ep_is_busy(ep_cfg)) {
		atomic_set_bit(&priv->transfer_new, gd32_ep_to_bnum(ep_cfg->addr));
		k_event_post(&priv->events, BIT(UDC_GD32_EVT_XFER_NEW));
	}

	return 0;
}

static int gd32_ep_dequeue(const struct device *dev,
			   struct udc_ep_config *const ep_cfg)
{
	unsigned int lock_key;
	int ret;

	lock_key = irq_lock();

	ret = gd32_ep_disable(dev, ep_cfg);

	udc_ep_cancel_queued(dev, ep_cfg);
	udc_ep_set_busy(ep_cfg, false);

	irq_unlock(lock_key);

	return ret;
}

static int gd32_set_address(const struct device *dev, const uint8_t addr)
{
	LOG_DBG("Set new address %u for %s", addr, dev->name);
	gd32_set_device_address(dev->config, addr);

	return 0;
}

static int gd32_host_wakeup(const struct device *dev)
{
	const struct udc_gd32_config *config = dev->config;

	LOG_DBG("Remote wakeup from %s", dev->name);

	*(volatile uint32_t *)(config->base + UDC_GD32_CTL_OFF) =
		*(volatile uint32_t *)(config->base + UDC_GD32_CTL_OFF) |
		UDC_GD32_CTL_RSREQ;

	return 0;
}

static enum udc_bus_speed gd32_device_speed(const struct device *dev)
{
	struct udc_data *data = dev->data;

	return data->caps.hs ? UDC_BUS_SPEED_HS : UDC_BUS_SPEED_FS;
}

static int gd32_driver_init_clock(const struct device *dev)
{
	const struct udc_gd32_config *config = dev->config;
	int err;

	printk("usb: clock on clk_dev=%s\n", config->clk_dev ? config->clk_dev->name :
	       "(null)");

	err = clock_control_on(config->clk_dev,
			       (clock_control_subsys_t)&config->clk_id);
	if (err != 0) {
		LOG_ERR("Failed to enable USBD clock (%d)", err);
		printk("usb: clock_control_on failed %d\n", err);
		return err;
	}

	printk("usb: reset toggle\n");
	err = reset_line_toggle_dt(&config->reset);
	if (err != 0) {
		LOG_ERR("Failed to reset USBD peripheral (%d)", err);
		printk("usb: reset_line_toggle_dt failed %d\n", err);
		return err;
	}

	return 0;
}

static int gd32_disable(const struct device *dev)
{
	const struct udc_gd32_config *config = dev->config;
	int err;

	printk("usb: disable %s\n", dev->name);
	config->irq_disable_func(dev);

	*(volatile uint32_t *)(config->base + UDC_GD32_CTL_OFF) =
		UDC_GD32_CTL_SETRST;

	if (config->disconnect_gpio.port != NULL) {
		err = gpio_pin_configure_dt(&config->disconnect_gpio,
					    GPIO_OUTPUT_INACTIVE);
		if (err != 0) {
			LOG_ERR("Failed to configure disconnect GPIO (%d)", err);
		}
	}

	if (udc_is_suspended(dev)) {
		udc_set_suspended(dev, false);
	}

	return 0;
}

static int gd32_enable(const struct device *dev)
{
	const struct udc_gd32_config *config = dev->config;
	int err;

	printk("usb: enable %s\n", dev->name);

	err = gd32_driver_init_clock(dev);
	if (err != 0) {
		return err;
	}

	/* perform a soft reset of the USB core */
	printk("usb: soft reset\n");
	*(volatile uint32_t *)(config->base + UDC_GD32_CTL_OFF) = UDC_GD32_CTL_SETRST;
	*(volatile uint32_t *)(config->base + UDC_GD32_CTL_OFF) = 0U;
	*(volatile uint32_t *)(config->base + UDC_GD32_INTF_OFF) = 0U;

	/* descriptor table at the start of the packet RAM */
	*(volatile uint32_t *)(config->base + UDC_GD32_BADDR_OFF) = 0U;

	gd32_ep0_arm(config);
	gd32_set_device_address(config, 0U);

	/* Public udc_ep_enable() rejects control endpoints, so the driver is
	 * responsible for enabling EP0 through the internal API. This also
	 * marks the endpoints as enabled for udc_ep_enqueue().
	 */
	err = udc_ep_enable_internal(dev, USB_CONTROL_EP_OUT,
				     USB_EP_TYPE_CONTROL, 64, 0);
	if (err != 0) {
		LOG_ERR("Failed to enable control endpoint (%d)", err);
		return err;
	}

	err = udc_ep_enable_internal(dev, USB_CONTROL_EP_IN,
				     USB_EP_TYPE_CONTROL, 64, 0);
	if (err != 0) {
		LOG_ERR("Failed to enable control endpoint (%d)", err);
		return err;
	}

	if (IS_ENABLED(CONFIG_UDC_ENABLE_SOF)) {
		*(volatile uint32_t *)(config->base + UDC_GD32_CTL_OFF) =
			UDC_GD32_INTEN | UDC_GD32_CTL_SOFIE;
	} else {
		*(volatile uint32_t *)(config->base + UDC_GD32_CTL_OFF) =
			UDC_GD32_INTEN;
	}

	config->irq_enable_func(dev);

	/*
	 * Assert the D+ pull-up: connect the device to the bus. The active
	 * level is taken from the devicetree flag, matching udc_stm32.
	 */
	if (config->disconnect_gpio.port != NULL) {
		printk("usb: assert disconnect gpio %s pin %u\n",
		       config->disconnect_gpio.port->name,
		       config->disconnect_gpio.pin);
		err = gpio_pin_configure_dt(&config->disconnect_gpio,
					    GPIO_OUTPUT_ACTIVE);
		if (err != 0) {
			LOG_ERR("Failed to configure disconnect GPIO (%d)", err);
			printk("usb: gpio_pin_configure_dt failed %d\n", err);
			return err;
		}
		printk("usb: gpio configured active, readback=%d\n",
		       gpio_pin_get_dt(&config->disconnect_gpio));
	}

	LOG_DBG("Enable device %s", dev->name);
	printk("usb: enabled\n");

	return 0;
}

static int gd32_init(const struct device *dev)
{
	LOG_DBG("Init device %s", dev->name);

	return 0;
}

static int gd32_shutdown(const struct device *dev)
{
	return gd32_disable(dev);
}

static int gd32_driver_preinit(const struct device *dev)
{
	struct udc_gd32_config *config = (struct udc_gd32_config *)dev->config;
	struct udc_gd32_data *priv = udc_get_private(dev);
	struct udc_data *data = dev->data;
	int err;

	k_mutex_init(&data->mutex);
	k_event_init(&priv->events);
	atomic_clear(&priv->transfer_finished);
	atomic_clear(&priv->transfer_new);

	data->caps.rwup = true;
	data->caps.mps0 = UDC_MPS0_64;
	data->caps.can_detect_vbus = false;

	for (int i = 0; i < config->num_of_eps; i++) {
		config->ep_cfg_out[i].caps.out = 1;
		if (i == 0) {
			config->ep_cfg_out[i].caps.control = 1;
			config->ep_cfg_out[i].caps.mps = 64;
		} else {
			config->ep_cfg_out[i].caps.bulk = 1;
			config->ep_cfg_out[i].caps.interrupt = 1;
			config->ep_cfg_out[i].caps.mps = 64;
		}
		config->ep_cfg_out[i].addr = USB_EP_DIR_OUT | i;
		err = udc_register_ep(dev, &config->ep_cfg_out[i]);
		if (err != 0) {
			LOG_ERR("Failed to register endpoint");
			return err;
		}
	}

	for (int i = 0; i < config->num_of_eps; i++) {
		config->ep_cfg_in[i].caps.in = 1;
		if (i == 0) {
			config->ep_cfg_in[i].caps.control = 1;
			config->ep_cfg_in[i].caps.mps = 64;
		} else {
			config->ep_cfg_in[i].caps.bulk = 1;
			config->ep_cfg_in[i].caps.interrupt = 1;
			config->ep_cfg_in[i].caps.mps = 64;
		}
		config->ep_cfg_in[i].addr = USB_EP_DIR_IN | i;
		err = udc_register_ep(dev, &config->ep_cfg_in[i]);
		if (err != 0) {
			LOG_ERR("Failed to register endpoint");
			return err;
		}
	}

	config->make_thread(dev);

	return 0;
}

static void gd32_lock(const struct device *dev)
{
	k_sched_lock();
	udc_lock_internal(dev, K_FOREVER);
}

static void gd32_unlock(const struct device *dev)
{
	udc_unlock_internal(dev);
	k_sched_unlock();
}

static const struct udc_api gd32_api = {
	.lock = gd32_lock,
	.unlock = gd32_unlock,
	.device_speed = gd32_device_speed,
	.init = gd32_init,
	.enable = gd32_enable,
	.disable = gd32_disable,
	.shutdown = gd32_shutdown,
	.set_address = gd32_set_address,
	.host_wakeup = gd32_host_wakeup,
	.ep_enable = gd32_ep_enable,
	.ep_disable = gd32_ep_disable,
	.ep_set_halt = gd32_ep_set_halt,
	.ep_clear_halt = gd32_ep_clear_halt,
	.ep_enqueue = gd32_ep_enqueue,
	.ep_dequeue = gd32_ep_dequeue,
};

#define UDC_GD32_IRQ_DEFINE(n)							\
	static void udc_gd32_irq_enable_func_##n(const struct device *dev)	\
	{									\
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority),		\
			    gd32_isr_handler, DEVICE_DT_INST_GET(n), 0);	\
		irq_enable(DT_INST_IRQN(n));					\
	}									\
	static void udc_gd32_irq_disable_func_##n(const struct device *dev)	\
	{									\
		irq_disable(DT_INST_IRQN(n));					\
	}

#define UDC_GD32_DISCONNECT_GPIO_GET(n)						\
	GPIO_DT_SPEC_INST_GET_OR(n, disconnect_gpios, {0})

#define UDC_GD32_DEVICE_DEFINE(n)						\
	K_THREAD_STACK_DEFINE(udc_gd32_stack_##n, CONFIG_UDC_GD32_STACK_SIZE);	\
										\
	static void udc_gd32_thread_##n(void *dev_ptr, void *u1, void *u2)	\
	{									\
		while (true) {							\
			gd32_thread_handler(dev_ptr);				\
		}								\
	}									\
										\
	static void udc_gd32_make_thread_##n(const struct device *dev)		\
	{									\
		struct udc_gd32_data *priv = udc_get_private(dev);		\
										\
		k_thread_create(&priv->thread_data,				\
				udc_gd32_stack_##n,				\
				K_THREAD_STACK_SIZEOF(udc_gd32_stack_##n),	\
				udc_gd32_thread_##n,				\
				(void *)dev, NULL, NULL,			\
				K_PRIO_COOP(CONFIG_UDC_GD32_THREAD_PRIORITY),	\
				K_ESSENTIAL, K_NO_WAIT);			\
		k_thread_name_set(&priv->thread_data, dev->name);		\
	}									\
										\
	static struct udc_ep_config						\
		ep_cfg_out[DT_INST_PROP(n, num_bidir_endpoints)];		\
	static struct udc_ep_config						\
		ep_cfg_in[DT_INST_PROP(n, num_bidir_endpoints)];		\
										\
	UDC_GD32_IRQ_DEFINE(n)							\
										\
	static const struct udc_gd32_config udc_gd32_config_##n = {		\
		.base = DT_INST_REG_ADDR(n),					\
		.ram = DT_INST_REG_ADDR(n) + 0x400U,				\
		.num_of_eps = DT_INST_PROP(n, num_bidir_endpoints),		\
		.ep_cfg_out = ep_cfg_out,					\
		.ep_cfg_in = ep_cfg_in,						\
		.clk_dev = DEVICE_DT_GET(DT_CLOCKS_CTLR(DT_DRV_INST(n))),	\
		.clk_id = DT_INST_CLOCKS_CELL(n, id),				\
		.reset = RESET_DT_SPEC_INST_GET(n),				\
		.disconnect_gpio = UDC_GD32_DISCONNECT_GPIO_GET(n),		\
		.irq_enable_func = udc_gd32_irq_enable_func_##n,		\
		.irq_disable_func = udc_gd32_irq_disable_func_##n,		\
		.make_thread = udc_gd32_make_thread_##n,			\
	};									\
										\
	static struct udc_gd32_data udc_priv_##n;				\
										\
	static struct udc_data udc_data_##n = {					\
		.mutex = Z_MUTEX_INITIALIZER(udc_data_##n.mutex),		\
		.priv = &udc_priv_##n,						\
	};									\
										\
	DEVICE_DT_INST_DEFINE(n, gd32_driver_preinit, NULL,			\
			      &udc_data_##n, &udc_gd32_config_##n,		\
			      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,	\
			      &gd32_api);

DT_INST_FOREACH_STATUS_OKAY(UDC_GD32_DEVICE_DEFINE)