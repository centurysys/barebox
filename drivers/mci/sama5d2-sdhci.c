/*
 * Microchip SAMA5D2 SDHCI MCI driver
 *
 * Pengutronix, Michael Grzeschik <mgr@pengutronix.de>
 * Sebastian Hesselbarth <sebastian.hesselbarth@gmail.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <clock.h>
#include <common.h>
#include <init.h>
#include <io.h>
#include <dma.h>
#include <malloc.h>
#include <mci.h>
#include <linux/clk.h>
#include <linux/err.h>

#include "sdhci.h"

#define SDMMC_MC1R		0x204
#define SDMMC_MC1R_DDR		BIT(3)
#define SDMMC_MC1R_FCD		BIT(7)
#define SDMMC_CACR		0x230
#define SDMMC_CACR_CAPWREN	BIT(0)
#define SDMMC_CACR_KEY		(0x46 << 8)

#define SDHCI_AT91_PRESET_COMMON_CONF	0x400 /* drv type B, programmable clock mode */

struct sama5d2_sdhci {
	struct device_d *dev;
	struct mci_host mci;
	void __iomem *base;
	struct sdhci sdhci;
	struct clk *mainclk;
	struct clk *hclk;
	struct clk *gclk;
};

#define SIZE_512K (1 << 19)
static char *dma_buffer = NULL;

#define priv_from_mci_host(h)	\
	container_of(h, struct sama5d2_sdhci, mci);

static void sama5d2_sdhci_writel(struct sdhci *sdhci, int reg, u32 val)
{
	struct sama5d2_sdhci *p = container_of(sdhci, struct sama5d2_sdhci, sdhci);

	writel(val, p->base + reg);
}

static void sama5d2_sdhci_writew(struct sdhci *sdhci, int reg, u16 val)
{
	struct sama5d2_sdhci *p = container_of(sdhci, struct sama5d2_sdhci, sdhci);

	writew(val, p->base + reg);
}

static void sama5d2_sdhci_writeb(struct sdhci *sdhci, int reg, u8 val)
{
	struct sama5d2_sdhci *p = container_of(sdhci, struct sama5d2_sdhci, sdhci);

	writeb(val, p->base + reg);
}

static u32 sama5d2_sdhci_readl(struct sdhci *sdhci, int reg)
{
	struct sama5d2_sdhci *p = container_of(sdhci, struct sama5d2_sdhci, sdhci);

	return readl(p->base + reg);
}

static u16 sama5d2_sdhci_readw(struct sdhci *sdhci, int reg)
{
	struct sama5d2_sdhci *p = container_of(sdhci, struct sama5d2_sdhci, sdhci);

	return readw(p->base + reg);
}

static u8 sama5d2_sdhci_readb(struct sdhci *sdhci, int reg)
{
	struct sama5d2_sdhci *p = container_of(sdhci, struct sama5d2_sdhci, sdhci);

	return readb(p->base + reg);
}

static int sama5d2_sdhci_wait_for_done(struct sama5d2_sdhci *host, u16 mask)
{
	u16 status;
	u64 start;

	start = get_time_ns();
	while (1) {
		status = sdhci_read16(&host->sdhci, SDHCI_INT_NORMAL_STATUS);
		if (status & SDHCI_INT_ERROR)
			return -EPERM;
		/* this special quirk is necessary, as the dma
		 * engine stops on dma boundary and will only
		 * restart after acknowledging it this way.
		 */
		if (status & SDHCI_INT_DMA) {
			u32 addr = sdhci_read32(&host->sdhci, SDHCI_DMA_ADDRESS);
			sdhci_write32(&host->sdhci, SDHCI_DMA_ADDRESS, addr);
		}
		if (status & mask) {
			sdhci_write16(&host->sdhci, SDHCI_INT_NORMAL_STATUS, mask);
			status = sdhci_read16(&host->sdhci, SDHCI_INT_NORMAL_STATUS);
			break;
		}
		if (is_timeout(start, 1000 * MSECOND)) {
			dev_err(host->mci.hw_dev, "SDHCI timeout while waiting for done\n");
			return -ETIMEDOUT;
		}
	}
	return 0;
}

static int sama5d2_sdhci_mci_send_cmd(struct mci_host *mci, struct mci_cmd *cmd,
				struct mci_data *data)
{
	u32 val;
	u32 command, xfer;
	u64 start;
	int ret, loops;
	unsigned int num_bytes = 0;
	struct sama5d2_sdhci *host = priv_from_mci_host(mci);

	sdhci_write32(&host->sdhci, SDHCI_INT_STATUS, ~0);

	/* Do not wait for CMD_INHIBIT_DAT on stop commands */
	if (cmd->cmdidx == MMC_CMD_STOP_TRANSMISSION)
		val = SDHCI_CMD_INHIBIT_CMD;
	else
		val = SDHCI_CMD_INHIBIT_CMD | SDHCI_CMD_INHIBIT_DATA;

	/* Wait for bus idle */
	start = get_time_ns();
	loops = 0;
	while (1) {
		if (!(sdhci_read32(&host->sdhci, SDHCI_PRESENT_STATE) & val))
			break;
		if (is_timeout(start, 50 * MSECOND)) {
			dev_err(host->mci.hw_dev, "SDHCI timeout while waiting for idle (%d)\n", loops);
			return -ETIMEDOUT;
		}
		loops++;
	}

	/* setup transfer data */
	if (data) {
		num_bytes = data->blocks * data->blocksize;

		if (data->flags & MMC_DATA_WRITE) {
			memcpy(dma_buffer, data->src, num_bytes);
		}

		sdhci_write32(&host->sdhci, SDHCI_DMA_ADDRESS, (u32) dma_buffer);
		sdhci_write16(&host->sdhci, SDHCI_BLOCK_SIZE, SDHCI_DMA_BOUNDARY_512K |
				SDHCI_TRANSFER_BLOCK_SIZE(data->blocksize));
		sdhci_write16(&host->sdhci, SDHCI_BLOCK_COUNT, data->blocks);
		sdhci_write8(&host->sdhci, SDHCI_TIMEOUT_CONTROL, 0xe);

		if (data->flags & MMC_DATA_WRITE)
			dma_sync_single_for_device((unsigned long) dma_buffer,
						    num_bytes, DMA_TO_DEVICE);
	}

	/* setup transfer mode */
	sdhci_set_cmd_xfer_mode(&host->sdhci, cmd, data, true, &command, &xfer);

	if (sdhci_read16(&host->sdhci, SDHCI_TRANSFER_MODE) != xfer) {
		sdhci_write16(&host->sdhci, SDHCI_TRANSFER_MODE, xfer);
	}
	sdhci_write32(&host->sdhci, SDHCI_ARGUMENT, cmd->cmdarg);
	sdhci_write16(&host->sdhci, SDHCI_COMMAND, command);

	ret = sama5d2_sdhci_wait_for_done(host, SDHCI_INT_CMD_COMPLETE);
	if (ret) {
		dev_err(host->mci.hw_dev, "error on command %d (ret = %d)\n",
			cmd->cmdidx, ret);
		dev_err(host->mci.hw_dev, "state = [24]%08x, interrupt = [30]%04x [32]%04x\n",
			sdhci_read32(&host->sdhci, SDHCI_PRESENT_STATE),
			sdhci_read16(&host->sdhci, SDHCI_INT_NORMAL_STATUS),
			sdhci_read16(&host->sdhci, SDHCI_INT_ERROR_STATUS));
		/* reset sdhci controller */
		sdhci_write8(&host->sdhci, SDHCI_SOFTWARE_RESET, 2);
		goto cmd_error;
	}

	sdhci_read_response(&host->sdhci, cmd);

	udelay(1000);

	if (data) {
		ret = sama5d2_sdhci_wait_for_done(host, SDHCI_INT_XFER_COMPLETE);
		if (ret) {
			dev_err(host->mci.hw_dev, "error while transfering data for command %d\n",
				cmd->cmdidx);
			dev_err(host->mci.hw_dev, "state = [24]%08x, interrupt = [30]%04x [32]%04x\n",
				sdhci_read32(&host->sdhci, SDHCI_PRESENT_STATE),
				sdhci_read16(&host->sdhci, SDHCI_INT_NORMAL_STATUS),
				sdhci_read16(&host->sdhci, SDHCI_INT_ERROR_STATUS));
			goto cmd_error;
		}

		if (data->flags & MMC_DATA_READ) {
			dma_sync_single_for_cpu((unsigned long) dma_buffer,
						num_bytes, DMA_FROM_DEVICE);
			memcpy(data->dest, dma_buffer, num_bytes);
		}
	}

cmd_error:
	sdhci_write16(&host->sdhci, SDHCI_INT_NORMAL_STATUS, ~0);
	sdhci_write16(&host->sdhci, SDHCI_INT_ERROR_STATUS, ~0);
	return ret;
}

static u16 sama5d2_sdhci_get_clock_divider(struct sama5d2_sdhci *host, u32 reqclk)
{
	u16 div;
	u32 gclk;

	gclk = clk_get_rate(host->gclk);
	div = gclk / reqclk;
	dev_dbg(host->dev, "gclk: %d, reqclk: %d --> div: %d\n", gclk, reqclk, div);

	return div;
}

static void sama5d2_sdhci_mci_set_ios(struct mci_host *mci, struct mci_ios *ios)
{
	u16 val, div, clk;
	u64 start;
	struct sama5d2_sdhci *host = priv_from_mci_host(mci);

	dev_dbg(host->dev, "clock = %u, bus-width = %d, timing = %02x\n",
		ios->clock, ios->bus_width, ios->timing);

	sdhci_write16(&host->sdhci, SDHCI_CLOCK_CONTROL, 0);

	/* disable on zero clock */
	if (!ios->clock)
		return;

	/* enable bus power */
	val = SDHCI_BUS_VOLTAGE_330;
	sdhci_write8(&host->sdhci, SDHCI_POWER_CONTROL, val | SDHCI_BUS_POWER_EN);
	udelay(400);

	/* set bus width */
	val = sdhci_read8(&host->sdhci, SDHCI_HOST_CONTROL) &
		~(SDHCI_DATA_WIDTH_4BIT | SDHCI_DATA_WIDTH_8BIT);

	switch (ios->bus_width) {
	case MMC_BUS_WIDTH_8:
		val |= SDHCI_DATA_WIDTH_8BIT;
		break;
	case MMC_BUS_WIDTH_4:
		val |= SDHCI_DATA_WIDTH_4BIT;
		break;
	}

	if (ios->clock > 26000000)
		val |= SDHCI_HIGHSPEED_EN;
	else
		val &= ~SDHCI_HIGHSPEED_EN;

	sdhci_write8(&host->sdhci, SDHCI_HOST_CONTROL, val);
	
	//clk = sdhci_read16(&host->sdhci, SDHCI_CLOCK_CONTROL);
	//clk &= SDHCI_INTCLOCK_EN;
	//sdhci_write16(&host->sdhci, SDHCI_CLOCK_CONTROL, clk);

	if (ios->clock == 0)
		return;
	
	/* set bus clock */
	div = sama5d2_sdhci_get_clock_divider(host, ios->clock) - 1;
	val = SDHCI_INTCLOCK_EN | ((div & 0x300) >> 2) | ((div & 0xff) << 8) | BIT(5);

	dev_dbg(host->dev, "SDHCI_CLOCK_CONTROL <-- 0x%04x\n", val);
	sdhci_write16(&host->sdhci, SDHCI_CLOCK_CONTROL, val);

	/* wait for internal clock stable */
	start = get_time_ns();
	while (!(sdhci_read16(&host->sdhci, SDHCI_CLOCK_CONTROL) &
			SDHCI_INTCLOCK_STABLE)) {
		if (is_timeout(start, 20 * MSECOND)) {
			dev_err(host->mci.hw_dev, "SDHCI clock stable timeout\n");
			return;
		}
	}

	/* enable bus clock */
	val = sdhci_read16(&host->sdhci, SDHCI_CLOCK_CONTROL);
	sdhci_write16(&host->sdhci, SDHCI_CLOCK_CONTROL, val | SDHCI_SDCLOCK_EN);
}

static int sama5d2_sdhci_mci_init(struct mci_host *mci, struct device_d *dev)
{
	u64 start;
	u16 val;
	struct sama5d2_sdhci *host = priv_from_mci_host(mci);

	/* reset sdhci controller */
	sdhci_write8(&host->sdhci, SDHCI_SOFTWARE_RESET, SDHCI_RESET_ALL);

	/* wait for reset completion */
	start = get_time_ns();
	while (1) {
		if ((sdhci_read8(&host->sdhci, SDHCI_SOFTWARE_RESET) &
				SDHCI_RESET_ALL) == 0)
			break;
		if (is_timeout(start, 100 * MSECOND)) {
			dev_err(dev, "SDHCI reset timeout\n");
			return -ETIMEDOUT;
		}
	}

	sdhci_write32(&host->sdhci, SDHCI_INT_STATUS, ~0);
	sdhci_write32(&host->sdhci, SDHCI_INT_ENABLE, 0x027f003b);
	sdhci_write32(&host->sdhci, SDHCI_SIGNAL_ENABLE, 0);
	sdhci_write8(&host->sdhci, 0x205, BIT(1));
	val = sdhci_read16(&host->sdhci, 0x3e);
	return 0;
}

static void sama5d2_sdhci_set_mci_caps(struct sama5d2_sdhci *host)
{
	struct device_d *dev = host->dev;
	u32 caps[2];
	u32 clk_base, clk_mul;
	u32 gck_rate, clk_base_rate;
	u32 preset_div;
	struct clk *clk_utmi;

	host->hclk = clk_get(dev,"hclock");
	host->gclk = clk_get(dev, "multclk");
	host->mainclk = clk_get(dev, "baseclk");

	clk_utmi = clk_lookup("utmick");
	if (clk_utmi) {
		clk_set_parent(host->gclk, clk_utmi);
	}

#define ATMEL_SDHC_GCK_RATE	240000000
	clk_set_rate(host->gclk, ATMEL_SDHC_GCK_RATE);
	clk_enable(host->hclk);

	caps[0] = sdhci_read32(&host->sdhci, SDHCI_CAPABILITIES);
	caps[1] = sdhci_read32(&host->sdhci, SDHCI_CAPABILITIES_1);

	dev_dbg(dev, "caps[0]: 0x%08x\n", caps[0]);
	dev_dbg(dev, "caps[1]: 0x%08x\n", caps[1]);

	/*
	* We experience some issues with SDR104. If the SD clock is higher
	* than 100 MHz, we can get data corruption. With a 100 MHz clock,
	* the tuning procedure may fail. For those reasons, it is useless to
	* advertise that we can use SDR104 mode, so remove it from
	* the capabilities.
	*/
	sdhci_write32(&host->sdhci, SDMMC_CACR, SDMMC_CACR_KEY | SDMMC_CACR_CAPWREN);
	caps[1] &= (~SDHCI_SUPPORT_SDR104);
	sdhci_write32(&host->sdhci, SDHCI_CAPABILITIES_1, caps[1]);
	sdhci_write32(&host->sdhci, SDMMC_CACR, 0);

	gck_rate = clk_get_rate(host->gclk);
	clk_base_rate = clk_get_rate(host->mainclk);

	dev_dbg(dev, "gck_rate: %d, clk_base_rate: %d\n", gck_rate, clk_base_rate);

	clk_base = clk_base_rate / 1000000;
	clk_mul = gck_rate / clk_base_rate - 1;

	dev_dbg(dev, "clk_base: %d, clk_mul: %d\n", clk_base, clk_mul);

	caps[0] &= (~SDHCI_CLOCK_V3_BASE_MASK);
	caps[0] |= ((clk_base << SDHCI_CLOCK_BASE_SHIFT) & SDHCI_CLOCK_V3_BASE_MASK);
	caps[1] &= (~SDHCI_CLOCK_MUL_MASK);
	caps[1] |= ((clk_mul << SDHCI_CLOCK_MUL_SHIFT) & SDHCI_CLOCK_MUL_MASK);

	/* Set capabilities in r/w mode. */
	sdhci_write32(&host->sdhci, SDMMC_CACR, SDMMC_CACR_KEY | SDMMC_CACR_CAPWREN);

	sdhci_write32(&host->sdhci, SDHCI_CAPABILITIES, caps[0]);
	sdhci_write32(&host->sdhci, SDHCI_CAPABILITIES_1, caps[1]);

	/* Set capabilities in ro mode. */
	sdhci_write32(&host->sdhci, SDMMC_CACR, 0);
	dev_dbg(dev, "update clk mul to %u as gck rate is %u Hz and clk base is %u Hz\n",
		clk_mul, gck_rate, clk_base_rate);

	dev_dbg(dev, "caps[0]: 0x%08x\n", caps[0]);
	dev_dbg(dev, "caps[1]: 0x%08x\n", caps[1]);

	/*
	 * We have to set preset values because it depends on the clk_mul
	 * value. Moreover, SDR104 is supported in a degraded mode since the
	 * maximum sd clock value is 120 MHz instead of 208 MHz. For that
	 * reason, we need to use presets to support SDR104.
	 */
	preset_div = DIV_ROUND_UP(gck_rate, 24000000) - 1;
	sdhci_write16(&host->sdhci, SDHCI_PRESET_FOR_SDR12, SDHCI_AT91_PRESET_COMMON_CONF | preset_div);
	preset_div = DIV_ROUND_UP(gck_rate, 50000000) - 1;
	sdhci_write16(&host->sdhci, SDHCI_PRESET_FOR_SDR25, SDHCI_AT91_PRESET_COMMON_CONF | preset_div);
	preset_div = DIV_ROUND_UP(gck_rate, 100000000) - 1;
	sdhci_write16(&host->sdhci, SDHCI_PRESET_FOR_SDR50, SDHCI_AT91_PRESET_COMMON_CONF | preset_div);
	preset_div = DIV_ROUND_UP(gck_rate, 120000000) - 1;
	sdhci_write16(&host->sdhci, SDHCI_PRESET_FOR_SDR104, SDHCI_AT91_PRESET_COMMON_CONF | preset_div);
	preset_div = DIV_ROUND_UP(gck_rate, 50000000) - 1;
	sdhci_write16(&host->sdhci, SDHCI_PRESET_FOR_DDR50, SDHCI_AT91_PRESET_COMMON_CONF | preset_div);

	if (caps[1] & SDHCI_HOSTCAP_VOLTAGE_180) {
		dev_dbg(dev, "caps[1] & SDHCI_HOSTCAP_VOLTAGE_180\n");
		host->mci.voltages |= MMC_VDD_165_195;
	}
	if (caps[1] & SDHCI_HOSTCAP_VOLTAGE_300) {
		dev_dbg(dev, "caps[1] & SDHCI_HOSTCAP_VOLTAGE_300\n");
		host->mci.voltages |= MMC_VDD_29_30 | MMC_VDD_30_31;
	}
	if (caps[1] & SDHCI_HOSTCAP_VOLTAGE_330) {
		dev_dbg(dev, "caps[1] & SDHCI_HOSTCAP_VOLTAGE_330\n");
		host->mci.voltages |= MMC_VDD_32_33 | MMC_VDD_33_34;
	}
	if (caps[1] & SDHCI_HOSTCAP_HIGHSPEED) {
		dev_dbg(dev, "caps[1] & SDHCI_HOSTCAP_HIGHSPEED\n");
		host->mci.host_caps |= (MMC_CAP_MMC_HIGHSPEED_52MHZ |
					MMC_CAP_MMC_HIGHSPEED |
					MMC_CAP_SD_HIGHSPEED);
	}

	/* parse board supported bus width capabilities */
	mci_of_parse(&host->mci);

	/* limit bus widths to controller capabilities */
	if ((caps[1] & SDHCI_HOSTCAP_8BIT) == 0) {
		dev_dbg(dev, "(caps[1] & SDHCI_HOSTCAP_8BIT) == 0\n");
		host->mci.host_caps &= ~MMC_CAP_8_BIT_DATA;
	}

	clk_enable(host->gclk);
	clk_enable(host->mainclk);
}

static int sama5d2_sdhci_detect(struct device_d *dev)
{
	struct sama5d2_sdhci *host = dev->priv;
	return mci_detect_card(&host->mci);
}

static int sama5d2_sdhci_probe(struct device_d *dev)
{
	struct sama5d2_sdhci *host;
	int ret;

	host = xzalloc(sizeof(*host));
	host->dev = dev;
	host->base = dev_request_mem_region(dev, 0);
	//host->mci.max_req_size = 0x8000;
	host->mci.hw_dev = dev;
	host->mci.send_cmd = sama5d2_sdhci_mci_send_cmd;
	host->mci.set_ios = sama5d2_sdhci_mci_set_ios;
	host->mci.init = sama5d2_sdhci_mci_init;
	host->mci.f_max = 50000000;
	host->mci.f_min = host->mci.f_max / 256;
	host->mci.voltages = MMC_VDD_32_33 | MMC_VDD_33_34;
	host->sdhci.read32 = sama5d2_sdhci_readl;
	host->sdhci.read16 = sama5d2_sdhci_readw;
	host->sdhci.read8 = sama5d2_sdhci_readb;
	host->sdhci.write32 = sama5d2_sdhci_writel;
	host->sdhci.write16 = sama5d2_sdhci_writew;
	host->sdhci.write8 = sama5d2_sdhci_writeb;
	dev->priv = host;
	dev->detect = sama5d2_sdhci_detect;

	sama5d2_sdhci_set_mci_caps(host);

	if (!dma_buffer) {
		dma_buffer = memalign(SIZE_512K, SIZE_512K);

		if (!dma_buffer) {
			free(host);
			ret = -ENOMEM;
			goto ret;
		}
	}

	ret = mci_register(&host->mci);
	if (ret)
		free(host);

ret:
	return ret;
}

static struct of_device_id sama5d2_sdhci_dt_ids[] = {
	{ .compatible = "atmel,sama5d2-sdhci", },
	{ }
};

static struct driver_d sama5d2_sdhci_driver = {
	.name = "sama5d2-sdhci",
	.probe = sama5d2_sdhci_probe,
	.of_compatible = DRV_OF_COMPAT(sama5d2_sdhci_dt_ids),
};
device_platform_driver(sama5d2_sdhci_driver);
