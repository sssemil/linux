/*
 * Synopsys Secure Digital Host Controller Interface.
 * Copyright (C) 2011 - 2012 Michal Simek <monstr@monstr.eu>
 * Copyright (c) 2012 Wind River Systems, Inc.
 * Copyright (C) 2013 Pengutronix e.K.
 * Copyright (C) 2013 Xilinx Inc.
 *
 * Based on sdhci-of-arasan.c
 *
 * Copyright (c) 2007 Freescale Semiconductor, Inc.
 * Copyright (c) 2009 MontaVista Software, Inc.
 *
 * Authors: Xiaobo Xie <X.Xie@freescale.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */
/*lint --e{750}*/
#include <linux/bootdevice.h>
#include <linux/module.h>
#include "sdhci-pltfm.h"
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_address.h>
#include <linux/mmc/mmc.h>
#include <linux/slab.h>
#include <linux/scatterlist.h>
#include <linux/mmc/cmdq_hci.h>
#include <linux/pm_runtime.h>
#include <linux/mmc/card.h>
#include <soc_pmctrl_interface.h>
#include <linux/regulator/consumer.h>
#include <linux/i2c.h>
#include "sdhci-dwc-mshc.h"

#define DRIVER_NAME "sdhci-mshc"

#define CLK_CTRL_TIMEOUT_SHIFT		16
#define CLK_CTRL_TIMEOUT_MASK		(0xf << CLK_CTRL_TIMEOUT_SHIFT)
#define CLK_CTRL_TIMEOUT_MIN_EXP	13

#define SDHCI_MSHC_MIN_FREQ				400000
#define SDHCI_MSHC_AHB_CLK_FREQ			120000000
#define SDHCI_MSHC_XIN_CLK_FREQ			200000000

static u32 xin_clk = SDHCI_MSHC_XIN_CLK_FREQ;
static u32 ahb_clk = SDHCI_MSHC_AHB_CLK_FREQ;

extern void sdhci_dumpregs(struct sdhci_host *host);

/**
 * struct sdhci_mshc_data
 * @clk:						Pointer to the sd clock
 * @clk_ahb:					Pointer to the AHB clock
 * @clock:					record current clock rate
 * @tuning_current_sample:		record current sample when soft tuning
 * @tuning_init_sample:			record the optimum sample of soft tuning
 * @tuning_sample_count:		record the tuning count for soft tuning
 * @tuning_move_sample:		record the move sample when data or cmd error occor
 * @tuning_move_count:		record the move count
 * @tuning_sample_flag:			record the sample OK or NOT of soft tuning
 * @tuning_strobe_init_sample:	default strobe sample
 * @tuning_strobe_move_sample:	record the strobe move sample when data or cmd error occor
 * @tuning_strobe_move_count:	record the strobe move count
 */
struct sdhci_mshc_data {
	struct clk *clk;
	struct clk *clk_ahb;
	unsigned int clock;
	unsigned int enhanced_strobe_enabled;
	int tuning_current_sample;
	int tuning_init_sample;
	int tuning_sample_count;
	int tuning_move_sample;
	int tuning_move_count;
	unsigned long tuning_sample_flag;
	int tuning_strobe_init_sample;
	int tuning_strobe_move_sample;
	int tuning_strobe_move_count;
};


#define MMC_I2C_SLAVE (0x5A)
#define MMC_I2C_BUS 3

/*lint -e785*/
static struct i2c_board_info mmc_i2c_board_info = {
    /* FIXME*/
    .type = "i2c_mmc",
    .addr = MMC_I2C_SLAVE,
};
/*lint +e785*/

static void sdhci_i2c_writel(struct sdhci_host *host, u32 val, u32 addr)
{
	int ret;

	union i2c_fmt {
		unsigned char chars[8];
		u32 addr_val[2];
	} data;
	data.addr_val[0] = cpu_to_be32(addr);
	data.addr_val[1] = val;
	if (host->i2c_client) {
		ret = i2c_master_send(host->i2c_client, (char *)data.chars,
				      (int)sizeof(union i2c_fmt));
		if (ret < 0)
			pr_err("%s fail\n", __func__);
	} else {
		pr_err("%s  fail client empty\n", __func__);
	}
}

static u32 sdhci_i2c_readl(struct sdhci_host *host, u32 addr)
{
	int ret;
	u32 val;
	u32 buf = cpu_to_be32(addr);
	struct i2c_msg msg[2];

	if (host->i2c_client) {
		msg[0].addr = host->i2c_client->addr;
		msg[0].flags = host->i2c_client->flags & I2C_M_TEN;
		msg[0].len = sizeof(addr);
		msg[0].buf = (char *)(&buf);

		msg[1].addr = host->i2c_client->addr;
		msg[1].flags = host->i2c_client->flags & I2C_M_TEN;
		msg[1].flags |= I2C_M_RD;
		msg[1].len = sizeof(val);
		msg[1].buf = (char *)&val;

		ret = i2c_transfer(host->i2c_client->adapter, (struct i2c_msg *)msg, 2);
		if (ret < 0)
			pr_err("%s  fail\n", __func__);
	} else {
		pr_err("%s  fail client empty\n", __func__);
	}

	return val;
}

static int create_i2c_client(struct sdhci_host *host)
{
	struct i2c_adapter *adapter;

	adapter = i2c_get_adapter(MMC_I2C_BUS);
	if (!adapter) {
		pr_err("%s i2c_get_adapter error\n", __func__);
		return -EIO;
	}
	host->i2c_client = i2c_new_device(adapter, &mmc_i2c_board_info);
	if (!host->i2c_client) {
		pr_err("%s i2c_new_device error\n", __func__);
		return -EIO;
	}
	return 0;
}

static void sdhci_mshc_i2c_gpio_config(struct device *dev)
{
	int err = 0;
	int chipsel_gpio;

	chipsel_gpio = of_get_named_gpio(dev->of_node, "i2c-gpios", 0);
	if (!gpio_is_valid(chipsel_gpio)) {
		pr_err("%s: chipsel_gpio %d is not valid,check DTS\n", __func__, chipsel_gpio);
	}
	err = gpio_request((unsigned int)chipsel_gpio, "i2c-gpio");
	if (err < 0) {
		pr_err("Can`t request chipsel gpio %d\n", chipsel_gpio);
	}
	err = gpio_direction_output((unsigned int)chipsel_gpio, 0);
	if (err < 0) {
		pr_err("%s: could not set gpio %d output push down\n", __func__, chipsel_gpio);
	}
}

static int sdhci_mshc_i2c_setup(struct platform_device *pdev)
{
	int ret = 0;
	struct sdhci_host *host = platform_get_drvdata(pdev);

	if ((host->quirks2 & SDHCI_QUIRK2_HISI_COMBO_PHY_TC) && !host->i2c_client) {
		sdhci_mshc_i2c_gpio_config(&pdev->dev);
		ret = create_i2c_client(host);
		if (ret)
			dev_err(&pdev->dev, "create i2c client error\n");
	}

	return ret;
}

void sdhci_mshc_dumpregs(struct sdhci_host *host)
{
/*
	pr_info(DRIVER_NAME ": PHY init ctrl: 0x%08x | init stat: 0x%08x\n",
		sdhci_phy_readl(host, COMBO_PHY_PHYINITCTRL),
		sdhci_phy_readl(host, COMBO_PHY_PHYINITSTAT));
	pr_info(DRIVER_NAME ": PHY imp ctrl: 0x%08x | imp stat: 0x%08x\n",
		sdhci_phy_readl(host, COMBO_PHY_IMPCTRL),
		sdhci_phy_readl(host, COMBO_PHY_IMPSTATUS));
	pr_info(DRIVER_NAME ": PHY dly ctrl1: 0x%08x | dly ctrl2: 0x%08x\n",
		sdhci_phy_readl(host, COMBO_PHY_DLY_CTL1),
		sdhci_phy_readl(host,  COMBO_PHY_DLY_CTL2));
	pr_info(DRIVER_NAME ": PHY drv1: 0x%08x | drv2: 0x%08x\n",
		sdhci_phy_readl(host, COMBO_PHY_IOCTL_RONSEL_1),
		sdhci_phy_readl(host, COMBO_PHY_IOCTL_RONSEL_2));
*/
}

void sdhci_hisi_dump_clk_reg(void)
{

}

static unsigned int sdhci_mshc_get_min_clock(struct sdhci_host *host)
{
	return SDHCI_MSHC_MIN_FREQ;
}

static unsigned int sdhci_mshc_get_timeout_clock(struct sdhci_host *host)
{
	u32 div;
	unsigned long freq;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_mshc_data *sdhci_mshc = pltfm_host->priv;

	div = sdhci_readl(host, SDHCI_CLOCK_CONTROL);
	div = (div & CLK_CTRL_TIMEOUT_MASK) >> CLK_CTRL_TIMEOUT_SHIFT;

	freq = clk_get_rate(sdhci_mshc->clk);
	freq /= 1 << (CLK_CTRL_TIMEOUT_MIN_EXP + div);

	pr_debug("%s: freq=%lx\n", __func__, freq);

	return freq;
}

static void sdhci_mshc_hardware_reset(struct sdhci_host *host)
{
	u32 count;

	if (!host->sysctrl) {
		pr_err("%s: sysctrl is null, can't reset mmc!\n", __func__);
		return;
	}

	/* eMMC reset */
	count = 0;
	sdhci_sctrl_writel(host, IP_RST_EMMC, SCTRL_SCPERRSTEN0);
	while(!(IP_RST_EMMC & sdhci_sctrl_readl(host, SCTRL_SCPERRSTSTAT0))) {
		if (count > 0xFFF) {
			pr_err("emmc reset timeout\n");
			break;
		}
		count++;
	}

	/* phy reset */
	sdhci_mmc_sys_writel(host, IP_PRST_EMMC_PHY_MASK, EMMC_SYS_CRG_CFG1);
	if (host->quirks2 & SDHCI_QUIRK2_HISI_COMBO_PHY_TC) {
		sdhci_i2c_writel(host, IP_RST_PHY_APB, SC_EMMC_RSTEN);
		mdelay(1);
	}

	/* phy dis-reset */
	sdhci_mmc_sys_writel(host, IP_PRST_EMMC_PHY | IP_PRST_EMMC_PHY_MASK,
				EMMC_SYS_CRG_CFG1);
	if (host->quirks2 & SDHCI_QUIRK2_HISI_COMBO_PHY_TC) {
		sdhci_i2c_writel(host, IP_RST_PHY_APB, SC_EMMC_RSTDIS);
		mdelay(1);
	}

	/* eMMC dis-reset */
	count = 0;
	sdhci_sctrl_writel(host, IP_RST_EMMC, SCTRL_SCPERRSTDIS0);
	while(IP_RST_EMMC & sdhci_sctrl_readl(host, SCTRL_SCPERRSTSTAT0)) {
		if (count > 0xFFF) {
			pr_err("emmc dis-reset timeout\n");
			break;
		}
		count++;
	}
}


void sdhci_combo_phy_delay_measurement(struct sdhci_host *host)
{
	u32 count;
	u32 reg_val;
	u32 delaymeas_code;

	reg_val = sdhci_phy_readl(host, COMBO_PHY_PHYINITCTRL);
	/*bit0:dlymeas_en, bit2:init_en*/
	reg_val |= (INIT_EN | DLYMEAS_EN);
	sdhci_phy_writel(host, reg_val, COMBO_PHY_PHYINITCTRL);

	reg_val =  sdhci_phy_readl(host, COMBO_PHY_PHYCTRL2);
	/*phy_dlymeas_update on*/
	reg_val |= PHY_DLYMEAS_UPDATE;
	sdhci_phy_writel(host, reg_val, COMBO_PHY_PHYCTRL2);

	count = 0;
	do {
		if (count > 1) {
			pr_err("%s wait for delay measurement done timeout\n", __func__);
			break;
		}
		mdelay(1);
		count++;
		reg_val = sdhci_phy_readl(host, COMBO_PHY_PHYINITCTRL);
	} while (reg_val & INIT_EN);

	reg_val = sdhci_phy_readl(host, COMBO_PHY_DLY_CTL);
	delaymeas_code = reg_val & DLY_CPDE_1T_MASK;

	reg_val = sdhci_phy_readl(host, COMBO_PHY_PHYCTRL2);
	/*phy_dlymeas_update off at dlymeas_done*/
	reg_val &= ~PHY_DLYMEAS_UPDATE;
	sdhci_phy_writel(host, reg_val, COMBO_PHY_PHYCTRL2);

	/*set tx_clk as 1/4T*/
	sdhci_phy_readl(host, COMBO_PHY_DLY_CTL1);
	reg_val &= ~(DLY_1_CODE_MASK << DLY_1_CODE_SHIFT);
	delaymeas_code = delaymeas_code>>2;
	reg_val |= ((DLY_1_CODE_MASK & (delaymeas_code)) << DLY_1_CODE_SHIFT);
	sdhci_phy_writel(host, reg_val, COMBO_PHY_DLY_CTL1);
}

static void sdhci_mshc_update_phy_control(struct sdhci_host *host, unsigned char timing)
{
	//sdhci_combo_phy_delay_measurement(host);
}

static void sdhci_combo_phy_zq_cal(struct sdhci_host *host)
{
	u32 count;
	u32 reg_val;

	reg_val = sdhci_phy_readl(host, COMBO_PHY_IMPCTRL);
	/*zcom_num set as d'20*/
	reg_val &= (~(IMPCTRL_ZCOM_NUM_MASK << IMPCTRL_ZCOM_NUM_SHIFT));
	reg_val |= ((IMPCTRL_ZCOM_NUM_MASK & 0x14) << IMPCTRL_ZCOM_NUM_SHIFT);
	/*zcom_rsp_dly set as d'60*/
	reg_val &= ~(IMPCTRL_ZCOM_RSP_DLY_MASK);
	reg_val |= (IMPCTRL_ZCOM_RSP_DLY_MASK & 0x3C);
	sdhci_phy_writel(host, reg_val, COMBO_PHY_IMPCTRL);

	reg_val = sdhci_phy_readl(host, COMBO_PHY_PHYINITCTRL);
	/*disable delay measuremnt to wait zq calc ready*/
	reg_val &= ~DLYMEAS_EN;
	/*zcal_en set 1 to start impedance cal*/
	reg_val |= ZCAL_EN;
	/*int_en & zcal_en all set to 1*/
	reg_val |= INIT_EN;
	sdhci_phy_writel(host, reg_val, COMBO_PHY_PHYINITCTRL);

	count = 0;
	do {
		if (count > 3) {
			pr_err("%s wait  timeout\n", __func__);
			break;
		}
		mdelay(1);
		count++;
		reg_val = sdhci_phy_readl(host, COMBO_PHY_PHYINITCTRL);
	} while (reg_val & (INIT_EN |ZCAL_EN));

	pr_err("%s the IMPSTATUS is %x\n", __func__,
			sdhci_phy_readl(host, COMBO_PHY_IMPSTATUS));
}

static void sdhci_combo_phy_init(struct sdhci_host *host)
{
	u32 reg_val;

	reg_val = PUPD_EN_DATA | PUPD_EN_STROBE |PUPD_EN_CMD
			| PULLUP_DATA | PULLUP_CMD;
	sdhci_phy_writel(host, reg_val, COMBO_PHY_IOCTL_PUPD);

	/*set drv 33o*/
	reg_val = EMMC_RONSEL_1;
	sdhci_phy_writel(host, reg_val, COMBO_PHY_IOCTL_RONSEL_1);
	reg_val = sdhci_phy_readl(host, COMBO_PHY_IOCTL_RONSEL_2);
	reg_val |= EMMC_RONSEL_2;
	sdhci_phy_writel(host, reg_val, COMBO_PHY_IOCTL_RONSEL_2);

	/*enable phy input output*/
	reg_val = DA_EMMC_E | DA_EMMC_IE;
	sdhci_phy_writel(host, reg_val, COMBO_PHY_IOCTL_IOE);

	reg_val = sdhci_phy_readl(host, COMBO_PHY_DLY_CTL1);
	reg_val |= INV_TX_CLK;
	sdhci_phy_writel(host, reg_val, COMBO_PHY_DLY_CTL1);

	/*do ZQ Calibration and so on, to be finish*/
	sdhci_combo_phy_zq_cal(host);
}

void sdhci_mshc_OD_enable(struct sdhci_host *host)
{

	/* configure OD enable */

	pr_debug("%s: end!\n", __func__);
}

int sdhci_disable_open_drain(struct sdhci_host *host)
{

	pr_debug("%s\n", __func__);
	return 0;
}

static int sdhci_of_mshc_enable_enhanced_strobe(struct sdhci_host *host)
{
	return 0;
}

static void sdhci_mshc_hw_reset(struct sdhci_host *host)
{
	sdhci_mshc_hardware_reset(host);
	sdhci_combo_phy_init(host);
}

void sdhci_chk_busy_before_send_cmd(struct sdhci_host *host,
	struct mmc_command* cmd)
{
	unsigned long timeout;

	/* We shouldn't wait for busy for stop commands */
	if ((cmd->opcode != MMC_STOP_TRANSMISSION) && (cmd->opcode != MMC_SEND_STATUS)) {
		/* Wait busy max 10 s */
		timeout = 10000;
		while (!(sdhci_readl(host, SDHCI_PRESENT_STATE) & SDHCI_DATA_0_LVL_MASK)) {
			if (timeout == 0) {
				pr_err("%s: wait busy 10s time out.\n", mmc_hostname(host->mmc));
				sdhci_dumpregs(host);
				cmd->error = -EIO;
				tasklet_schedule(&host->finish_tasklet);
				return;
			}
			timeout--;
			mdelay(1);
		}
	}
	return;
}

int sdhci_mshc_enable_dma(struct sdhci_host *host)
{
	u16 ctrl;

	if (!(host->quirks2 & SDHCI_QUIRK2_BROKEN_64_BIT_DMA)){
		ctrl = sdhci_readw(host, SDHCI_HOST_CONTROL2);
		ctrl |= BIT(13);
		sdhci_writew(host, ctrl, SDHCI_HOST_CONTROL2);
	}
	return 0;
}

extern void sdhci_set_transfer_irqs(struct sdhci_host *host);
static void sdhci_mshc_restore_transfer_para(struct sdhci_host *host)
{
	u8 ctrl;
	u16 mode;

	sdhci_mshc_enable_dma(host);

	/*
	 * Always adjust the DMA selection as some controllers
	 * (e.g. JMicron) can't do PIO properly when the selection
	 * is ADMA.
	 */
	if (host->version >= SDHCI_SPEC_200) {
		ctrl = sdhci_readb(host, SDHCI_HOST_CONTROL);
		ctrl &= ~SDHCI_CTRL_DMA_MASK;
		if ((host->flags & SDHCI_REQ_USE_DMA) &&
			(host->flags & SDHCI_USE_ADMA)) {
			if (host->flags & SDHCI_USE_64_BIT_DMA)
				ctrl |= SDHCI_CTRL_ADMA64;
			else
				ctrl |= SDHCI_CTRL_ADMA32;
		} else {
			ctrl |= SDHCI_CTRL_SDMA;
		}
		sdhci_writeb(host, ctrl, SDHCI_HOST_CONTROL);
	}
	sdhci_set_transfer_irqs(host);

	mode = SDHCI_TRNS_BLK_CNT_EN;
	mode |= SDHCI_TRNS_MULTI;
	if (host->flags & SDHCI_REQ_USE_DMA)
		mode |= SDHCI_TRNS_DMA;
	sdhci_writew(host, mode, SDHCI_TRANSFER_MODE);
}

void sdhci_mshc_select_card_type(struct sdhci_host *host)
{
	u16 reg_val;

	reg_val = sdhci_readw(host, EMMC_CTRL_R);
	reg_val |= CARD_IS_EMMC;
	sdhci_writew(host, reg_val, EMMC_CTRL_R);
}

static struct sdhci_ops sdhci_mshc_ops = {
	.get_min_clock = sdhci_mshc_get_min_clock,
	.set_clock = sdhci_set_clock,
	.enable_dma = sdhci_mshc_enable_dma,
	.get_max_clock = sdhci_pltfm_clk_get_max_clock,
	.get_timeout_clock = sdhci_mshc_get_timeout_clock,
	.set_bus_width = sdhci_set_bus_width,
	.reset = sdhci_reset,
	.set_uhs_signaling = sdhci_set_uhs_signaling,
	.update_phy_control = sdhci_mshc_update_phy_control,
	.enable_enhanced_strobe = sdhci_of_mshc_enable_enhanced_strobe,
	.check_busy_before_send_cmd = sdhci_chk_busy_before_send_cmd,
	.restore_transfer_para = sdhci_mshc_restore_transfer_para,
	.hw_reset = sdhci_mshc_hw_reset,
	.select_card_type = sdhci_mshc_select_card_type,
	.dumpregs = sdhci_mshc_dumpregs,
};

static struct sdhci_pltfm_data sdhci_mshc_pdata = {
	.ops = &sdhci_mshc_ops,
	.quirks = SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN,
	.quirks2 = SDHCI_QUIRK2_PRESET_VALUE_BROKEN |
			SDHCI_QUIRK2_USE_1_8_V_VMMC,
};

#ifdef CONFIG_PM_SLEEP
/**
 * sdhci_mshc_suspend - Suspend method for the driver
 * @dev:	Address of the device structure
 * Returns 0 on success and error value on error
 *
 * Put the device in a low power state.
 */
static int sdhci_mshc_suspend(struct device *dev)
{
	return 0;
}

/**
 * sdhci_mshc_resume - Resume method for the driver
 * @dev:	Address of the device structure
 * Returns 0 on success and error value on error
 *
 * Resume operation after suspend
 */
static int sdhci_mshc_resume(struct device *dev)
{

	return 0;
}
#endif /* ! CONFIG_PM_SLEEP */

#ifdef CONFIG_PM
static int sdhci_mshc_runtime_suspend(struct device *dev)
{
	return 0;
}

static int sdhci_mshc_runtime_resume(struct device *dev)
{

	return 0;
}
#endif

static const struct dev_pm_ops sdhci_mshc_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(sdhci_mshc_suspend, sdhci_mshc_resume)
	SET_RUNTIME_PM_OPS(sdhci_mshc_runtime_suspend, sdhci_mshc_runtime_resume,
						   NULL)
};

static int sdhci_mshc_get_resource(struct platform_device *pdev, struct sdhci_host *host)
{
	struct device_node *np = NULL;
	struct resource *mem_res;

	/* get resource of mmc phy ctrl */
	mem_res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	host->mmc_phy = devm_ioremap_resource(&pdev->dev, mem_res);
	if (!host->mmc_phy) {
		dev_err(&pdev->dev, "cannot ioremap for mmc phy ctrl register\n");
		return -ENOMEM;
	}

	/* get resource of mmc sys ctrl */
	mem_res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	host->mmc_sys = devm_ioremap_resource(&pdev->dev, mem_res);
	if (!host->mmc_sys) {
		dev_err(&pdev->dev, "cannot ioremap for mmc sys ctrl register\n");
		return -ENOMEM;
	}

	np = of_find_compatible_node(NULL, NULL, "hisilicon,sysctrl");
	if (!np) {
		dev_err(&pdev->dev,
			"can't find device node \"hisilicon,sysctrl\"\n");
		return -ENXIO;
	}

	host->sysctrl = of_iomap(np, 0);
	if (!host->sysctrl) {
		printk("sysctrl iomap error\n");
		return -ENOMEM;
	}

	return 0;
}


void sdhci_retry_req(struct sdhci_host *host,struct mmc_request *mrq)
{
	return;
}

void sdhci_set_vmmc_power(struct sdhci_host *host, unsigned short vdd)
{
	struct mmc_host *mmc = host->mmc;

	if (host->quirks2 & SDHCI_QUIRK2_USE_1_8_V_VMMC) {
		if (vdd == 0) {
			if (mmc->regulator_enabled) {
				if (!regulator_disable(mmc->supply.vmmc))
					mmc->regulator_enabled = false;
				else
					pr_err("%s: regulator vmmc disable failed !\n", __func__);
			}
		}
		else {
			if (regulator_set_voltage(mmc->supply.vmmc, 2950000,
						    2950000)) {
				pr_err("%s: regulator vmmc set_voltage failed !\n", __func__);
			}
			if (!mmc->regulator_enabled) {
				if (!regulator_enable(mmc->supply.vmmc))
					mmc->regulator_enabled = true;
				else
					pr_err("%s: regulator vmmc enable failed !\n", __func__);
			}
		}
	}
	else {
		mmc_regulator_set_ocr(mmc, mmc->supply.vmmc, vdd);
	}
}

static void sdhci_mshc_populate_dt(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	 struct sdhci_host *host = platform_get_drvdata(pdev);

	if (!np) {
		dev_err(&pdev->dev, "can not find device node\n");
		return;
	}

	if (of_get_property(np, "use-combo-phy-tc", NULL)) {
		host->quirks2 |= SDHCI_QUIRK2_HISI_COMBO_PHY_TC;
		pr_err("use combo phy testchip\n");
	}

}

static void sdhci_mshc_data_init(struct sdhci_mshc_data *sdhci_mshc)
{
	sdhci_mshc->enhanced_strobe_enabled = 0;
	sdhci_mshc->tuning_current_sample = -1;
	sdhci_mshc->tuning_init_sample = 0xff;
	sdhci_mshc->tuning_sample_count = 0;
	sdhci_mshc->tuning_move_sample = 0xff;
	sdhci_mshc->tuning_move_count = 0;
	sdhci_mshc->tuning_strobe_init_sample = 0;	/* 0x0, 0x1, 0x2, 0x3 */
	sdhci_mshc->tuning_strobe_move_sample = 0xff;
	sdhci_mshc->tuning_strobe_move_count = 0;
	sdhci_mshc->tuning_sample_flag = 0;
}

static int sdhci_mshc_clock_init(struct platform_device *pdev, struct sdhci_mshc_data *sdhci_mshc)
{
	int ret;
	struct device_node *np = pdev->dev.of_node;

	sdhci_mshc->clk_ahb = devm_clk_get(&pdev->dev, "clk_ahb");
	if (IS_ERR(sdhci_mshc->clk_ahb)) {
		dev_err(&pdev->dev, "clk_ahb clock not found.\n");
		ret = PTR_ERR(sdhci_mshc->clk_ahb);
		return ret;
	}

	sdhci_mshc->clk = devm_clk_get(&pdev->dev, "clk_xin");
	if (IS_ERR(sdhci_mshc->clk)) {
		dev_err(&pdev->dev, "clk_xin clock not found.\n");
		ret = PTR_ERR(sdhci_mshc->clk);
		return ret;
	}

	if (of_device_is_available(np)) {
		if (of_property_read_u32(np, "clk-ahb", &ahb_clk))
			dev_err(&pdev->dev, "clk-ahb cannot get from dts, use the default value!\n");

		if (of_property_read_u32(np, "clk-xin", &xin_clk))
			dev_err(&pdev->dev, "clk-xin  cannot get from dts, use the default value!\n");
	}

	ret = clk_set_rate(sdhci_mshc->clk_ahb, ahb_clk);
	if (ret)
		dev_err(&pdev->dev, "Error setting desired ahb_clk=%u, get clk=%lu.\n",
			ahb_clk, clk_get_rate(sdhci_mshc->clk_ahb));
	ret = clk_prepare_enable(sdhci_mshc->clk_ahb);
	if (ret) {
		dev_err(&pdev->dev, "Unable to enable AHB clock.\n");
		return ret;
	}

	ret = clk_set_rate(sdhci_mshc->clk, xin_clk);
	if (ret)
		dev_err(&pdev->dev, "Error setting desired xin_clk=%u, get clk=%lu.\n",
			xin_clk, clk_get_rate(sdhci_mshc->clk));
	ret = clk_prepare_enable(sdhci_mshc->clk);
	if (ret) {
		dev_err(&pdev->dev, "Unable to enable SD clock.\n");
		clk_disable_unprepare(sdhci_mshc->clk_ahb);
		return ret;

	}

	pr_info("%s: clk_ahb=%lu, clk_xin=%lu\n", __func__,
		clk_get_rate(sdhci_mshc->clk_ahb), clk_get_rate(sdhci_mshc->clk));

	return 0;
}


#ifdef CONFIG_MMC_CQ_HCI
#define HW_CMDQ_REG_OFF                0x180
static void sdhci_mshc_cmdq_init(struct platform_device *pdev)
{
	int ret;
	struct sdhci_host *host = platform_get_drvdata(pdev);

	if (host->mmc->caps2 & MMC_CAP2_CMD_QUEUE) {
		host->cq_host = cmdq_pltfm_init(pdev, host->ioaddr + HW_CMDQ_REG_OFF);
		if (IS_ERR(host->cq_host)) {
			ret = PTR_ERR(host->cq_host);
			dev_err(&pdev->dev, "cmd queue platform init failed (%u)\n", ret);
			host->mmc->caps2 &= ~MMC_CAP2_CMD_QUEUE;
		}

		if (!(host->quirks2 & SDHCI_QUIRK2_BROKEN_64_BIT_DMA)) {
			host->cq_host->caps |= CMDQ_TASK_DESC_SZ_128;
		}
	}
}
#endif

/**
 * BUG: device rename krees old name, which would be realloced for other
 * device, pdev->name points to freed space, driver match may cause a panic
 * for wrong device
 */
static int sdhci_rename(struct platform_device *pdev)
{
	int ret = 0;
	static const char *const hi_mci0 = "hi_mci.0";
	struct sdhci_host *host = platform_get_drvdata(pdev);

	pdev->name = hi_mci0;
	ret = device_rename(&pdev->dev, hi_mci0);
	if (ret < 0) {
		dev_err(&pdev->dev, "dev set name hi_mci.0 fail \n");
		/* if failed, keep pdev->name same to dev.kobj.name */
		pdev->name = pdev->dev.kobj.name;
	}
	/* keep pdev->name pointing to dev.kobj.name */
	pdev->name = pdev->dev.kobj.name;
	host->hw_name = dev_name(&pdev->dev);

#ifdef CONFIG_HISI_BOOTDEVICE
	if (get_bootdevice_type() == BOOT_DEVICE_EMMC)
		set_bootdevice_name(&pdev->dev);
#endif

	return ret;
}

static int sdhci_mshc_probe(struct platform_device *pdev)
{
	int ret;
	struct sdhci_host *host;
	struct sdhci_pltfm_host *pltfm_host;
	struct sdhci_mshc_data *sdhci_mshc;

	if (get_bootdevice_type() != BOOT_DEVICE_EMMC) {
		pr_err("not boot from emmc\n");
		return -ENODEV;
	}

	host = sdhci_pltfm_init(pdev, &sdhci_mshc_pdata, sizeof(*sdhci_mshc));
	if (IS_ERR(host))
		return PTR_ERR(host);

	ret = sdhci_mshc_get_resource(pdev, host);
	if (ret)
		goto err_pltfm_free;

	pltfm_host = sdhci_priv(host);
	sdhci_mshc = sdhci_pltfm_priv(pltfm_host);

	ret = sdhci_mshc_clock_init(pdev, sdhci_mshc);
	if (ret)
		goto err_pltfm_free;

	pltfm_host->clk = sdhci_mshc->clk;
	sdhci_mshc_data_init(sdhci_mshc);
	sdhci_get_of_property(pdev);
	sdhci_mshc_populate_dt(pdev);

	sdhci_mshc_hardware_reset(host);

	sdhci_mshc_i2c_setup(pdev);

	sdhci_combo_phy_init(host);

#ifdef CONFIG_MMC_CQ_HCI
	sdhci_mshc_cmdq_init(pdev);
#endif

	if (host->mmc->pm_caps & MMC_PM_KEEP_POWER) {
		host->mmc->pm_flags |= MMC_PM_KEEP_POWER;
		host->quirks2 |= SDHCI_QUIRK2_HOST_OFF_CARD_ON;
	}

	/* import, ADMA support 64 or 32 bit address, here we use 32 bit. SDMA only support 32 bit mask. */
	if (!(host->quirks2 & SDHCI_QUIRK2_BROKEN_64_BIT_DMA)) {
		host->dma_mask = DMA_BIT_MASK(64);
	} else {
		host->dma_mask = DMA_BIT_MASK(32);
	}
	mmc_dev(host->mmc)->dma_mask = &host->dma_mask;

	ret = sdhci_rename(pdev);
	if (ret)
		goto clk_disable_all;

	ret = sdhci_add_host(host);
	if (ret)
		goto clk_disable_all;

	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);
	pm_runtime_set_autosuspend_delay(&pdev->dev, 50);
	pm_runtime_use_autosuspend(&pdev->dev);
	pm_suspend_ignore_children(&pdev->dev, 1);

	pr_info("%s: done!\n", __func__);

	return 0;

clk_disable_all:
	clk_disable_unprepare(sdhci_mshc->clk);
	clk_disable_unprepare(sdhci_mshc->clk_ahb);
err_pltfm_free:
	sdhci_pltfm_free(pdev);

	pr_err("%s: error = %d!\n", __func__, ret);

	return ret;
}

static int sdhci_mshc_remove(struct platform_device *pdev)
{
	struct sdhci_host *host = platform_get_drvdata(pdev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_mshc_data *sdhci_mshc = pltfm_host->priv;

	pr_debug("%s:\n", __func__);

	pm_runtime_get_sync(&pdev->dev);
	sdhci_remove_host(host, 1);
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	clk_disable_unprepare(sdhci_mshc->clk);
	clk_disable_unprepare(sdhci_mshc->clk_ahb);

	sdhci_pltfm_free(pdev);

	return 0;
}

static const struct of_device_id sdhci_mshc_of_match[] = {
	{.compatible = "mshc,sdhci"},
	{}
};

MODULE_DEVICE_TABLE(of, sdhci_mshc_of_match);

static struct platform_driver sdhci_mshc_driver = {
	.driver = {
			   .name = "sdhci-mshc",
			   .of_match_table = sdhci_mshc_of_match,
			   .pm = &sdhci_mshc_dev_pm_ops,
			   },
	.probe = sdhci_mshc_probe,
	.remove = sdhci_mshc_remove,
};

module_platform_driver(sdhci_mshc_driver);

MODULE_DESCRIPTION("Driver for the Synopsys SDHCI Controller");
MODULE_AUTHOR("Yongheng Qin <qinyongheng@huawei.com>");
MODULE_LICENSE("GPL");
