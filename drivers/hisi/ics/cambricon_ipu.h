/*
 * Generic driver head file for the cambricon ipu device.
 *
 * Copyright (C) 2016 Cambricon Limited
 *
 * Licensed under the GPL v2 or later.
 */
#ifndef _CAMBRICON_IPU_H
#define _CAMBRICON_IPU_H

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/ioctl.h>
#include <linux/cdev.h>
#include <linux/semaphore.h>
#include <linux/version.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/compiler.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/pm_runtime.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/input.h>
#include <linux/wakelock.h>
#include <linux/clk.h>
#include <linux/hwspinlock.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include "ipu_smmu_drv.h"
#include "ipu_clock.h"

#include <asm/io.h>
#include <linux/ion.h>
#define COMP_CAMBRICON_IPU_DRV_NAME "hisilicon,cambricon-ipu"
/* ipu base address */
#define IPU_NAME	"ipu"

/* configure registers info */
#define IPU_CONF_REG_BASE	(IPU_BASE_ADDRESS + 0x00000000)
#define IPU_CONF_REG_SIZE	0x00100000
/* instruction RAM info */
#define IPU_INST_RAM_BASE	(IPU_BASE_ADDRESS + 0x00100000)
#define IPU_INST_RAM_SIZE	0x00100000

/* ipu configure register offset */
#define IPU_START_REG 0x18	/* IPU start up reg */
#define IPU_STATUS_REG 0x20	/* IPU payload finish status reg */
#define IPU_BASE_ADDR_REG 0x28	/* IPU access external DDR address */
#define IPU_SRAM_CTRL_REG 0x30	/* IPU internal SRAM configure reg */

/* reserved DDR memory info */
#define DMA_BUFFER_START 0x20c00000
#define DMA_BUFFER_SIZE	 (500 * 1024 * 1024)

/* Configuration Registers Operations: ioctl */
#define MAGIC_NUM	100
#define RDCONFIG_DWORD	_IOR(MAGIC_NUM, 3, unsigned int)
#define WRCONFIG_DWORD	_IOW(MAGIC_NUM, 6, unsigned int*)
#define SETCONFIG_MAP	_IOW(MAGIC_NUM, 7, unsigned int*)
#define SETCONFIG_UNMAP	_IOW(MAGIC_NUM, 8, unsigned int*)
#define SETCONFIG_RESET_VIRT_ADDR  _IOW(MAGIC_NUM, 9, unsigned int)
#define SETCONFIG_RESET_STATISTIC  _IOW(MAGIC_NUM, 10, unsigned int)
#define SETCONFIG_REPORT_STATISTIC _IOW(MAGIC_NUM, 11, unsigned int*)
#define SETCONFIG_UPDATE_PTE	   _IOW(MAGIC_NUM, 12, unsigned int)


struct irq_reg_offset {
	unsigned int ics_irq_base_addr;
	unsigned int ics_irq_mask_ns;
	unsigned int ics_irq_clr_ns;
};

struct ics_noc_bus_reg_offset {
	unsigned int base_addr;
	unsigned int qos_type;
	unsigned int factor;
	unsigned int saturation;
};

struct pmctrl_reg_offset {
	unsigned int base_addr;
	unsigned int noc_power_idle_req;
	unsigned int noc_power_idle_ack;
	unsigned int noc_power_idle_stat;
};

struct pctrl_reg_offset {
	unsigned int base_addr;
	unsigned int peri_stat3;
};

struct media2_reg_offset {
	unsigned int base_addr;
	unsigned int peren0;
	unsigned int perdis0;
	unsigned int perrsten0;
	unsigned int perrstdis0;
};

struct peri_reg_offset {
	unsigned int base_addr;
	unsigned int clkdiv18;
};

struct ics_feature_tree {
	bool finish_irq_expand_ns;
	bool finish_irq_expand_p;
	bool finish_irq_expand_s;
	bool finish_irq_to_hifi;
	bool finish_irq_to_ivp;
	bool finish_irq_to_isp;
	bool finish_irq_to_lpm3;
	bool finish_irq_to_iocmu;
	bool smmu_port_select;
	bool ipu_reset_when_in_error;
};

/* cambricon ipu private data */
struct cambricon_ipu_private
{
	const char *name;
	unsigned int irq;
	int ipu_open_count;

	/* config reg addr */
	unsigned int config_reg_length;
	phys_addr_t config_reg_phys_addr;
	void __iomem *config_reg_virt_addr;

	/* inst reg addr */
	unsigned int inst_ram_size;
	phys_addr_t inst_ram_phys_addr;
	void __iomem *inst_ram_virt_addr;

	/* ioremap addr */
	void __iomem *ics_irq_io_addr;
	void __iomem *noc_bus_io_addr;
	void __iomem *pmctrl_io_addr;
	void __iomem *pctrl_io_addr;
	void __iomem *media2_io_addr;
	void __iomem *peri_io_addr;

	struct semaphore config_reg_sem;
	struct semaphore inst_ram_sem;
	struct semaphore llseek_sem;

	/* char device */
	dev_t chrdev; /* ipu char device number */
	struct cdev	cdev; /* ipu char device */

	/* platform device resource */
	struct resource *inst_mem, *cfg_mem;
	struct regulator *vipu_ip;

	/* clock */
	struct clk *clock;
	unsigned int clock_start_rate;
	unsigned int clock_stop_rate;

	/* platform info, init in cambricon_ipu_probe from dtsi */
	bool ipu_cs_platform;

	unsigned long va_addr;
	unsigned long smmu_ttbr0;
	void *smmu_rw_err_phy_addr;

	struct irq_reg_offset irq_reg_offset;
	struct ics_noc_bus_reg_offset ics_noc_bus_reg_offset;
	struct pmctrl_reg_offset pmctrl_reg_offset;
	struct pctrl_reg_offset pctrl_reg_offset;
	struct media2_reg_offset media2_reg_offset;
	struct peri_reg_offset peri_reg_offset;

	struct ics_feature_tree feature_tree;
};


#endif
