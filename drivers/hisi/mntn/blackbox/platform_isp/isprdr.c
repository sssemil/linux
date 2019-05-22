/*
 * record the data to rdr. (RDR: kernel run data recorder.)
 * This file wraps the ring buffer.
 *
 * Copyright (c) 2013 Hisilicon Technologies CO., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
/*lint -e715 -e838 -e529 -e438 -e30 -e142 -e528 -e750 -e753 -e754 -e785 -e655 -e749 -e732 -e747 -e708 -e712 -e64 -e661 -e574 -e737 -e713 -e826 -e530 -e570
 -esym(715,*) -esym(838,*) -esym(529,*) -esym(438,*) -esym(30,*) -esym(142,*) -esym(528,*) -esym(750,*) -esym(753,*) -esym(754,*) -esym(785,*) -esym(655,*) -esym(749,*) -esym(732,*) -esym(747,*) -esym(708,*) -esym(712,*) -esym(64,*) -esym(661,*) -esym(574,*) -esym(737,*) -esym(713,*) -esym(826,*) -esym(530,*) -esym(570,*)*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/printk.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/hisi/rdr_pub.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/rproc_share.h>
#include <linux/version.h>
#include <linux/of_irq.h>

#include "isprdr.h"
#include "rdr_print.h"
#include "rdr_inner.h"
#include "rdr_field.h"

#include <soc_acpu_baseaddr_interface.h>
#include <soc_isp_watchdog_interface.h>
#include <soc_sctrl_interface.h>
#include <soc_pctrl_interface.h>

#define SOC_ACPU_ISP_WDT_BASE_ADDR  SOC_ACPU_ISP_Watchdog_BASE_ADDR
enum RDR_ISP_SYSTEM_ERROR_TYPE {
    ISP_MODID_START = HISI_BB_MOD_ISP_START,
    ISP_WDT_TIMEOUT = 0x81fffdfe,
    ISP_SYSTEM_STATES,
    ISP_MODID_END = HISI_BB_MOD_ISP_END,
    ISP_SYSTEM_INVALID,
} rdr_isp_sys_err_t;

struct rdr_err_type {
    struct list_head node;
    enum RDR_ISP_SYSTEM_ERROR_TYPE type;
};

struct rdr_sys_err_dump_info {
    struct list_head node;
    u32 modid;
    u64 coreid;
    pfn_cb_dump_done cb;
};

struct rdr_isp_device {
    void __iomem *sctrl_addr;
    void __iomem *wdt_addr;
    void __iomem *pctrl_addr;
    void __iomem *rdr_addr;
    struct workqueue_struct *wq;
    struct work_struct err_work;
    struct work_struct dump_work;
    struct list_head err_list;
    struct list_head dump_list;
    spinlock_t err_list_lock;
    spinlock_t dump_list_lock;
    int wdt_irq;
    bool wdt_enable_flag;
    unsigned int offline_rdrlevel;
    unsigned char irq[IRQ_NUM];
    int isprdr_initialized;
    u64 isprdr_addr;
    u64 core_id;
    struct rdr_register_module_result current_info;
} rdr_isp_dev;

static struct rdr_exception_info_s exc_info[] = {
    [0] = {
           .e_modid = ISP_WDT_TIMEOUT,
           .e_modid_end = ISP_WDT_TIMEOUT,
           .e_process_priority = RDR_ERR,
           .e_reboot_priority = RDR_REBOOT_WAIT,
           .e_notify_core_mask = RDR_AP | RDR_ISP,
           .e_reset_core_mask = RDR_AP,
           .e_reentrant = RDR_REENTRANT_DISALLOW,
           .e_exce_type = ISP_S_ISPWD,
           .e_from_core = RDR_ISP,
           .e_from_module = MODULE_NAME,
           .e_desc = "RDR ISP WDT TIMEOUT",
           },
};

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 13, 0))
extern char *hisp_get_comp_name(void);
int hisp_get_wdt_irq(void)
{
    struct device_node *np = NULL;
    char *name = NULL;
    int irq = 0;

    name = hisp_get_comp_name();

    np = of_find_compatible_node(NULL, NULL, name);
    if (!np) {
        pr_err("%s: of_find_compatible_node failed, %s\n", __func__, name);
        return -ENXIO;
    }

    irq = irq_of_parse_and_map(np, 0);
    if (!irq) {
        pr_err("%s: irq_of_parse_and_map failed, irq.%d\n", __func__, irq);
        return -ENXIO;
    }

    pr_info("%s: comp.%s, wdt irq.%d.\n", __func__, name, irq);
    return irq;
}
#endif

u64 get_isprdr_addr(void)
{
    struct rdr_isp_device *dev = &rdr_isp_dev;
    pr_info("%s: isprdr_addr.0x%llx, isprdr_addr.0x%x",
            __func__, dev->isprdr_addr, (unsigned int)dev->isprdr_addr);

    return dev->isprdr_addr;
}

static void rdr_system_err_dump_work(struct work_struct *work)
{
    struct rdr_isp_device *dev = &rdr_isp_dev;
    struct rdr_sys_err_dump_info *entry, *tmp;
    unsigned int sync_word;
    int timeout = 20;

    list_for_each_entry_safe(entry, tmp, &dev->dump_list, node) {
        if (ISP_WDT_TIMEOUT == entry->modid) {
            /* check sync word */
            do {
                sync_word =
                    readl(dev->rdr_addr + RDR_SYNC_WORD_OFF);
                msleep(100);
            } while (RDR_ISP_SYNC != sync_word && timeout-- > 0);
            pr_info("%s: sync_word = 0x%x, timeout = %d.\n",
                __func__, sync_word, timeout);
        }
        entry->cb(entry->modid, entry->coreid);

        spin_lock(&dev->dump_list_lock);
        list_del(&entry->node);
        spin_unlock(&dev->dump_list_lock);

        kfree(entry);
    }
}
/*lint -save -e429*/
static void rdr_isp_dump(u32 modid, u32 etype, u64 coreid,
             char *pathname, pfn_cb_dump_done pfn_cb)
{
    struct rdr_isp_device *dev = &rdr_isp_dev;
    struct rdr_sys_err_dump_info *dump_info;
    pr_info("%s: enter.\n", __func__);

    dump_info = kzalloc(sizeof(struct rdr_sys_err_dump_info), GFP_KERNEL);
    if (!dump_info) {
        pr_err("%s: kzalloc failed.\n", __func__);
        return;
    }

    dump_info->modid = modid;
    dump_info->coreid = dev->core_id;
    dump_info->cb = pfn_cb;

    spin_lock(&dev->dump_list_lock);
    list_add_tail(&dump_info->node, &dev->dump_list);
    spin_unlock(&dev->dump_list_lock);

    queue_work(dev->wq, &dev->dump_work);
    pr_info("%s: exit.\n", __func__);
    return;
}
/*lint -restore */
static void rdr_isp_reset(u32 modid, u32 etype, u64 coreid)
{
    pr_info("%s: enter.\n", __func__);
    return;
}
/*lint -save -e429*/
static irqreturn_t isp_wdt_irq_handler(int irq, void *data)
{
    struct rdr_isp_device *dev = &rdr_isp_dev;
    struct rdr_err_type *err_info;
    pr_info("%s:enter.\n", __func__);

    /* disable wdt */
    writel(WDT_UNLOCK, SOC_ISP_WatchDog_WDG_LOCK_ADDR(dev->wdt_addr));
    writel(0, SOC_ISP_WatchDog_WDG_CONTROL_ADDR(dev->wdt_addr));
    writel(WDT_LOCK, SOC_ISP_WatchDog_WDG_LOCK_ADDR(dev->wdt_addr));

    /* init sync work */
    writel(0, dev->rdr_addr + RDR_SYNC_WORD_OFF);

    /* send fiq to isp_a7 */
    writel(0, SOC_PCTRL_PERI_CTRL27_ADDR(dev->pctrl_addr));
    writel(1, SOC_PCTRL_PERI_CTRL27_ADDR(dev->pctrl_addr));
    writel(0, SOC_PCTRL_PERI_CTRL27_ADDR(dev->pctrl_addr));

    err_info = kzalloc(sizeof(struct rdr_err_type), GFP_ATOMIC);
    if (!err_info) {
        pr_info("%s: kzalloc failed.\n", __func__);
        return IRQ_NONE;
    }

    err_info->type = ISP_WDT_TIMEOUT;

    spin_lock(&dev->err_list_lock);
    list_add_tail(&err_info->node, &dev->err_list);
    spin_unlock(&dev->err_list_lock);

    queue_work(dev->wq, &dev->err_work);
    pr_info("%s:exit.\n", __func__);
    return IRQ_HANDLED;
}
/*lint -restore */
static void rdr_system_err_work(struct work_struct *work)
{
    struct rdr_isp_device *dev = &rdr_isp_dev;
    struct rdr_err_type *entry, *tmp;

    list_for_each_entry_safe(entry, tmp, &dev->err_list, node) {
        if (ISP_WDT_TIMEOUT == entry->type)
            rdr_system_error(entry->type, dev->wdt_irq, 0);
        else
            rdr_system_error(entry->type, 0, 0);

        spin_lock_irq(&dev->err_list_lock);
        list_del(&entry->node);
        spin_unlock_irq(&dev->err_list_lock);

        kfree(entry);
    }
}

static int rdr_isp_wdt_init(struct rdr_isp_device *dev)
{
    int ret = 0;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 13, 0))
       int irq = 0;
#endif
    pr_info("%s: enter.\n", __func__);
    if (!dev->wdt_enable_flag) {
        pr_info("%s: isp wdt is disabled.\n", __func__);
        return 0;
    }

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 13, 0))
    irq = hisp_get_wdt_irq();
    if (irq <= 0) {
        pr_err("%s: hisp_get_wdt_irq failed, irq.0x%d\n", __func__, irq);
        return -EINVAL;
    }
    dev->wdt_irq = irq;
#else
    dev->wdt_irq = ISP_WDT_IRQ;
#endif

    ret =
        request_irq(dev->wdt_irq, isp_wdt_irq_handler, 0, "isp wtd hanler",
            NULL);
    if (0 != ret)
        pr_err("%s: request_irq failed, irq.%d, ret.%d.\n", __func__, dev->wdt_irq, ret);

    pr_info("%s: exit.\n", __func__);
    return 0;
}

static int rdr_isp_module_register(void)
{
    struct rdr_isp_device *dev = &rdr_isp_dev;
    struct rdr_module_ops_pub module_ops;
    struct rdr_register_module_result ret_info;
    int ret = 0;

    pr_info("%s: enter.\n", __func__);
    module_ops.ops_dump = rdr_isp_dump;
    module_ops.ops_reset = rdr_isp_reset;

    dev->core_id = RDR_ISP;
    ret = rdr_register_module_ops(dev->core_id, &module_ops, &ret_info);
    if (ret != 0) {
        pr_err("%s: rdr_register_module_ops failed! return %d\n",
               __func__, ret);
        return ret;
    }

    dev->current_info.log_addr = ret_info.log_addr;
    dev->current_info.log_len = ret_info.log_len;
    dev->current_info.nve = ret_info.nve;
    dev->isprdr_addr = ret_info.log_addr;

    pr_info("%s: log_addr = 0x%llx, log_len = 0x%x, nve = 0x%llx, isprdr_addr = 0x%llx\n",
         __func__, ret_info.log_addr, ret_info.log_len, ret_info.nve,
         dev->isprdr_addr);

    dev->rdr_addr = hisi_bbox_map((phys_addr_t)dev->isprdr_addr,
                                    dev->current_info.log_len);
    if (!dev->rdr_addr) {
        pr_err("%s: hisi_bbox_map rdr_addr failed.\n", __func__);
        return -ENOMEM;
    }

    dev->isprdr_initialized = 1;
    pr_info("%s: exit.\n", __func__);
    return 0;
}

static int rdr_isp_exception_register(struct rdr_isp_device *dev)
{
    int i, ret;

    pr_info("%s: enter.\n", __func__);
    for (i = 0; i < sizeof(exc_info) / sizeof(struct rdr_exception_info_s);
         i++) {
        pr_info("%s: register rdr exception, i = %d, type:%d", __func__,
            i, exc_info[i].e_exce_type);

        if (exc_info[i].e_modid == ISP_WDT_TIMEOUT)
            if (!dev->wdt_enable_flag)
                continue;

        ret = rdr_register_exception(&exc_info[i]);
        if (ret != exc_info[i].e_modid_end) {
            pr_info("%s: rdr_register_exception failed, ret.%d.\n",
                __func__, ret);
            return -EINVAL;
        }
    }

    pr_info("%s: exit.\n", __func__);
    return 0;
}

static int rdr_isp_dev_map(struct rdr_isp_device *dev)
{
    unsigned int value;
    bool wdt_flag = false;

    pr_info("%s: enter.\n", __func__);
    dev->wdt_enable_flag = wdt_flag;

    dev->sctrl_addr = ioremap((phys_addr_t) SOC_ACPU_SCTRL_BASE_ADDR, SZ_4K);
    if (!dev->sctrl_addr) {
        pr_err("%s: ioremp sctrl failed.\n", __func__);
        return -ENOMEM;
    }

    value = readl(SOC_SCTRL_SCBAKDATA10_ADDR(dev->sctrl_addr));

    if (value & (1 << SC_ISP_WDT_BIT))
        wdt_flag = true;

    if (wdt_flag) {
        dev->wdt_addr = ioremap((phys_addr_t) SOC_ACPU_ISP_WDT_BASE_ADDR, SZ_4K);
        if (!dev->wdt_addr) {
            pr_err("%s: ioremp wdt failed.\n", __func__);
            goto wdt_err;
        }

        dev->pctrl_addr = ioremap((phys_addr_t) SOC_ACPU_PCTRL_BASE_ADDR, SZ_4K);
        if (!dev->pctrl_addr) {
            pr_err("%s: ioremp pctrl failed.\n", __func__);
            goto pctrl_err;
        }
    }

    dev->wdt_enable_flag = wdt_flag;
    pr_info("%s: exit.\n", __func__);
    return 0;

pctrl_err:
    iounmap(dev->wdt_addr);
wdt_err:
    iounmap(dev->sctrl_addr);

    pr_info("%s: error, exit.\n", __func__);
    return -ENOMEM;
}



int find_irq_num(char *cmp, const char *input)
{
    unsigned long data;
    int ret, len_cmp;

    len_cmp = strlen(cmp);
    ret = kstrtoul((input + len_cmp), 0, &data);
    if (ret < 0) {
        pr_err("%s: strict_strtoul failed, ret.%d\n", __func__, ret);
        return -EINVAL;
    }

    pr_info("%s: number is %d.\n", __func__, (int)data);
    return (int)data;
}

struct level_switch_s rdrlevel[] = {
        {
        ISPCPU_RDR_USE_APCTRL, "yes", "no", "RDR Controlled by AP"}, {
        ISPCPU_RDR_RESERVE_30, "enable", "disable","reserved 30"}, {
        ISPCPU_RDR_RESERVE_29, "enable", "disable", "reserved 29"}, {
        ISPCPU_RDR_RESERVE_28, "enable", "disable", "reserved 28"}, {
        ISPCPU_RDR_RESERVE_27, "enable", "disable", "reserved 27"}, {
        ISPCPU_RDR_RESERVE_26, "enable", "disable", "reserved 26"}, {
        ISPCPU_RDR_RESERVE_25, "enable", "disable", "reserved 25"}, {
        ISPCPU_RDR_RESERVE_24, "enable", "disable", "reserved 24"}, {
        ISPCPU_RDR_RESERVE_23, "enable", "disable", "reserved 23"}, {
        ISPCPU_RDR_RESERVE_22, "enable", "disable", "reserved 22"}, {
        ISPCPU_RDR_RESERVE_21, "enable", "disable", "reserved 21"}, {
        ISPCPU_RDR_RESERVE_20, "enable", "disable", "reserved 20"}, {
        ISPCPU_RDR_RESERVE_19, "enable", "disable", "reserved 19"}, {
        ISPCPU_RDR_RESERVE_18, "enable", "disable", "reserved 18"}, {
        ISPCPU_RDR_RESERVE_17, "enable", "disable", "reserved 17"}, {
        ISPCPU_RDR_RESERVE_16, "enable", "disable", "reserved 16"}, {
        ISPCPU_RDR_RESERVE_15, "enable", "disable", "reserved 15"}, {
        ISPCPU_RDR_RESERVE_14, "enable", "disable", "reserved 14"}, {
        ISPCPU_RDR_RESERVE_13, "enable", "disable", "reserved 13"}, {
        ISPCPU_RDR_RESERVE_12, "enable", "disable", "reserved 12"}, {
        ISPCPU_RDR_RESERVE_11, "enable", "disable", "reserved 11"}, {
        ISPCPU_RDR_RESERVE_10, "enable", "disable", "reserved 10"}, {
        ISPCPU_RDR_RESERVE_9, "enable", "disable", "reserved 9"}, {
        ISPCPU_RDR_RESERVE_8, "enable", "disable", "reserved 8"}, {
        ISPCPU_RDR_RESERVE_7, "enable", "disable", "reserved 7"}, {
        ISPCPU_RDR_LEVEL_CPUP, "enable", "disable", "task cpup"}, {
        ISPCPU_RDR_LEVEL_TASK, "enable", "disable", "task switch"}, {
        ISPCPU_RDR_LEVEL_IRQ, "enable", "disable", "irq"}, {
        ISPCPU_RDR_LEVEL_CVDR, "enable", "disable", "cvdr"}, {
        ISPCPU_RDR_LEVEL_ALGO, "enable", "disable", "algo"}, {
        ISPCPU_RDR_LEVEL_LAST_WORD, "enable", "disable", "last word"}, {
        ISPCPU_RDR_LEVEL_TRACE, "enable", "disable", "trace"},};
static void usage_isprdrctrl(void)
{
    int i = 0;

    pr_info("<Usage: >\n");
    for (i = 0; i < (int)((int)sizeof(rdrlevel) / (int)sizeof(struct level_switch_s)); i ++) {
        if (rdrlevel[i].level == ISPCPU_RDR_USE_APCTRL)
            continue;
        pr_info("echo <%s>:<%s/%s> > rdr_isp\n", rdrlevel[i].info,
        rdrlevel[i].enable_cmd, rdrlevel[i].disable_cmd);
    }
    /* SIRQ Handle */
    pr_info("echo <%s>:<%s/%s> > rdr_isp\n", "sirqX",
        "enable", "disable");
    pr_info("sirqX,X[0-%d]\n",IRQ_NUM);
}
static ssize_t rdr_isp_store(struct device *pdev, struct device_attribute *attr,
                                                                        const char *buf, size_t count)
{
        int i = 0, len = 0, flag = 0;
        int irq_num;

        char *p = NULL;
        char *q = NULL;
        struct rdr_isp_device *dev = &rdr_isp_dev;
        struct rproc_shared_para *param =  rproc_get_share_para();;

        p = memchr(buf, ':', count);
        if(!p)
            return -EINVAL;
        len = p - buf;
        for (i = 0; i < (int)((int)sizeof(rdrlevel) / (int)sizeof(struct level_switch_s)); i ++) {
            if (rdrlevel[i].level == ISPCPU_RDR_USE_APCTRL)
                continue;

            if (!strncmp(buf, rdrlevel[i].info, len)) {
                flag = 1;
                p += 1;
                if (!strncmp(p, rdrlevel[i].enable_cmd,
                    (int)strlen(rdrlevel[i].enable_cmd)))
                    dev->offline_rdrlevel |= rdrlevel[i].level;
                else if (!strncmp(p, rdrlevel[i].disable_cmd,
                    (int)strlen(rdrlevel[i].disable_cmd)))
                    dev->offline_rdrlevel &= ~(rdrlevel[i].level);
                else
                    flag = 0;
                break;
            }
       }
       /*sirq should special handle  example:sirq32:disable or sirq32:enable*/
       if(!strncmp(buf, "sirq", (int)strlen("sirq")))
       {
            flag = 0;
            p += 1;

            q = kzalloc(strlen(buf)+1, GFP_KERNEL);
            if (!q) {
                pr_err("%s: kzalloc failed.\n", __func__);
                return -EINVAL;
            }
            memcpy(q,buf,len);
            q[len] ='\0';

            irq_num=find_irq_num("sirq", q);
            kzfree(q);


            if (irq_num < 0 || irq_num > IRQ_NUM) {
                pr_err("%s: SIRQ irq number overflow.\n", __func__);
                goto EXIT;
            }

            pr_info("%s: SIRQ .%d\n", __func__, irq_num);
            if (!param) {
                pr_err("%s:rproc_get_share_para failed.\n", __func__);
                goto EXIT;
            }

            if(!strncmp(p, "enable", (int)strlen("enable")))
            {
                flag =1 ;
                param->irq[irq_num] = 1;
                dev->irq[irq_num] = 1;
            }
            else if(!strncmp(p, "disable", (int)strlen("disable")))
            {
                flag=1;
                param->irq[irq_num] = 0;
                dev->irq[irq_num] = 0;
            }
            else
                flag = 0;
       }
EXIT:
    if (!flag)
        usage_isprdrctrl();

    if (param != NULL) {
        param->rdr_enable_type =
            (param->rdr_enable_type & ~ISPCPU_RDR_LEVEL_MASK) |
        (dev->offline_rdrlevel & ISPCPU_RDR_LEVEL_MASK);
        pr_info("[%s] rdrlevel.0x%x >> online.0x%x\n", __func__,
                            dev->offline_rdrlevel, param->rdr_enable_type);
    }

    return (ssize_t)count;

}
static ssize_t rdr_isp_show(struct device *pdev,
                                                                            struct device_attribute *attr, char *buf)
{
    struct rdr_isp_device *dev = &rdr_isp_dev;
    struct rproc_shared_para *param;
    unsigned int rdr_enable_type = 0;
    char *s = buf;
    ssize_t size;
    int i;

    param = rproc_get_share_para();
    if (param != NULL)
        rdr_enable_type = param->rdr_enable_type;

    for (i = 0; i < (int)((int)sizeof(rdrlevel) / (int)sizeof(struct level_switch_s)); i ++) {
                // cppcheck-suppress *
                s += sprintf(s, "[%s.%s] : %s\n",
                                        (param ? ((rdr_enable_type & rdrlevel[i].level)
                                        ? rdrlevel[i].enable_cmd : rdrlevel[i].disable_cmd) : "ispoffline"),
                                        ((dev->offline_rdrlevel & rdrlevel[i].
                                        level) ? rdrlevel[i].enable_cmd : rdrlevel[i].disable_cmd),
                                        rdrlevel[i].info);/*lint !e421 */
    }
    /*handle  sirq */
   for(i=1;i<IRQ_NUM;i++)
   {
        s += sprintf(s, "[%s.%s] : sirq%d\n",
            (param ?((param->irq[i])
             ?"enable":"disable"):"ispoffline"),
             ((dev->irq[i])?"enable":"disable"),
             i);/*lint !e421 */
   }

    size = s - buf;
    return size;
}
/*lint -e846 -e514 -e778 -e866 -e84*/
static DEVICE_ATTR(rdr_isp, (S_IRUGO | S_IWUSR | S_IWGRP), rdr_isp_show,
           rdr_isp_store);
/*lint +e846 +e514 +e778 +e866 +e84*/
static struct miscdevice isprdr_miscdev = {
    .minor = 255,
    .name = "isp_rdr",
};

static void rdr_isp_dev_unmap(struct rdr_isp_device *dev)
{
    iounmap(dev->sctrl_addr);

    if (dev->wdt_enable_flag) {
        iounmap(dev->wdt_addr);
        iounmap(dev->pctrl_addr);
    }

    return;
}

int __init rdr_isp_init(void)
{
    struct rdr_isp_device *dev = &rdr_isp_dev;
    int ret = 0;
    int i;

    pr_info("[%s] +\n", __func__);
    ret = rdr_isp_dev_map(dev);
    if (0 != ret) {
        pr_err("%s: rdr_isp_dev_map failed.\n", __func__);
        return ret;
    }

    ret = rdr_isp_wdt_init(dev);
    if (0 != ret) {
        pr_err("%s: rdr_isp_wdt_init failed.\n", __func__);
        return ret;
    }

    ret = rdr_isp_module_register();
    if (0 != ret) {
        pr_err("%s: rdr_isp_module_register failed.\n", __func__);
        return ret;
    }

    ret = rdr_isp_exception_register(dev);
    if (0 != ret) {
        pr_err("%s: rdr_isp_exception_register failed.\n", __func__);
        return ret;
    }

    dev->wq = create_singlethread_workqueue(MODULE_NAME);
    if (!dev->wq) {
        pr_err("%s: create_singlethread_workqueue failed.\n", __func__);
        return -1;
    }

    INIT_WORK(&dev->dump_work, rdr_system_err_dump_work);
    INIT_WORK(&dev->err_work, rdr_system_err_work);
    INIT_LIST_HEAD(&dev->err_list);
    INIT_LIST_HEAD(&dev->dump_list);

    spin_lock_init(&dev->err_list_lock);
    spin_lock_init(&dev->dump_list_lock);

    ret = misc_register((struct miscdevice *)&isprdr_miscdev);
    if (0 != ret) {
        pr_err("%s: misc_register failed, ret.%d.\n", __func__, ret);
        return ret;
    }

    ret = device_create_file(isprdr_miscdev.this_device, &dev_attr_rdr_isp);
    if (0 != ret) {
        pr_err("%s: Faield : isprdr device_create_file.%d\n",
            __func__, ret);
        return ret;
    }

    dev->offline_rdrlevel = ISPCPU_DEFAULT_RDR_LEVEL;
    for(i=0;i<IRQ_NUM;i++)
        dev->irq[i]=0;

    pr_info("[%s] -\n", __func__);

    return ret;
}

static void __exit rdr_isp_exit(void)
{
    struct rdr_isp_device *dev = &rdr_isp_dev;

    rdr_isp_dev_unmap(dev);
    destroy_workqueue(dev->wq);
    misc_deregister((struct miscdevice *)&isprdr_miscdev);

    return;
}

subsys_initcall(rdr_isp_init);
module_exit(rdr_isp_exit);
MODULE_LICENSE("GPL v2");
