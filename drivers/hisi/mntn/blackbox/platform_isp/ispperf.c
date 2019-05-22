/*
 * Hisilicon ispperf server, ispperf.c
 *
 * Copyright (c) 2013 Hisilicon Technologies CO., Ltd.
 *
 */
/*lint -e715 -e838 -e529 -e438 -e30 -e142 -e528 -e750 -e753 -e754 -e785 -e655 -e749 -e732 -e747 -e708 -e712 -e64 -e666 -e713 -e665 -e40 -e774 -e845 -e578
 -esym(715,*) -esym(838,*) -esym(529,*) -esym(438,*) -esym(30,*) -esym(142,*) -esym(528,*) -esym(750,*) -esym(753,*) -esym(754,*) -esym(785,*) -esym(655,*) -esym(749,*) -esym(732,*) -esym(747,*) -esym(708,*) -esym(712,*) -esym(64,*) -esym(666,*) -esym(713,*) -esym(665,*) -esym(40,*) -esym(774,*) -esym(845,*) -esym(578,*)*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/atomic.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/ion.h>
#include <linux/hisi/hisi_ion.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/rproc_share.h>
#include <linux/platform_data/remoteproc-hisi.h>
#include <uapi/linux/histarisp.h>
#include "isprdr.h"

#define DEBUG_BIT   (1 << 2)
#define INFO_BIT    (1 << 1)
#define ERROR_BIT   (1 << 0)
static int kmsgcat_mask = (INFO_BIT | ERROR_BIT);
/*lint -e21 -e846 -e514 -e778 -e866 -e84*/
module_param_named(kmsgcat_mask, kmsgcat_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);
/*lint +e21 +e846 +e514 +e778 +e866 +e84*/
#define D(fmt, args...) \
    do { \
        if (kmsgcat_mask & DEBUG_BIT) \
            printk("[ispperf][%s] " fmt, __func__, ##args); \
    } while (0)
#define I(fmt, args...) \
    do { \
        if (kmsgcat_mask & INFO_BIT) \
            printk("[ispperf][%s] " fmt, __func__, ##args); \
    } while (0)
#define E(fmt, args...) \
    do { \
        if (kmsgcat_mask & ERROR_BIT) \
            printk("[ispperf][%s] " fmt, __func__, ##args); \
    } while (0)

#define IONCLIENT_NAME_SZ   (64)

struct ispperf_device_s {
    int initialized;
    atomic_t open_cnt;
    struct ion_client *ion_client;
    struct ion_handle *ion_handle;
    unsigned int local_perf;
    wait_queue_head_t wait;
} ispperf_dev;

struct ispperf_param_s {
    unsigned int moduleaddr;
    unsigned int iova;
    unsigned int sharedfd;
    unsigned int type;
    unsigned int prot;
    unsigned int size;
};

#define HISPPERF_START    _IOWR('i', 0x1210, struct ispperf_param_s)
#define HISPPERF_STOP     _IOWR('i', 0x1211, struct ispperf_param_s)

static int ispperf_open(struct inode *inode, struct file *filp)
{
    struct ispperf_device_s *dev = (struct ispperf_device_s *)&ispperf_dev;

    D("[%s] +\n", __func__);
    if (!dev->initialized) {
        pr_err("[%s] Failed : ispperf not ready\n", __func__);
        return -ENXIO;
    }

    if (0 != atomic_read(&dev->open_cnt)) {
        pr_err("[%s] Failed : opened.0x%x\n", __func__, (unsigned int)atomic_read(&dev->open_cnt));
        return -EBUSY;
    }

    atomic_inc(&dev->open_cnt);
    D("[%s] -\n", __func__);

    return 0;
}

static long ispperf_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct ispperf_device_s *dev = (struct ispperf_device_s *)&ispperf_dev;
    struct ispperf_param_s params;
    struct rproc_shared_para *param;
    int ret = 0, timeout = 10000;

    D("+ cmd.0x%x\n", cmd);
    if (!dev->initialized) {
        pr_err("[%s] Failed : ispperf not ready\n", __func__);
        return -ENXIO;
    }

    if ((param = rproc_get_share_para()) == NULL)
        return -EINVAL;

    if ((void __user *)arg == NULL) {
        pr_err("[%s] Failed : cmd.0x%x, arg.0x%lx\n", __func__, cmd, arg);
        return -EINVAL;
    }

    if ((ret = copy_from_user(&params, (void __user *)arg, (int)sizeof(params))) < 0) {
        pr_err("[%s] Failed : copy_from_user.%d\n", __func__, ret);
        return -EFAULT;
    }

    D("params  sharefd = 0x%x, size = 0x%x, prot = %x, type = 0x%x\n",
        params.sharedfd, params.size, params.prot, params.type);

    switch(cmd) {
        case HISPPERF_START: {
            param->perf_addr    = params.iova;
            param->perf_switch  = ISPCPU_PERF_RECORD;
            D("[%s] perf_addr.0x%llx, perf_switch.0x%x\n", __func__, param->perf_addr, param->perf_switch);
            do {
                ret = wait_event_interruptible_timeout(dev->wait, param->perf_switch == 0, msecs_to_jiffies(10));
                if(ret < 0) {
                    E("ret =%d, param->perf_switch = 0x%x, timeout = %d",
                        ret, param->perf_switch , timeout);
                }
            } while((param->perf_switch == ISPCPU_PERF_RECORD) && timeout-- > 0);
            break;
        }
        case HISPPERF_STOP:
            param->perf_switch  = 0;
            break;
        default:
            pr_err("[%s] Failed : Defaule cmd.%d\n", __func__, cmd);
            break;
    }
    D("- cmd.0x%x\n", cmd);

    return 0;
}

void ispperf_stop_record(void)
{
    struct ispperf_device_s *dev = (struct ispperf_device_s *)&ispperf_dev;
    struct rproc_shared_para *param;

    D("[%s] +\n", __func__);
    if (!dev->initialized) {
        pr_err("[%s] Failed : ispperf not ready\n", __func__);
        return ;
    }

    if ((param = rproc_get_share_para()) == NULL)
        return;

    param->perf_switch  = 0;
    wake_up_interruptible(&dev->wait);
    D("[%s] -\n", __func__);
}

static int ispperf_release(struct inode *inode, struct file *filp)
{
    struct ispperf_device_s *dev = (struct ispperf_device_s *)&ispperf_dev;

    D("[%s] +\n", __func__);
    if (!dev->initialized) {
        pr_err("[%s] Failed : ispperf not ready\n", __func__);
        return -ENXIO;
    }

    if (0 >= atomic_read(&dev->open_cnt)) {
        pr_err("[%s] Failed : release.0x%x\n", __func__, (unsigned int)atomic_read(&dev->open_cnt));
        return -ENXIO;
    }
    atomic_dec(&dev->open_cnt);
    D("[%s] -\n", __func__);

    return 0;
}

struct level_switch_s perftable[] = {
    {
    ISPCPU_PERF_RECORD, "enable", "disable", "perf record"}, {
    ISPCPU_PERF_RESERVE_30, "enable", "disable", "reserved 30"}, {
    ISPCPU_PERF_RESERVE_29, "enable", "disable", "reserved 29"}, {
    ISPCPU_PERF_YUVNF, "enable", "disable", "YUVNF"}, {
    ISPCPU_PERF_WARP, "enable", "disable", "WARP"}, {
    ISPCPU_PERF_TNR, "enable", "disable", "TNR"}, {
    ISPCPU_PERF_STAT3A, "enable", "disable", "STAT3A"}, {
    ISPCPU_PERF_SCALER, "enable", "disable", "SCALER"}, {
    ISPCPU_PERF_RGB2YUV, "enable", "disable", "RGB2YUV"}, {
    ISPCPU_PERF_RAWNF, "enable", "disable", "RAWNF"}, {
    ISPCPU_PERF_PDAF, "enable", "disable", "PDAF"}, {
    ISPCPU_PERF_OIS, "enable", "disable", "OIS"}, {
    ISPCPU_PERF_MINILSC, "enable", "disable", "MINILSC"}, {
    ISPCPU_PERF_LUT3D, "enable", "disable", "LUT3D"}, {
    ISPCPU_PERF_LSC, "enable", "disable", "LSC"}, {
    ISPCPU_PERF_LASER, "enable", "disable", "LASER"}, {
    ISPCPU_PERF_IF, "enable", "disable", "IF"}, {
    ISPCPU_PERF_GCD, "enable", "disable", "GCD"}, {
    ISPCPU_PERF_GAMMA, "enable", "disable", "GAMMA"}, {
    ISPCPU_PERF_FLASH, "enable", "disable", "FLASH"}, {
    ISPCPU_PERF_FD, "enable", "disable", "FD"}, {
    ISPCPU_PERF_DRC, "enable", "disable", "DRC"}, {
    ISPCPU_PERF_DPCC, "enable", "disable", "DPCC"}, {
    ISPCPU_PERF_DIS, "enable", "disable", "DIS"}, {
    ISPCPU_PERF_DEGAMMA, "enable", "disable", "DEGAMMA"}, {
    ISPCPU_PERF_CE, "enable", "disable", "CE"}, {
    ISPCPU_PERF_CC, "enable", "disable", "CC"}, {
    ISPCPU_PERF_BLC, "enable", "disable", "BLC"}, {
    ISPCPU_PERF_BAS, "enable", "disable", "BAS"}, {
    ISPCPU_PERF_AWB, "enable", "disable", "AWB"}, {
    ISPCPU_PERF_AF, "enable", "disable", "AF"}, {
    ISPCPU_PERF_AE, "enable", "disable", "AE"},};

void ispperfctrl_update(void)
{
    struct ispperf_device_s *dev = (struct ispperf_device_s *)&ispperf_dev;
    struct rproc_shared_para *param;

    if ((param = rproc_get_share_para()) == NULL)
        return;

    param->perf_switch = dev->local_perf;
    pr_info("[%s] perftable.0x%x >> online.0x%x\n", __func__,
        dev->local_perf, param->perf_switch);
}

static ssize_t ispperfctrl_show(struct device *pdev,
                   struct device_attribute *attr, char *buf)
{
    struct ispperf_device_s *dev = (struct ispperf_device_s *)&ispperf_dev;
    struct rproc_shared_para *param;
    unsigned int perf_switch = 0;
    char *s = buf;
    ssize_t size;
    int i;

    param = rproc_get_share_para();
    if (param != NULL)
        perf_switch = param->perf_switch;

    for (i = 0; i < (int)((int)sizeof(perftable) / (int)sizeof(struct level_switch_s)); i ++) {
        // cppcheck-suppress *
        s += sprintf(s, "[%s.%s] : %s\n",
                 (param ? ((perf_switch & perftable[i].level)
                 ? perftable[i].enable_cmd : perftable[i].disable_cmd) : "ispoffline"),
                 ((dev->local_perf & perftable[i].
                  level) ? perftable[i].enable_cmd : perftable[i].disable_cmd),
                 perftable[i].info);/*lint !e421 */
    }

    size = s - buf;
    return size;
}

static void usage_ispperfctrl(void)
{
    int i = 0;

    pr_info("<Usage: >\n");
    for (i = 0; i < (int)((int)sizeof(perftable) / (int)sizeof(struct level_switch_s)); i++)
        pr_info("echo <%s>:<%s/%s> > ispperfctrl\n", perftable[i].info,
               perftable[i].enable_cmd, perftable[i].disable_cmd);
}

static ssize_t ispperfctrl_store(struct device *pdev,
                struct device_attribute *attr, const char *buf,
                size_t count)
{
    struct ispperf_device_s *dev = (struct ispperf_device_s *)&ispperf_dev;
    int i = 0, len = 0, flag = 0;
    char *p = NULL;

    p = memchr(buf, ':', count);
    if (!p)
        return (ssize_t)count;

    len = (int)(p - buf);
    for (i = 0; i < (int)((int)sizeof(perftable) / (int)sizeof(struct level_switch_s)); i ++) {
        if (perftable[i].level == ISPCPU_PERF_RECORD)
            continue;
        if (!strncmp(buf, perftable[i].info, len)) {
            flag = 1;
            p += 1;
            if (!strncmp(p, perftable[i].enable_cmd,
                (int)strlen(perftable[i].enable_cmd)))
                dev->local_perf = perftable[i].level;
            else if (!strncmp(p, perftable[i].disable_cmd,
                  (int)strlen(perftable[i].disable_cmd)))
                dev->local_perf &= ~(perftable[i].level);
            else
                flag = 0;
            break;
        }
    }

    if (!flag)
        usage_ispperfctrl();

    ispperfctrl_update();

    return (ssize_t)count;
}
/*lint -e846 -e514 -e778 -e866 -e84*/
static DEVICE_ATTR(ispperfctrl, (S_IRUGO | S_IWUSR | S_IWGRP), ispperfctrl_show,
           ispperfctrl_store);
/*lint +e846 +e514 +e778 +e866 +e84*/
static const struct file_operations ispperf_ops = {
    .open           = ispperf_open,
    .unlocked_ioctl = ispperf_ioctl,
    .compat_ioctl   = ispperf_ioctl,
    .release        = ispperf_release,
    .owner          = THIS_MODULE,
};

static struct miscdevice ispperf_miscdev = {
    .minor  = 255,
    .name   = "isp_perf",
    .fops   = &ispperf_ops,
};

static int __init ispperf_init(void)
{
    struct ispperf_device_s *dev = (struct ispperf_device_s *)&ispperf_dev;
    int ret = 0;

    pr_info("[%s] +\n", __func__);
    dev->initialized = 0;

    if ((ret = misc_register((struct miscdevice *)&ispperf_miscdev)) != 0) {
        pr_err("[%s] Failed : misc_register.%d\n", __func__, ret);
        return ret;
    }

    ret = device_create_file(ispperf_miscdev.this_device,
                   &dev_attr_ispperfctrl);
    if (0 != ret) {
        pr_err("[%s] Faield : ispperfctrl device_create_file.%d\n", __func__,
               ret);
        return ret;
    }

    atomic_set(&dev->open_cnt, 0);
    dev->initialized = 1;
    init_waitqueue_head(&dev->wait);
    pr_info("[%s] -\n", __func__);

    return 0;
}

static void __exit ispperf_exit(void)
{
    struct ispperf_device_s *dev = (struct ispperf_device_s *)&ispperf_dev;

    pr_info("[%s] +\n", __func__);
    misc_deregister((struct miscdevice *)&ispperf_miscdev);
    dev->initialized = 0;
    pr_info("[%s] -\n", __func__);
}

module_init(ispperf_init);
module_exit(ispperf_exit);
MODULE_LICENSE("GPL v2");
