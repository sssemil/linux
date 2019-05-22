/*
 * Remote Processor - Histar ISP remoteproc platform data.
 * include/linux/platform_data/remoteproc-hisi.h
 *
 * Copyright (c) 2013-2014 Hisilicon Technologies CO., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _PLAT_REMOTEPROC_HISI_ISP_H
#define _PLAT_REMOTEPROC_HISI_ISP_H

#include <linux/rproc_share.h>
#include <linux/firmware.h>
#include <linux/remoteproc.h>

struct rproc_ops;
struct platform_device;
struct rproc;
struct scatterlist;

/*
 * struct omap_rproc_pdata - omap remoteproc's platform data
 * @name: the remoteproc's name
 * @oh_name: omap hwmod device
 * @oh_name_opt: optional, secondary omap hwmod device
 * @firmware: name of firmware file to load
 * @mbox_name: name of omap mailbox device to use with this rproc
 * @ops: start/stop rproc handlers
 * @device_enable: omap-specific handler for enabling a device
 * @device_shutdown: omap-specific handler for shutting down a device
 * @set_bootaddr: omap-specific handler for setting the rproc boot address
 */

struct hisi_rproc_data {
    const char *name;
    const char *firmware;
	const char *bootware;
    const char *mbox_name;
    const struct rproc_ops *ops;
    unsigned int ipc_addr;
    int (*device_enable) (struct platform_device *pdev);
    int (*device_shutdown) (struct platform_device *pdev);
    void(*set_bootaddr)(u32);
};

enum a7mappint_type {
    A7BOOT = 0,
    A7TEXT,
    A7DATA,
    A7PGM,
    A7PTE,
    A7RDR,
    A7SHARED,
    A7VQ,
    A7VRING0,
    A7VRING1,
    A7HEAP,
    A7DYNAMIC,
    A7REGISP,
    A7REGIPCS,
    A7REGIPCNS,
    A7REGPCTRL,
    A7REGSCTRL,
    A7REGPCFG,
    A7REGGIC,
    A7REGSYSC,
    A7REGUART,
    A7REGGPIO,
    A7REGGPIO25,
    A7REGIOC,
    MAXA7MAPPING
};

enum hisi_isp_rproc_case_attr {
    DEFAULT_CASE = 0,
    SEC_CASE,
    NONSEC_CASE,
    INVAL_CASE,
};

enum HISP_CLK_TYPE {
    ISPCPU_CLK = 0,
    ISPFUNC_CLK = 1,
    ISPI2C_CLK = 2,
    ISP_CLK_MAX
};

extern const struct rproc_fw_ops rproc_elf_fw_ops;
extern const struct rproc_fw_ops rproc_bin_fw_ops;

void hisi_ispsec_share_para_set(void);
int hisi_isp_rproc_case_set(enum hisi_isp_rproc_case_attr);
enum hisi_isp_rproc_case_attr hisi_isp_rproc_case_get(void);
int hisp_powerup(void);
int hisp_powerdn(void);

int hisi_atfisp_cma_alloc(void);
void hisi_atfisp_cma_free(void);
void *hisi_fstcma_alloc(dma_addr_t *dma_handle, size_t size, gfp_t flag);
void hisi_fstcma_free(void *va, dma_addr_t dma_handle, size_t size);

void atfisp_set_nonsec(void);
void atfisp_disreset_a7(u64);
int use_nonsec_isp(void);
int use_sec_isp(void);
u64 get_a7sharedmem_addr(void);
u64 get_a7remap_addr(void);
void *getsec_a7sharedmem_addr(void);
unsigned long long get_nonsec_pgd(void);
void *get_a7remap_va(void);
void *get_a7sharedmem_va(void);
void set_a7mem_pa(u64 addr);
void set_a7mem_va(void *addr);
void set_a7sharedmem_addr(unsigned int addr);
int hisi_isp_nsec_probe(struct platform_device *pdev);
int hisi_isp_nsec_remove(struct platform_device *pdev);
int hisi_isp_rproc_pgd_set(struct rproc *rproc);
int nonsec_isp_device_enable(void);
int nonsec_isp_device_disable(void);
int hisp_nsec_jpeg_powerup(void);
int hisp_nsec_jpeg_powerdn(void);
int sync_isplogcat(void);
int start_isplogcat(void);
void stop_isplogcat(void);
void set_rpmsg_status(int status);
int is_ispcpu_powerup(void);
void hisp_sendin(void);
void hisp_sendx(void);
void hisp_recvin(void);
void hisp_recvx(void);
void hisp_rpmsgrefs_dump(void);
void hisp_rpmsgrefs_reset(void);
void hisp_recvtask(void);
int hisp_sec_jpeg_powerup(void);
int hisp_sec_jpeg_powerdn(void);
int hisp_jpeg_powerdn(void);
int hisp_jpeg_powerup(void);
int check_dvfs_valid(void);
int hisp_set_clk_rate(unsigned int type, unsigned int rate);
int secnsec_setclkrate(unsigned int type, unsigned int rate);
int nsec_setclkrate(unsigned int type, unsigned int rate);
int sec_setclkrate(unsigned int type, unsigned int rate);
int bypass_power_updn(void);
int set_power_updn(int bypass);
extern void hisi_isp_boot_stat_dump(void);
extern u64 hisi_getcurtime(void);
extern size_t print_time(u64 ts, char *buf);
extern unsigned int get_slice_time(void);
unsigned int a7_mmu_map(struct scatterlist *sgl, unsigned int size, unsigned int prot, unsigned int flag);
void a7_mmu_unmap(unsigned int va, unsigned int size);
extern u64 get_isprdr_addr(void);
void ispperfctrl_update(void);
void isploglevel_update(void);
void ispperf_stop_record(void);

extern int rproc_enable(struct rproc *rproc);
extern int rproc_bootware_attach(struct rproc *rproc, const char *bootware);
extern int rproc_enablerproc_enable(struct rproc *rproc);
extern struct rproc_shared_para *isp_share_para;
extern void isp_loglevel_init(struct rproc_shared_para *param);
extern struct rproc_shared_para *rproc_get_share_para(void);
extern void init_isp_shared_params(struct rproc_shared_para *p, unsigned int len);
extern void *rproc_da_to_va_priv(struct rproc *rproc, u64 da, int len);
extern int rproc_handle_version(struct rproc *rproc, struct fw_rsc_version *rsc, int offset, int avail);
extern int rproc_handle_cda(struct rproc *rproc, struct fw_rsc_cda *rsc, int offset, int avail);
extern void rproc_memory_cache_flush(struct rproc *rproc);
extern int vaddr_to_sgl(struct rproc *rproc, void **vaddr, unsigned int length, struct sg_table **table);
extern int rproc_handle_dynamic_memory(struct rproc *rproc, struct fw_rsc_dynamic_memory *rsc, int offset, int avail);
extern int rproc_handle_reserved_memory(struct rproc *rproc, struct fw_rsc_reserved_memory *rsc, int offset, int avail);
extern int rproc_handle_rdr_memory(struct rproc *rproc, struct fw_rsc_carveout *rsc, int offset, int avail);
extern int rproc_handle_shared_memory(struct rproc *rproc, struct fw_rsc_carveout *rsc, int offset, int avail);
extern int rproc_bw_load(struct rproc *rproc, const struct firmware *fw);
extern int rproc_set_shared_para(void);

extern int hisi_atfisp_probe(struct platform_device *pdev);
extern int hisi_atfisp_remove(struct platform_device *pdev);
extern int secisp_device_enable(void);
extern int secisp_device_disable(void);
extern void rproc_resource_cleanup(struct rproc *rproc);
extern void rproc_fw_config_virtio(const struct firmware *fw, void *context);
extern int hisp_meminit(unsigned int etype, unsigned long paddr);
extern int sec_rproc_boot(struct rproc *rproc);
extern int hisp_rsctable_init(void);
extern void sec_rscwork_func(struct work_struct *work);
extern void hisi_secisp_dump(void);
extern int hw_is_fpga_board(void);
extern unsigned long hisp_mem_pool_alloc_iova(unsigned int size, unsigned int pool_num);
extern unsigned int hisp_mem_pool_free_iova(unsigned int pool_num, unsigned int va, unsigned int size);
extern unsigned int hisp_mem_map_steup(struct scatterlist *sgl, unsigned int iova, unsigned int size,
                        unsigned int prot, unsigned int pool_num, unsigned int flag,unsigned int align);
extern void hisp_mem_pool_destroy(unsigned int pool_num);
extern void hisp_mem_pool_init(void);

#endif /* _PLAT_REMOTEPROC_HISI_ISP_H */

