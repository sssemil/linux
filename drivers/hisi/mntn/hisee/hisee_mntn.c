#include <linux/dma-mapping.h>
#include <linux/device.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/hisi/rdr_pub.h>
#include <linux/hisi/hisi_rproc.h>
#include <linux/hisi/util.h>
#include <linux/hisi/ipc_msg.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <asm/compiler.h>
#include <m3_interrupts.h>

#include "hisee_mntn.h"
#include "../mntn_filesys.h"

/*we should handle like this, for the differences between chicago and boston*/
/*define here for compiling, if this macro isn't defined in m3_interrupts.h
for chicago and boston es*/
#ifndef IRQ_INTR_HISEE_SENC2AP_IRQ
#define IRQ_INTR_HISEE_SENC2AP_IRQ 0
#endif
#ifndef IRQ_INTR_HISEE_EH2H_SLV
#define IRQ_INTR_HISEE_EH2H_SLV 0
#endif
/*ends*/
/*define here for compiling, if this macro isn't defined in m3_interrupts.h
for boston cs, starts*/
#ifndef IRQ_INTR_HISEE_SENC2AP_IRQ0
#define IRQ_INTR_HISEE_SENC2AP_IRQ0 0
#endif
#ifndef IRQ_INTR_HISEE_SENC2AP_IRQ1
#define IRQ_INTR_HISEE_SENC2AP_IRQ1 0
#endif
/*ends*/
/*lint -e785*/
struct rdr_exception_info_s hisee_excetption_info[] = {
	{
		.e_modid            = (u32)MODID_HISEE_EXC_SENSOR_CTRL,
		.e_modid_end        = (u32)MODID_HISEE_EXC_SENSOR_CTRL,
		.e_process_priority = RDR_ERR,
		.e_reboot_priority  = RDR_REBOOT_NO,
		.e_notify_core_mask = RDR_HISEE,
		.e_reset_core_mask  = RDR_HISEE,
		.e_from_core        = RDR_HISEE,
		.e_reentrant        = (u32)RDR_REENTRANT_DISALLOW,
		.e_exce_type        = HISEE_S_EXCEPTION,
		.e_upload_flag      = (u32)RDR_UPLOAD_YES,
		.e_from_module      = "HISEE SENSOR",
		.e_desc             = "HISEE",
	},
	{
		.e_modid            = (u32)MODID_HISEE_EXC_SIC,
		.e_modid_end        = (u32)MODID_HISEE_EXC_SIC,
		.e_process_priority = RDR_ERR,
		.e_reboot_priority  = RDR_REBOOT_NO,
		.e_notify_core_mask = RDR_HISEE,
		.e_reset_core_mask  = RDR_HISEE,
		.e_from_core        = RDR_HISEE,
		.e_reentrant        = (u32)RDR_REENTRANT_DISALLOW,
		.e_exce_type        = HISEE_S_EXCEPTION,
		.e_upload_flag      = (u32)RDR_UPLOAD_YES,
		.e_from_module      = "HISEE SIC",
		.e_desc             = "HISEE",
	},
	{
		.e_modid            = (u32)MODID_HISEE_EXC_MED_ROM,
		.e_modid_end        = (u32)MODID_HISEE_EXC_MED_ROM,
		.e_process_priority = RDR_ERR,
		.e_reboot_priority  = RDR_REBOOT_NO,
		.e_notify_core_mask = RDR_HISEE,
		.e_reset_core_mask  = RDR_HISEE,
		.e_from_core        = RDR_HISEE,
		.e_reentrant        = (u32)RDR_REENTRANT_DISALLOW,
		.e_exce_type        = HISEE_S_EXCEPTION,
		.e_upload_flag      = (u32)RDR_UPLOAD_YES,
		.e_from_module      = "HISEE ROM",
		.e_desc             = "HISEE",
	},
	{
		.e_modid            = (u32)MODID_HISEE_EXC_MED_RAM,
		.e_modid_end        = (u32)MODID_HISEE_EXC_MED_RAM,
		.e_process_priority = RDR_ERR,
		.e_reboot_priority  = RDR_REBOOT_NO,
		.e_notify_core_mask = RDR_HISEE,
		.e_reset_core_mask  = RDR_HISEE,
		.e_from_core        = RDR_HISEE,
		.e_reentrant        = (u32)RDR_REENTRANT_DISALLOW,
		.e_exce_type        = HISEE_S_EXCEPTION,
		.e_upload_flag      = (u32)RDR_UPLOAD_YES,
		.e_from_module      = "HISEE RAM",
		.e_desc             = "HISEE",
	},
	{
		.e_modid            = (u32)MODID_HISEE_EXC_OTPC,
		.e_modid_end        = (u32)MODID_HISEE_EXC_OTPC,
		.e_process_priority = RDR_ERR,
		.e_reboot_priority  = RDR_REBOOT_NO,
		.e_notify_core_mask = RDR_HISEE,
		.e_reset_core_mask  = RDR_HISEE,
		.e_from_core        = RDR_HISEE,
		.e_reentrant        = (u32)RDR_REENTRANT_DISALLOW,
		.e_exce_type        = HISEE_S_EXCEPTION,
		.e_upload_flag      = (u32)RDR_UPLOAD_YES,
		.e_from_module      = "HISEE OTPC",
		.e_desc             = "HISEE",
	},
	{
		.e_modid            = (u32)MODID_HISEE_EXC_HARD,
		.e_modid_end        = (u32)MODID_HISEE_EXC_HARD,
		.e_process_priority = RDR_ERR,
		.e_reboot_priority  = RDR_REBOOT_NO,
		.e_notify_core_mask = RDR_HISEE,
		.e_reset_core_mask  = RDR_HISEE,
		.e_from_core        = RDR_HISEE,
		.e_reentrant        = (u32)RDR_REENTRANT_DISALLOW,
		.e_exce_type        = HISEE_S_EXCEPTION,
		.e_upload_flag      = (u32)RDR_UPLOAD_YES,
		.e_from_module      = "HISEE HARD",
		.e_desc             = "HISEE",
	},
	{
		.e_modid            = (u32)MODID_HISEE_EXC_IPC_MAILBOX,
		.e_modid_end        = (u32)MODID_HISEE_EXC_IPC_MAILBOX,
		.e_process_priority = RDR_ERR,
		.e_reboot_priority  = RDR_REBOOT_NO,
		.e_notify_core_mask = RDR_HISEE,
		.e_reset_core_mask  = RDR_HISEE,
		.e_from_core        = RDR_HISEE,
		.e_reentrant        = (u32)RDR_REENTRANT_DISALLOW,
		.e_exce_type        = HISEE_S_EXCEPTION,
		.e_upload_flag      = (u32)RDR_UPLOAD_YES,
		.e_from_module      = "HISEE IPC MAILBOX",
		.e_desc             = "HISEE",
	},
	{
		.e_modid            = (u32)MODID_HISEE_EXC_MPU,
		.e_modid_end        = (u32)MODID_HISEE_EXC_MPU,
		.e_process_priority = RDR_ERR,
		.e_reboot_priority  = RDR_REBOOT_NO,
		.e_notify_core_mask = RDR_HISEE,
		.e_reset_core_mask  = RDR_HISEE,
		.e_from_core        = RDR_HISEE,
		.e_reentrant        = (u32)RDR_REENTRANT_DISALLOW,
		.e_exce_type        = HISEE_S_EXCEPTION,
		.e_upload_flag      = (u32)RDR_UPLOAD_YES,
		.e_from_module      = "HISEE MPU",
		.e_desc             = "HISEE",
	},
	{
		.e_modid            = (u32)MODID_HISEE_EXC_BUS,
		.e_modid_end        = (u32)MODID_HISEE_EXC_BUS,
		.e_process_priority = RDR_ERR,
		.e_reboot_priority  = RDR_REBOOT_NO,
		.e_notify_core_mask = RDR_HISEE,
		.e_reset_core_mask  = RDR_HISEE,
		.e_from_core        = RDR_HISEE,
		.e_reentrant        = (u32)RDR_REENTRANT_DISALLOW,
		.e_exce_type        = HISEE_S_EXCEPTION,
		.e_upload_flag      = (u32)RDR_UPLOAD_YES,
		.e_from_module      = "HISEE BUS",
		.e_desc             = "HISEE",
	},
	{
		.e_modid            = (u32)MODID_HISEE_EXC_TIMER,
		.e_modid_end        = (u32)MODID_HISEE_EXC_TIMER,
		.e_process_priority = RDR_ERR,
		.e_reboot_priority  = RDR_REBOOT_NO,
		.e_notify_core_mask = RDR_HISEE,
		.e_reset_core_mask  = RDR_HISEE,
		.e_from_core        = RDR_HISEE,
		.e_reentrant        = (u32)RDR_REENTRANT_DISALLOW,
		.e_exce_type        = HISEE_S_EXCEPTION,
		.e_upload_flag      = (u32)RDR_UPLOAD_YES,
		.e_from_module      = "HISEE TIMER",
		.e_desc             = "HISEE",
	},
	{
		.e_modid            = (u32)MODID_HISEE_EXC_SEC_EXTERN,
		.e_modid_end        = (u32)MODID_HISEE_EXC_SEC_EXTERN,
		.e_process_priority = RDR_ERR,
		.e_reboot_priority  = RDR_REBOOT_NO,
		.e_notify_core_mask = RDR_HISEE,
		.e_reset_core_mask  = RDR_HISEE,
		.e_from_core        = RDR_HISEE,
		.e_reentrant        = (u32)RDR_REENTRANT_DISALLOW,
		.e_exce_type        = HISEE_S_EXCEPTION,
		.e_upload_flag      = (u32)RDR_UPLOAD_YES,
		.e_from_module      = "HISEE EXTERN",
		.e_desc             = "HISEE",
	},
	{
		.e_modid            = (u32)MODID_HISEE_EXC_WDG,
		.e_modid_end        = (u32)MODID_HISEE_EXC_WDG,
		.e_process_priority = RDR_ERR,
		.e_reboot_priority  = RDR_REBOOT_NO,
		.e_notify_core_mask = RDR_HISEE,
		.e_reset_core_mask  = RDR_HISEE,
		.e_from_core        = RDR_HISEE,
		.e_reentrant        = (u32)RDR_REENTRANT_DISALLOW,
		.e_exce_type        = HISEE_S_EXCEPTION,
		.e_upload_flag      = (u32)RDR_UPLOAD_YES,
		.e_from_module      = "HISEE WDG",
		.e_desc             = "HISEE",
	},
	{
		.e_modid            = (u32)MODID_HISEE_EXC_SYSALARM,
		.e_modid_end        = (u32)MODID_HISEE_EXC_SYSALARM,
		.e_process_priority = RDR_ERR,
		.e_reboot_priority  = RDR_REBOOT_NO,
		.e_notify_core_mask = RDR_HISEE,
		.e_reset_core_mask  = RDR_HISEE,
		.e_from_core        = RDR_HISEE,
		.e_reentrant        = (u32)RDR_REENTRANT_DISALLOW,
		.e_exce_type        = HISEE_S_EXCEPTION,
		.e_upload_flag      = (u32)RDR_UPLOAD_YES,
		.e_from_module      = "HISEE SYSALARM",
		.e_desc             = "HISEE",
	},
	{
		.e_modid            = (u32)MODID_HISEE_EXC_SECENG_TRNG,
		.e_modid_end        = (u32)MODID_HISEE_EXC_SECENG_TRNG,
		.e_process_priority = RDR_ERR,
		.e_reboot_priority  = RDR_REBOOT_NO,
		.e_notify_core_mask = RDR_HISEE,
		.e_reset_core_mask  = RDR_HISEE,
		.e_from_core        = RDR_HISEE,
		.e_reentrant        = (u32)RDR_REENTRANT_DISALLOW,
		.e_exce_type        = HISEE_S_EXCEPTION,
		.e_upload_flag      = (u32)RDR_UPLOAD_YES,
		.e_from_module      = "HISEE SECENG TRNG",
		.e_desc             = "HISEE",
	},
	{
		.e_modid            = (u32)MODID_HISEE_EXC_SECENG_TRIM,
		.e_modid_end        = (u32)MODID_HISEE_EXC_SECENG_TRIM,
		.e_process_priority = RDR_ERR,
		.e_reboot_priority  = RDR_REBOOT_NO,
		.e_notify_core_mask = RDR_HISEE,
		.e_reset_core_mask  = RDR_HISEE,
		.e_from_core        = RDR_HISEE,
		.e_reentrant        = (u32)RDR_REENTRANT_DISALLOW,
		.e_exce_type        = HISEE_S_EXCEPTION,
		.e_upload_flag      = (u32)RDR_UPLOAD_YES,
		.e_from_module      = "HISEE SECENG TRIM",
		.e_desc             = "HISEE",
	},
	{
		.e_modid            = (u32)MODID_HISEE_EXC_SECENG_SCE,
		.e_modid_end        = (u32)MODID_HISEE_EXC_SECENG_SCE,
		.e_process_priority = RDR_ERR,
		.e_reboot_priority  = RDR_REBOOT_NO,
		.e_notify_core_mask = RDR_HISEE,
		.e_reset_core_mask  = RDR_HISEE,
		.e_from_core        = RDR_HISEE,
		.e_reentrant        = (u32)RDR_REENTRANT_DISALLOW,
		.e_exce_type        = HISEE_S_EXCEPTION,
		.e_upload_flag      = (u32)RDR_UPLOAD_YES,
		.e_from_module      = "HISEE SECENG SCE",
		.e_desc             = "HISEE",
	},
	{
		.e_modid            = (u32)MODID_HISEE_EXC_SECENG_RSA,
		.e_modid_end        = (u32)MODID_HISEE_EXC_SECENG_RSA,
		.e_process_priority = RDR_ERR,
		.e_reboot_priority  = RDR_REBOOT_NO,
		.e_notify_core_mask = RDR_HISEE,
		.e_reset_core_mask  = RDR_HISEE,
		.e_from_core        = RDR_HISEE,
		.e_reentrant        = (u32)RDR_REENTRANT_DISALLOW,
		.e_exce_type        = HISEE_S_EXCEPTION,
		.e_upload_flag      = (u32)RDR_UPLOAD_YES,
		.e_from_module      = "HISEE SECENG RSA",
		.e_desc             = "HISEE",
	},
	{
		.e_modid            = (u32)MODID_HISEE_EXC_SECENG_SM2,
		.e_modid_end        = (u32)MODID_HISEE_EXC_SECENG_SM2,
		.e_process_priority = RDR_ERR,
		.e_reboot_priority  = RDR_REBOOT_NO,
		.e_notify_core_mask = RDR_HISEE,
		.e_reset_core_mask  = RDR_HISEE,
		.e_from_core        = RDR_HISEE,
		.e_reentrant        = (u32)RDR_REENTRANT_DISALLOW,
		.e_exce_type        = HISEE_S_EXCEPTION,
		.e_upload_flag      = (u32)RDR_UPLOAD_YES,
		.e_from_module      = "HISEE SECENG SM2",
		.e_desc             = "HISEE",
	},
	{
		.e_modid            = (u32)MODID_HISEE_EXC_SECENG_KM,
		.e_modid_end        = (u32)MODID_HISEE_EXC_SECENG_KM,
		.e_process_priority = RDR_ERR,
		.e_reboot_priority  = RDR_REBOOT_NO,
		.e_notify_core_mask = RDR_HISEE,
		.e_reset_core_mask  = RDR_HISEE,
		.e_from_core        = RDR_HISEE,
		.e_reentrant        = (u32)RDR_REENTRANT_DISALLOW,
		.e_exce_type        = HISEE_S_EXCEPTION,
		.e_upload_flag      = (u32)RDR_UPLOAD_YES,
		.e_from_module      = "HISEE SECENG KM",
		.e_desc             = "HISEE",
	},
	{
		.e_modid            = (u32)MODID_HISEE_EXC_SECENG_SCRAMBLING,
		.e_modid_end        = (u32)MODID_HISEE_EXC_SECENG_SCRAMBLING,
		.e_process_priority = RDR_ERR,
		.e_reboot_priority  = RDR_REBOOT_NO,
		.e_notify_core_mask = RDR_HISEE,
		.e_reset_core_mask  = RDR_HISEE,
		.e_from_core        = RDR_HISEE,
		.e_reentrant        = (u32)RDR_REENTRANT_DISALLOW,
		.e_exce_type        = HISEE_S_EXCEPTION,
		.e_upload_flag      = (u32)RDR_UPLOAD_YES,
		.e_from_module      = "HISEE SECENG SCRAMBLING",
		.e_desc             = "HISEE",
	},
	{
		.e_modid            = (u32)MODID_HISEE_EXC_BOTTOM,
		.e_modid_end        = (u32)MODID_HISEE_EXC_BOTTOM,
		.e_process_priority = RDR_ERR,
		.e_reboot_priority  = RDR_REBOOT_NO,
		.e_notify_core_mask = RDR_HISEE,
		.e_reset_core_mask  = RDR_HISEE,
		.e_from_core        = RDR_HISEE,
		.e_reentrant        = (u32)RDR_REENTRANT_DISALLOW,
		.e_exce_type        = HISEE_S_EXCEPTION,
		.e_upload_flag      = (u32)RDR_UPLOAD_YES,
		.e_from_module      = "HISEE BOTTOM",
		.e_desc             = "HISEE",
	},
	{
		.e_modid            = (u32)MODID_HISEE_EXC_ALARM0,
		.e_modid_end        = (u32)MODID_HISEE_EXC_ALARM0,
		.e_process_priority = RDR_ERR,
		.e_reboot_priority  = RDR_REBOOT_NO,
		.e_notify_core_mask = RDR_HISEE,
		.e_reset_core_mask  = RDR_HISEE,
		.e_from_core        = RDR_HISEE,
		.e_reentrant        = (u32)RDR_REENTRANT_DISALLOW,
		.e_exce_type        = HISEE_S_EXCEPTION,
		.e_upload_flag      = (u32)RDR_UPLOAD_YES,
		.e_from_module      = "HISEE ALARM0",
		.e_desc             = "HISEE",
	},
	{
		.e_modid            = (u32)MODID_HISEE_EXC_ALARM1,
		.e_modid_end        = (u32)MODID_HISEE_EXC_ALARM1,
		.e_process_priority = RDR_ERR,
		.e_reboot_priority  = RDR_REBOOT_NO,
		.e_notify_core_mask = RDR_HISEE,
		.e_reset_core_mask  = RDR_HISEE,
		.e_from_core        = RDR_HISEE,
		.e_reentrant        = (u32)RDR_REENTRANT_DISALLOW,
		.e_exce_type        = HISEE_S_EXCEPTION,
		.e_upload_flag      = (u32)RDR_UPLOAD_YES,
		.e_from_module      = "HISEE ALARM1",
		.e_desc             = "HISEE",
	},
	{
		.e_modid            = (u32)MODID_HISEE_EXC_AS2AP_IRQ,
		.e_modid_end        = (u32)MODID_HISEE_EXC_AS2AP_IRQ,
		.e_process_priority = RDR_ERR,
		.e_reboot_priority  = RDR_REBOOT_NO,
		.e_notify_core_mask = RDR_HISEE,
		.e_reset_core_mask  = RDR_HISEE,
		.e_from_core        = RDR_HISEE,
		.e_reentrant        = (u32)RDR_REENTRANT_DISALLOW,
		.e_exce_type        = HISEE_S_EXCEPTION,
		.e_upload_flag      = (u32)RDR_UPLOAD_YES,
		.e_from_module      = "HISEE AS2AP IRQ",
		.e_desc             = "HISEE",
	},
	{
		.e_modid            = (u32)MODID_HISEE_EXC_DS2AP_IRQ,
		.e_modid_end        = (u32)MODID_HISEE_EXC_DS2AP_IRQ,
		.e_process_priority = RDR_ERR,
		.e_reboot_priority  = RDR_REBOOT_NO,
		.e_notify_core_mask = RDR_HISEE,
		.e_reset_core_mask  = RDR_HISEE,
		.e_from_core        = RDR_HISEE,
		.e_reentrant        = (u32)RDR_REENTRANT_DISALLOW,
		.e_exce_type        = HISEE_S_EXCEPTION,
		.e_upload_flag      = (u32)RDR_UPLOAD_YES,
		.e_from_module      = "HISEE DS2AP IRQ",
		.e_desc             = "HISEE",
	},
	{
		.e_modid            = (u32)MODID_HISEE_EXC_SENC2AP_IRQ,
		.e_modid_end        = (u32)MODID_HISEE_EXC_SENC2AP_IRQ,
		.e_process_priority = RDR_ERR,
		.e_reboot_priority  = RDR_REBOOT_NO,
		.e_notify_core_mask = RDR_HISEE,
		.e_reset_core_mask  = RDR_HISEE,
		.e_from_core        = RDR_HISEE,
		.e_reentrant        = (u32)RDR_REENTRANT_DISALLOW,
		.e_exce_type        = HISEE_S_EXCEPTION,
		.e_upload_flag      = (u32)RDR_UPLOAD_YES,
		.e_from_module      = "HISEE SENC2AP IRQ",
		.e_desc             = "HISEE",
	},
	{
		.e_modid            = (u32)MODID_HISEE_EXC_SENC2AP_IRQ0,
		.e_modid_end        = (u32)MODID_HISEE_EXC_SENC2AP_IRQ0,
		.e_process_priority = RDR_ERR,
		.e_reboot_priority  = RDR_REBOOT_NO,
		.e_notify_core_mask = RDR_HISEE,
		.e_reset_core_mask  = RDR_HISEE,
		.e_from_core        = RDR_HISEE,
		.e_reentrant        = (u32)RDR_REENTRANT_DISALLOW,
		.e_exce_type        = HISEE_S_EXCEPTION,
		.e_upload_flag      = (u32)RDR_UPLOAD_YES,
		.e_from_module      = "HISEE SENC2AP IRQ0",
		.e_desc             = "HISEE",
	},
	{
		.e_modid            = (u32)MODID_HISEE_EXC_SENC2AP_IRQ1,
		.e_modid_end        = (u32)MODID_HISEE_EXC_SENC2AP_IRQ1,
		.e_process_priority = RDR_ERR,
		.e_reboot_priority  = RDR_REBOOT_NO,
		.e_notify_core_mask = RDR_HISEE,
		.e_reset_core_mask  = RDR_HISEE,
		.e_from_core        = RDR_HISEE,
		.e_reentrant        = (u32)RDR_REENTRANT_DISALLOW,
		.e_exce_type        = HISEE_S_EXCEPTION,
		.e_upload_flag      = (u32)RDR_UPLOAD_YES,
		.e_from_module      = "HISEE SENC2AP IRQ1",
		.e_desc             = "HISEE",
	},
	{
		.e_modid            = (u32)MODID_HISEE_EXC_LOCKUP,
		.e_modid_end        = (u32)MODID_HISEE_EXC_LOCKUP,
		.e_process_priority = RDR_ERR,
		.e_reboot_priority  = RDR_REBOOT_NO,
		.e_notify_core_mask = RDR_HISEE,
		.e_reset_core_mask  = RDR_HISEE,
		.e_from_core        = RDR_HISEE,
		.e_reentrant        = (u32)RDR_REENTRANT_DISALLOW,
		.e_exce_type        = HISEE_S_EXCEPTION,
		.e_upload_flag      = (u32)RDR_UPLOAD_YES,
		.e_from_module      = "HISEE LOCKUP",
		.e_desc             = "HISEE",
	},
	{
		.e_modid            = (u32)MODID_HISEE_EXC_EH2H_SLV,
		.e_modid_end        = (u32)MODID_HISEE_EXC_EH2H_SLV,
		.e_process_priority = RDR_ERR,
		.e_reboot_priority  = RDR_REBOOT_NO,
		.e_notify_core_mask = RDR_HISEE,
		.e_reset_core_mask  = RDR_HISEE,
		.e_from_core        = RDR_HISEE,
		.e_reentrant        = (u32)RDR_REENTRANT_DISALLOW,
		.e_exce_type        = HISEE_S_EXCEPTION,
		.e_upload_flag      = (u32)RDR_UPLOAD_YES,
		.e_from_module      = "HISEE EH2H SLV",
		.e_desc             = "HISEE",
	},
	{
		.e_modid            = (u32)MODID_HISEE_EXC_TSENSOR1,
		.e_modid_end        = (u32)MODID_HISEE_EXC_TSENSOR1,
		.e_process_priority = RDR_ERR,
		.e_reboot_priority  = RDR_REBOOT_NO,
		.e_notify_core_mask = RDR_HISEE,
		.e_reset_core_mask  = RDR_HISEE,
		.e_from_core        = RDR_HISEE,
		.e_reentrant        = (u32)RDR_REENTRANT_DISALLOW,
		.e_exce_type        = HISEE_S_EXCEPTION,
		.e_upload_flag      = (u32)RDR_UPLOAD_YES,
		.e_from_module      = "HISEE TSENSOR1",
		.e_desc             = "HISEE",
	},
	{
		.e_modid            = (u32)MODID_HISEE_EXC_UNKNOWN,
		.e_modid_end        = (u32)MODID_HISEE_EXC_UNKNOWN,
		.e_process_priority = RDR_ERR,
		.e_reboot_priority  = RDR_REBOOT_NO,
		.e_notify_core_mask = RDR_HISEE,
		.e_reset_core_mask  = RDR_HISEE,
		.e_from_core        = RDR_HISEE,
		.e_reentrant        = (u32)RDR_REENTRANT_DISALLOW,
		.e_exce_type        = HISEE_S_EXCEPTION,
		.e_upload_flag      = (u32)RDR_UPLOAD_YES,
		.e_from_module      = "HISEE UNKNOWN",
		.e_desc             = "HISEE",
	}
};
/*lint +e785*/

/*for translation from original irq no to exception type that module id*/
hisee_exc_trans_s hisee_exc_trans[] = {
	{IRQ_INTR_HISEE_ALARM0, MODID_HISEE_EXC_ALARM0},
	{IRQ_INTR_HISEE_ALARM1, MODID_HISEE_EXC_ALARM1},
	{IRQ_INTR_HISEE_AS2AP_IRQ, MODID_HISEE_EXC_AS2AP_IRQ},
	{IRQ_INTR_HISEE_DS2AP_IRQ, MODID_HISEE_EXC_DS2AP_IRQ},
	{IRQ_INTR_HISEE_SENC2AP_IRQ, MODID_HISEE_EXC_SENC2AP_IRQ},
	{IRQ_INTR_HISEE_SENC2AP_IRQ0, MODID_HISEE_EXC_SENC2AP_IRQ0},
	{IRQ_INTR_HISEE_SENC2AP_IRQ1, MODID_HISEE_EXC_SENC2AP_IRQ1},
	{IRQ_INTR_HISEE_LOCKUP, MODID_HISEE_EXC_LOCKUP},
	{IRQ_INTR_HISEE_EH2H_SLV, MODID_HISEE_EXC_EH2H_SLV},
	{IRQ_INTR_HISEE_TSENSOR1, MODID_HISEE_EXC_TSENSOR1},

	/*Please add your new member above!!!!*/
};

static u32 hisee_exception_modid;
static u32	 g_log_out_offset = 0;
static struct notifier_block hisee_ipc_block;
static struct rdr_register_module_result hisee_info;
static void *hisee_mntn_addr;
static dma_addr_t hisee_log_phy;
static void __iomem *hisee_log_addr;
static struct task_struct *hisee_mntn_thread = NULL;
static DECLARE_COMPLETION(hisee_mntn_complete);

static struct ipc_msg g_msg;
static hisee_mntn_state	g_hisee_mntn_state = HISEE_STATE_INVALID;

/********************************************************************
Description:	send msg to lpm3 by rproc
input:	u32 data0, u32 data1, u32 data2
output:
return:	NA
********************************************************************/
static int hisee_mntn_send_msg_to_lpm3(u32 *p_data, u32 len)
{
	int	ret = -1;

	if (NULL == p_data)
		return ret;

	ret = RPROC_ASYNC_SEND(HISI_RPROC_LPM3_MBX17,
		(mbox_msg_t *)p_data, (rproc_msg_len_t)len);
	if (ret != 0)
		pr_err("%s:RPROC_ASYNC_SEND failed! return 0x%x, msg:(%x)\n",
			__func__, ret, *p_data);
	return ret;
}
/********************************************************************
description:  translate the irq number to the exception type defined by kernel,
		so kernel can know what exception it is.
input: irq_no, irq number id.
output: NA
return: exception type
********************************************************************/
static unsigned int translate_exc_type(u32 irq_no)
{
	unsigned long i;
	u32 module_id = (u32)MODID_HISEE_EXC_UNKNOWN;

	for (i = 0; i < sizeof(hisee_exc_trans) /
		sizeof(hisee_exc_trans_s); i++)
		if (irq_no == hisee_exc_trans[i].irq_value) {
			module_id = hisee_exc_trans[i].module_value;
			break;
		}

	return module_id;
}

/********************************************************************
Description:	kenrel send msg to ATF
input:	NA
output:	NA
return:	NA
********************************************************************/
/*lint -e715*/
noinline u64 atfd_hisi_service_hisee_mntn_smc(u64 function_id,
	u64 arg0,
	u64 arg1,
	u64 arg2)
{
	asm volatile(
		__asmeq("%0", "x0")
		__asmeq("%1", "x1")
		__asmeq("%2", "x2")
		__asmeq("%3", "x3")
		"smc    #0\n"
		: "+r" (function_id)
		: "r" (arg0), "r" (arg1), "r" (arg2));

	return (u64)function_id;
}

/********************************************************************
Description: save hisee print log data in file
input:	NA
output:	NA
return:	NA
********************************************************************/
void rdr_hisee_save_print_log(void)
{
	u32 tmpsize;
	char path[HISEE_MNTN_PATH_MAXLEN] = {0};
	char	*p_timestr = rdr_get_timestamp();
	int ret;
	hlog_header *hisee_log_head = (hlog_header *)hisee_log_addr;

	pr_err(" ====================================\n");
	pr_err(" save hisee print log now\n");
	pr_err(" ====================================\n");

	snprintf(path, (size_t)HISEE_MNTN_PATH_MAXLEN, "%s", PATH_ROOT);
	/*return if there is no free space in hisi_logs*/
	tmpsize = (u32)rdr_dir_size(path, (bool)true);
	if (tmpsize > rdr_get_logsize()) {
		pr_err("hisi_logs dir is full!!\n");
		return;
	}
	atfd_hisi_service_hisee_mntn_smc((u64)HISEE_MNTN_ID,
		(u64)HISEE_SMC_LOG_OUT, (u64)g_log_out_offset, (u64)0x0);

	snprintf(path, (size_t)HISEE_MNTN_PATH_MAXLEN, "%s%s", PATH_ROOT, HISEE_PRINTLOG_FLIENAME);

	/*save current time in hisee_printlog*/
	/*lint -e124*/
	if (NULL != p_timestr) {
		ret = mntn_filesys_write_log(path,
				(void *)(p_timestr),
				(unsigned int)strlen(p_timestr),
				HISEE_FILE_PERMISSION);
		pr_err(" save hisee print log time, time is %s, ret = %d\n", p_timestr, ret);
	}
	/* save hisee log to data/hisi_logs/time/hisee_printlog */
	ret = mntn_filesys_write_log(path,
			(void *)(hisee_log_addr + sizeof(hlog_header)),
			(unsigned int)hisee_log_head->real_size,
			HISEE_FILE_PERMISSION);
	/*lint +e124*/
	pr_err(" ====================================\n");
	pr_err(" save hisee print log end, ret = %d\n", ret);
	pr_err(" ====================================\n");
	/*lint +e124*/
}
/********************************************************************
Description: hisee mntn thread
input:	void *arg
output:	NA
return:	NA
********************************************************************/
int rdr_hisee_thread(void *arg)
{
	while (!kthread_should_stop()) {
		wait_for_completion(&hisee_mntn_complete);
		switch (g_hisee_mntn_state) {
		case HISEE_STATE_HISEE_EXC:
			pr_err("fi[0x%x] fv[0x%x] ss[0x%x] sc[0x%x]\n", g_msg.data[2], g_msg.data[3], g_msg.data[4], g_msg.data[5]);
			memset(&g_msg, 0, sizeof(struct ipc_msg));
			rdr_system_error(hisee_exception_modid, 0, 0);
			break;
		case HISEE_STATE_LOG_OUT:
			rdr_hisee_save_print_log();
			break;
		default:
			break;
		}
		g_hisee_mntn_state = HISEE_STATE_READY;
	}
	return 0;
}
/********************************************************************
Description:	receive ipc message
input:	msg£ºipc message
output:	NA
return:	NA
********************************************************************/
/*lint -e715*/
int rdr_hisee_msg_handler(struct notifier_block *nb,
	unsigned long action,
	void *msg)
{
	if (NULL == msg) {
		pr_err("%s:msg is NULL!\n", __func__);
		return 0;
	}

	g_msg = *(struct ipc_msg *)msg;
	switch (g_msg.data[0]) {
	case HISEE_LOG_OUT:
		g_log_out_offset = g_msg.data[1];
		g_hisee_mntn_state = HISEE_STATE_LOG_OUT;
		complete(&hisee_mntn_complete);
		break;
	case HISEE_EXCEPTION:
		hisee_exception_modid = g_msg.data[1] + MODID_HISEE_START;
		if (hisee_exception_modid >= (u32)MODID_HISEE_EXC_BOTTOM)
			hisee_exception_modid = (u32)MODID_HISEE_EXC_UNKNOWN;

		g_hisee_mntn_state = HISEE_STATE_HISEE_EXC;
		complete(&hisee_mntn_complete);
		break;
	case HISEE_IRQ:
		hisee_exception_modid = translate_exc_type(g_msg.data[1]);
		g_hisee_mntn_state = HISEE_STATE_HISEE_EXC;
		complete(&hisee_mntn_complete);
		break;
	case HISEE_TIME:
		pr_err("%s:sync time with hisee, mark value is:%d\n",
				__func__, g_msg.data[1]);
		break;
	default:
		/*nothing to do, other modules' msg*/
		break;
	}
	return 0;
}
/*lint +e715*/

/********************************************************************
Description: notify lpm3 reset hisee
input:	modid:module id
		etype:exception type
		coreid:core id
output:	NA
return:	NA
********************************************************************/
void rdr_hisee_reset(u32 modid, u32 etype, u64 coreid)
{
	unsigned int msg[2];

#ifdef FACTORY_TEST_HISEE
#ifdef CONFIG_PN547_NFC_64
	int ret;
	char buf[128] = {0};

	snprintf(buf, sizeof(buf), "modid:0x%x; etype:0x%x; coreid:0x%llx\n", modid, etype, coreid);
	ret = nfc_record_dmd_info(DSM_NFC_HISEE_POWER_ON_OFF_ERROR_NO, (const char *)buf);
	if (0 != ret)
		pr_err("%s(): nfc_record_dmd_for_hisee return %d failed\n", __func__, ret);
#endif
#endif
	msg[0] = LPM3_HISEE_MNTN;
	msg[1] = HISEE_RESET;

	pr_err(" ====================================\n");
	pr_err(" modid:          [0x%x]\n", modid);
	pr_err(" coreid:         [0x%llx]\n", coreid);
	pr_err(" exce tpye:      [0x%x]\n", etype);
	pr_err(" ====================================\n");

	hisee_mntn_send_msg_to_lpm3(msg, 2);
}
/********************************************************************
Description: get the pointer to the name str of mod
input:	modid: module id
output:	NA
return:	pointer to the name str of mod
********************************************************************/
static char *hisee_mntn_get_mod_name_str(u32 modid)
{
	u32	i;
	char	*p_name = NULL;

	for (i = 0; i < (sizeof(hisee_excetption_info) / sizeof(hisee_excetption_info[0]));i++) {
		if (hisee_excetption_info[i].e_modid == modid) {
			p_name = (char *)(hisee_excetption_info[i].e_from_module);
			break;
		}
	}
	return p_name;
}
/********************************************************************
Description: save hisee log to file system
input:	modid: module id
		etype:exception type
		coreid: core id
		pathname: log path
		pfn_cb: callback function
output:	NA
return:	NA
********************************************************************/
void rdr_hisee_dump(u32 modid,
	u32 etype,
	u64 coreid,
	char *pathname,
	pfn_cb_dump_done pfn_cb)
{
	char path[HISEE_MNTN_PATH_MAXLEN] = {0};
	int ret;
	hlog_header *hisee_log_head = (hlog_header *)hisee_log_addr;
	char	*p_name_str = hisee_mntn_get_mod_name_str(modid);

	pr_err(" ====================================\n");
	pr_err(" modid:          [0x%x]\n",   modid);
	pr_err(" coreid:         [0x%llx]\n", coreid);
	pr_err(" exce tpye:      [0x%x]\n",   etype);
	pr_err(" path name:      [%s]\n",     pathname);
	atfd_hisi_service_hisee_mntn_smc((u64)HISEE_MNTN_ID,
		(u64)HISEE_SMC_GET_LOG, (u64)0x0, (u64)0x0);
	snprintf(path, (unsigned long)HISEE_MNTN_PATH_MAXLEN, "%s/%s", pathname, HISEE_LOG_FLIENAME);

	/* save hisee log to data/hisi_logs/time/hisee_log */
	/*lint -e124*/
	ret = mntn_filesys_write_log(path,
			(void *)(hisee_log_addr + sizeof(hlog_header)),
			(unsigned int)hisee_log_head->real_size,
			HISEE_FILE_PERMISSION);
	/*lint +e124*/
	if (0 == ret)
		pr_err("%s:hisee log save fail\n", __func__);

	/*lint -e124*/
	if (NULL != p_name_str) {
		ret = mntn_filesys_write_log(path,
				(void *)p_name_str,
				(unsigned int)MODULE_NAME_LEN,
				HISEE_FILE_PERMISSION);		
		pr_err("mod name:      [%s]\n", p_name_str);
	}
	else {
		ret = mntn_filesys_write_log(path,
				(void *)(&modid),
				(unsigned int)sizeof(modid),
				HISEE_FILE_PERMISSION);
	}
	/*lint +e124*/
	pr_err(" ====================================\n");
	if (0 == ret)
		pr_err("%s:hisee mod id save fail\n", __func__);
	/* save to 8M */
	memcpy(hisee_mntn_addr, hisee_log_addr, (unsigned long)hisee_info.log_len);

	if (pfn_cb)
		pfn_cb(modid, coreid);

}

/********************************************************************
Description:	register hisee dump and reset function
input:	NA
output:	NA
return:	NA
********************************************************************/
static int hisee_register_core(void)
{
	int ret;
	struct rdr_module_ops_pub s_soc_ops;

	s_soc_ops.ops_dump = rdr_hisee_dump;
	s_soc_ops.ops_reset = rdr_hisee_reset;
	/* register hisee core dump and reset function */
	ret = rdr_register_module_ops((u64)RDR_HISEE, &s_soc_ops, &hisee_info);
	if (ret < 0) {
		pr_err("%s:hisee core register fail, ret = [%d]\n",
			__func__, ret);
		return ret;
	};
	return 0;
}

/********************************************************************
Description:	register hisee exception
input:	hisee_einfo: hisee exception list
output:
return:	NA
********************************************************************/
static void hisee_register_exception(void)
{
	u32 ret;
	unsigned long i;

	for (i = 0; i < sizeof(hisee_excetption_info) /
		sizeof(struct rdr_exception_info_s); i++) {
		/* error return 0, ok return modid */
		ret = rdr_register_exception(&hisee_excetption_info[i]);
		if (!ret)
			pr_err("register hisee exception fail hisee_einfo[%lu]\n", i);
	}
}
/********************************************************************
Description: Check whether hisee chip is valid
input:	NA
output:	NA
return:	false:invalid
		true:valid
********************************************************************/
int is_hisee_efuse_valid(void)
{
	int ret;
	struct device_node *pn_hisee;
	u32 hisee_sm_flag_pos;
	u32 group;
	u32 offset;
	unsigned int hisee_value[2] = {0};

	pn_hisee = of_find_compatible_node(NULL, NULL, "hisilicon,hisee-device");
	if (!pn_hisee) {
		pr_err("%s hisee dts not found\n", __func__);
		return false;
	}
	if (0 != of_property_read_u32(pn_hisee, "sm_flag_pos", &hisee_sm_flag_pos)) {
		pr_err("%s hisee sm flag pos invalid\n", __func__);
		return false;
	}
	group = hisee_sm_flag_pos / 32;
	offset = hisee_sm_flag_pos % 32;
	if(group >= 2) {
		pr_err("%s hisee sm flag pos %u invalid\n", __func__, hisee_sm_flag_pos);
		return false;
	}
	pr_info("hisee mntn efuse pos %u, %u\n", group, offset);
	ret = get_efuse_hisee_value((unsigned char *)hisee_value, EFUSE_LENGTH, EFUSE_READ_TIMEOUT);
	if (ret) {
		pr_err("%s() get_efuse_hisee_value failed,ret=%d\n", __func__, ret);
		return false;
	}
	if (!((hisee_value[group] >> offset) & 1))
		return false;

	return true;
}
#ifdef CONFIG_HISI_HISEE_MNTN_TEST
/********************************************************************
Description:	set mntn ctrl level according to the nv value
input:	NONE
output:
return:	NA
********************************************************************/
static void hisee_mntn_set_ctrl_level(void)
{
	unsigned int msg[4];
	int	value = get_himntn_value(HIMNTN_HISEE);

	msg[0] = LPM3_HISEE_MNTN;
	msg[1] = HISEE_MNTN_CTRL;
	msg[3] = HISEE_MNTN_LOG_CTRL_OUT_2_KERNEL;
	switch (value) {
	case '0':
		msg[2] = (u32)HISEE_MNTN_DISABLE;
		break;
	case '1':
		msg[2] = (u32)HISEE_MNTN_USR;
		break;
	case '2':
		msg[2] = (u32)HISEE_MNTN_BETA;
		break;
	case '3':
		msg[2] = (u32)HISEE_MNTN_ENG;
		break;
	case '4':
		msg[2] = (u32)HISEE_MNTN_ENG;
		msg[3] = HISEE_MNTN_LOG_CTRL_NOT_2_KERNEL;
		break;
	default:
		break;
	}

	hisee_mntn_send_msg_to_lpm3(msg, 4);
	pr_err("%s:hisee mntn ctrl level is %d\n", __func__, msg[2]);
}
/********************************************************************
Description:	hisee mntn test, only for test
input:	modid:module id
output:	NA
return:	NA
********************************************************************/
void hisee_mntn_make_data_full(void)
{
#define DATA_FULL_BUF_LEN	0x400000
	u64	total_cnt = 0;
	u32	cnt_1 = 0;
	char path[HISEE_MNTN_PATH_MAXLEN] = {0};
	u8	*p_buf = (u8 *)kmalloc((size_t)DATA_FULL_BUF_LEN, GFP_KERNEL);

	if (NULL != p_buf) {
		memset((void *)p_buf, 0xab, (size_t)DATA_FULL_BUF_LEN);
		pr_err(" ============start to make data full++!!==============\n");
		snprintf(path, (size_t)HISEE_MNTN_PATH_MAXLEN, "%s%s", PATH_ROOT, "hisee_datafull");
		while (1) {
			/* save hisee log to data/hisi_logs/hisee/hisee_log */
			cnt_1 = mntn_filesys_write_log((const char *)path,
					(void *)p_buf,
					(unsigned int)DATA_FULL_BUF_LEN,
					HISEE_FILE_PERMISSION);/*lint !e732*/
			total_cnt = total_cnt + cnt_1;
			if (cnt_1 < DATA_FULL_BUF_LEN || total_cnt > rdr_get_logsize()) {
				break;
			}
		}
		kfree(p_buf);
		pr_err(" ============start to make data full --!!==============\n");
	}
}
/********************************************************************
Description:	hisee mntn test, only for test
input:	modid:module id
output:	NA
return:	NA
********************************************************************/
int hisee_mntn_test(u32 data1, u32 data2)
{
	int ret;
	unsigned long i;
	unsigned int msg[4];
	u32 modid = data2;
	u32	irq_no = 0;

	msg[0] = LPM3_HISEE_MNTN;
	msg[1] = HISEE_TEST;
	switch (data1) {
	case 0:
		if (modid < (u32)HISI_BB_MOD_HISEE_START ||
			modid > (u32)HISI_BB_MOD_HISEE_END)
			return -ENOEXEC;

		for (i = 0; i < sizeof(hisee_exc_trans) /
			sizeof(hisee_exc_trans_s); i++)
			if (modid == hisee_exc_trans[i].module_value) {
				irq_no = hisee_exc_trans[i].irq_value;
				break;
			}
		if (irq_no) {
			msg[2] = HISEE_TEST_IRQ;
			msg[3] = irq_no;
		} else {
			msg[2] = HISEE_TEST_EXCEPTION;
			msg[3] = modid - MODID_HISEE_START;
		}
		break;
	case 1:
		msg[2] = HISEE_TEST_PRINT_PLAIN;
		msg[3] = data2;
		break;
	case 2:
		msg[2] = HISEE_TEST_RUN_COS;
		msg[3] = data2;
		break;
	case 3:
		msg[2] = HISEE_TEST_PRINT_ERR;
		msg[3] = data2;
		break;
	case 4:
		msg[2] = HISEE_TEST_PRINT_CRYPTO;
		msg[3] = data2;
		break;
	default:
		break;
	}

	ret = hisee_mntn_send_msg_to_lpm3(msg, 4);
	return ret;
}

/********************************************************************
Description: only for debug!!!the way of ecall, to dump hisee data when hisee is runing, req from hisee team, Wujianhua.
input:	NONE
output:	NA
return:	int, 0 success; -1, fail
********************************************************************/
int hisee_mntn_debug_dump(void)
{
	char path[HISEE_MNTN_PATH_MAXLEN] = {0};
	int ret;
	hlog_header *hisee_log_head = (hlog_header *)hisee_log_addr;

	pr_err(" ============somebody is using hisee mntn debug!!==============\n");
	atfd_hisi_service_hisee_mntn_smc((u64)HISEE_MNTN_ID,
		(u64)HISEE_SMC_GET_LOG, (u64)0x0, (u64)0x0);
	if (0 == hisee_log_head->real_size)
		pr_err("Hisee is pwr off!!!\n");
	else {
		snprintf(path, (size_t)HISEE_MNTN_PATH_MAXLEN, "%s%s", PATH_ROOT, HISEE_LOG_FLIENAME);

		/* save hisee log to data/hisi_logs/hisee/hisee_log */
		/*lint -e124*/
		ret = mntn_filesys_write_log((const char *)path,
				(void *)(hisee_log_addr + sizeof(hlog_header)),
				(unsigned int)hisee_log_head->real_size,
				HISEE_FILE_PERMISSION);
		/*lint +e124*/
		if (0 == ret)
			pr_err("%s:hisee log save fail\n", __func__);
	}
	return 0;
}
#endif
/********************************************************************
Description:	hisee mntn initialization
input:	NA
output:	NA
return:	NA
********************************************************************/
static int hisee_mntn_prepare_logbuf(struct platform_device *pdev)
{
	hisee_mntn_addr = hisi_bbox_map(
		(phys_addr_t)hisee_info.log_addr, (size_t)hisee_info.log_len);
	if (NULL == hisee_mntn_addr) {
		pr_err("%s:memory map fail\n", __func__);
		return -EFAULT;
	}
	/*lint -e747 -esym(747,*)*/
	hisee_log_addr = dma_alloc_coherent(&pdev->dev,
		(size_t)hisee_info.log_len, &hisee_log_phy, GFP_KERNEL);
	/*lint -e747 +esym(747,*)*/
	if (!hisee_log_addr) {
		pr_err("%s:memory alloc fail\n", __func__);
		return -ENOMEM;
	}
	pr_info("%s : v:%pK, phy : %llx\n", __func__,
		hisee_log_addr, (u64)hisee_log_phy);
	return 0;
}
/********************************************************************
Description:	hisee mntn initialization
input:	NA
output:	NA
return:	NA
********************************************************************/
static int hisee_mntn_probe(struct platform_device *pdev)
{
	int ret;
#ifndef FACTORY_TEST_HISEE
	unsigned int msg[3];

	msg[0] = LPM3_HISEE_MNTN;
	msg[1] = HISEE_MNTN_CTRL;
	msg[2] = HISEE_MNTN_DISABLE;
	if (false == is_hisee_efuse_valid()) {
		ret = hisee_mntn_send_msg_to_lpm3(msg, 3);
		pr_err("%s:hisee mntn disable, ret = %d \n", __func__, ret);
		return -EACCES;
	}
#endif
#ifdef CONFIG_HISI_HISEE_MNTN_TEST
	/*set ctrl level, usr, beta, eng, disable, etc according to the value of himntn nv*/
	hisee_mntn_set_ctrl_level();
#endif

	ret = of_reserved_mem_device_init(&pdev->dev);
	if (0 != ret) {
		pr_err("%s: init failed, ret.%d\n", __func__, ret);
		return ret;
	}

	/* register hisee exception */
	hisee_register_exception();

	/* register hisee dump and reset function */
	ret = hisee_register_core();
	if (ret < 0)
		return ret;

	ret = hisee_mntn_prepare_logbuf(pdev);
	if (ret < 0)
		return ret;
	/* initialization mailbox */
	hisee_ipc_block.next = NULL;
	hisee_ipc_block.notifier_call = rdr_hisee_msg_handler;
	ret = RPROC_MONITOR_REGISTER(HISI_RPROC_LPM3_MBX0, &hisee_ipc_block);
	if (ret != 0) {
		pr_err("%s:RPROC_MONITOR_REGISTER failed\n", __func__);
		return ret;
	}
	init_completion(&hisee_mntn_complete);
	hisee_mntn_thread = kthread_run(rdr_hisee_thread, NULL, "hisee_mntn");
	if (!hisee_mntn_thread)
		pr_err("create hisee mntn thread faild.\n");

	atfd_hisi_service_hisee_mntn_smc((u64)HISEE_MNTN_ID,
		(u64)HISEE_SMC_INIT, hisee_log_phy, (u64)hisee_info.log_len);
	pr_err("exit %s\n", __func__);

	g_hisee_mntn_state = HISEE_STATE_READY;
	return 0;
}

static int hisee_mntn_remove(struct platform_device *pdev)
{
	/*lint -e747 -esym(747,*)*/
	dma_free_coherent(&pdev->dev, (size_t)hisee_info.log_len,
		&hisee_log_phy, GFP_KERNEL);
	/*lint -e747 +esym(747,*)*/
	kthread_stop(hisee_mntn_thread);
	hisee_mntn_thread = NULL;
	return 0;
}
/*lint -e785*/
static const struct of_device_id hisee_mntn_match[] = {
	{.compatible = "hisee-mntn"},
	{}
};

static struct platform_driver hisee_mntn_driver = {
	.probe = hisee_mntn_probe,
	.remove = hisee_mntn_remove,
	.driver = {
		   .name = "hisee-mntn",
		   .of_match_table = hisee_mntn_match,
	},
};
/*lint +e785*/

static int __init hisee_mntn_init(void)
{
	/*lint -e64 -esym(64,*)*/
	return platform_driver_register(&hisee_mntn_driver);
	/*lint -e64 +esym(64,*)*/
}

static void __exit hisee_mntn_exit(void)
{
	platform_driver_unregister(&hisee_mntn_driver);
}
/*lint -e528 -esym(528,*)*/
module_init(hisee_mntn_init);
module_exit(hisee_mntn_exit);
/*lint -e528 +esym(528,*)*/
/*lint -e753 -esym(753,*)*/
MODULE_LICENSE("GPL");
/*lint -e753 +esym(753,*)*/

