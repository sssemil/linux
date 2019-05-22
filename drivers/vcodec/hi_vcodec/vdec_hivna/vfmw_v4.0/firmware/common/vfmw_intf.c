/*
 * vfmw interface
 *
 * Copyright (c) 2017 Hisilicon Limited
 *
 * Author: gaoyajun<gaoyajun@hisilicon.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation.
 *
 */
#include <linux/module.h>
#include <linux/printk.h>
#include <linux/kern_levels.h>

#include "basedef.h"
#include "public.h"
#include "vfmw_intf.h"

#ifdef VFMW_VC1_SUPPORT
#endif
#ifdef HIVDEC_SMMU_SUPPORT
#include "smmu.h"
#include "smmu_regs.h"
#endif
#include "vdm_hal_api.h"

#ifndef IRQ_HANDLED
#define IRQ_HANDLED                 (1)
#endif

#ifndef IRQF_DISABLED
#define IRQF_DISABLED               (0x00000020)
#endif
#define VDM_TIMEOUT               (1000)//ms
#define VDM_FPGA_TIMEOUT    (500000)//ms
#define SCD_TIMEOUT                 (2000)//ms
#define SCD_FPGA_TIMEOUT      (200000)//ms

#define TIME_PERIOD(begin, end) ((end >= begin)? (end-begin):(0xffffffff - begin + end))

#ifdef HI_ADVCA_FUNCTION_RELEASE
#define ModPrint(format,arg...)    ({do{}while(0);0;})
#else
#define ModPrint(format,arg...) \
do { \
    printk(KERN_ALERT format, ## arg); \
} while (0)
#endif

#define  DO_OPEN_DRV_ERR()                                  \
do {                                                        \
    VCTRL_CloseVfmw();                                \
    return VCTRL_ERR;                                       \
} while(0)

static DRV_MEM_S g_RegsBaseAddr;

Vfmw_Osal_Func_Ptr g_vfmw_osal_fun_ptr;

VOID VCTRL_Suspend(VOID)
{
    UINT8  isScdSleep=0, isVdmSleep=0;
    UINT32 j;
    UINT32 SleepCount=0;
    UINT32 BeginTime, EntrTime, CurTime;

    EntrTime = VFMW_OSAL_GetTimeInMs();

    SCDDRV_PrepareSleep();

    for (j=0; j<MAX_VDH_NUM; j++)
    {
        VDMHAL_PrepareSleep(j);
    }

    BeginTime = VFMW_OSAL_GetTimeInMs();
    do
    {
        if (SCDDRV_SLEEP_STAGE_SLEEP == SCDDRV_GetSleepStage())
        {
            isScdSleep = 1;
        }

        for (j=0; j<MAX_VDH_NUM; j++)
        {
            if (VDMHAL_GetSleepStage(j) != VDMDRV_SLEEP_STAGE_SLEEP)
            {
                break;
            }
            if (j == MAX_VDH_NUM-1)
            {
                isVdmSleep = 1;
            }
        }

        if (1==isScdSleep && 1==isVdmSleep)
        {
            break;
        }
        else
        {
            if (SleepCount > 30)
            {
                CurTime = VFMW_OSAL_GetTimeInMs();
                dprint(PRN_FATAL,"Wait sleep time out %d ms (isScdSleep=%d, isVdmSleep=%d)!\n", TIME_PERIOD(BeginTime, CurTime), isScdSleep, isVdmSleep);
                if (isScdSleep != 1)
                {
                    dprint(PRN_FATAL,"Force scd sleep.\n");
                    SCDDRV_ForceSleep();
                }
                if(isVdmSleep != 1)
                {
                    for (j=0; j<MAX_VDH_NUM; j++)
                    {
                        dprint(PRN_FATAL,"Force vdm %d sleep.\n", j);
                        VDMHAL_ForceSleep(j);
                    }
                }
                break;
            }

            VFMW_OSAL_mSleep(10);
            SleepCount++;
        }
    }while(isScdSleep!=1 || isVdmSleep!=1);

    CurTime = VFMW_OSAL_GetTimeInMs();
    dprint(PRN_FATAL,"Vfmw suspend totally take %d ms\n", TIME_PERIOD(EntrTime, CurTime));

    return;
}

VOID VCTRL_Resume(VOID)
{
    SINT32 j;
    UINT32 EntrTime, CurTime;

    EntrTime = VFMW_OSAL_GetTimeInMs();

    //VDMHAL_IMP_WriteScdEMARID();
    SMMU_InitGlobalReg();

    SCDDRV_ExitSleep();

    for (j=0; j<MAX_VDH_NUM; j++)
    {
        VDMHAL_ExitSleep(j);
    }

    CurTime = VFMW_OSAL_GetTimeInMs();
    dprint(PRN_FATAL,"Vfmw resume totally take %d ms\n", TIME_PERIOD(EntrTime, CurTime));

    return;
}

static SINT32 VCTRL_MFDE_ISR(SINT32 irq, VOID *dev_id)
{
    VDMHAL_ISR(0);
    return IRQ_HANDLED;
}

static SINT32 VCTRL_SCD_ISR(SINT32 irq, VOID *dev_id)
{
    SCDDRV_ISR();

    return IRQ_HANDLED;
}

static SINT32 VCTRL_SMMU_ISR(SINT32 irq, VOID *dev_id)
{
    SMMU_IntServProc();

    return IRQ_HANDLED;
}

static VOID VCTRL_ISR(VOID)
{
    UINT32 D32;
    D32 = RD_SCDREG(REG_SCD_INI_CLR)&0x1;
    if(1 == D32)
    {
        dprint(PRN_ALWS, "%s(), %d, SCDDRV_ISR\n", __func__, __LINE__);
        SCDDRV_ISR();
    }

    RD_VREG(VREG_INT_STATE, D32, 0);
    if(1 == D32)
    {
        dprint(PRN_ALWS, "%s(), %d, VDMHAL_ISR\n", __func__, __LINE__);
        VDMHAL_ISR(0);
    }

    /* RD_VREG(SMMU_INTSTAT_NS, D32, 0);
    if(D32 != 0)
    {
        SMMU_IntServProc();
    }*/
}

static SINT32 VCTRL_RequestIrq(UINT32 IrqNumNorm, UINT32 IrqNumProt, UINT32 IrqNumSafe)
{
#if !defined(VDM_BUSY_WAITTING)
    if (VFMW_OSAL_RequestIrq(IrqNumNorm, VCTRL_ISR, IRQF_DISABLED, "vdec_norm_irq", NULL) != 0)//for 2.6.24以后
    {
        dprint(PRN_FATAL, "Request vdec norm irq %d failed!\n", IrqNumNorm);
        return VCTRL_ERR;
    }
#endif

#if !defined(SMMU_BUSY_WAITTING)
#ifdef ENV_SOS_KERNEL
    if (VFMW_OSAL_RequestIrq(IrqNumProt, VCTRL_ISR, IRQF_DISABLED, "vdec_prot_smmu_irq", NULL) != 0)//for 2.6.24以后
    {
        dprint(PRN_FATAL, "Request vdec prot irq %d failed!\n", IrqNumProt);
        return VCTRL_ERR;
    }
#endif
#endif

    return VCTRL_OK;
}

static VOID VCTRL_FreeIrq(UINT32 IrqNumNorm, UINT32 IrqNumProt, UINT32 IrqNumSafe)
{
#if !defined(VDM_BUSY_WAITTING)
    VFMW_OSAL_FreeIrq(IrqNumNorm, NULL);
#endif

#if !defined(SMMU_BUSY_WAITTING)
#ifdef ENV_SOS_KERNEL
    VFMW_OSAL_FreeIrq(IrqNumProt, NULL);
#endif
#endif
}

static SINT32 VCTRL_HalInit(VOID)
{
#ifdef HIVDEC_SMMU_SUPPORT
    if ( SMMU_OK != SMMU_Init() )
    {
        dprint(PRN_FATAL, "SMMU_Init failed!\n");
        return VCTRL_ERR;
    }
#endif

    SCDDRV_init();
    VDMHAL_IMP_Init();
    SMMU_InitGlobalReg();

    return VCTRL_OK;
}

static VOID VCTRL_HalDeInit(VOID)
{
#ifdef HIVDEC_SMMU_SUPPORT
    SMMU_DeInit();
#endif
    VDMHAL_IMP_DeInit();
    SCDDRV_DeInit();
}

SINT32 VCTRL_OpenDrivers(VOID)
{
    MEM_RECORD_S *pstMem;
    SINT32  ret = VCTRL_ERR;

    dprint(PRN_ALWS, "%s, %d gVdhRegBaseAddr 0x%x", __func__, __LINE__, gVdhRegBaseAddr);
    pstMem = &g_RegsBaseAddr.stVdhReg;
    if( MEM_MAN_OK == MEM_MapRegisterAddr(gVdhRegBaseAddr, 256*1024, pstMem ) )
    {
        if (MEM_AddMemRecord(pstMem->PhyAddr, pstMem->VirAddr, pstMem->Length) != MEM_MAN_OK)
        {
            dprint(PRN_ERROR, "%s %d MEM_AddMemRecord failed!\n", __func__, __LINE__);
            goto exit;
        }
    }
    else
    {
        dprint(PRN_FATAL, "Map vdh register failed! gVdhRegBaseAddr = 0x%x, gVdhRegRange = %d\n", gVdhRegBaseAddr, gVdhRegRange);
        goto exit;
    }

#if 1   // del PERICRG_REG
    pstMem = &g_RegsBaseAddr.stCrgReg;
    if( MEM_MAN_OK == MEM_MapRegisterAddr( gPERICRG_RegBaseAddr, 0x100, pstMem ) )
    {
        if (MEM_AddMemRecord(pstMem->PhyAddr, pstMem->VirAddr, pstMem->Length) != MEM_MAN_OK)
        {
            dprint(PRN_ERROR, "%s %d MEM_AddMemRecord failed!\n", __func__, __LINE__);
            goto exit;
        }
    }
    else
    {
        dprint(PRN_FATAL, "Map crg register failed! gPERICRG_RegBaseAddr = 0x%x, RegRange = 0x100\n", gPERICRG_RegBaseAddr);
        goto exit;
    }
#endif

    ret = VCTRL_RequestIrq(gVdecIrqNumNorm, gVdecIrqNumProt, gVdecIrqNumSafe);
    if (ret != VCTRL_OK)
    {
        dprint(PRN_FATAL, "VCTRL_RequestIrq failed!\n");
        goto exit;
    }

    if (VCTRL_OK != VCTRL_HalInit())
    {
        dprint(PRN_FATAL, "VCTRL_HalInit failed!\n");
        goto exit;
    }

    return VCTRL_OK;

exit:
    DO_OPEN_DRV_ERR();
}

SINT32 VCTRL_OpenVfmw(VOID)
{
    memset(&g_RegsBaseAddr, 0, sizeof(DRV_MEM_S));

    MEM_InitMemManager();
    if (VCTRL_OK != VCTRL_OpenDrivers())
    {
        dprint(PRN_FATAL, "OpenDrivers fail\n");
        return VCTRL_ERR;
    }


    return VCTRL_OK;
}

SINT32 VCTRL_CloseVfmw(VOID)
{
    MEM_RECORD_S *pstMem;

    VCTRL_HalDeInit();

    pstMem = &g_RegsBaseAddr.stVdhReg;
    if( 0 != pstMem->Length )
    {
        MEM_UnmapRegisterAddr(pstMem->PhyAddr, pstMem->VirAddr, pstMem->Length);
        memset(pstMem, 0, sizeof(MEM_RECORD_S));
    }
    MEM_DelMemRecord(pstMem->PhyAddr, pstMem->VirAddr, pstMem->Length);

    pstMem = &g_RegsBaseAddr.stCrgReg;
    if( 0 != pstMem->Length )
    {
        MEM_UnmapRegisterAddr(pstMem->PhyAddr, pstMem->VirAddr, pstMem->Length);
        memset(pstMem, 0, sizeof(MEM_RECORD_S));
    }
    MEM_DelMemRecord(pstMem->PhyAddr, pstMem->VirAddr, pstMem->Length);

    VCTRL_FreeIrq(gVdecIrqNumNorm, gVdecIrqNumProt, gVdecIrqNumSafe);

    return VCTRL_OK;

}

SINT32 VCTRL_VDMHal_Process(OMXVDH_REG_CFG_S *pVdmRegCfg, VDMHAL_BACKUP_S *pVdmRegState)
{
        HI_S32 Ret = HI_SUCCESS;
        CONFIG_VDH_CMD cmd = pVdmRegCfg->vdh_cmd;
#if 0
        if (pVdmRegCfg->GlbResetFlag) {
                VDMDRV_GlbReset();
                VDMDRV_InitConfig();
        }

#endif
        if (pVdmRegCfg->vdh_reset_flag) {
               VDMHAL_IMP_ResetVdm(0);
        }

        switch (cmd) {
        case CONFIG_VDH_AfterDec_CMD:
                VDMHAL_AfterDec(pVdmRegCfg);
                break;
        case CONFIG_VDH_ACTIVATEDEC_CMD:
                VDMHAL_ActivateVDH(pVdmRegCfg);
                break;
        default:
                dprint(PRN_FATAL," %s  , cmd type unknown:%d\n", cmd, __func__);
                return HI_FAILURE;
                break;
       }

    Ret = VFMW_OSAL_WaitEvent(G_VDMHWDONEEVENT, ((0 == gIsFPGA) ? VDM_TIMEOUT : VDM_FPGA_TIMEOUT));
    if (Ret == HI_SUCCESS) {
        VDMHAL_GetRegState(pVdmRegState);
    }
    else
    {
        dprint(PRN_FATAL, "VFMW_OSAL_WaitEvent wait time out!\n");
        VDMHAL_IMP_ResetVdm(0);
        return HI_FAILURE;
    }

    return HI_SUCCESS;
}

SINT32 VCTRL_SCDHal_Process(OMXSCD_REG_CFG_S *pScdRegCfg,SCD_STATE_REG_S *pScdStateReg)
{
    HI_S32 Ret = HI_SUCCESS;
    CONFIG_SCD_CMD cmd = pScdRegCfg->cmd;
#if 0
   if (pScdRegCfg->GlbResetFlag) {
       VDMDRV_GlbReset();
       VDMDRV_InitConfig();
   }

#endif
    if (pScdRegCfg->SResetFlag)
    {
        if (SCDDRV_ResetSCD() != HI_SUCCESS)
        {
            dprint(PRN_FATAL, "VDEC_IOCTL_SCD_WAIT_HW_DONE  Reset SCD failed!\n");
            return HI_FAILURE;
        }
    }

    switch (cmd)
    {
        case CONFIG_SCD_REG_CMD:
            SCDDRV_WriteReg(&pScdRegCfg->SmCtrlReg);
            Ret = VFMW_OSAL_WaitEvent(G_SCDHWDONEEVENT, ((0 == gIsFPGA) ? SCD_TIMEOUT : SCD_FPGA_TIMEOUT));
            if (Ret == HI_SUCCESS) {
                SCDDRV_GetRegState(pScdStateReg);
            }
            else
            {
                dprint(PRN_ALWS, "VDEC_IOCTL_SCD_WAIT_HW_DONE  wait time out!\n");
                SCDDRV_ResetSCD();
                return HI_FAILURE;
            }
            break;
        default:
            dprint(PRN_ALWS," cmd type unknown:%d\n", cmd);
            return HI_FAILURE;
    }

    return HI_SUCCESS;
}

SINT32 VCTRL_BPDHal_Process(OMXBPD_REG_S *pBpdReg)
{
#if 0
    if (pBpdReg->GlbResetFlag) {
        VDMDRV_GlbReset();
        VDMDRV_InitConfig();
    }
#endif

    if (BPD_ConfigReg(pBpdReg)) {
        dprint(PRN_FATAL, "omxvdec_config_bpd:  failed!\n");
        return HI_FAILURE;
    }
    return HI_SUCCESS;
}

SINT32 VCTRL_VDMHAL_IsRun(VOID)
{
    return VDMHAL_IsVdmRun(0);
}

HI_S32 VFMW_DRV_ModInit(HI_VOID)
{
    OSAL_InitInterface();
    VFMW_OSAL_SemaInit(G_VfmwSem);
    VFMW_OSAL_InitEvent(G_INTEVENT, 1);
    VFMW_OSAL_InitEvent(G_SCDHWDONEEVENT, 0);
    VFMW_OSAL_InitEvent(G_VDMHWDONEEVENT, 0);

    VFMW_OSAL_SpinLockInit(G_SPINLOCK_THREAD);
    VFMW_OSAL_SpinLockInit(G_SPINLOCK_RECORD);
    VFMW_OSAL_SpinLockInit(G_SPINLOCK_VOQUEUE);
    VFMW_OSAL_SpinLockInit(G_SPINLOCK_FSP);
    VFMW_OSAL_SpinLockInit(G_SPINLOCK_DESTROY);

#ifdef MODULE
    ModPrint("Load hi_vfmw.ko (%d) success.\n", VFMW_VERSION_NUM);
#endif

    return 0;
}

HI_VOID VFMW_DRV_ModExit(HI_VOID)
{
#ifdef MODULE
    ModPrint("Unload hi_vfmw.ko (%d) success.\n", VFMW_VERSION_NUM);
#endif

    return ;
}

module_init(VFMW_DRV_ModInit);
module_exit(VFMW_DRV_ModExit);

MODULE_AUTHOR("gaoyajun");
MODULE_LICENSE("GPL");

