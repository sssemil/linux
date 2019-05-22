/*
 * vdec driver for scd master
 *
 * Copyright (c) 2017 Hisilicon Limited
 *
 * Author: gaoyajun<gaoyajun@hisilicon.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation.
 *
 */
#include "public.h"
#include "scd_drv.h"
#include "vfmw_intf.h"
#include "vdm_hal_api.h"
#ifdef HIVDEC_SMMU_SUPPORT
#include "smmu.h"
#endif

static SCDDRV_SLEEP_STAGE_E  s_eScdDrvSleepStage = SCDDRV_SLEEP_STAGE_NONE;
static SINT32 IsScdDrvInit;
static SCD_STATE_REG_S gScdStateReg;
SINT32 s_SCDState;

static VOID PrintScdVtrlReg(VOID);

SINT32 SCDDRV_ResetSCD(VOID)
{
    UINT32 tmp;
    UINT32 i;
    UINT32 reg_rst_ok;
    UINT32 reg;
    UINT32 div_value;
    UINT32 *pScdResetReg = NULL;
    UINT32 *pScdResetOkReg = NULL;

    pScdResetReg   = (UINT32*)MEM_Phy2Vir(gSOFTRST_REQ_Addr);
    pScdResetOkReg = (UINT32*)MEM_Phy2Vir(gSOFTRST_OK_ADDR);

    if( NULL == pScdResetReg || NULL == pScdResetOkReg)
    {
        dprint(PRN_FATAL,"scd reset register map fail!\n");
        return VF_ERR_SYS;
    }

    tmp = RD_SCDREG(REG_SCD_INT_MASK);

    div_value = VDMHAL_EXT_ReduceFrequency(SCD_RESET_REQUIRE);

    reg= *(volatile UINT32 *)pScdResetReg;
    *(volatile UINT32 *)pScdResetReg = reg | (UINT32)(1<<SCD_RESET_CTRL_BIT);

    for(i=0; i<100; i++)
    {
        reg_rst_ok =*(volatile UINT32 *)pScdResetOkReg;
        if( reg_rst_ok & (1 <<SCD_RESET_OK_BIT))
        {
            break;
        }
        VFMW_OSAL_uDelay(10);
    }

    if (i >= 100)
    {
        dprint(PRN_FATAL, "%s reset failed!\n", __func__);
    }
    else
    {
        dprint(PRN_ALWS,  "%s reset success!\n", __func__);
    }

    *(volatile UINT32 *)pScdResetReg = reg & (UINT32)(~(1<<SCD_RESET_CTRL_BIT));

    VDMHAL_EXT_RestoreFrequency(SCD_RESET_REQUIRE, div_value);

    WR_SCDREG(REG_SCD_INT_MASK, tmp);

    s_SCDState = 0;
    return FMW_OK;
}

SINT32 SCDDRV_PrepareSleep(VOID)
{
    SINT32 ret = SCDDRV_OK;

    if (s_eScdDrvSleepStage == SCDDRV_SLEEP_STAGE_NONE)
    {
        if (0 == s_SCDState)
        {
            s_eScdDrvSleepStage = SCDDRV_SLEEP_STAGE_SLEEP;
        }
        else
        {
            s_eScdDrvSleepStage = SCDDRV_SLEEP_STAGE_PREPARE;
        }

        ret = SCDDRV_OK;
    }
    else
    {
        ret = SCDDRV_ERR;
    }

    return ret;
}

SCDDRV_SLEEP_STAGE_E SCDDRV_GetSleepStage(VOID)
{
    return s_eScdDrvSleepStage;
}

VOID SCDDRV_ForceSleep(VOID)
{
    if (s_eScdDrvSleepStage != SCDDRV_SLEEP_STAGE_SLEEP)
    {
        s_eScdDrvSleepStage = SCDDRV_SLEEP_STAGE_SLEEP;
    }

    return;
}

VOID SCDDRV_ExitSleep(VOID)
{
    s_eScdDrvSleepStage = SCDDRV_SLEEP_STAGE_NONE;
    return;
}

VOID SCDDRV_WriteReg(SM_CTRLREG_S *pSmCtrlReg)
{
    WR_SCDREG(REG_SCD_INI_CLR, 1);

    // LIST_ADDRESS
    WR_SCDREG(REG_LIST_ADDRESS, (unsigned int)pSmCtrlReg->DownMsgPhyAddr);

    // UP_ADDRESS
    WR_SCDREG(REG_UP_ADDRESS, (UINT32)pSmCtrlReg->UpMsgPhyAddr);

    // UP_LEN
    WR_SCDREG(REG_UP_LEN,(UINT32) pSmCtrlReg->UpLen);

    // BUFFER_FIRST
    WR_SCDREG(REG_BUFFER_FIRST, (UINT32)pSmCtrlReg->BufferFirst);

    // BUFFER_LAST
    WR_SCDREG(REG_BUFFER_LAST, (UINT32)pSmCtrlReg->BufferLast);

    // BUFFER_INI
    WR_SCDREG(REG_BUFFER_INI, (UINT32)pSmCtrlReg->BufferIni);

    // AVS_FLAG
    WR_SCDREG(REG_AVS_FLAG, (UINT32)pSmCtrlReg->reg_avs_flag);

#ifdef VFMW_SCD_LOWDLY_SUPPORT
    // SCD LOW DELAY
    WR_SCDREG(REG_DSP_SPS_MSG_ADDRESS, (UINT32)pSmCtrlReg->DspSpsMsgMemAddr);
    WR_SCDREG(REG_DSP_PPS_MSG_ADDRESS, (UINT32)pSmCtrlReg->DspPpsMsgMemAddr);
    WR_SCDREG(REG_DVM_MSG_ADDRESS,     (UINT32)pSmCtrlReg->DvmMemAddr);
    WR_SCDREG(REG_SED_TOP_ADDRESS,     (UINT32)pSmCtrlReg->DspSedTopMemAddr);
    WR_SCDREG(REG_CA_MN_ADDRESS,       (UINT32)pSmCtrlReg->DspCaMnMemAddr);
#endif

    // SCD_PROTOCOL
#ifdef ENV_SOS_KERNEL
    WR_SCDREG(REG_SCD_PROTOCOL, (UINT32)((pSmCtrlReg->ScdLowdlyEnable << 8)
                                      | (1<<7) /* sec mode */
                                      | ((pSmCtrlReg->SliceCheckFlag << 4) & 0x10)
                                      | (pSmCtrlReg->ScdProtocol& 0x0f)));
#ifdef HIVDEC_SMMU_SUPPORT
    SMMU_SetMasterReg(SCD, SECURE_ON,  SMMU_OFF);
#endif

#else
    WR_SCDREG(REG_SCD_PROTOCOL, (UINT32)((pSmCtrlReg->ScdLowdlyEnable << 8)
                                      | ((pSmCtrlReg->SliceCheckFlag << 4) & 0x10)
                                      | (pSmCtrlReg->ScdProtocol& 0x0f)));
#ifdef HIVDEC_SMMU_SUPPORT
    SMMU_SetMasterReg(SCD, SECURE_OFF, SMMU_ON);
#endif

#endif

#ifndef SCD_BUSY_WAITTING
    WR_SCDREG(REG_SCD_INT_MASK, 0);
#endif

    PrintScdVtrlReg();

    // SCD_START
    WR_SCDREG(REG_SCD_START, 0);
    WR_SCDREG(REG_SCD_START, (UINT32)(pSmCtrlReg->ScdStart & 0x01));
}

VOID SCDDRV_SaveStateReg(VOID)
{
    gScdStateReg.ScdProtocol = RD_SCDREG(REG_SCD_PROTOCOL);
    gScdStateReg.Scdover = RD_SCDREG(REG_SCD_OVER);
    gScdStateReg.ScdInt = RD_SCDREG(REG_SCD_INT);
    gScdStateReg.ScdNum = RD_SCDREG(REG_SCD_NUM);
    gScdStateReg.ScdRollAddr = RD_SCDREG(REG_ROLL_ADDR);
    gScdStateReg.SrcEaten = RD_SCDREG(REG_SRC_EATEN);
    gScdStateReg.UpLen = RD_SCDREG(REG_UP_LEN);
}

static SINT32 CheckScdStateReg(SM_STATEREG_S *pSmStateReg, SINT32 StdType)
{
    FMW_ASSERT_RET((0 != pSmStateReg->ScdRollAddr), FMW_ERR_SCD);
    return FMW_OK;
}

VOID SCDDRV_init(VOID)
{
    memset(&gScdStateReg, 0, sizeof(gScdStateReg));
    s_eScdDrvSleepStage = SCDDRV_SLEEP_STAGE_NONE;
    IsScdDrvInit = 1;
    s_SCDState = 0;
}

VOID SCDDRV_DeInit(VOID)
{
    s_eScdDrvSleepStage = SCDDRV_SLEEP_STAGE_NONE;
    IsScdDrvInit = 0;
    s_SCDState = 0;
}

VOID SCDDRV_ISR( VOID )
{
	SINT32 dat = 0;
	VFMW_OSAL_SpinLock(G_SPINLOCK_THREAD);
	dat = RD_SCDREG(REG_SCD_OVER) & 0x01;
	if((dat & 1) == 0)
    {
		VFMW_OSAL_SpinUnLock(G_SPINLOCK_THREAD);
		printk(KERN_ERR "End0: SM_SCDIntServeProc()!\n");
		return;
	}

	WR_SCDREG(REG_SCD_INI_CLR, 1);
	SCDDRV_SaveStateReg();
	VFMW_OSAL_GiveEvent(G_SCDHWDONEEVENT);
    VFMW_OSAL_SpinUnLock(G_SPINLOCK_THREAD);
}

VOID SCDDRV_GetRegState(SCD_STATE_REG_S *pScdStateReg)
{
	memcpy(pScdStateReg, &gScdStateReg, sizeof(SCD_STATE_REG_S));
}

SINT32 WaitSCDFinish(VOID)
{
    SINT32 i;

    if ( 1 == s_SCDState )
    {
        for ( i=0; i<SCD_TIME_OUT_COUNT; i++ )
        {
            if ( (RD_SCDREG( REG_SCD_OVER ) & 1) )
            {
                return FMW_OK;
            }
            else
            {
                //OSAL_MSLEEP(10);
            }
        }

        return FMW_ERR_SCD;
    }
    else
    {
        return FMW_OK;
    }
}

static VOID PrintScdVtrlReg(VOID)
{
    SM_CTRLREG_S SmCtrlReg;
    memset(&SmCtrlReg, 0, sizeof(SmCtrlReg));

    SmCtrlReg.DownMsgPhyAddr = RD_SCDREG(REG_LIST_ADDRESS);
    SmCtrlReg.UpMsgPhyAddr = RD_SCDREG(REG_UP_ADDRESS);
    SmCtrlReg.UpLen = RD_SCDREG(REG_UP_LEN);
    SmCtrlReg.BufferFirst = RD_SCDREG(REG_BUFFER_FIRST);
    SmCtrlReg.BufferLast = RD_SCDREG(REG_BUFFER_LAST);
    SmCtrlReg.BufferIni= RD_SCDREG(REG_BUFFER_INI);
    SmCtrlReg.ScdProtocol = RD_SCDREG(REG_SCD_PROTOCOL);
    SmCtrlReg.ScdStart = RD_SCDREG(REG_SCD_START);

    dprint(PRN_SCD_REGMSG,"***Print Scd Vtrl Reg Now\n");
    dprint(PRN_SCD_REGMSG,"DownMsgPhyAddr = %x\n", SmCtrlReg.DownMsgPhyAddr);
    dprint(PRN_SCD_REGMSG,"UpMsgPhyAddr = %x\n", SmCtrlReg.UpMsgPhyAddr);
    dprint(PRN_SCD_REGMSG,"UpLen = %x\n", SmCtrlReg.UpLen);
    dprint(PRN_SCD_REGMSG,"BufferFirst = %x\n", SmCtrlReg.BufferFirst);
    dprint(PRN_SCD_REGMSG,"BufferLast = %x\n", SmCtrlReg.BufferLast);
    dprint(PRN_SCD_REGMSG,"BufferIni = %x\n", SmCtrlReg.BufferIni);
    dprint(PRN_SCD_REGMSG,"ScdProtocol = %x\n", SmCtrlReg.ScdProtocol);
    dprint(PRN_SCD_REGMSG,"ScdStart = %x\n", SmCtrlReg.ScdStart);
}

#if 0 //debug info
static VOID PrintScdStateReg(SM_STATEREG_S *pSmStateReg)
{
    dprint(PRN_SCD_REGMSG, "***Print Scd State Reg\n");
    dprint(PRN_SCD_REGMSG, "Scdover = %d \n",pSmStateReg->Scdover);
    dprint(PRN_SCD_REGMSG, "ScdInt = %d \n",pSmStateReg->ScdInt);
    dprint(PRN_SCD_REGMSG, "ShortScdNum = %d \n",pSmStateReg->ShortScdNum);
    dprint(PRN_SCD_REGMSG, "ScdNum = %d \n",pSmStateReg->ScdNum);
    dprint(PRN_SCD_REGMSG, "ScdRollAddr = %0x \n",pSmStateReg->ScdRollAddr);
    dprint(PRN_SCD_REGMSG, "SrcEaten = %d \n",pSmStateReg->SrcEaten);
}
#endif

#ifdef ENV_ARMLINUX_KERNEL
SINT32 SCDDRV_IsScdIdle(VOID)
{
   SINT32 ret = VFMW_TRUE;
   if (0 == s_SCDState)
   {
       ret = VFMW_TRUE;
   }
   else if (1 == s_SCDState)
   {
       ret = VFMW_FALSE;
   }
   else
   {
       ret = VFMW_FALSE;
       dprint(PRN_ERROR, "%s(): s_SCDState(%d) is wrong.\n", __func__, s_SCDState);
   }
   return ret;
}

#endif

