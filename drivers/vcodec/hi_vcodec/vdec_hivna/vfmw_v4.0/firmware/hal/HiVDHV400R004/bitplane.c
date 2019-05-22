/*
 * vdec driver for bpd master
 *
 * Copyright (c) 2017 Hisilicon Limited
 *
 * Author: gaoyajun<gaoyajun@hisilicon.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation.
 *
 */
#include "vfmw_intf.h"
#include "vdm_hal_api.h"
#include "bitplane.h"
#ifdef HIVDEC_SMMU_SUPPORT
#include "smmu.h"
#endif

#define BPD_CLK_CTRL_OFF    (0xaeb)

VOID BPD_Start(VOID);
SINT32 BPDDRV_WaitBpdReadyIfNoIsr(VOID);

VOID BPD_ClkControl_Off(VOID)
{
    UINT32 Data;

    RD_VREG(VREG_CRG_CLK_EN, Data, 0);

    if (Data != BPD_CLK_CTRL_OFF)
    {
        WR_VREG(VREG_CRG_CLK_EN, BPD_CLK_CTRL_OFF, 0);
    }
}

static SINT32 BPD_CfgReg(OMXBPD_REG_CFG_S *pBpdRegCfg)
{
    // clear int
    WR_BPD_VREG( REG_BPD_INT_STATE, pBpdRegCfg->BpdIntState);

    WR_BPD_VREG( REG_BPD_CFG0, pBpdRegCfg->BpdCfg0);

    WR_BPD_VREG( REG_BPD_CFG1, pBpdRegCfg->BpdCfg1 );

    WR_BPD_VREG( REG_BPD_CFG2, pBpdRegCfg->BpdCfg2);

    WR_BPD_VREG( REG_BPD_CFG3, pBpdRegCfg->BpdCfg3 );

    WR_BPD_VREG( REG_BPD_CFG4, pBpdRegCfg->BpdCfg4 );

    WR_BPD_VREG( REG_BPD_CFG5, pBpdRegCfg->BpdCfg5 );

    WR_BPD_VREG( REG_BPD_CFG6, pBpdRegCfg->BpdCfg6 );

    WR_BPD_VREG( REG_BPD_CFG7, pBpdRegCfg->BpdCfg7 );

    WR_BPD_VREG( REG_BPD_CFG8, pBpdRegCfg->BpdCfg8 );

    WR_BPD_VREG( REG_BPD_CFG9, pBpdRegCfg->BpdCfg9 );

    WR_BPD_VREG( REG_BPD_CFG10, pBpdRegCfg->BpdCfg10 );

    WR_BPD_VREG( REG_BPD_CFG11, pBpdRegCfg->BpdCfg11 );
    return VDMHAL_OK;
}

SINT32 BPD_GetParam(OMXBPD_REG_STATE_S* pBpdState)
{
    UINT32 data0;
    UINT32 data1;
    BPD_OUT0 *pbpd_out0;
    BPD_OUT1 *pbpd_out1;

    RD_BPD_VREG(REG_BPD_OUT0, data0);
    pbpd_out0 = (BPD_OUT0 *)(&data0);

    RD_BPD_VREG(REG_BPD_OUT1, data1);
    pbpd_out1 = (BPD_OUT1 *)(&data1);

    pBpdState->BpdOut0EatenLenth = pbpd_out0->eaten_lenth;

    pBpdState->BpdOut1MvtypembMode = pbpd_out1->MVTYPEMBMode;
    pBpdState->BpdOut1AcpredMode = pbpd_out1->ACPREDMode;
    pBpdState->BpdOut1OverflagsMode = pbpd_out1->OVERFLAGSMode;
    pBpdState->BpdOut1FieldtxMode = pbpd_out1->FIELDTXMode;
    pBpdState->BpdOut1DirectmbMode = pbpd_out1->DIRECTMBMode;
    pBpdState->BpdOut1ForwardmbMode = pbpd_out1->FORWARDMBMode;
    pBpdState->BpdOut1SkipmbMode = pbpd_out1->SKIPMBMode;
    pBpdState->BpdOut1Condover = pbpd_out1->CONDOVER;

    return VFMW_TRUE;
}

SINT32 BPD_ConfigReg(OMXBPD_REG_S *pBpdReg)
{
    SINT32 ret;
    SINT32 Data32;

    BPD_ClkControl_Off();
    BPD_CfgReg(&(pBpdReg->BpsRegCfg));

    BPD_Start();

    ret = BPDDRV_WaitBpdReadyIfNoIsr();
    if (VFMW_TRUE == ret)
    {
        BPD_GetParam(&(pBpdReg->BpsRegState));
        return VFMW_TRUE;
    }
    return VFMW_FALSE;
}

VOID BPD_Reset( VOID )
{
    UINT32 tmp;
    UINT32 i;
    UINT32 reg_rst_ok;
    UINT32 reg;
    UINT32 div_value;
    UINT32 *pBpdResetReg = NULL;
    UINT32 *pBpdResetOkReg = NULL;

    pBpdResetReg   = (UINT32*)MEM_Phy2Vir(gSOFTRST_REQ_Addr);
    pBpdResetOkReg = (UINT32*)MEM_Phy2Vir(gSOFTRST_OK_ADDR);

    if( NULL == pBpdResetReg || NULL == pBpdResetOkReg)
    {
        dprint(PRN_FATAL, "BPD_Reset: map register fail, vir(pBpdResetReg) = (%pK), vir(pBpdResetOkReg) = (%pK)\n",
               pBpdResetReg, pBpdResetOkReg);
        return;
    }

    RD_BPD_VREG(REG_BPD_INT_MASK, tmp);

    div_value = VDMHAL_EXT_ReduceFrequency(BPD_RESET_REQUIRE);

    /* require bpd reset */
    reg= *(volatile UINT32 *)pBpdResetReg;
    *(volatile UINT32 *)pBpdResetReg = reg | (UINT32)(1<<BPD_RESET_CTRL_BIT);

    /*wait for rest ok */
    for (i=0; i<100; i++)
    {
        reg_rst_ok =*(volatile UINT32 *)pBpdResetOkReg;
        if (reg_rst_ok & (1<<BPD_RESET_OK_BIT))
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
        //dprint(PRN_ALWS,  "%s reset success!\n", __func__);
    }

    /* clear reset require */
    *(volatile UINT32 *)pBpdResetReg = reg & (UINT32)(~(1<<BPD_RESET_CTRL_BIT));

    VDMHAL_EXT_RestoreFrequency(BPD_RESET_REQUIRE, div_value);

    WR_BPD_VREG( REG_BPD_INT_MASK, tmp );

    return;
}

VOID BPD_Start(VOID)
{
#ifdef HIVDEC_SMMU_SUPPORT
#ifdef ENV_SOS_KERNEL
    SMMU_SetMasterReg(BPD, SECURE_ON,  SMMU_OFF);
#else
    SMMU_SetMasterReg(BPD, SECURE_OFF, SMMU_ON);
#endif
#endif

    // Start BPD
    WR_BPD_VREG(REG_BPD_START, 0);
    WR_BPD_VREG(REG_BPD_START, 1);
    WR_BPD_VREG(REG_BPD_START, 0);
}

SINT32 IsBpd_Ready(VOID)
{
    SINT32 Data32 = 0;

    VDMHAL_ASSERT_RET( g_HwMem[0].pBpdRegVirAddr != NULL, "BPD register not mapped yet!" );

    RD_BPD_VREG(REG_BPD_STATE, Data32);

    Data32 = Data32 & 1;
    Data32 = (Data32 == 0) ? 0 : 1;

    return Data32;
}

SINT32 BPDDRV_WaitBpdReadyIfNoIsr(VOID)
{
    UINT32 cnt_wait = 0, max_wait = 10 * WAIT_NO_ISR_MAX;
    UINT32 StartTimeInMs = 0;
    UINT32 CurTimeInMs = 0;

    StartTimeInMs = VFMW_OSAL_GetTimeInMs();

    for ( cnt_wait = 0; cnt_wait < max_wait; )
    {
        if (IsBpd_Ready())
        {
            break;
        }
        else
        {
            CurTimeInMs = VFMW_OSAL_GetTimeInMs();

            if (CurTimeInMs < StartTimeInMs)
            {
                StartTimeInMs = 0;
            }

            cnt_wait = CurTimeInMs - StartTimeInMs;
        }
    }

    if (cnt_wait < max_wait)
    {
        return VFMW_TRUE;
    }
    else
    {
        dprint(PRN_FATAL, "BPD TimeOut!\n");
        BPD_Reset();
        return VFMW_FALSE;
    }
}

