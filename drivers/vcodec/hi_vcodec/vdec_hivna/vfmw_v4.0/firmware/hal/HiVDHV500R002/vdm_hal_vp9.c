/*-----------------------------------------------------------------------*/
/*!!Warning: Huawei key information asset. No spread without permission. */
/*CODEMARK:EG4uRhTwMmgcVFBsBnYHCDadN5jJKSuVyxmmaCmKFU6eJEbB2fyHF9weu4/jer/hxLHb+S1e
E0zVg4C3NiZh4Rryzsvo1gOdvy7M+qFCBFQKTTAFAVC3Q4e533WXdeQrddo4r2cqTmRg3Xeb
SI3trXaSV012ETxvJrJ/pkfs27/lT6wemL9iW3PaGW8//pmW7hQ7qCDBgWp7sMvcMuyYAWRh
jMb6+4xlgVl55z+iUl5XDCi0pMRG2hXB2hXZd5i/HJastZrWJFR4dVOatPlImg==#*/
/*--!!Warning: Deleting or modifying the preceding information is prohibited.--*/



/******************************************************************************

  版权所有 (C), 2001-2015, 华为技术有限公司

******************************************************************************
    文 件 名   : vdm_hal_vp9.c
    版 本 号   : 初稿
    作        者   : z00290437
    生成日期: 2015-02-03
    最近修改 :
    功能描述 : VDMV300 硬件抽象

  修改历史     :
    1.日    期       :
    2.作    者       :
    3.修改内容:

******************************************************************************/

#ifndef __VDM_HAL_VP9_C__
#define __VDM_HAL_VP9_C__

#include "public.h"
#include "vfmw.h"
#include "vdm_hal.h"
#include "vdm_hal_local.h"
#include "vdm_hal_vp9.h"

extern UINT32 g_ddr_interleave_value;

UINT8 g_colmvBuf[1024*1024];

/************************************************************************/
/*  函数实现                                                            */
/************************************************************************/

SINT32 VP9HAL_CfgReg(OMXVDH_REG_CFG_S *pVdhRegCfg)
{
    UINT32 D32;

    //BASIC_CFG0
    WR_VREG(VREG_BASIC_CFG0, pVdhRegCfg->VdhBasicCfg0, 0);
    dprint(PRN_FATAL, "VREG_BASIC_CFG0 = 0x%x\n", pVdhRegCfg->VdhBasicCfg0);

    //BASIC_CFG1
    WR_VREG(VREG_BASIC_CFG1, pVdhRegCfg->VdhBasicCfg1, 0);
    dprint(PRN_VDMREG, "BASIC_CFG1 = 0x%x\n", pVdhRegCfg->VdhBasicCfg1);

    //AVM_ADDR
    WR_VREG(VREG_AVM_ADDR, pVdhRegCfg->VdhAvmAddr, 0);
    dprint(PRN_VDMREG, "AVM_ADDR = 0x%x\n", pVdhRegCfg->VdhAvmAddr);

    //VAM_ADDR
    WR_VREG(VREG_VAM_ADDR, pVdhRegCfg->VdhVamAddr, 0);
    dprint(PRN_VDMREG, "VAM_ADDR = 0x%x\n", pVdhRegCfg->VdhVamAddr);

    //STREAM_BASE_ADDR
    WR_VREG(VREG_STREAM_BASE_ADDR, pVdhRegCfg->VdhStreamBaseAddr, 0);
    dprint(PRN_VDMREG, "STREAM_BASE_ADDR = 0x%x\n", pVdhRegCfg->VdhStreamBaseAddr);

    //PPFD_BUF_ADDR
    WR_VREG(VREG_PPFD_BUF_ADDR, pVdhRegCfg->VdhPpfdBufAddr, 0);
    dprint(PRN_VDMREG, "PPFD_BUF_ADDR = 0x%x\n", pVdhRegCfg->VdhPpfdBufAddr);

    //PPFD_BUF_LEN
    WR_VREG(VREG_PPFD_BUF_LEN, pVdhRegCfg->VdhPpfdBufLen, 0);
    dprint(PRN_VDMREG, "PPFD_BUF_LEN = 0x%x\n", pVdhRegCfg->VdhPpfdBufLen);

    //PRC_CACHE_TYPE
    WR_VREG(VREG_FF_APT_EN, pVdhRegCfg->VdhFfAptEn, 0);
    dprint(PRN_VDMREG, "VREG_FF_APT_EN = 0x%x\n", pVdhRegCfg->VdhFfAptEn);

    //TIME_OUT
    D32 = 0x00300C03;
    WR_VREG(VREG_SED_TO,    D32, 0);
    WR_VREG(VREG_ITRANS_TO, D32, 0);
    WR_VREG(VREG_PMV_TO,    D32, 0);
    WR_VREG(VREG_PRC_TO,    D32, 0);
    WR_VREG(VREG_RCN_TO,    D32, 0);
    WR_VREG(VREG_DBLK_TO,   D32, 0);
    WR_VREG(VREG_PPFD_TO,   D32, 0);

    //DEC_OVER_INT_LEVEL
    WR_VREG(VREG_PART_DEC_OVER_INT_LEVEL, pVdhRegCfg->VdhPartLevel, 0);
    dprint(PRN_VDMREG, "VREG_PART_DEC_OVER_INT_LEVEL=0x%x\n", pVdhRegCfg->VdhPartLevel);

    //YSTADDR_1D
    WR_VREG(VREG_YSTADDR_1D, pVdhRegCfg->VdhYstAddr, 0);
    dprint(PRN_VDMREG, "YSTADDR_1D = 0x%x\n", pVdhRegCfg->VdhYstAddr);

    //YSTRIDE_1D
    WR_VREG(VREG_YSTRIDE_1D, pVdhRegCfg->VdhYstride, 0);
    dprint(PRN_VDMREG, "YSTRIDE_1D = 0x%x\n", pVdhRegCfg->VdhYstride);

    //UVOFFSET_1D
    WR_VREG(VREG_UVOFFSET_1D, pVdhRegCfg->VdhUvoffset, 0);
    dprint(PRN_VDMREG, "UVOFFSET_1D = 0x%x\n", pVdhRegCfg->VdhUvoffset);

    //UVSTRIDE_1D
    WR_VREG(VREG_UVSTRIDE_1D, pVdhRegCfg->VdhUvstride, 0);
    dprint(PRN_VDMREG, "UVSTRIDE_1D = 0x%x\n", pVdhRegCfg->VdhUvstride);

    //CFGINFO_ADDR
    WR_VREG(VREG_CFGINFO_ADDR, pVdhRegCfg->VdhCfgInfoAddr, 0);
    dprint(PRN_VDMREG, "pPicParam->cfginfo_msg_addr:%x\n", pVdhRegCfg->VdhCfgInfoAddr);

     //DDR_INTERLEAVE_MODE
    WR_VREG(VREG_DDR_INTERLEAVE_MODE, pVdhRegCfg->VdhDdrInterleaveMode, 0);

    return VDMHAL_OK;
}

SINT32 VP9HAL_StartDec(OMXVDH_REG_CFG_S *pVdhRegCfg)
{
    VP9HAL_CfgReg(pVdhRegCfg);

    return VDMHAL_OK;
}

#endif
