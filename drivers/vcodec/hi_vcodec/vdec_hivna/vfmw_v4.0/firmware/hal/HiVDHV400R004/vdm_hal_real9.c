/*
 * vdec hal for rv9
 *
 * Copyright (c) 2017 Hisilicon Limited
 *
 * Author: gaoyajun<gaoyajun@hisilicon.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation.
 *
 */

#ifndef __VDM_HAL_REAL9_C__
#define __VDM_HAL_REAL9_C__

#include "public.h"
#include "vdm_hal_api.h"
#include "vdm_hal_local.h"
#include "vdm_hal_real9.h"
#include "vdm_hal.h"

VOID RV9HAL_WriteReg(OMXVDH_REG_CFG_S *pVdhRegCfg);

SINT32 RV9HAL_StartDec(OMXVDH_REG_CFG_S *pVdhRegCfg)
{
    RV9HAL_WriteReg(pVdhRegCfg);
    return VDMHAL_OK;
}

VOID RV9HAL_WriteReg(OMXVDH_REG_CFG_S *pVdhRegCfg)
{
    UINT32 datTmp;
    SINT32 D32;

    dprint(PRN_CTRL, "configuring VDM registers...\n");

    WR_VREG( VREG_BASIC_CFG0, pVdhRegCfg->VdhBasicCfg0, 0 );
    dprint(PRN_VDMREG, "BASIC_CFG0 = 0x%x\n", pVdhRegCfg->VdhBasicCfg0);

    /*set uv order 0: v first; 1: u first*/
    WR_VREG( VREG_BASIC_CFG1, pVdhRegCfg->VdhBasicCfg1, 0 );
    dprint(PRN_VDMREG, "BASIC_CFG1=0x%x\n", pVdhRegCfg->VdhBasicCfg1);

    //AVM_ADDR
    WR_VREG( VREG_AVM_ADDR, pVdhRegCfg->VdhAvmAddr, 0 );
    dprint(PRN_VDMREG, "AVM_ADDR=0x%x\n", pVdhRegCfg->VdhAvmAddr);

    //VAM_ADDR
    WR_VREG( VREG_VAM_ADDR, pVdhRegCfg->VdhVamAddr, 0 );
    dprint(PRN_VDMREG, "VAM_ADDR=0x%x\n", pVdhRegCfg->VdhVamAddr);

    //STREAM_BASE_ADDR
    WR_VREG( VREG_STREAM_BASE_ADDR, pVdhRegCfg->VdhStreamBaseAddr, 0 );
    dprint(PRN_VDMREG, "STREAM_BASE_ADDR=0x%x\n", pVdhRegCfg->VdhStreamBaseAddr);

    //EMAR_ID
    datTmp = RD_SCDREG(REG_EMAR_ID);
    if (0 == pVdhRegCfg->VdhEmarId)
    {
        datTmp = datTmp & (~(0x10000));
    }
    else
    {
        datTmp = datTmp | (0x10000);
    }
    WR_SCDREG(REG_EMAR_ID, datTmp);
    //TIME_OUT
    D32 = 0x00300C03;
    WR_VREG( VREG_SED_TO,    D32, 0 );
    WR_VREG( VREG_ITRANS_TO, D32, 0 );
    WR_VREG( VREG_PMV_TO,    D32, 0 );
    WR_VREG( VREG_PRC_TO,    D32, 0 );
    WR_VREG( VREG_RCN_TO,    D32, 0 );
    WR_VREG( VREG_DBLK_TO,   D32, 0 );
    WR_VREG( VREG_PPFD_TO,   D32, 0 );
    dprint(PRN_VDMREG, "TIME_OUT = 0x%x\n", D32);

    //YSTADDR_1D
    WR_VREG( VREG_YSTADDR_1D, pVdhRegCfg->VdhYstAddr, 0 );
    dprint(PRN_VDMREG, "YSTADDR_1D = 0x%x\n", pVdhRegCfg->VdhYstAddr);

    //YSTRIDE_1D
    WR_VREG( VREG_YSTRIDE_1D, pVdhRegCfg->VdhYstride, 0 );
    dprint(PRN_VDMREG, "YSTRIDE_1D = 0x%x\n", pVdhRegCfg->VdhYstride);

    //UVOFFSET_1D
    WR_VREG( VREG_UVOFFSET_1D, pVdhRegCfg->VdhUvoffset, 0 );
    dprint(PRN_VDMREG, "UVOFFSET_1D = 0x%x\n", pVdhRegCfg->VdhUvoffset);

    //PRCNUM_1D_CNT
    WR_VREG( VREG_HEAD_INF_OFFSET, pVdhRegCfg->VdhHeadInfOffset, 0);
    //  dprint(PRN_VDMREG, "PRCNUM_1D_CNT = 0x%x\n", D32);
    //
    //REF_PIC_TYPE
    D32 = 0;
    WR_VREG( VREG_REF_PIC_TYPE, pVdhRegCfg->VdhRefPicType, 0 );
    dprint(PRN_VDMREG, "REF_PIC_TYPE=0x%x\n", pVdhRegCfg->VdhRefPicType);

    //FF_APT_EN
    WR_VREG( VREG_FF_APT_EN, pVdhRegCfg->VdhFfAptEn, 0 );
    dprint(PRN_VDMREG, "FF_APT_EN=0x%x\n", pVdhRegCfg->VdhFfAptEn);
}

#endif
