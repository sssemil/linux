/*
 * vdec hal for vp8
 *
 * Copyright (c) 2017 Hisilicon Limited
 *
 * Author: gaoyajun<gaoyajun@hisilicon.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation.
 *
 */
#ifndef __VDM_HAL_VP8_C__
#define __VDM_HAL_VP8_C__


#include "public.h"
#include "vdm_hal.h"
#include "vdm_hal_api.h"
#include "vdm_hal_local.h"
#include "vdm_hal_vp8.h"


SINT32 VP8HAL_CfgReg(OMXVDH_REG_CFG_S *pVdhRegCfg)
{
    UINT32 datTmp;

    //BASIC_CFG0
    WR_VREG( VREG_BASIC_CFG0, pVdhRegCfg->VdhBasicCfg0, 0);
    dprint(PRN_VDMREG, "BASIC_CFG0 = 0x%x\n", pVdhRegCfg->VdhBasicCfg0);

    //BASIC_CFG1
    /*set uv order 0: v first; 1: u first*/
    WR_VREG( VREG_BASIC_CFG1, pVdhRegCfg->VdhBasicCfg1, 0);
    dprint(PRN_VDMREG, "BASIC_CFG1 = 0x%x\n", pVdhRegCfg->VdhBasicCfg1);

    //AVM_ADDR
    WR_VREG( VREG_AVM_ADDR, pVdhRegCfg->VdhAvmAddr, 0);
    dprint(PRN_VDMREG, "AVM_ADDR = 0x%x\n", pVdhRegCfg->VdhAvmAddr);

    //VAM_ADDR
    WR_VREG( VREG_VAM_ADDR, pVdhRegCfg->VdhVamAddr, 0);
    dprint(PRN_VDMREG, "VAM_ADDR = 0x%x\n", pVdhRegCfg->VdhVamAddr);

    //STREAM_BASE_ADDR
    WR_VREG( VREG_STREAM_BASE_ADDR, pVdhRegCfg->VdhStreamBaseAddr, 0);
    dprint(PRN_VDMREG, "STREAM_BASE_ADDR = 0x%x\n", pVdhRegCfg->VdhStreamBaseAddr);


    //PPFD_BUF_ADDR
    WR_VREG( VREG_PPFD_BUF_ADDR, pVdhRegCfg->VdhPpfdBufAddr, 0);
    dprint(PRN_VDMREG, "PPFD_BUF_ADDR = 0x%x\n", pVdhRegCfg->VdhPpfdBufAddr);

    //PPFD_BUF_LEN
    WR_VREG( VREG_PPFD_BUF_LEN, pVdhRegCfg->VdhPpfdBufLen, 0);
    dprint(PRN_VDMREG, "PPFD_BUF_LEN = 0x%x\n", pVdhRegCfg->VdhPpfdBufLen);

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

    //YSTADDR_1D
    WR_VREG( VREG_YSTADDR_1D, pVdhRegCfg->VdhYstAddr, 0);
    dprint(PRN_VDMREG, "YSTADDR_1D = 0x%x\n", pVdhRegCfg->VdhYstAddr);

    //YSTRIDE_1D
    //  ystride = pVp8DecParam->ddr_stride * 32;
    //  ystride = (( pVp8DecParam->pic_width_in_mb + 15) / 16) * 16*16*16;
    WR_VREG( VREG_YSTRIDE_1D, pVdhRegCfg->VdhYstride, 0);
    dprint(PRN_VDMREG, "YSTRIDE_1D = 0x%x\n", pVdhRegCfg->VdhYstride);

    //UVOFFSET_1D

    WR_VREG( VREG_UVOFFSET_1D, pVdhRegCfg->VdhUvoffset, 0);
    dprint(PRN_VDMREG, "UVOFFSET_1D = 0x%x\n", pVdhRegCfg->VdhUvoffset);

    WR_VREG( VREG_HEAD_INF_OFFSET, pVdhRegCfg->VdhHeadInfOffset, 0);
    return VDMHAL_OK;
}

SINT32 VP8HAL_StartDec(OMXVDH_REG_CFG_S *pVdhRegCfg)
{
    VP8HAL_CfgReg(pVdhRegCfg);
    return VDMHAL_OK;
}

#endif

