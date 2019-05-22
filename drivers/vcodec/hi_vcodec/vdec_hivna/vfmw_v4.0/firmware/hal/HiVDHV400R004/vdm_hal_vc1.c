/*
 * vdec hal for vc1
 *
 * Copyright (c) 2017 Hisilicon Limited
 *
 * Author: gaoyajun<gaoyajun@hisilicon.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation.
 *
 */

#ifndef __VDM_HAL_VC1_C__
#define __VDM_HAL_VC1_C__

#include "basedef.h"
#include "public.h"
#include "vdm_hal_api.h"
#include "vdm_hal_local.h"
#include "vdm_hal_vc1.h"

SINT32 VC1HAL_CfgReg(OMXVDH_REG_CFG_S *pVdhRegCfg);

SINT32 VC1HAL_StartDec(OMXVDH_REG_CFG_S *pVdhRegCfg)
{
    VC1HAL_CfgReg(pVdhRegCfg);
    return VDMHAL_OK;
}

SINT32 VC1HAL_CfgReg(OMXVDH_REG_CFG_S *pVdhRegCfg)
{
    UINT32 datTmp;
    SINT32 D32;

    WR_VREG( VREG_BASIC_CFG0, pVdhRegCfg->VdhBasicCfg0, 0 );

    /*set uv order 0: v first; 1: u first*/
    WR_VREG( VREG_BASIC_CFG1, pVdhRegCfg->VdhBasicCfg1, 0 );

    WR_VREG( VREG_AVM_ADDR,         pVdhRegCfg->VdhAvmAddr, 0 );
    WR_VREG( VREG_VAM_ADDR,         pVdhRegCfg->VdhVamAddr, 0);
    WR_VREG( VREG_STREAM_BASE_ADDR, pVdhRegCfg->VdhStreamBaseAddr, 0 );

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

    WR_VREG( VREG_YSTADDR_1D, pVdhRegCfg->VdhYstAddr, 0 );
    WR_VREG( VREG_YSTRIDE_1D , pVdhRegCfg->VdhYstride, 0 );
    WR_VREG( VREG_UVOFFSET_1D, pVdhRegCfg->VdhUvoffset, 0 );
    WR_VREG( VREG_FF_APT_EN, pVdhRegCfg->VdhFfAptEn, 0 );
    WR_VREG( VREG_REF_PIC_TYPE, pVdhRegCfg->VdhRefPicType, 0 );

    //PRCNUM_1D_CNT
    WR_VREG( VREG_HEAD_INF_OFFSET, pVdhRegCfg->VdhHeadInfOffset, 0 );
    return VDMHAL_OK;
}

#endif
