/*
 * vdec hal for hevc
 *
 * Copyright (c) 2017 Hisilicon Limited
 *
 * Author: gaoyajun<gaoyajun@hisilicon.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation.
 *
 */
#include "basedef.h"
#include "vfmw.h"
#include "mem_manage.h"
#include "public.h"
#include "vdm_hal_api.h"
#include "vdm_hal_local.h"
#include "vdm_hal_hevc.h"
#include "vfmw_intf.h"

SINT32 HEVCHAL_CfgVdmReg(OMXVDH_REG_CFG_S *pVdhRegCfg)
{
    UINT32 D32;

    //BASIC_CFG0
    WR_VREG( VREG_BASIC_CFG0, pVdhRegCfg->VdhBasicCfg0, 0 );

    //BASIC_CFG1
    /*set uv order 0: v first; 1: u first*/
    WR_VREG( VREG_BASIC_CFG1, pVdhRegCfg->VdhBasicCfg1, 0 );

    //AVM_ADDR
    WR_VREG( VREG_AVM_ADDR, pVdhRegCfg->VdhAvmAddr, 0 );
    dprint(PRN_VDMREG, "AVM_ADDR = 0x%x\n", pVdhRegCfg->VdhAvmAddr);

    //VAM_ADDR
    WR_VREG( VREG_VAM_ADDR, pVdhRegCfg->VdhVamAddr, 0 );

    //STREAM_BASE_ADDR
    WR_VREG( VREG_STREAM_BASE_ADDR, pVdhRegCfg->VdhStreamBaseAddr, 0 );

    //TIME_OUT
    D32 = 0x00300C03;
    WR_VREG( VREG_SED_TO,    D32, 0 );
    WR_VREG( VREG_ITRANS_TO, D32 , 0);
    WR_VREG( VREG_PMV_TO,    D32, 0 );
    WR_VREG( VREG_PRC_TO,    D32, 0 );
    WR_VREG( VREG_RCN_TO,    D32, 0 );
    WR_VREG( VREG_DBLK_TO,   D32 , 0);
    WR_VREG( VREG_PPFD_TO,   D32, 0 );

    WR_VREG( VREG_YSTADDR_1D, pVdhRegCfg->VdhYstAddr, 0 );

    //YSTRIDE_1D
    WR_VREG( VREG_YSTRIDE_1D, pVdhRegCfg->VdhYstride, 0 );

    //UVOFFSET_1D
    WR_VREG( VREG_UVOFFSET_1D, pVdhRegCfg->VdhUvoffset, 0 );

    //HEAD_INF_OFFSET
    WR_VREG( VREG_HEAD_INF_OFFSET, pVdhRegCfg->VdhHeadInfOffset, 0 );

    //PPFD_BUF_ADDR
    WR_VREG( VREG_PPFD_BUF_ADDR, pVdhRegCfg->VdhPpfdBufAddr, 0 );

    //PPFD_BUF_LEN
    //((PPFD_BUF_LEN*)(&D32))->ppfd_buf_len = pHwMem->ppfd_buf_len;
    WR_VREG( VREG_PPFD_BUF_LEN, pVdhRegCfg->VdhPpfdBufLen, 0 );

    //UVSTRIDE_1D
    WR_VREG( VREG_UVSTRIDE_1D, pVdhRegCfg->VdhUvstride, 0 );

    //CFGINFO_ADDR
    WR_VREG(VREG_CFGINFO_ADDR, pVdhRegCfg->VdhCfgInfoAddr, 0);
    dprint(PRN_VDMREG, "pPicParam->cfginfo_msg_addr:%x\n", pVdhRegCfg->VdhCfgInfoAddr);

    //DDR_INTERLEAVE_MODE
    WR_VREG(VREG_DDR_INTERLEAVE_MODE, pVdhRegCfg->VdhDdrInterleaveMode, 0);

    //FF_APT_EN
    WR_VREG( VREG_FF_APT_EN, pVdhRegCfg->VdhFfAptEn, 0);
    dprint(PRN_VDMREG, "VREG_FF_APT_EN = 0x%x\n", pVdhRegCfg->VdhFfAptEn);

    return VDMHAL_OK;
}


SINT32 HEVCHAL_StartDec(OMXVDH_REG_CFG_S *pVdhRegCfg)
{
    HEVCHAL_CfgVdmReg(pVdhRegCfg);
    return VDMHAL_OK;
}
