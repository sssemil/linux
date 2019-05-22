/*
 * vdm hal interface
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
#include "scd_drv.h"
#include "vdm_hal_api.h"
#include "vdm_hal_local.h"
#ifdef VFMW_MPEG2_SUPPORT
#include "vdm_hal_mpeg2.h"
#endif
#ifdef VFMW_H264_SUPPORT
#include "vdm_hal_h264.h"
#endif
#ifdef VFMW_HEVC_SUPPORT
#include "vdm_hal_hevc.h"
#endif
#ifdef VFMW_MPEG4_SUPPORT
#include "vdm_hal_mpeg4.h"
#endif
#ifdef VFMW_AVS_SUPPORT
#include "vdm_hal_avs.h"
#endif
#ifdef VFMW_REAL8_SUPPORT
#include "vdm_hal_real8.h"
#endif
#ifdef VFMW_REAL9_SUPPORT
#include "vdm_hal_real9.h"
#endif
#ifdef VFMW_VC1_SUPPORT
#include "vdm_hal_vc1.h"
#endif
#ifdef VFMW_DIVX3_SUPPORT
#include "vdm_hal_divx3.h"
#endif
#ifdef VFMW_VP6_SUPPORT
#include "vdm_hal_vp6.h"
#endif
#ifdef VFMW_VP8_SUPPORT
#include "vdm_hal_vp8.h"
#endif
#ifdef VFMW_VP9_SUPPORT
#include "vdm_hal_vp9.h"
#endif
#include "vfmw_intf.h"
#ifdef HIVDEC_SMMU_SUPPORT
#include "smmu.h"
#endif

UINT8 g_not_direct_8x8_inference_flag;

VDMDRV_PARAM_S g_VdmDrvParam[MAX_VDH_NUM];
VDMHAL_HWMEM_S g_HwMem[MAX_VDH_NUM];
VDMHAL_BACKUP_S       g_BackUp[MAX_VDH_NUM];
VDMHAL_REPAIR_PARAM_S g_RepairParam[MAX_VDH_NUM][2];
VDMHAL_BACKUP_S     g_VdmRegState;

static VDMDRV_SLEEP_STAGE_E  s_eVdmDrvSleepState[MAX_VDH_NUM] = {VDMDRV_SLEEP_STAGE_NONE};

inline UINT32 VDMHAL_GetIntMaskCfg( SINT32 ModuleLowlyEnable )
{
    UINT32 D32 = 0;

#ifdef VDM_BUSY_WAITTING
    //mask   int
    D32 = 0xFFFFFFFF;
#else
    //enable int
    if (1 == ModuleLowlyEnable)
    {
        D32 = 0xFFFFFFFA;
    }
    else
    {
        D32 = 0xFFFFFFFE;
    }
#endif

    return D32;
}

VOID VDMHAL_EnableInt( SINT32 VdhId )
{
    UINT32 D32 = 0;
    SINT32 *p32 = NULL;
    UADDR vdm_reg_phy_addr = 0;
    SINT32 ModuleLowlyEnable = 0;
    ModuleLowlyEnable = 0;

    switch (VdhId)
    {
        case 0:
            vdm_reg_phy_addr = gVdhRegBaseAddr;
            break;

        default:
            dprint(PRN_FATAL, "VdhId is wrong! VDMHAL_EnableInt\n");
            return;
    }

    if (VdhId > (MAX_VDH_NUM - 1))
    {
        dprint(PRN_FATAL, "%s: VdhId(%d) > %d\n", __func__, VdhId, (MAX_VDH_NUM - 1));
        return ;
    }

    if ( NULL == g_HwMem[VdhId].pVdmRegVirAddr )
    {
        if ( NULL != (p32 = (SINT32 *)MEM_Phy2Vir(vdm_reg_phy_addr)) )
        {
            g_HwMem[VdhId].pVdmRegVirAddr = p32;
        }
        else
        {
            dprint(PRN_FATAL, "vdm register virtual address not mapped, reset failed!\n");
            return;
        }
    }

    D32 = VDMHAL_GetIntMaskCfg(ModuleLowlyEnable);

    WR_VREG( VREG_INT_MASK, D32, VdhId );

    return;
}


SINT32 VDMHAL_CfgRpMsg(VDMHAL_REPAIR_PARAM_S *pRepairParam, VDMHAL_HWMEM_S *pHwMem, SINT32 VdhId)
{
    SINT32 *pMsgBase;
    UINT32 D32;
    SINT32 i;
    UINT32 align_mb = 0;
    SINT32 pic_width, pic_height, SlotWidth, Stride1D, uvstride_1d, ChromaOffset;
    SINT32 cur_EndMb = 0, cur_StartMb = 0, front_StartMb = 0, front_EndMb = 0, next_StartMb = 0, next_EndMb = 0;
    SINT32 start_Mb = 0, end_Mb = 0;
    SINT32 max_repairTime = 0, actual_repairTime = 0;
    SINT32 ImgHeightInMb;
    SINT32 ImgHeightInPixAln;
    UINT32 fld_image_w, fld_image_h;

    pMsgBase = (SINT32 *)MEM_Phy2Vir(pHwMem->MsgSlotAddr[RP_MSG_SLOT_INDEX]);
    VDMHAL_ASSERT_RET( NULL != pMsgBase , "can not map repair msg virtual address!");

    if ( (pRepairParam->ValidGroupNum <= 0) || (pRepairParam->ValidGroupNum > MAX_UP_MSG_SLICE_NUM) )
    {
        dprint(PRN_FATAL, "ValidGroupNum=%d out of range!\n", pRepairParam->ValidGroupNum);
        return VDMHAL_ERR;
    }

    pic_width  = pRepairParam->ImageWidth * (1 << pRepairParam->CtbSize);
    pic_height = pRepairParam->ImageHeight * (1 << pRepairParam->CtbSize);

    ImgHeightInMb = (pic_height + 15) >> 4;
    ImgHeightInPixAln   =  ((ImgHeightInMb + 1) / 2) * 2 * 16;

    SlotWidth  = (pic_width + (ALIGN_LEN - 1)) & (~(ALIGN_LEN- 1));

    //Stride1D = SlotWidth;

    //ChromaOffset = SlotWidth * ImgHeightInPixAln;
    //HeadInfOffset = 0;

     Stride1D = pRepairParam->Stride1D;
     uvstride_1d = pRepairParam->uvstride_1d;
     ChromaOffset = pRepairParam->uvOffset;

    if(pRepairParam->Compress_en == 0)
    {
         Stride1D = pRepairParam->Stride1D/8;
         uvstride_1d = pRepairParam->Stride1D/8;
    }
    D32 = 0;
    ((VDMRPMSG_D0 *)(&D32))->src_luma_addr = pRepairParam->RefImageAddr;
    WR_MSGWORD( pMsgBase, D32);
    dprint(PRN_VDMREG, "repair: src_luma_addr = 0x%x\n", D32);

    D32 = 0;
    ((VDMRPMSG_D1 *)(&D32))->src_chroma_addr = pRepairParam->RefImageAddr + ChromaOffset;
    WR_MSGWORD( pMsgBase + 1, D32);
    dprint(PRN_VDMREG, "repair: src_chroma_addr = 0x%x\n", D32);

    D32 = 0;
    ((VDMRPMSG_D2 *)(&D32))->dst_luma_addr = pRepairParam->ImageAddr;
    WR_MSGWORD( pMsgBase + 2, D32);
    dprint(PRN_VDMREG, "repair: dst_luma_addr = 0x%x\n", D32);

    D32 = 0;
    ((VDMRPMSG_D3 *)(&D32))->dst_chroma_addr = pRepairParam->ImageAddr + ChromaOffset;
    WR_MSGWORD( pMsgBase + 3, D32);
    dprint(PRN_VDMREG, "repair: dst_chroma_addr = 0x%x\n", D32);

    D32 = 0;
    ((VDMRPMSG_D4 *)(&D32))->stride_1d = Stride1D;
    WR_MSGWORD( pMsgBase + 4, D32);
    dprint(PRN_VDMREG, "repair: stride_1d = 0x%x\n", D32);

    D32 = 0;
    //((VDMRPMSG_D5 *)(&D32))->headInfOffset = HeadInfOffset;
    D32 = ((pRepairParam->head_stride)<<18)|uvstride_1d;
    WR_MSGWORD( pMsgBase + 5, D32);
    dprint(PRN_VDMREG, "repair: head_stride = 0x%x\n", D32);

    D32 = 0;
    ((VDMRPMSG_D6 *)(&D32))->pic_width_in_mb = (pRepairParam->ImageWidth) - 1;
    ((VDMRPMSG_D6 *)(&D32))->pic_height_in_mb = (pRepairParam->ImageHeight) - 1;
    WR_MSGWORD( pMsgBase + 6, D32);
    dprint(PRN_VDMREG, "repair: pic_width_in_mb = 0x%x\n",  (pRepairParam->ImageWidth) - 1);
    dprint(PRN_VDMREG, "repair: pic_height_in_mb = 0x%x\n", (pRepairParam->ImageHeight) - 1);

     D32 = 0;
     ((VDMRPMSG_D8 *)(&D32))->src_head_ystaddr = pRepairParam->src_head_ystaddr;
     WR_MSGWORD( pMsgBase+8, D32);
     D32 = 0;
     ((VDMRPMSG_D9 *)(&D32))->src_head_cstaddr = pRepairParam->src_head_cstaddr;
     WR_MSGWORD( pMsgBase+9, D32);
     D32 = 0;
     ((VDMRPMSG_D10 *)(&D32))->dst_head_ystaddr = pRepairParam->dst_head_ystaddr;
     WR_MSGWORD( pMsgBase+10, D32);
     D32 = 0;
     ((VDMRPMSG_D11 *)(&D32))->dst_head_cstaddr = pRepairParam->dst_head_cstaddr;
     WR_MSGWORD( pMsgBase+11, D32);

    fld_image_w = pRepairParam->ImageWidth;
    fld_image_h = (1 == pRepairParam->Pic_type || 2 == pRepairParam->Pic_type)? pRepairParam->ImageHeight/2: pRepairParam->ImageHeight;

    /*计算一个最大的修补次数，当修补次数要超过最大次数的时候就将修补变成整帧修补*/
    max_repairTime = (RP_MSG_SLOT_NUM * MSG_SLOT_SIZE - 8) / 2;

    if (pRepairParam->CtbSize == 6)
    {
        align_mb = 2;
    }
    else if (pRepairParam->CtbSize == 5)
    {
        align_mb = 4;
    }
    else if (pRepairParam->CtbSize == 4)
    {
        align_mb = 8;
    }
    else
    {
        align_mb = 1;
        dprint(PRN_ERROR, "%s CtbSize %d align_mb error\n", __func__, pRepairParam->CtbSize);
    }

    if (0 == pRepairParam->FullRepair)
    {
        for (i = 0; i < pRepairParam->ValidGroupNum; i++)
        {
            //8mb(宏块)对齐，其实宏块向前移动，结束宏块向后移动
            cur_StartMb = (pRepairParam->MbGroup[i].StartMbn) / align_mb * align_mb;
            cur_EndMb = (pRepairParam->MbGroup[i].EndMbn + align_mb - 1) / align_mb * align_mb;
            next_StartMb = (pRepairParam->MbGroup[i+1].StartMbn)/align_mb*align_mb;
            next_EndMb = (pRepairParam->MbGroup[i+1].EndMbn+align_mb-1)/align_mb*align_mb;
            if (i > 0)
            {
                front_StartMb = (pRepairParam->MbGroup[i - 1].StartMbn) / align_mb * align_mb;
                front_EndMb = (pRepairParam->MbGroup[i - 1].EndMbn + align_mb - 1) / align_mb * align_mb;
            }

            if (i != 0 && (cur_EndMb <= front_EndMb))
            {
                /*如果当前修补的结束宏块号小于等于上次的结束宏块好，则不进行本次修补*/
                continue;
            }

            start_Mb = cur_StartMb;

            if (i > 0)
            {
                /*如果下次的修补和本次修补的宏块有重叠或者下次修补紧邻本次修补，那么将本次修补和下次修补在下次一起修补，不启动本次修补*/
                while ((i + 1 < pRepairParam->ValidGroupNum ) && ((pRepairParam->MbGroup[i].EndMbn + align_mb - 1) / align_mb * align_mb > (pRepairParam->MbGroup[i + 1].StartMbn) / align_mb * align_mb
                        || (pRepairParam->MbGroup[i].EndMbn + align_mb - 1) / align_mb * align_mb == ((pRepairParam->MbGroup[i + 1].StartMbn) / align_mb * align_mb + 1)
                        || (pRepairParam->MbGroup[i].EndMbn + align_mb - 1) / align_mb * align_mb <= (pRepairParam->MbGroup[i - 1].EndMbn + align_mb - 1) / align_mb * align_mb))
                {
                    i++;
                }
            }

            end_Mb = (pRepairParam->MbGroup[i].EndMbn + align_mb - 1) / align_mb * align_mb;

          	if(end_Mb >= pRepairParam->ImageWidth * pRepairParam->ImageHeight)
            {
                end_Mb =pRepairParam->ImageWidth * pRepairParam->ImageHeight - 1;
            }

            if (start_Mb > end_Mb)
            {
                start_Mb = end_Mb;
            }

            actual_repairTime ++;

            /*如果修补次数超过了最大修补次数，则变为整帧修补*/
            if (actual_repairTime > max_repairTime)
            {
                pRepairParam->FullRepair = 1;
                break;
            }

            D32 = 0;
           ((VDMRPMSG_D12 *)(&D32))->start_mbx = start_Mb % (pRepairParam->ImageWidth);
           ((VDMRPMSG_D12 *)(&D32))->start_mby = start_Mb / (pRepairParam->ImageWidth);
           WR_MSGWORD( pMsgBase+2*(actual_repairTime-1)+12, D32);

            D32 = 0;
            ((VDMRPMSG_D13 *)(&D32))->end_mbx = (end_Mb) % (pRepairParam->ImageWidth);
            ((VDMRPMSG_D13 *)(&D32))->end_mby = (end_Mb) / (pRepairParam->ImageWidth);
            WR_MSGWORD( pMsgBase+2*(actual_repairTime-1)+13, D32);

        }
    }

    if ( 1 == pRepairParam->FullRepair)
    {
        pRepairParam->ValidGroupNum  = 1;
        actual_repairTime = pRepairParam->ValidGroupNum;

        D32 = 0;
        ((VDMRPMSG_D12 *)(&D32))->start_mbx = 0;
        ((VDMRPMSG_D12 *)(&D32))->start_mby = 0;
        WR_MSGWORD( pMsgBase + 12, D32);
        dprint(PRN_VDMREG, "repair: start_mbx = 0x%x\n", 0);
        dprint(PRN_VDMREG, "repair: start_mby = 0x%x\n", 0);

        D32 = 0;
        ((VDMRPMSG_D13 *)(&D32))->end_mbx = fld_image_w - 1;
        ((VDMRPMSG_D13 *)(&D32))->end_mby = fld_image_h - 1;
        WR_MSGWORD( pMsgBase + 13, D32);
        dprint(PRN_VDMREG, "repair: end_mbx = 0x%x\n", fld_image_w - 1);
        dprint(PRN_VDMREG, "repair: end_mby = 0x%x\n", fld_image_h - 1);
    }

	D32 = 0;
    ((VDMRPMSG_D7 *)(&D32))->total_grp_num_minus1 = actual_repairTime - 1;
    ((VDMRPMSG_D7 *)(&D32))->compress_flag = pRepairParam->Compress_en;
	((VDMRPMSG_D7 *)(&D32))->ddr_interleave_mode = 0x3;
    ((VDMRPMSG_D7 *)(&D32))->dst_store_mode = pRepairParam->Pic_type;
    ((VDMRPMSG_D7 *)(&D32))->src_load_mode = pRepairParam->Pic_type;
    ((VDMRPMSG_D7 *)(&D32))->ctb_size = pRepairParam->CtbSize - 4;
	((VDMRPMSG_D7 *)(&D32))->bit_depth = pRepairParam->BitDepth;
    ((VDMRPMSG_D7 *)(&D32))->vdh_2d_en = pRepairParam->vdh_2d_en;
    WR_MSGWORD( pMsgBase + 7, D32);
    dprint(PRN_VDMREG, "repair: total_grp_num_minus1 = 0x%x\n", actual_repairTime - 1);
    dprint(PRN_VDMREG, "repair: compress_flag = 0x%x\n", pRepairParam->Compress_en);
	dprint(PRN_VDMREG, "repair: g_ddr_interleave_value = 0x%x\n", ((VDMRPMSG_D7 *)(&D32))->ddr_interleave_mode);
    dprint(PRN_VDMREG, "repair: dst_store_mode = 0x%x\n", pRepairParam->Pic_type);
    dprint(PRN_VDMREG, "repair: src_load_mode = 0x%x\n", pRepairParam->Pic_type);
    dprint(PRN_VDMREG, "repair: ctb_size = 0x%x\n", pRepairParam->CtbSize - 4);
	dprint(PRN_VDMREG, "repair: bit_depth = 0x%x\n", pRepairParam->BitDepth);
    dprint(PRN_VDMREG, "repair: vdh_2d_en = 0x%x\n", pRepairParam->vdh_2d_en);

    return VDMHAL_OK;
}

UINT32 VDMHAL_ReduceCoreFrequency(VOID)
{
    UINT32 div_value = 0;
    UINT32 sc_div_vdec = 0;

    RD_CRG_VREG(PERI_CRG_CORE_DIV, sc_div_vdec);    //读VDEC core当前的分频比
    div_value = sc_div_vdec & 0x1F;

    sc_div_vdec = ((sc_div_vdec & 0x1F) << 1) + 1;  //取低5bit，乘2加1
    if (sc_div_vdec > 0x1F)
    {
        sc_div_vdec = 0x1F;
    }

    //16 ~ 20 bit置1, 使能低5bit生效
    WR_CRG_VREG(PERI_CRG_CORE_DIV, 0x1F0000 | sc_div_vdec);
    VFMW_OSAL_uDelay(1);

    return div_value;
}

VOID VDMHAL_RestoreCoreFrequency(UINT32 DivValue)
{
    WR_CRG_VREG(PERI_CRG_CORE_DIV, 0x1F0000 | DivValue);
    VFMW_OSAL_uDelay(1);

    return;
}

UINT32 VDMHAL_ReduceAXIFrequency(VOID)
{
    UINT32 div_value = 0;
    UINT32 sc_div_vcodecbus = 0;

    RD_CRG_VREG(PERI_CRG_AXI_DIV, sc_div_vcodecbus);
    div_value = sc_div_vcodecbus & 0x1F;

    sc_div_vcodecbus = ((sc_div_vcodecbus & 0x1F) << 1) + 1;
    if (sc_div_vcodecbus > 0x1F)
    {
        sc_div_vcodecbus = 0x1F;
    }

    WR_CRG_VREG(PERI_CRG_AXI_DIV, 0x1F0000 | sc_div_vcodecbus);
    VFMW_OSAL_uDelay(1);

    return div_value;
}

VOID VDMHAL_RestoreAXIFrequency(UINT32 DivValue)
{
    WR_CRG_VREG(PERI_CRG_AXI_DIV, 0x1F0000 | DivValue);
    VFMW_OSAL_uDelay(1);

    return;
}

SINT32 VDMHAL_CfgRpReg(OMXVDH_REG_CFG_S *pVdhRegCfg)
{
    SINT32 D32 = 0;
    WR_VREG( VREG_AVM_ADDR, pVdhRegCfg->VdhAvmAddr, 0);

    D32 = 0x2000C203;
    WR_VREG( VREG_BASIC_CFG1, D32, 0 );

    D32 = 0x00300C03;
    WR_VREG( VREG_SED_TO, D32, 0 );
    WR_VREG( VREG_ITRANS_TO, D32, 0 );
    WR_VREG( VREG_PMV_TO, D32, 0 );
    WR_VREG( VREG_PRC_TO, D32, 0 );
    WR_VREG( VREG_RCN_TO, D32, 0 );
    WR_VREG( VREG_DBLK_TO, D32, 0 );
    WR_VREG( VREG_PPFD_TO, D32, 0 );

    return VDMHAL_OK;
}

VOID VDMHAL_IMP_Init(VOID)
{
    memset( &g_HwMem[0], 0, sizeof(VDMHAL_HWMEM_S) );
    memset(&g_VdmDrvParam[0], 0, sizeof(VDMDRV_PARAM_S));
    memset(&g_VdmRegState, 0, sizeof(g_VdmRegState));

    g_HwMem[0].pVdmRegVirAddr  = (SINT32 *)MEM_Phy2Vir(gVdhRegBaseAddr);
    g_HwMem[0].pPERICRGVirAddr = (SINT32 *)MEM_Phy2Vir(gPERICRG_RegBaseAddr);
    g_HwMem[0].pBpdRegVirAddr  = (SINT32 *)MEM_Phy2Vir(gBpdRegBaseAddr);

    VDMHAL_IMP_GlbReset();
//    VDMHAL_IMP_WriteScdEMARID();
    s_eVdmDrvSleepState[0] = VDMDRV_SLEEP_STAGE_NONE;
}

VOID VDMHAL_IMP_DeInit(VOID)
{
    s_eVdmDrvSleepState[0] = VDMDRV_SLEEP_STAGE_NONE;
}

VOID VDMHAL_IMP_ResetVdm( SINT32 VdhId)
{
    SINT32 i;
    SINT32 tmp = 0;
    UINT32 reg;
    UINT32 reg_rst_ok;
    UINT32 div_value;
    UINT32 *pVdmResetVirAddr, *pVdmResetOkVirAddr;

    pVdmResetVirAddr   = (SINT32 *)MEM_Phy2Vir(gSOFTRST_REQ_Addr);
    pVdmResetOkVirAddr = (SINT32 *)MEM_Phy2Vir(gSOFTRST_OK_ADDR);

    if ( NULL == pVdmResetVirAddr  || NULL == pVdmResetOkVirAddr)
    {
        dprint(PRN_FATAL, "VDMHAL_ResetVdm: map vdm register fail, vir(pVdmResetVirAddr) = (%pK), vir(pVdmResetOkVirAddr) = (%pK)\n",
               pVdmResetVirAddr, pVdmResetOkVirAddr);
        return;
    }

    RD_VREG( VREG_INT_MASK, tmp, VdhId );

    div_value = VDMHAL_EXT_ReduceFrequency(MFDE_RESET_REQUIRE);

    /* require mfde reset */
    reg = *(volatile UINT32 *)pVdmResetVirAddr;
    *(volatile UINT32 *)pVdmResetVirAddr = reg | (UINT32)(1 << MFDE_RESET_CTRL_BIT);

    /* wait for reset ok */
    for (i = 0; i < 100; i++)
    {
        reg_rst_ok = *(volatile UINT32 *)pVdmResetOkVirAddr;
        if ( reg_rst_ok & (1 <<MFDE_RESET_OK_BIT))
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
    *(volatile UINT32 *)pVdmResetVirAddr = reg & (UINT32)(~(1 << MFDE_RESET_CTRL_BIT));

    VDMHAL_EXT_RestoreFrequency(MFDE_RESET_REQUIRE, div_value);

    WR_VREG( VREG_INT_MASK, tmp, VdhId );

    return;
}


VOID VDMHAL_IMP_GlbReset(VOID)
{
    SINT32 i;
    UINT32 reg, reg_rst_ok;
    UINT32 div_value;
    UINT32 *pResetVirAddr = NULL;
    UINT32 *pResetOKVirAddr = NULL;

    pResetVirAddr   = (SINT32 *)MEM_Phy2Vir(gSOFTRST_REQ_Addr);
    pResetOKVirAddr = (SINT32 *)MEM_Phy2Vir(gSOFTRST_OK_ADDR);

    if ( NULL == pResetVirAddr  || NULL == pResetOKVirAddr)
    {
        dprint(PRN_FATAL, "VDMHAL_GlbReset: map vdm register fail, vir(pResetVirAddr) = (%pK), vir(pResetOKVirAddr) = (%pK)\n",
               pResetVirAddr, pResetOKVirAddr);
        return;
    }

    div_value = VDMHAL_EXT_ReduceFrequency(GLB_RESET_REQUIRE);

    /* require all reset, include mfde scd bpd */
    reg = *(volatile UINT32 *)pResetVirAddr;
    *(volatile UINT32 *)pResetVirAddr = reg | (UINT32)(1 << ALL_RESET_CTRL_BIT);

    /* wait for reset ok */
    for (i = 0; i < 100; i++)
    {
        reg_rst_ok = *(volatile UINT32 *)pResetOKVirAddr;
        if ( reg_rst_ok & (1 << ALL_RESET_OK_BIT))
        {
            break;
        }
        VFMW_OSAL_uDelay(10);
    }

    if (i >= 100)
    {
        dprint(PRN_FATAL, "Glb Reset Failed!!!\n");
    }
    else
    {
        //dprint(PRN_ALWS, "Glb Reset Success!\n");
    }

    /* clear reset require */
    *(volatile UINT32 *)pResetVirAddr = reg & (UINT32)(~(1 << ALL_RESET_CTRL_BIT));

    VDMHAL_EXT_RestoreFrequency(GLB_RESET_REQUIRE, div_value);

    // after GlbReset, if not used reset value, need to reconfig these value.
    //WR_VREG( VREG_CRG_CLK_EN,  CRG_CLK_EN_DEFAULT_VALUE,  0 );
    //WR_VREG( VREG_MFDE_CLK_EN, MFDE_CLK_EN_DEFAULT_VALUE, 0 );

    return;

}

VOID VDMHAL_IMP_ClearIntState( SINT32 VdhId )
{
    SINT32 *p32;
    SINT32 D32 = 0xFFFFFFFF;

    if (VdhId > (MAX_VDH_NUM - 1))
    {
        dprint(PRN_FATAL, "%s: VdhId(%d) > %d\n", __func__, VdhId, (MAX_VDH_NUM - 1));
        return ;
    }

    if ( NULL == g_HwMem[VdhId].pVdmRegVirAddr )
    {
        if ( NULL != (p32 = (SINT32 *)MEM_Phy2Vir(gVdhRegBaseAddr)) )
        {
            g_HwMem[VdhId].pVdmRegVirAddr = p32;
        }
        else
        {
            dprint(PRN_FATAL, " %s %d vdm register virtual address not mapped, reset failed!\n", __func__, __LINE__);
            return;
        }
    }

    WR_VREG( VREG_INT_STATE, D32, VdhId );

    return;
}

SINT32 VDMHAL_IMP_CheckReg(REG_ID_E reg_id, SINT32 VdhId)
{
    SINT32 *p32;
    SINT32 dat = 0;
    UINT32 reg_type;

    if (VdhId > (MAX_VDH_NUM - 1))
    {
        dprint(PRN_FATAL, "%s: VdhId(%d) Invalid!\n", __func__, VdhId);
        return VDMHAL_ERR;
    }

    if ( NULL == g_HwMem[VdhId].pVdmRegVirAddr )
    {
        if ( NULL != (p32 = (SINT32 *)MEM_Phy2Vir(gVdhRegBaseAddr)) )
        {
            g_HwMem[VdhId].pVdmRegVirAddr = p32;
        }
        else
        {
            dprint(PRN_FATAL, " %s %d vdm register virtual address not mapped, reset failed!\n", __func__, __LINE__);
            return 0;
        }
    }

    switch (reg_id)
    {
        case VDH_STATE_REG:
            reg_type = VREG_VDH_STATE;
            break;

        case INT_STATE_REG:
            reg_type = VREG_INT_STATE;
            break;

        case INT_MASK_REG:
            reg_type = VREG_INT_MASK;
            break;

        case VCTRL_STATE_REG:
            reg_type = VREG_VCTRL_STATE;
            break;

        default:
            dprint(PRN_FATAL, "%s: unkown reg_id = %d\n", __func__, reg_id);
            return 0;
    }

    RD_VREG(reg_type, dat, 0);
    return dat;
}

SINT32 VDMHAL_IMP_PrepareDec(OMXVDH_REG_CFG_S *pVdhRegCfg)
{
    if (VFMW_AVS == pVdhRegCfg->VidStd)
    {
        WR_SCDREG(REG_AVS_FLAG, 0x00000001);
    }
    else
    {
        WR_SCDREG(REG_AVS_FLAG, 0x00000000);
    }

    WR_SCDREG(REG_VDH_SELRST, 0x00000001);

    switch (pVdhRegCfg->VidStd)
    {
#ifdef VFMW_H264_SUPPORT

        case VFMW_H264:
            return H264HAL_StartDec(pVdhRegCfg);
            break;
#endif
#ifdef VFMW_HEVC_SUPPORT

        case VFMW_HEVC:
            return HEVCHAL_StartDec(pVdhRegCfg);
            break;
#endif
#ifdef VFMW_MPEG2_SUPPORT

        case VFMW_MPEG2:
            return MP2HAL_StartDec(pVdhRegCfg);
            break;
#endif
#ifdef VFMW_MPEG4_SUPPORT
        case VFMW_MPEG4:
            return MP4HAL_StartDec(pVdhRegCfg);
            break;
#endif
#ifdef VFMW_REAL8_SUPPORT

        case VFMW_REAL8:
            return RV8HAL_StartDec(pVdhRegCfg);
            break;
#endif
#ifdef VFMW_REAL9_SUPPORT

        case VFMW_REAL9:
            return RV9HAL_StartDec(pVdhRegCfg);
            break;
#endif
#ifdef VFMW_DIVX3_SUPPORT

        case VFMW_DIVX3:
            return DIVX3HAL_StartDec(pVdhRegCfg);
            break;
#endif
#ifdef VFMW_VC1_SUPPORT

        case VFMW_VC1:
            return VC1HAL_StartDec(pVdhRegCfg);
            break;
#endif
#ifdef VFMW_VP8_SUPPORT

        case VFMW_VP8:
            return VP8HAL_StartDec(pVdhRegCfg);
            break;
#endif
#ifdef VFMW_VP9_SUPPORT

        case VFMW_VP9:
            return VP9HAL_StartDec(pVdhRegCfg);
            break;
#endif
#ifdef VFMW_MVC_SUPPORT

        case VFMW_MVC:
            return H264HAL_StartDec(pVdhRegCfg);
            break;
#endif
        default:
            break;
    }

    return VDMHAL_ERR;
}

/*****************************************************************************************
  原型    SINT32 VDMHAL_IsVdmReady( )
  功能    检查VDM是否ready
  参数    无

  返回值  类布尔数据，1表示VDM已经ready，反之返回0
 ******************************************************************************************/
SINT32 VDMHAL_IMP_IsVdmReady(SINT32 VdhId)
{
    SINT32 Data32 = 0;

    VDMHAL_ASSERT_RET( g_HwMem[VdhId].pVdmRegVirAddr != NULL, "VDM register not mapped yet!" );

    RD_VREG(VREG_VDH_STATE, Data32, VdhId);

    Data32 = (Data32 >> 17) & 1;
    Data32 = (Data32 == 0) ? 0 : 1;

    return Data32;
}

SINT32 VDMHAL_IsVdmRun(SINT32 VdhId)
{
    SINT32 Data32 = 0;

    if ( g_HwMem[VdhId].pVdmRegVirAddr == NULL)
    {
        dprint(PRN_FATAL, "VDM register not mapped yet!\n" );
        return 0;
    }

    RD_VREG(VREG_VCTRL_STATE, Data32, VdhId);

    if (Data32 == 1)
    {
        return 0;
    }
    else
    {
        return 1;//work
    }
}

SINT32 VDMHAL_IMP_BackupInfo(VOID)
{
    SINT32 i = 0;

    g_VdmRegState.Int_State_Reg = VDMHAL_IMP_CheckReg(INT_STATE_REG, 0);

    RD_VREG( VREG_BASIC_CFG1, g_VdmRegState.BasicCfg1, 0 );
    RD_VREG( VREG_VDH_STATE, g_VdmRegState.VdmState, 0 );

    RD_VREG( VREG_MB0_QP_IN_CURR_PIC, g_VdmRegState.Mb0QpInCurrPic, 0 );
    RD_VREG( VREG_SWITCH_ROUNDING, g_VdmRegState.SwitchRounding, 0 );

    {
        RD_VREG( VREG_SED_STA, g_VdmRegState.SedSta, 0 );
        RD_VREG( VREG_SED_END0, g_VdmRegState.SedEnd0, 0 );
        RD_VREG(VREG_DEC_CYCLEPERPIC, g_VdmRegState.DecCyclePerPic, 0 );
        RD_VREG(VREG_RD_BDWIDTH_PERPIC, g_VdmRegState.RdBdwidthPerPic, 0 );
        RD_VREG(VREG_WR_BDWIDTH_PERPIC, g_VdmRegState.WrBdWidthPerPic, 0 );
        RD_VREG(VREG_RD_REQ_PERPIC, g_VdmRegState.RdReqPerPic, 0 );
        RD_VREG(VREG_WR_REQ_PERPIC, g_VdmRegState.WrReqPerPic, 0 );
        RD_VREG( VREG_LUMA_SUM_LOW, g_VdmRegState.LumaSumLow, 0 );
        RD_VREG( VREG_LUMA_SUM_HIGH, g_VdmRegState.LumaSumHigh, 0 );
    }

    for (i = 0; i < 32; i++)
    {
        RD_VREG( VREG_LUMA_HISTORGRAM + i * 4, g_VdmRegState.LumaHistorgam[i], 0 );
    }

    return VDMHAL_OK;
}

VOID VDMHAL_GetRegState(VDMHAL_BACKUP_S *pVdmRegState)
{
	memcpy(pVdmRegState, &g_VdmRegState, sizeof(VDMHAL_BACKUP_S));
}

SINT32 VDMHAL_IMP_PrepareRepair(OMXVDH_REG_CFG_S *pVdhRegCfg)
{
    if (FIRST_REPAIR == pVdhRegCfg->RepairTime)
    {
        if (pVdhRegCfg->ValidGroupNum0 > 0)
        {
            VDMHAL_CfgRpReg(pVdhRegCfg);
        }
        else
        {
            return VDMHAL_ERR;
        }
    }
    else if (SECOND_REPAIR == pVdhRegCfg->RepairTime)
    {
        dprint(PRN_FATAL, "SECOND_REPAIR Parameter Error!\n");
        return VDMHAL_ERR;
    }

    return VDMHAL_OK;
}

VOID VDMHAL_IMP_StartHwRepair(SINT32 VdhId)
{
    SINT32 D32 = 0;

    RD_VREG(VREG_BASIC_CFG0, D32, VdhId);
#ifdef ENV_SOS_KERNEL
    D32 = 0x84000000;
#else
    D32 = 0x4000000;
#endif
    WR_VREG(VREG_BASIC_CFG0, D32, VdhId);

#ifdef HIVDEC_SMMU_SUPPORT
#ifdef ENV_SOS_KERNEL
    SMMU_SetMasterReg(MFDE, SECURE_ON,  SMMU_OFF);
#else
    SMMU_SetMasterReg(MFDE, SECURE_OFF, SMMU_ON);
#endif
#endif

    VDMHAL_IMP_ClearIntState(VdhId);
    VDMHAL_EnableInt(VdhId);

    VFMW_OSAL_Mb();
    WR_VREG( VREG_VDH_START, 0, VdhId );
    WR_VREG( VREG_VDH_START, 1, VdhId );
    WR_VREG( VREG_VDH_START, 0, VdhId );

    return;
}

VOID VDMHAL_IMP_StartHwDecode(SINT32 VdhId)
{

#ifdef HIVDEC_SMMU_SUPPORT
#ifdef ENV_SOS_KERNEL
    SMMU_SetMasterReg(MFDE, SECURE_ON,  SMMU_OFF);
#else
    SMMU_SetMasterReg(MFDE, SECURE_OFF, SMMU_ON);
#endif
#endif

    VDMHAL_IMP_ClearIntState(VdhId);
    VDMHAL_EnableInt(VdhId);

    VFMW_OSAL_Mb();
    WR_VREG( VREG_VDH_START, 0,0);
    WR_VREG( VREG_VDH_START, 1,0);
    WR_VREG( VREG_VDH_START, 0,0);

    return;
}

VOID VDMHAL_IMP_WriteScdEMARID(VOID)
{
//    WR_SCDREG(REG_EMAR_ID, DEFAULT_EMAR_ID_VALUE);
}

UINT32 VDMHAL_EXT_ReduceFrequency(RESET_REQUIRE_TYPE_E eResetRequire)
{
#ifdef PLATFORM_HI3660
    UINT32 div_value;
    if (eResetRequire == SCD_RESET_REQUIRE || eResetRequire == MFDE_RESET_REQUIRE)
    {
        div_value = VDMHAL_ReduceCoreFrequency();
    }
    else
    {
        div_value = VDMHAL_ReduceAXIFrequency();
    }

    return div_value;
#endif
}

VOID VDMHAL_EXT_RestoreFrequency(RESET_REQUIRE_TYPE_E eResetRequire, UINT32 DivValue)
{
#ifdef PLATFORM_HI3660
    if (eResetRequire == SCD_RESET_REQUIRE || eResetRequire == MFDE_RESET_REQUIRE)
    {
        VDMHAL_RestoreCoreFrequency(DivValue);
    }
    else
    {
        VDMHAL_RestoreAXIFrequency(DivValue);
    }

    return;
#endif
}

VOID VDMHAL_ISR(SINT32 VdhId)
{
    if (VdhId > (MAX_VDH_NUM - 1))
    {
        dprint(PRN_FATAL, "%s: VdhId(%d) > %d\n",__func__, VdhId, (MAX_VDH_NUM - 1));
        return;
    }

    VDMDRV_STATEMACHINE_E *pVdmStateMachine = &g_VdmDrvParam[VdhId].VdmStateMachine;
    VFMW_OSAL_SpinLock(G_SPINLOCK_THREAD);

    VDMHAL_IMP_BackupInfo();
    VDMHAL_IMP_ClearIntState(VdhId);
    VFMW_OSAL_GiveEvent(G_VDMHWDONEEVENT);
    VFMW_OSAL_SpinUnLock(G_SPINLOCK_THREAD);
}

VOID VDMHAL_AfterDec(OMXVDH_REG_CFG_S *pVdhRegCfg)
{
    if (VDM_DECODE_STATE == pVdhRegCfg->VdmStateMachine)
    {
        if (pVdhRegCfg->ErrRationAndRpStratageFlag)
        {
            VDMHAL_ActivateVDH(pVdhRegCfg);
        }
        else if (VDMHAL_OK == VDMHAL_IMP_PrepareRepair(pVdhRegCfg))
        {
            VDMHAL_IMP_StartHwRepair(0);
        }
    }
    else if (VDM_REPAIR_STATE_0 == pVdhRegCfg->VdmStateMachine)
    {
        if ( 1 == pVdhRegCfg->AvsSecondFld)
        {
        }
        else if ( 1 == pVdhRegCfg->IsMpeg4Nvop)
        {
        #ifdef VFMW_MPEG4_SUPPORT
            VDMHAL_ActivateVDH(pVdhRegCfg);
        #endif
        }
        else if ( 1 == pVdhRegCfg->IsVc1Skpic)//vc1 copy
        {
        #ifdef VFMW_VC1_SUPPORT
            VDMHAL_ActivateVDH(pVdhRegCfg);
        #endif
        }
        else
        {
            VDMHAL_ActivateVDH(pVdhRegCfg);
        }
    }
    else if (VDM_REPAIR_STATE_1 == pVdhRegCfg->VdmStateMachine)
    {

        VDMHAL_ActivateVDH(pVdhRegCfg);
    }
}

VOID VDMHAL_ActivateVDH(OMXVDH_REG_CFG_S *pVdhRegCfg)
{
    if(0)
    {
    }
#ifdef VFMW_MPEG4_SUPPORT
    else if ( 1 == pVdhRegCfg->IsMpeg4Nvop)
    {
        VDMHAL_IMP_PrepareRepair(pVdhRegCfg);
        VDMHAL_IMP_StartHwRepair(0);
    }
#endif
#ifdef VFMW_VC1_SUPPORT
    else if ( 1 == pVdhRegCfg->IsVc1Skpic )
    {
        VDMHAL_IMP_PrepareRepair(pVdhRegCfg);
        VDMHAL_IMP_StartHwRepair(0);
    }
#endif
    else
    {
        VDMHAL_IMP_PrepareDec(pVdhRegCfg);
        VDMHAL_IMP_StartHwDecode(0);
    }
}

SINT32 VDMHAL_PrepareSleep(SINT32 VdhId)
{
    SINT32 ret = VDMDRV_OK;

    if (s_eVdmDrvSleepState[VdhId] == VDMDRV_SLEEP_STAGE_NONE)
    {
        if (g_VdmDrvParam[VdhId].VdmStateMachine == VDM_NULL_STATE)
        {
            s_eVdmDrvSleepState[VdhId] = VDMDRV_SLEEP_STAGE_SLEEP;
        }
        else
        {
            s_eVdmDrvSleepState[VdhId] = VDMDRV_SLEEP_STAGE_PREPARE;
        }

        ret = VDMDRV_OK;
    }
    else
    {
        ret = VDMDRV_ERR;
    }

    return ret;
}

VOID VDMHAL_ForceSleep(SINT32 VdhId)
{
    if (s_eVdmDrvSleepState[VdhId] != VDMDRV_SLEEP_STAGE_SLEEP)
    {
        //VDMDRV_Reset(VdhId);
        VDMHAL_IMP_ResetVdm(VdhId);
        s_eVdmDrvSleepState[VdhId] = VDMDRV_SLEEP_STAGE_SLEEP;
    }

    dprint(PRN_FATAL,"====== forece VDMHAL sleep! ======\n");
}

VOID VDMHAL_ExitSleep(SINT32 VdhId)
{
    s_eVdmDrvSleepState[VdhId] = VDMDRV_SLEEP_STAGE_NONE;
}

VDMDRV_SLEEP_STAGE_E VDMHAL_GetSleepStage(SINT32 VdhId)
{
    return s_eVdmDrvSleepState[VdhId];
}
