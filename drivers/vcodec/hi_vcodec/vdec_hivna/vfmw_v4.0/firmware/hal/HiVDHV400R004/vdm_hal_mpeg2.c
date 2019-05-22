/*
 * vdec hal for mp2
 *
 * Copyright (c) 2017 Hisilicon Limited
 *
 * Author: gaoyajun<gaoyajun@hisilicon.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation.
 *
 */
#ifndef __VDM_HAL_MPEG2_C__
#define __VDM_HAL_MPEG2_C__

#include 	"public.h"
#include    "vdm_hal_api.h"
#include    "vdm_hal_local.h"
#include 	"vdm_hal_mpeg2.h"


SINT32 MP2HAL_CfgReg(OMXVDH_REG_CFG_S *pVdhRegCfg);
SINT32 MP2HAL_CfgDnMsg(MP2_DEC_PARAM_S *pMp2DecParam, VDMHAL_HWMEM_S *pHwMem, SINT32 VdhId, UADDR StreamBaseAddr);

static SINT32 FrameNum = -1;


SINT32 MP2HAL_StartDec(OMXVDH_REG_CFG_S *pVdhRegCfg)
{
    if (VDMHAL_OK != MP2HAL_CfgReg(pVdhRegCfg))
    {
        dprint(PRN_ERROR, "MP2HAL_CfgReg ERROR!\n");
        return VDMHAL_ERR;
    }
    return VDMHAL_OK;
}

SINT32 MP2HAL_WriteSliceMsg(MP2_DEC_PARAM_S *pMp2DecParam,  SINT32 *SlcDnMsgVirAddr, UADDR SlcDnMsgPhyAddr, UADDR StreamBaseAddr)
{
    SINT32 i, j;
    UINT32 D32 = 0;
    SINT32 SliceNum = pMp2DecParam->SlcNum;
    MP2_SLCSTR_S *SliceInfo = pMp2DecParam->SlcStr;
    SINT32 slice_end_mbn = 0;
    UADDR next_slice_para_addr = 0,  bit_stream_address_0,  bit_stream_address_1;
    SINT32 bit_offset_0, bit_offset_1;
    SINT32 AddSlice0Flag = 0;

    if (SliceInfo[0].slice_start_mbn != 0)
    {
        i = 0;
        //D0
        D32 = 0;
        bit_stream_address_0 = (SliceInfo[i].BsPhyAddr[0] & 0xFFFFFFF0);
        bit_offset_0 = SliceInfo[i].BsBitOffset[0] + (SliceInfo[i].BsPhyAddr[0] & 0x0F) * 8;
        ((MP2SLCDNMSG_D0 *)(&D32))->bit_len_0 = 1;
        ((MP2SLCDNMSG_D0 *)(&D32))->bit_offset_0 = bit_offset_0;
        WR_MSGWORD(SlcDnMsgVirAddr + i * 8 + 0, D32);
        dprint(PRN_DNMSG, "D0 = %#x \n", D32);

        //D1
        D32 = 0;
        ((MP2SLCDNMSG_D1 *)(&D32))->bit_stream_address_0 = bit_stream_address_0 - StreamBaseAddr;
        WR_MSGWORD(SlcDnMsgVirAddr + i * 8 + 1, D32);
        dprint(PRN_DNMSG, "D1 = %#x \n", D32);

        //D2
        D32 = 0;
        bit_stream_address_1 = (SliceInfo[i].BsPhyAddr[1] & 0xFFFFFFF0);
        bit_offset_1 = SliceInfo[i].BsBitOffset[1] + (SliceInfo[i].BsPhyAddr[1] & 0x0F) * 8;
        ((MP2SLCDNMSG_D2 *)(&D32))->bit_len_1 = 0;
        ((MP2SLCDNMSG_D2 *)(&D32))->bit_offset_1 = bit_offset_1;
        WR_MSGWORD(SlcDnMsgVirAddr + i * 8 + 2, D32);
        dprint(PRN_DNMSG, "D2 = %#x \n", D32);

        //D3
        D32 = 0;

        if (SliceInfo[i].BsPhyAddr[1] > 0)
        {
            ((MP2SLCDNMSG_D3 *)(&D32))->bit_stream_address_1 = bit_stream_address_1 - StreamBaseAddr;
        }
        else
        {
            ((MP2SLCDNMSG_D3 *)(&D32))->bit_stream_address_1 = 0;
        }

        WR_MSGWORD(SlcDnMsgVirAddr + i * 8 + 3, D32);
        dprint(PRN_DNMSG, "D3 = %#x \n", D32);

        //D4
        D32 = 0;
        ((MP2SLCDNMSG_D4 *)(&D32))->quantiser_scale_code = SliceInfo[i].quantiser_scale_code;
        ((MP2SLCDNMSG_D4 *)(&D32))->intra_slice = SliceInfo[i].intra_slice;
        WR_MSGWORD(SlcDnMsgVirAddr + i * 8 + 4, D32);
        dprint(PRN_DNMSG, "D4 = %#x \n", D32);

        //D5
        D32 = 0;
        ((MP2SLCDNMSG_D5 *)(&D32))->slice_start_mbn = 0;
        WR_MSGWORD(SlcDnMsgVirAddr + i * 8 + 5, D32);
        dprint(PRN_DNMSG, "D5 = %#x \n", D32);

        //D6
        D32 = 0;
        slice_end_mbn = (SliceInfo[0].slice_start_mbn) - 1;
        ((MP2SLCDNMSG_D6 *)(&D32))->slice_end_mbn = slice_end_mbn;
        WR_MSGWORD(SlcDnMsgVirAddr + i * 8 + 6, D32);
        dprint(PRN_DNMSG, "D6 = %#x \n", D32);
        //D7
        D32 = 0;
        next_slice_para_addr = SlcDnMsgPhyAddr + (i + 1) * 4 * 8;
        ((MP2SLCDNMSG_D7 *)(&D32))->next_slice_para_addr = next_slice_para_addr;
        WR_MSGWORD(SlcDnMsgVirAddr + i * 8 + 7, D32);

        AddSlice0Flag = 1;
    }

    for (i = 0; i < SliceNum; i++)
    {
        if ((SliceInfo[i].slice_start_mbn <= SliceInfo[i - 1].slice_start_mbn) && (i > 0))
        {
            continue;
        }

        //D0
        D32 = 0;
        bit_stream_address_0 = (SliceInfo[i].BsPhyAddr[0] & 0xFFFFFFF0);
        bit_offset_0 = SliceInfo[i].BsBitOffset[0] + (SliceInfo[i].BsPhyAddr[0] & 0x0F) * 8;
        ((MP2SLCDNMSG_D0 *)(&D32))->bit_len_0 = SliceInfo[i].BsLenInBit[0];
        ((MP2SLCDNMSG_D0 *)(&D32))->bit_offset_0 = bit_offset_0;
        WR_MSGWORD(SlcDnMsgVirAddr + i * 8 + 0 + AddSlice0Flag * 4 * 8, D32);
        dprint(PRN_DNMSG, "D0 = %#x \n", D32);

        //D1
        D32 = 0;
        ((MP2SLCDNMSG_D1 *)(&D32))->bit_stream_address_0 = bit_stream_address_0 - StreamBaseAddr;
        WR_MSGWORD(SlcDnMsgVirAddr + i * 8 + 1 + AddSlice0Flag * 4 * 8, D32);
        dprint(PRN_DNMSG, "D1 = %#x \n", D32);

        //D2
        D32 = 0;
        bit_stream_address_1 = (SliceInfo[i].BsPhyAddr[1] & 0xFFFFFFF0);
        bit_offset_1 = SliceInfo[i].BsBitOffset[1] + (SliceInfo[i].BsPhyAddr[1] & 0x0F) * 8;
        ((MP2SLCDNMSG_D2 *)(&D32))->bit_len_1 = SliceInfo[i].BsLenInBit[1];
        ((MP2SLCDNMSG_D2 *)(&D32))->bit_offset_1 = bit_offset_1;
        WR_MSGWORD(SlcDnMsgVirAddr + i * 8 + 2 + AddSlice0Flag * 4 * 8, D32);
        dprint(PRN_DNMSG, "D2 = %#x \n", D32);

        //D3
        D32 = 0;

        if (SliceInfo[i].BsPhyAddr[1] > 0)
        {
            ((MP2SLCDNMSG_D3 *)(&D32))->bit_stream_address_1 = bit_stream_address_1 - StreamBaseAddr;
        }
        else
        {
            ((MP2SLCDNMSG_D3 *)(&D32))->bit_stream_address_1 = 0;
        }

        WR_MSGWORD(SlcDnMsgVirAddr + i * 8 + 3 + AddSlice0Flag * 4 * 8, D32);
        dprint(PRN_DNMSG, "D3 = %#x \n", D32);

        //D4
        D32 = 0;
        ((MP2SLCDNMSG_D4 *)(&D32))->quantiser_scale_code = SliceInfo[i].quantiser_scale_code;
        ((MP2SLCDNMSG_D4 *)(&D32))->intra_slice = SliceInfo[i].intra_slice;
        WR_MSGWORD(SlcDnMsgVirAddr + i * 8 + 4 + AddSlice0Flag * 4 * 8, D32);
        dprint(PRN_DNMSG, "D4 = %#x \n", D32);

        //D5
        D32 = 0;
        ((MP2SLCDNMSG_D5 *)(&D32))->slice_start_mbn = SliceInfo[i].slice_start_mbn;
        WR_MSGWORD(SlcDnMsgVirAddr + i * 8 + 5 + AddSlice0Flag * 4 * 8, D32);
        dprint(PRN_DNMSG, "D5 = %#x \n", D32);

        for (j = (i + 1); j < SliceNum; j++)
        {
            if (SliceInfo[i].slice_start_mbn < SliceInfo[j].slice_start_mbn)
            {
                break;
            }
        }

        if (j == SliceNum)
        {
            slice_end_mbn = ((pMp2DecParam->PicWidthInMb) * (pMp2DecParam->PicHeightInMb)) - 1;
            next_slice_para_addr = 0;
        }
        else
        {
            slice_end_mbn = (SliceInfo[j].slice_start_mbn) - 1;
            next_slice_para_addr = SlcDnMsgPhyAddr + j * 4 * 8 + AddSlice0Flag * 4 * 8;
        }

        //D6
        D32 = 0;
        ((MP2SLCDNMSG_D6 *)(&D32))->slice_end_mbn = slice_end_mbn;
        WR_MSGWORD(SlcDnMsgVirAddr + i * 8 + 6 + AddSlice0Flag * 4 * 8, D32);
        dprint(PRN_DNMSG, "D6 = %#x \n", D32);
        //D7
        D32 = 0;
        ((MP2SLCDNMSG_D7 *)(&D32))->next_slice_para_addr = next_slice_para_addr;
        WR_MSGWORD(SlcDnMsgVirAddr + i * 8 + 7 + AddSlice0Flag * 4 * 8, D32);
        dprint(PRN_DNMSG, "D7 = %#x \n", D32);

        i = j - 1;
    }

    return 0;
}

SINT32 MP2HAL_CfgReg(OMXVDH_REG_CFG_S *pVdhRegCfg)
{
    UINT32 datTmp;
    SINT32 D32;
    WR_VREG( VREG_BASIC_CFG0, pVdhRegCfg->VdhBasicCfg0, 0  );

    /*set uv order 0: v first; 1: u first*/
    WR_VREG( VREG_BASIC_CFG1, pVdhRegCfg->VdhBasicCfg1, 0 );

    WR_VREG( VREG_AVM_ADDR, pVdhRegCfg->VdhAvmAddr, 0  );

    WR_VREG( VREG_VAM_ADDR, pVdhRegCfg->VdhVamAddr, 0  );

    WR_VREG( VREG_STREAM_BASE_ADDR, pVdhRegCfg->VdhStreamBaseAddr, 0  );

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
    WR_VREG( VREG_YSTRIDE_1D, pVdhRegCfg->VdhYstride, 0 );
    WR_VREG( VREG_UVOFFSET_1D, pVdhRegCfg->VdhUvoffset, 0 );

    WR_VREG( VREG_REF_PIC_TYPE, pVdhRegCfg->VdhRefPicType, 0 );
    WR_VREG( VREG_FF_APT_EN, pVdhRegCfg->VdhFfAptEn, 0 );

    //HEAD_INF_OFFSET
    WR_VREG( VREG_HEAD_INF_OFFSET, pVdhRegCfg->VdhHeadInfOffset, 0 );
    dprint(PRN_VDMREG, "HEAD_INF_OFFSET = 0x%x\n", pVdhRegCfg->VdhHeadInfOffset);

    return VDMHAL_OK;
}


SINT32 MP2HAL_CfgDnMsg(MP2_DEC_PARAM_S *pMp2DecParam, VDMHAL_HWMEM_S *pHwMem, SINT32 VdhId, UADDR StreamBaseAddr)
{
    SINT32 *pMsgBase;
    UINT32 D32 = 0;
    SINT32 i, j;
    UINT32 BytePos0, BytePos1;
    SINT8  *StreamVirAddr[2];
    UADDR stream_base_addr, SlcDnMsgPhyAddr;
    SINT32 *SlcDnMsgVirAddr;

    pMsgBase = (SINT32 *)MEM_Phy2Vir(pHwMem->MsgSlotAddr[DN_MSG_SLOT_INDEX]);
    VDMHAL_ASSERT_RET( NULL != pMsgBase, "can not map down msg virtual address!" );

    VDMHAL_ASSERT_RET( pMp2DecParam->PicWidthInMb <= MAX_IMG_WIDTH_IN_MB, "picture width out of range");
    VDMHAL_ASSERT_RET( pMp2DecParam->PicHeightInMb <= MAX_IMG_HEIGHT_IN_MB, "picture height out of range");

    D32 = 0;
    ((MP2DNMSG_D0 *)(&D32))->pic_width_in_mb = pMp2DecParam->PicWidthInMb - 1;
    ((MP2DNMSG_D0 *)(&D32))->pic_height_in_mb = pMp2DecParam->PicHeightInMb - 1;
    ((MP2DNMSG_D0 *)(&D32))->mpeg1_flag = pMp2DecParam->Mpeg1Flag;
    WR_MSGWORD( pMsgBase, D32);

    D32 = 0;
    ((MP2DNMSG_D1 *)(&D32))->frame_pred_frame_dct = pMp2DecParam->FramePredFrameDct;
    ((MP2DNMSG_D1 *)(&D32))->picture_structure = pMp2DecParam->PictureStructure;
    ((MP2DNMSG_D1 *)(&D32))->second_field_flag = pMp2DecParam->SecondFieldFlag;
    ((MP2DNMSG_D1 *)(&D32))->concealment_motion_vectors = pMp2DecParam->ConcealmentMotionVectors;
    ((MP2DNMSG_D1 *)(&D32))->pic_coding_type = pMp2DecParam->PicCodingType;
    ((MP2DNMSG_D1 *)(&D32))->full_pel_forward_vector = pMp2DecParam->Mp1FwdmvFullPel;
    ((MP2DNMSG_D1 *)(&D32))->full_pel_backward_vector = pMp2DecParam->Mp1BwdmvFullPel;
    WR_MSGWORD( pMsgBase + 1, D32);

    D32 = 0;
    ((MP2DNMSG_D2 *)(&D32))->fcode_11 = pMp2DecParam->Fcode[3];
    ((MP2DNMSG_D2 *)(&D32))->fcode_10 = pMp2DecParam->Fcode[2];
    ((MP2DNMSG_D2 *)(&D32))->fcode_01 = pMp2DecParam->Fcode[1];
    ((MP2DNMSG_D2 *)(&D32))->fcode_00 = pMp2DecParam->Fcode[0];
    ((MP2DNMSG_D2 *)(&D32))->top_field_first = pMp2DecParam->TopFieldFirst;
    WR_MSGWORD( pMsgBase + 2, D32);

    D32 = 0;
    ((MP2DNMSG_D3 *)(&D32))->intra_dc_precision = pMp2DecParam->IntraDcPrecision;
    ((MP2DNMSG_D3 *)(&D32))->q_scale_type = pMp2DecParam->QuantType;
    ((MP2DNMSG_D3 *)(&D32))->intra_vlc_format = pMp2DecParam->IntraVlcFormat;
    ((MP2DNMSG_D3 *)(&D32))->alternate_scan = pMp2DecParam->AlternateScan;
    WR_MSGWORD( pMsgBase + 3, D32);

    D32 = 0;
    ((MP2DNMSG_D4 *)(&D32))->bwd_address = (pMp2DecParam->BwdRefPhyAddr + 0xF) & 0xFFFFFFF0;
    WR_MSGWORD( pMsgBase + 4, D32);

    D32 = 0;
    ((MP2DNMSG_D5 *)(&D32))->fwd_address = (pMp2DecParam->FwdRefPhyAddr + 0xF) & 0xFFFFFFF0;
    WR_MSGWORD( pMsgBase + 5, D32);

    D32 = 0;
    ((MP2DNMSG_D6 *)(&D32))->rcn_address = (pMp2DecParam->DispFramePhyAddr + 0xF) & 0xFFFFFFF0;
    WR_MSGWORD( pMsgBase + 6, D32);

    D32 = 0;
    ((MP2DNMSG_D7 *)(&D32))->current_pmv_addr = (pMp2DecParam->PmvColmbPhyAddr + 0xF) & 0xFFFFFFF0;
    WR_MSGWORD( pMsgBase + 7, D32);

    StreamVirAddr[0] = (SINT8 *)MEM_Phy2Vir(pMp2DecParam->StreamPhyAddr[0]);
    StreamVirAddr[1] = (SINT8 *)MEM_Phy2Vir(pMp2DecParam->StreamPhyAddr[1]);

    if (0 == pMp2DecParam->StreamLength[1])
    {
        BytePos0 = pMp2DecParam->StreamPhyAddr[0] + pMp2DecParam->StreamBitOffset[0] / 8;
        stream_base_addr = BytePos0 & 0xFFFFFFF0;

        D32 = 0;
        ((MP2DNMSG_D8 *)(&D32))->bit_stream_address_0 = 0;
        WR_MSGWORD( pMsgBase + 8, D32);

        D32 = 0;
        ((MP2DNMSG_D9 *)(&D32))->bit_len_0 = pMp2DecParam ->StreamLength[0] + MPEG2_DUMMY_BITS;
        ((MP2DNMSG_D9 *)(&D32))->bit_offset_0 = 8 * (BytePos0 & 0x0F) + (pMp2DecParam->StreamBitOffset[0] % 8);
        WR_MSGWORD( pMsgBase + 9, D32);

        if ( NULL != StreamVirAddr[0] )
        {
            UINT8 *rp = StreamVirAddr[0] + pMp2DecParam->StreamBitOffset[0] / 8;
            dprint(PRN_STR_HEAD, "Stream Head (8bytes): 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
                   rp[0], rp[1], rp[2], rp[3], rp[4], rp[5], rp[6], rp[7]);
            rp = StreamVirAddr[0] + pMp2DecParam->StreamBitOffset[0] / 8 + pMp2DecParam ->StreamLength[0] / 8 - 8;
            dprint(PRN_STR_TAIL, "Stream Tail (8bytes): 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
                   rp[0], rp[1], rp[2], rp[3], rp[4], rp[5], rp[6], rp[7]);
        }

        D32 = 0;
        ((MP2DNMSG_D10 *)(&D32))->bit_stream_address_1 = 0;
        WR_MSGWORD( pMsgBase + 10, D32);

        D32 = 0;
        ((MP2DNMSG_D11 *)(&D32))->bit_len_1 = 0;
        ((MP2DNMSG_D11 *)(&D32))->bit_offset_1 = 0;
        WR_MSGWORD( pMsgBase + 11, D32);
    }
    else
    {
        BytePos1 = pMp2DecParam->StreamPhyAddr[1] + pMp2DecParam->StreamBitOffset[1] / 8;
        stream_base_addr = BytePos1 & 0xFFFFFFF0;
        BytePos0 = pMp2DecParam->StreamPhyAddr[0] + pMp2DecParam->StreamBitOffset[0] / 8;

        D32 = 0;
        ((MP2DNMSG_D8 *)(&D32))->bit_stream_address_0 = (BytePos0 & 0xFFFFFFF0) - stream_base_addr;
        WR_MSGWORD( pMsgBase + 8, D32);

        D32 = 0;
        ((MP2DNMSG_D9 *)(&D32))->bit_len_0 = pMp2DecParam ->StreamLength[0];
        ((MP2DNMSG_D9 *)(&D32))->bit_offset_0 = 8 * (BytePos0 & 0x0F) + (pMp2DecParam->StreamBitOffset[0] % 8);
        WR_MSGWORD( pMsgBase + 9, D32);

        if ( NULL != StreamVirAddr[0] )
        {
            UINT8 *rp = StreamVirAddr[0] + pMp2DecParam->StreamBitOffset[0] / 8;
            dprint(PRN_STR_HEAD, "1p Stream Head (8bytes): 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
                   rp[0], rp[1], rp[2], rp[3], rp[4], rp[5], rp[6], rp[7]);
            rp = StreamVirAddr[0] + pMp2DecParam->StreamBitOffset[0] / 8 + pMp2DecParam ->StreamLength[0] / 8 - 8;
            dprint(PRN_STR_TAIL, "1p Stream Tail (8bytes): 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
                   rp[0], rp[1], rp[2], rp[3], rp[4], rp[5], rp[6], rp[7]);
            dprint(PRN_DBG, "1p last phy addr = 0x%x\n", pMp2DecParam->StreamPhyAddr[0] + pMp2DecParam->StreamLength[0]);
        }

        D32 = 0;
        ((MP2DNMSG_D10 *)(&D32))->bit_stream_address_1 = 0;
        WR_MSGWORD( pMsgBase + 10, D32);

        D32 = 0;
        ((MP2DNMSG_D11 *)(&D32))->bit_len_1 = pMp2DecParam ->StreamLength[1] + MPEG2_DUMMY_BITS;
        ((MP2DNMSG_D11 *)(&D32))->bit_offset_1 = 8 * (BytePos1 & 0x0F) + (pMp2DecParam->StreamBitOffset[1] % 8);
        WR_MSGWORD( pMsgBase + 11, D32);

        if ( NULL != StreamVirAddr[1] )
        {
            UINT8 *rp = StreamVirAddr[1] + pMp2DecParam->StreamBitOffset[1] / 8;
            dprint(PRN_STR_HEAD, "2p Stream Head (8bytes): 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
                   rp[0], rp[1], rp[2], rp[3], rp[4], rp[5], rp[6], rp[7]);
            rp = StreamVirAddr[1] + pMp2DecParam->StreamBitOffset[1] / 8 + pMp2DecParam ->StreamLength[1] / 8 - 8;
            dprint(PRN_STR_TAIL, "2p Stream Tail (8bytes): 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
                   rp[0], rp[1], rp[2], rp[3], rp[4], rp[5], rp[6], rp[7]);
        }
    }

    for (i = 0; i < 8; i++)
    {
        for (j = 0; j < 2; j++)
        {
            ((MP2DNMSG_D16_31 *)(&D32))->intra_quantiser_matrix_0 =  pMp2DecParam->IntraQuantTab[i + 8 * j + 0];
            ((MP2DNMSG_D16_31 *)(&D32))->intra_quantiser_matrix_1 = pMp2DecParam->IntraQuantTab[i + 8 * j + 16];
            ((MP2DNMSG_D16_31 *)(&D32))->intra_quantiser_matrix_2 = pMp2DecParam->IntraQuantTab[i + 8 * j + 32];
            ((MP2DNMSG_D16_31 *)(&D32))->intra_quantiser_matrix_3 = pMp2DecParam->IntraQuantTab[i + 8 * j + 48];

            WR_MSGWORD( pMsgBase + 16 + 2 * i + j, D32);

            ((MP2DNMSG_D32_47 *)(&D32))->non_intra_quantiser_matrix_0 = pMp2DecParam->NonIntraQuantTab[i + 8 * j + 0];
            ((MP2DNMSG_D32_47 *)(&D32))->non_intra_quantiser_matrix_1 = pMp2DecParam->NonIntraQuantTab[i + 8 * j + 16];
            ((MP2DNMSG_D32_47 *)(&D32))->non_intra_quantiser_matrix_2 = pMp2DecParam->NonIntraQuantTab[i + 8 * j + 32];
            ((MP2DNMSG_D32_47 *)(&D32))->non_intra_quantiser_matrix_3 = pMp2DecParam->NonIntraQuantTab[i + 8 * j + 48];

            WR_MSGWORD( pMsgBase + 32 + 2 * i + j, D32);
        }
    }

    D32 = 0;
    ((MP2DNMSG_D48 *)(&D32))->pmv_top_addr = (pHwMem->PmvTopAddr + 0xF) & 0xFFFFFFF0;
    WR_MSGWORD( pMsgBase + 48, D32);

    /* ?????? */
    SlcDnMsgPhyAddr = (pHwMem->MsgSlotAddr[DN_MSG_SLOT_INDEX] & 0xFFFFFFF0) + 64 * 4;
    SlcDnMsgVirAddr = MEM_Phy2Vir(SlcDnMsgPhyAddr);
    VDMHAL_ASSERT_RET(NULL != SlcDnMsgVirAddr, "can not map a valid SlcDnMsgVirAddr address");

    D32 = 0;
    ((MP2DNMSG_D63 *)(&D32))->first_slc_dnmsg_addr = SlcDnMsgPhyAddr;
    WR_MSGWORD( pMsgBase + 63, D32);

    MP2HAL_WriteSliceMsg(pMp2DecParam, SlcDnMsgVirAddr, SlcDnMsgPhyAddr, StreamBaseAddr);
    {
        UINT32 i_cnt;
        static UINT32 num = 0;

        if ((g_PrintEnable & (1 << PRN_DNMSG)) != 0)
        {
            num++;
            dprint(PRN_DNMSG, "\n*****No.%2d Down Msg (phy addr: %#8x) *****\n", num, pHwMem->MsgSlotAddr[DN_MSG_SLOT_INDEX]);

            for (i_cnt = 0; i_cnt < 64 - 3; i_cnt += 4)
            {
                dprint(PRN_DNMSG, "\n0x%02x 0x%08x 0x%08x 0x%08x 0x%08x\n",
                       i_cnt, *(pMsgBase + i_cnt), *(pMsgBase + i_cnt + 1), *(pMsgBase + i_cnt + 2), *(pMsgBase + i_cnt + 3));
            }

            dprint(PRN_DNMSG, "\n***** Down Msg print finished *****\n");
        }
    }
    return VDMHAL_OK;
}

#endif
