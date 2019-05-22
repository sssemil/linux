#ifndef __VDM_HAL__HEVC_H__
#define __VDM_HAL__HEVC_H__

#include "basedef.h"
#include "mem_manage.h"
#define HEVC_I_SLICE                   (2)
#define HEVC_P_SLICE                   (1)
#define HEVC_B_SLICE                   (0)

SINT32 HEVCHAL_InitHal(VDMHAL_HWMEM_S *pHwMem);
SINT32 HEVCHAL_StartDec(OMXVDH_REG_CFG_S *pVdhRegCfg);

#endif
