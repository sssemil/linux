#ifndef __OMXVDEC_H__
#define __OMXVDEC_H__

#include "platform.h"
#include "drv_omxvdec.h"

#define OMXVDEC_VERSION	 		          (2017032300)
#define MAX_OPEN_COUNT                    (32)

#define OMX_ALWS                          (30)
#define OMX_FATAL                         (0)
#define OMX_ERR                           (1)
#define OMX_WARN                          (2)
#define OMX_INFO                          (3)
#define OMX_TRACE                         (4)
#define OMX_INBUF                         (5)
#define OMX_OUTBUF                        (6)
#define OMX_VPSS                          (7)
#define OMX_VER                           (8)
#define OMX_PTS                           (9)
#define OMX_MEM                           (10)

extern HI_U32 g_TraceOption;

#ifndef HI_ADVCA_FUNCTION_RELEASE
#define OmxPrint(flag, format,arg...) \
    do { \
        if (OMX_ALWS == flag || (0 != (g_TraceOption & (1 << flag)))) \
            printk(KERN_ALERT format, ## arg); \
    } while (0)
#else
#define OmxPrint(flag, format,arg...)    ({do{}while(0);0;})
#endif

/*
   g_TraceOption

   1:      OMX_FATAL
   2:      OMX_ERR
   4:      OMX_WARN
   8:      OMX_INFO
   16:     OMX_TRACE
   32:     OMX_INBUF
   64:     OMX_OUTBUF
   128:    OMX_VPSS
   256:    OMX_RAWCTRL
   512:    OMX_PTS
   1024:   OMX_MEM

   3:      OMX_FATAL & OMX_ERR
   7:      OMX_FATAL & OMX_ERR & OMX_WARN
   11:     OMX_FATAL & OMX_ERR & OMX_INFO
   19:     OMX_FATAL & OMX_ERR & OMX_TRACE
   96:     OMX_INBUF & OMX_OUTBUF
   35:     OMX_FATAL & OMX_ERR & OMX_INBUF
   67:     OMX_FATAL & OMX_ERR & OMX_OUTBUF
   99:     OMX_FATAL & OMX_ERR & OMX_INBUF & OMX_OUTBUF

*/

typedef struct {
    HI_U32              open_count;
    atomic_t            nor_chan_num;
    atomic_t            sec_chan_num;
    struct semaphore    omxvdec_mutex;
    struct cdev         cdev;
    struct device      *device;
}OMXVDEC_ENTRY;

#endif

