#ifndef __HI_TYPE_H__
#define __HI_TYPE_H__

typedef unsigned char           HI_U8;
typedef unsigned char           HI_UCHAR;
typedef unsigned short          HI_U16;
typedef unsigned int            HI_U32;

typedef signed char             HI_S8;
typedef signed short            HI_S16;
typedef signed int              HI_S32;

#ifndef _M_IX86
typedef unsigned long long      HI_U64;
typedef long long               HI_S64;
#else
typedef __int64                 HI_U64;
typedef __int64                 HI_S64;
#endif

typedef char                    HI_CHAR;
typedef char*                   HI_PCHAR;

typedef float                   HI_FLOAT;
typedef double                  HI_DOUBLE;

/*typedef void                  HI_VOID;*/
#define HI_VOID                 void

typedef unsigned long           HI_SIZE_T;
typedef unsigned long           HI_LENGTH_T;

typedef HI_U32                  HI_HANDLE;

typedef unsigned long  	        HI_VIRT_ADDR_T;

typedef unsigned long  	        HI_ULONG;

typedef enum {
    HI_FALSE    = 0,
    HI_TRUE     = 1,
} HI_BOOL;

#ifndef NULL
#define NULL              (0L)
#endif

#define HI_NULL           (0L)
#define HI_NULL_PTR       (0L)

#define HI_SUCCESS        (0)
#define HI_FAILURE        (-1)

#define HI_INVALID_HANDLE (0xffffffff)

#define HI_INVALID_PTS    (0xffffffff)
#define HI_INVALID_TIME   (0xffffffff)

#define HI_OS_LINUX       (0xabcd)
#define HI_OS_WIN32       (0xcdef)

#ifdef _WIN32
#define HI_OS_TYPE        HI_OS_WIN32
#else
#define __OS_LINUX__
#define HI_OS_TYPE        HI_OS_LINUX
#endif

#ifdef HI_ADVCA_SUPPORT
#define __INIT__
#define __EXIT__
#else
#define __INIT__  __init
#define __EXIT__  __exit
#endif

#define HI_HANDLE_MAKEHANDLE(mod, privatedata, chnid)  (HI_HANDLE)( (((mod)& 0xffff) << 16) | ((((privatedata)& 0xff) << 8) ) | (((chnid) & 0xff)) )

#define HI_HANDLE_GET_MODID(handle)    (((handle) >> 16) & 0xffff)
#define HI_HANDLE_GET_PriDATA(handle)  (((handle) >> 8) & 0xff)
#define HI_HANDLE_GET_CHNID(handle)    (((handle)) & 0xff)

#define UNUSED(x) ((x)=(x))


#if defined(__KERNEL__)

#include <linux/version.h>

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 36))
#define DECLARE_MUTEX DEFINE_SEMAPHORE
#endif

#endif

#endif

