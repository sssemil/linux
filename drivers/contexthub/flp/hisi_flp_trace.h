#if !defined(_HISI_FLP_TRACE_H_) || defined(TRACE_HEADER_MULTI_READ)
#define _DRM_TRACE_H_

#include <linux/stringify.h>
#include <linux/types.h>
#include <linux/tracepoint.h>

#undef TRACE_SYSTEM
#define TRACE_SYSTEM hisi_flp
#undef TRACE_SYSTEM_STRING
#define TRACE_SYSTEM_STRING __stringify(TRACE_SYSTEM)
#define TRACE_INCLUDE_FILE hisi_flp_trace

TRACE_EVENT(flp_pdr_event, /* [false alarm]:ftrace_raw_output_flp_pdr_event */
	    TP_PROTO(int x,  int y),
	    TP_ARGS(x, y),
	    TP_STRUCT__entry(
		    __field(int, x)
		    __field(int, y)
		    ),
	    TP_fast_assign(
		    __entry->x = x;
		    __entry->y = y;
		    ),
	    TP_printk("PDRDATA %d:%d", __entry->x, __entry->y)
);
TRACE_EVENT(flp_ar_event, /* [false alarm]:ftrace_raw_output_flp_ar_event */
	    TP_PROTO(unsigned int a,  unsigned int b, unsigned int c, unsigned long d, unsigned int e, unsigned int f, unsigned int g, unsigned int h, unsigned int i, unsigned short int l),
	    TP_ARGS(a, b, c, d, e, f, g, h, i, l),
	    TP_STRUCT__entry(
		    __field(unsigned int, a)
		    __field(unsigned int, b)
		    __field(unsigned int, c)
		    __field(unsigned long, d)
		    __field(unsigned int, e)
		    __field(unsigned int, f)
		    __field(unsigned int, g)
		    __field(unsigned int, h)
		    __field(unsigned int, i)
		    __field(unsigned int, l)
		    ),
	    TP_fast_assign(
		    __entry->a = a;
		    __entry->b = b;
		    __entry->c = c;
		    __entry->d = d;
		    __entry->e = e;
		    __entry->f = f;
		    __entry->g = g;
		    __entry->h = h;
		    __entry->i = i;
		    __entry->l = l;
		    ),
	    TP_printk("ARDATA [%u][%u][%u][%lu][%u][%u][%u][%u][%u][%x]", __entry->a, __entry->b,__entry->c, __entry->d,__entry->e, __entry->f,__entry->g, __entry->h, __entry->i, __entry->l)
); /* [false alarm]:fortify */

TRACE_EVENT(flp_env_event, /* [false alarm]:ftrace_raw_output_flp_ar_event */
	    TP_PROTO(unsigned int a,  unsigned int b, unsigned int c, unsigned long d, unsigned int e, unsigned int f, unsigned int g, unsigned int h),
	    TP_ARGS(a, b, c, d, e, f, g, h),
	    TP_STRUCT__entry(
		    __field(unsigned int, a)
		    __field(unsigned int, b)
		    __field(unsigned int, c)
		    __field(unsigned long, d)
		    __field(unsigned int, e)
		    __field(unsigned int, f)
		    __field(unsigned int, g)
		    __field(unsigned int, h)
		    ),
	    TP_fast_assign(
		    __entry->a = a;
		    __entry->b = b;
		    __entry->c = c;
		    __entry->d = d;
		    __entry->e = e;
		    __entry->f = f;
		    __entry->g = g;
		    __entry->h = h;
		    ),
	    TP_printk("ENVDATA [%u][%u][%u][%lu][%u][%u][%u][%u]", __entry->a, __entry->b,__entry->c, __entry->d,__entry->e, __entry->f,__entry->g, __entry->h)
); /* [false alarm]:fortify */
TRACE_EVENT(flp_cmd, /* [false alarm]:ftrace_raw_output_flp_cmd */
	    TP_PROTO(unsigned int x),
	    TP_ARGS(x),
	    TP_STRUCT__entry(
		    __field(unsigned int, x)
		    ),
	    TP_fast_assign(
		    __entry->x = x;
		    ),
	    TP_printk("FLP CMD:%x", __entry->x)
); /* [false alarm]:fortify */

#endif /* _HISI_FLP_TRACE_H_ */

/* This part must be outside protection */
#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#undef TRACE_SYSTEM_STRING
#include <trace/define_trace.h>
