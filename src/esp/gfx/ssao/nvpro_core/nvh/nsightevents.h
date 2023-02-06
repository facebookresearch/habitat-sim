#ifndef __NSIGHTEVENTS__
#define __NSIGHTEVENTS__

/// \nodoc (keyword to exclude this file from automatic README.md generation)

//-----------------------------------------------------------------------------
// NSIGHT
//-----------------------------------------------------------------------------
#ifdef NVP_SUPPORTS_NVTOOLSEXT
// NSight perf markers - take the whole stuff from "C:\Program Files
// (x86)\NVIDIA GPU Computing Toolkit\nvToolsExt"
#include "../nvtx3/nvToolsExt.h>

typedef int(NVTX_API* nvtxRangePushEx_Pfn)(
    const nvtxEventAttributes_t* eventAttrib);
typedef int(NVTX_API* nvtxRangePush_Pfn)(const char* message);
typedef int(NVTX_API* nvtxRangePop_Pfn)();
extern nvtxRangePushEx_Pfn nvtxRangePushEx_dyn;
extern nvtxRangePush_Pfn nvtxRangePush_dyn;
extern nvtxRangePop_Pfn nvtxRangePop_dyn;
extern nvtxEventAttributes_t eventAttr;

#define NX_RANGE nvtxRangeId_t
#define NX_MARK(name) nvtxMark(name)
#define NX_RANGESTART(name) nvtxRangeStart(name)
#define NX_RANGEEND(id) nvtxRangeEnd(id)
#define NX_RANGEPUSH(name) nvtxRangePush(name)
#define NX_RANGEPUSHCOL(name, c)                       \
  {                                                    \
    nvtxEventAttributes_t eventAttrib = {0};           \
    eventAttrib.version = NVTX_VERSION;                \
    eventAttrib.size = NVTX_EVENT_ATTRIB_STRUCT_SIZE;  \
    eventAttrib.colorType = NVTX_COLOR_ARGB;           \
    eventAttrib.color = c;                             \
    eventAttrib.messageType = NVTX_MESSAGE_TYPE_ASCII; \
    eventAttrib.message.ascii = name;                  \
    nvtxRangePushEx(&eventAttrib);                     \
  }
#define NX_RANGEPOP() nvtxRangePop()
struct NXProfileFunc {
  NXProfileFunc(const char* name, uint32_t c, /*int64_t*/ uint32_t p = 0) {
    nvtxEventAttributes_t eventAttrib = {0};
    // set the version and the size information
    eventAttrib.version = NVTX_VERSION;
    eventAttrib.size = NVTX_EVENT_ATTRIB_STRUCT_SIZE;
    // configure the attributes.  0 is the default for all attributes.
    eventAttrib.colorType = NVTX_COLOR_ARGB;
    eventAttrib.color = c;
    eventAttrib.messageType = NVTX_MESSAGE_TYPE_ASCII;
    eventAttrib.message.ascii = name;
    eventAttrib.payloadType = NVTX_PAYLOAD_TYPE_INT64;
    eventAttrib.payload.llValue = (int64_t)p;
    eventAttrib.category = (uint32_t)p;
    nvtxRangePushEx(&eventAttrib);
  }
  ~NXProfileFunc() { nvtxRangePop(); }
};
#ifdef NXPROFILEFUNC
#undef NXPROFILEFUNC
#undef NXPROFILEFUNCCOL
#undef NXPROFILEFUNCCOL2
#endif
#define NXPROFILEFUNC(name) NXProfileFunc nxProfileMe(name, 0xFF0000FF)
#define NXPROFILEFUNCCOL(name, c) NXProfileFunc nxProfileMe(name, c)
#define NXPROFILEFUNCCOL2(name, c, p) NXProfileFunc nxProfileMe(name, c, p)
#else
#define NX_RANGE int
#define NX_MARK(name)
#define NX_RANGESTART(name) 0
#define NX_RANGEEND(id)
#define NX_RANGEPUSH(name)
#define NX_RANGEPUSHCOL(name, c)
#define NX_RANGEPOP()
#define NXPROFILEFUNC(name)
#define NXPROFILEFUNCCOL(name, c)
#define NXPROFILEFUNCCOL2(name, c, a)
#endif

#endif  //__NSIGHTEVENTS__
