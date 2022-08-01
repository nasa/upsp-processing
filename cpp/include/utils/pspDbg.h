#ifndef _psp_dbg_h_
#define _psp_dbg_h_

#include "pspError.h"

#define _FL_ std::cerr<<__FILE__<<":"<<__LINE__<<": "

#ifdef DBG
#define FANCYMESG(m)  { _FL_<<"===== ["<<m<<"] =====\n";}
#define FUNC(m)  { _FL_<<"===== ["<<m<<"] =====\n";}
#define MESG(m)  { _FL_<<"=== ["<<m<<"] ===\n";}
#define FANCY(m) { _FL_<<"=== ["<<#m<<" = "<<m<<"] ===\n";}
#define VAR(v)   { _FL_<<#v<<"=["<<v<<"]\n";}
#define VAR2(u,v) { _FL_<<"["<<#u<<","<<#v<<"]="<<u<<","<<v<<std::endl;}
#define VAR3(u,v,w) { _FL_<<"["<<#u<<","<<#v<<","<<#w<<"]="<<u<<","<<v<<","<<w<<std::endl;}
#define VAR3V(v) { _FL_<<"["<<#v<<"]="<<v[0]<<","<<v[1]<<","<<v[2]<<std::endl;}
#define VARHEX(v){_FL_<<#v<<" = 0x"<<std::hex<<(void*)v<<std::dec<<std::endl;}
#else
#define FANCYMESG(f)
#define FUNC(f)
#define MESG(m)
#define FANCY(m)
#define VAR(v)
#define VAR2(u,v)
#define VAR3(u,v,w)
#define VAR3V(v)
#define VARHEX(v)
#endif

#endif // _psp_dbg_h_

