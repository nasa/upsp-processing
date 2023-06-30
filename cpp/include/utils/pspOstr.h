#ifndef _psp_ostr_h_
#define _psp_ostr_h_

#include <iostream>
#include <iomanip>
#include <vector>
#include <map>
#include <list>
#include <sstream>
#include <set>
#include <deque>
#include <string>
#include <math.h>
#include <stdint.h>
#include <stdio.h> // for perror
#include <errno.h>

namespace std {
  template <typename T>
  inline T clamp (T val, T lo, T hi)
  {
    return std::max(lo,std::min(val,hi));
  }
}

namespace std
{
template <typename T>
inline std::ostream& operator<<(std::ostream& ostr, std::vector<T>& cont)
{
  if(cont.empty())return ostr<<"[empty]";
  int e = (int)floor(log10((double)cont.size())+1.);
  ostr << "===== [vector size=" << cont.size() << "] =====" << std::endl;
  typename std::vector<T>::iterator iter = cont.begin();
  for(int i=0 ; iter != cont.end() ; ++iter,++i ){
    ostr << "[";
    ostr.width(e);
    ostr.fill(' ');
    ostr<<i;
    ostr<<"]: " << *iter << std::endl;
  }
  return ostr;
}

template <typename T>
inline std::ostream& operator<<(std::ostream& ostr, std::set<T>& cont )
{
  if(cont.empty())return ostr<<"[empty]";
  ostr << "===== [set size=" << cont.size() << "] =====" << std::endl;
  typename std::set<T>::iterator iter = cont.begin();
  for( ; iter != cont.end() ; ++iter )
    ostr << *iter << std::endl;
  return ostr;
}

template <typename T>
inline std::ostream& operator<<(std::ostream& ostr, std::list<T>& cont )
{
  if(cont.empty())return ostr<<"[empty]";
  ostr << "===== [list size=" << cont.size() << "] =====" << std::endl;
  typename std::list<T>::iterator iter = cont.begin();
  for( ; iter != cont.end() ; ++iter )
    ostr << *iter << std::endl;
  return ostr;
}

template <typename S,typename T>
  inline std::ostream& operator<<(std::ostream& ostr, std::map<S,T>& cont )
{
  if(cont.empty())return ostr<<"[empty]";
  ostr << "===== [map size=" << cont.size() << "] =====" << std::endl;
  typename std::map<S,T>::iterator iter = cont.begin();
  for( ; iter != cont.end() ; ++iter )
    ostr << "["<<(*iter).first << "] => ["<<(*iter).second <<"]"<< std::endl;
  return ostr;
}

template <typename S,typename T>
  inline std::ostream& operator<<(std::ostream& ostr, std::multimap<S,T>& cont )
{
  if(cont.empty())return ostr<<"[empty]";
  ostr << "===== [multimap size=" << cont.size() << "] =====" << std::endl;
  typename std::multimap<S,T>::iterator iter = cont.begin();
  for( ; iter != cont.end() ; ++iter )
    ostr << "["<<(*iter).first << "] => ["<<(*iter).second <<"]"<< std::endl;
  return ostr;
}

template <typename S, typename T>
  inline std::ostream& operator<<(std::ostream& ostr, std::pair<S,T>& p )
{
  return ostr<<"["<<p.first<<","<<p.second<<"]";
}

template <typename T>
inline std::ostream& operator<<(std::ostream& ostr, std::deque<T>& d )
{
  if(d.empty())return ostr<<"[empty]";
  ostr << "===== [deque size=" << d.size() << "] =====" << std::endl;
  typename std::deque<T>::iterator iter = d.begin();
  for( ; iter != d.end() ; ++iter )
    ostr << *iter << std::endl;
  return ostr;
}
}

template <typename T>
inline std::string toa(T val, int width=0, char fill=' ')
{
  std::ostringstream ostr;
  if(width) ostr.width(width);
  if(fill !=' ') ostr.fill(fill);
  ostr<<val;
  return ostr.str();
}

template <typename T>
inline std::string itoa(T val, int width=0, char fill=' ')
{
  std::ostringstream ostr;
  if(width) ostr.width(width);
  if(fill !=' ') ostr.fill(fill);
  ostr<<val;
  return ostr.str();
}

inline std::string dtoa(double val)
{
  std::ostringstream ostr;
  ostr.precision(16);
  ostr.width(24);
  ostr.fill(' ');
  ostr.setf(std::ios::right);
  ostr<<std::scientific<<val;
  return ostr.str();
}
inline std::string ftoa(double val)
{
  std::ostringstream ostr;
  ostr.precision(6);
  ostr.width(12);
  ostr.fill(' ');
  ostr.setf(std::ios::right);
  //ostr<<std::scientific<<val;
  ostr<<val;
  return ostr.str();
}

inline std::string utoa(uint64_t val)
{
  std::ostringstream ostr;
  ostr<<val;
  return ostr.str();
}

template <typename T>
inline std::string centered(T val, size_t lw, const char fillc=' ')
{
  std::ostringstream ostr;
  size_t pad = 2;
  ostr<<val;
  std::string res=ostr.str();
  size_t slen = res.size();
  int l = (lw-(slen+pad))/2 + slen%2;
  int r = lw-(slen+pad+l);
  return std::string(l,fillc)+std::string(" ")+res+std::string(" ")+std::string(r,fillc);
}

inline std::string memsize(size_t srcbytes)
{
  const size_t KB=1000UL;
  const size_t MB=1000UL*KB;
  const size_t GB=1000UL*MB;
  const size_t TB=1000UL*GB;
  const size_t PB=1000UL*TB;
  std::ostringstream ostr;
  double kb = (double)srcbytes/KB;
  double mb = (double)srcbytes/MB;
  double gb = (double)srcbytes/GB;
  double tb = (double)srcbytes/TB;
  double pb = (double)srcbytes/PB;

  ostr.precision(2);
  ostr.width(6);
  ostr.fill(' ');
  ostr.setf(std::ios::right);
  if( kb<1000 ){
    ostr<<std::fixed<<kb<<" KB";
  } else if( mb<1000 ){
    ostr<<std::fixed<<mb<<" MB";
  } else if( gb<1000 ){
    ostr<<std::fixed<<gb<<" GB";
  } else if( tb<1000 ){
    ostr<<std::fixed<<tb<<" TB";
  } else {
    ostr<<std::fixed<<pb<<" PB";
  }
  return ostr.str();
}

inline std::string commas(size_t srcbytes)
{
  int e = (int)floor(1. + log10((double)srcbytes));
  int g = e/3 -1;
  if( e%3>0 ) ++g;
  std::ostringstream ostr;
  ostr<<srcbytes;
  std::string astr = ostr.str();
  ostr.str("");

  int idx=e;
  while( idx>=0 ){
    ostr<<astr[e-idx];
    --idx;
    if( idx%3 == 0 && idx>0) ostr<<",";
  }
  return ostr.str();
}

/*
  namespace std {

  template< typename S, typename T>
  std::istream& operator>>(std::istream& in, std::pair<S,T>& p)
  {
  in>>p.first>>p.second;
  return in;
  }
  };
*/
#endif

