#ifndef _psp_timer_h_
#define _psp_timer_h_

#include <iostream>
#include <string>
#include <chrono>

namespace psp {

    class BlockTimer {
      std::string label;
      bool enabled;

      std::chrono::time_point<std::chrono::system_clock> bgn;

    public:
      BlockTimer(bool dbg, std::string n, bool showstart=false) :
	label(n), enabled(dbg)
      {
	if( enabled ) {
	  bgn = std::chrono::system_clock::now();
	  if(showstart) std::cerr<<"[ "<<label<<" starting ]\n";
	}
      }

      BlockTimer(std::string n, bool showstart=false) :
	label(n) , enabled(true)
      {
	bgn = std::chrono::system_clock::now();
	if(showstart) std::cerr<<"[ "<<label<<" starting ]\n";
      }

      ~BlockTimer(void){
	if( enabled ){
	  auto end = std::chrono::system_clock::now();
	  std::chrono::duration<double> elapsed_seconds = end-bgn;
	  std::cerr<<"[ "<<label<<" done in "<<elapsed_seconds.count()<<"s ]\n";
	}
      }

    };
} // namespace psp

#endif
