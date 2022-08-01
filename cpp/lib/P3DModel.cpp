/*
 * P3DModel.cpp
 *
 *  Created on: July 17, 2017
 *      Author: jmpowel2
 */

#include "P3DModel.h"

namespace upsp {

/*********************************************************************  
 * Additional Functions 
 ********************************************************************/ 

/*****************************************************************************/
std::ostream& operator<<(std::ostream& os, const GridIndex& gidx) { 
    os << "Grid " << gidx.zone << ": " << gidx.j << ", " << gidx.k; 
    os << ", " << gidx.l; 
    return os; 
} 

} /* end namespace upsp */
