/** @file
 *  @brief  Main uPSP Header
 *  @date   Nov 20, 2018
 *  @author jmpowel2
 */

#ifndef UFML_UPSP_H_
#define UFML_UPSP_H_

// Add the utilities
#include "utils/clustering.h"
#include "utils/cv_extras.h"
#include "utils/file_readers.h"
#include "utils/general_utils.h"

// Add core capabilities
#include "data_structs.h"
#include "filtering.h"
#include "grids.h"
#include "image_processing.h"
#include "integration.h"
#include "interpolation.h"
#include "kulites.h"
#include "models.h"
#include "non_cv_upsp.h"
#include "patches.h"
#include "projection.h"
#include "registration.h"
#include "upsp_inputs.h"

// Add data structures
#include "CameraCal.h"
#include "CineReader.h"
#include "MrawReader.h"
#include "Model.h"
#include "Octree.h"
#include "P3DModel.h"
#include "PSPHDF5.h"
#include "TriModel.h"

#endif /* UFML_UPSP_H_ */
