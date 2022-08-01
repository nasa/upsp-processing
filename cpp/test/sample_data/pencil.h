#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

#include "upsp.h"

/** Define a pencil grid */
template<typename FP>
struct Pencil {

    Pencil() {

        // Define the size of the grid
        unsigned int n_azim = 21;

        sgrid.grid_size.resize(3);
        sgrid.grid_size[0].resize(3);
        sgrid.grid_size[0][0] = n_azim;
        sgrid.grid_size[0][1] = 11;
        sgrid.grid_size[0][2] = 1;
        sgrid.grid_size[1].resize(3);
        sgrid.grid_size[1][0] = n_azim;
        sgrid.grid_size[1][1] = 41;
        sgrid.grid_size[1][2] = 1;
        sgrid.grid_size[2].resize(3);
        sgrid.grid_size[2][0] = n_azim;
        sgrid.grid_size[2][1] = 3;
        sgrid.grid_size[2][2] = 1;

        unsigned int n_long = 0;
        for (unsigned int i=0; i < sgrid.grid_size.size(); ++i) {
            n_long += sgrid.grid_size[i][1];
        }
        
        sgrid.x.resize(n_azim*n_long);
        sgrid.y.resize(n_azim*n_long);
        sgrid.z.resize(n_azim*n_long);

        // define the x,y,z positions of each grid point
        float x = 0.0;
        unsigned int node = 0;
        for (unsigned int z=0; z < sgrid.grid_size.size(); ++z) {
            for (unsigned int i=0; i < sgrid.grid_size[z][1]; ++i) {

                // Set the radius of the body at different x-stations
                float radius = 0.0;
                if (z == 2) {
                    if (i == 0) {
                        radius = 7.5;
                    } else if (i == 1) {
                        radius = 3.75;
                    } else {
                        radius = 0.0;
                    }
                } else if (x >= 10) {
                    radius = 7.5;
                } else {
                    radius = x * (7.5/10);
                }

                // Define the x,y,z position for each node
                for (unsigned int j=0; j < sgrid.grid_size[z][0]; ++j) {
                    sgrid.x[node] = x;
                    float incr = (float) j / (float) (n_azim-1);
                    if ( (j == 0) || (j == (n_azim-1)) ) {
                        // want exact values for exact intersection
                        sgrid.y[node] = radius;
                        sgrid.z[node] = 0.0;
                    } else {
                        sgrid.y[node] = radius * cos(incr * 2.0 * PI);
                        sgrid.z[node] = radius * sin(incr * 2.0 * PI);
                    }
                    ++node;
                }

                // Update x position
                if ( (z != 2) && ( i != sgrid.grid_size[z][1]-1) ) {
                    x += 1.0;
                }
            }
        }
    }

    /** Return a p3d model version of the pencil grid */
    upsp::P3DModel_<FP> get_p3d_model() const {
        return upsp::P3DModel_<FP>(sgrid);
    }

    /** Return a tri model version of the pencil grid
     *
     * @param[in] intersect     if true, intersect the grid to 
     *                          remove redundant nodes and collapsed 
     *                          triangles
     * @return                  tri model
     *
     * @post if !intersect, @return.size() == sgrid.size()
     */
    upsp::TriModel_<FP> get_tri_model(bool intersect) const {
        return upsp::TriModel_<FP>(get_p3d_model(), intersect);
    }

    /*****************************************************/

    upsp::StructuredGrid<FP> sgrid;

};

