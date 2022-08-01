#include "utils/file_writers.h"
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>

namespace upsp {

int fwrite(const std::string& fname, const std::vector<float>& v,
          const int maxels) {
    if (v.empty()) return -1;
    int numels = maxels > 0 ? maxels : v.size();
    float *outp = (float*) malloc(sizeof(float) * numels);
    if (!outp) return -1;
    int step = v.size() < numels ? 1 : v.size() / numels;
    int ii = 0, jj = 0;
    while (ii < numels && jj < v.size()) {
        outp[ii] = v[jj];
        ii = ii + 1;
        jj = jj + step;
    }
    FILE *fp = fopen(fname.c_str(), "wb");
    if (!fp) {
        free(outp);
        return -1;
    }
    int res = fwrite((void*) outp, sizeof(float), ii, fp);
    fclose(fp);
    free(outp);
    return res;
}

}
