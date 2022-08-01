/** @file
 *  @brief  1D Clustering
 *  @date   April 26, 2018
 *  @author jmpowel2
 */

namespace upsp {

/*****************************************************************************/
template<typename T>
void find_peaks(std::vector<T> data, std::vector<unsigned int>& peaks, 
        unsigned int separation /*= 0*/) {
    peaks.clear();

    if (data.size() < 3) {
        return;
    }

    // Find local maxima / or inf and add to peaks if it is greater than
    // any other peak within the separation band
    bool plateau = false;
    unsigned int plateau_begin = 0;
    for (unsigned int i=1; i < data.size()-1; ++i) {
        if ( std::isinf(data[i]) || ( (data[i] > data[i-1]) && (data[i] > data[i+1]) ) ) {
            if (peaks.size() > 0) {
                if ( (i - peaks[peaks.size()-1]) < separation) {
                    if (data[peaks[peaks.size()-1]] < data[i]) {
                        peaks[peaks.size()-1] = i;
                    }
                    break;
                }
            }
            peaks.push_back(i);
        } else if ( (data[i] > data[i-1]) && (data[i] == data[i+1]) ) {
            plateau = true;
            plateau_begin = i;
        } else if (plateau) {
            if (data[i] < data[i+1]) {
                plateau = false; // plateau was not an extended peak
            } else if (data[i] > data[i+1]) {
                // reached end of plateau and it is an extended peak
                plateau = false;
                // set the peak in the middle of the plateau (floor)
                unsigned int plateau_i = (i+plateau_begin) / 2;
                if (peaks.size() > 0) {
                    if ( (plateau_i - peaks[peaks.size()-1]) < separation) {
                        if (data[peaks[peaks.size()-1]] < data[plateau_i]) {
                            peaks[peaks.size()-1] = plateau_i;
                        }
                        break;
                    }
                }
                peaks.push_back(plateau_i);
            }
        }
    
    }

}

/*****************************************************************************/
template<typename T>
unsigned int first_min_threshold(const std::vector<T>& counts, 
        unsigned int separation /*=1*/) {

    // Find peaks
    std::vector<unsigned int> max_peaks;
    find_peaks(counts, max_peaks, separation);

    if (max_peaks.size() == 0) {
        return 0;
    }

    std::vector<unsigned int> min_peaks;
    std::vector<double> inverse_counts(counts.size());
    for (unsigned int i=0; i < inverse_counts.size(); ++i) {
        inverse_counts[i] = 1.0 / counts[i];
    }
    find_peaks(inverse_counts, min_peaks, separation);

    if (min_peaks.size() == 0) {
        return 0;
    }

    // find first minimum peak after the first maximum peak
    unsigned int first_max = max_peaks[0];
    //std::cout << "First Maximum " << first_max << " = " << counts[first_max] << std::endl;
    for (unsigned int i=0; i < min_peaks.size(); ++i) {
        if (min_peaks[i] > first_max) {
            //std::cout << "First Minimum " << min_peaks[i] << " = " << counts[min_peaks[i]] << std::endl;
            return min_peaks[i];
        }
    }

    return 0;
}

/*****************************************************************************/
template<typename T>
unsigned int otsu_threshold(const std::vector<T>& counts) {
    T total = std::accumulate(counts.begin(), counts.end(), 0);
    boost::counting_iterator<T> cit(0);
    T sum1 = std::inner_product(counts.begin(), counts.end(), cit, 0);
    T sum0 = 0;

    //std::cout << "index,sigma" << std::endl;

    unsigned int best_idx = 0;
    double best_wsv = 0.0;
    double w0=0.0, w1=0.0, mu1=0.0, sigma=0.0;
    for (unsigned int i = 0; i < counts.size(); ++i) {
        w0 += counts[i];
        w1 = total - w0;
        if ( (w0 == 0.0) || (w1 == 0.0) ) {
            //std::cout << i << ",0.0" << std::endl;
            continue;
        }

        sum0 += ((T) i)*counts[i];
        mu1 = (sum1 - sum0) / w1;
        sigma = w0 * w1 * ( ((double) sum0 / w0) - mu1) * (((double) sum0 / w0) - mu1);
        //std::cout << i << "," << sigma << std::endl;
        if (sigma >= best_wsv) {
            best_idx = i;
            best_wsv = sigma;
        }
    }

    return best_idx;
} 

} /* end namespace upsp */

