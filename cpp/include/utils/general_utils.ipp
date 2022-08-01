/** @file
 *
 * @brief   General Utilities
 */

/*****************************************************************************/
template<typename T, size_t Length>
bool CircularArray<T,Length>::operator==(const CircularArray& ca) const {

    // find a matching element
    size_t idx = 0;
    bool found_match = false;
    for (unsigned int i=0; i < Length; ++i) {
        if (ca[i] == arr_[0]) {
            idx = i;
            found_match = true;
            break;
        }
    }
    if (!found_match) {
        return false;
    }

    // loop through each array, checking all elements (except the matched)
    for (unsigned int i=1; i < Length; ++i) {
        if (arr_[i] != ca[idx+i]) {
            return false;
        }
    }

    return true;
}

/*****************************************************************************/
template<typename T, size_t Length>
std::ostream& operator<<(std::ostream& os, const CircularArray<T,Length>& carr) {
    os << "[ ";
    for (unsigned int i=0; i < Length-1; ++i) {
        os << carr[i] << ", ";
    }
    os << (carr[Length-1]) << " ]";
    return os;
}

/*****************************************************************************/
template<typename T>
T rad2_deg(T angle) {
    return angle * 180.0 / PI;
}

/*****************************************************************************/
template<typename T>
T deg2_rad(T angle) {
    return angle * PI / 180.0;
}

/*****************************************************************************/
template<typename T>
unsigned int min_ind(T p0, T p1, T p2) {
    if (p0 < p1) {
        if (p0 < p2) {
            return 0;
        } else {
            return 2;
        }
    } else {
        if (p1 < p2) {
            return 1;
        } else {
            return 2;
        }
    }
}

/*****************************************************************************/
template<typename T>
unsigned int max_ind(T p0, T p1, T p2) {
    if (p0 > p1) {
        if (p0 > p2) {
            return 0;
        } else {
            return 2;
        }
    } else {
        if (p1 > p2) {
            return 1;
        } else {
            return 2;
        }
    }
}

/*****************************************************************************/
template<typename T>
T min(T p1, T p2, T p3) {
    if (p1 < p2) {
        if (p1 < p3) {
            return p1;
        } else {
            return p3;
        }
    } else {
        if (p2 < p3) {
            return p2;
        } else {
            return p3;
        }
    }
}

/*****************************************************************************/
template<typename T>
T max(T p1, T p2, T p3) {
    if (p1 > p2) {
        if (p1 > p3) {
            return p1;
        } else {
            return p3;
        }
    } else {
        if (p2 > p3) {
            return p2;
        } else {
            return p3;
        }
    }
}

/*****************************************************************************/
template<typename T>
std::vector<T> erase_indices(const std::vector<T>& data, 
        const std::vector<unsigned int>& idcs) {

    if (idcs.size() == 0) {
        return data;
    }

    std::vector<T> ret;
    ret.reserve(data.size() - idcs.size());

    // Copy between each deleted index
    auto d_start = data.begin();
    for (auto it = idcs.begin(); it != idcs.end(); ++it) {
        auto d_end = data.begin() + *it;
        if (d_start != d_end) {
            std::copy(d_start, d_end, std::back_inserter(ret));
        }
        d_start = d_end + 1;
    }

    // Copy over the last chunk
    if (d_start != data.end()) {
        std::copy(d_start, data.end(), std::back_inserter(ret));
    }

    return ret;
}

/*****************************************************************************/
template<typename T>
inline void transpose_scalar_block(T* A, T* B, unsigned int rows,
        unsigned int cols, unsigned int block_size) {

    for (unsigned int i=0; i < block_size; ++i) {
        for (unsigned int j=0; j < block_size; ++j) {
            B[j*rows + i] = A[i*cols + j];
        }
    }

}

/*****************************************************************************/
template<typename T>
inline void transpose_block(T* A, T* B, unsigned int rows, 
        unsigned int cols, unsigned int block_size) {

    // handle up to mutliple of block size regularly
    unsigned int row_max = round_down(rows, block_size);
    unsigned int col_max = round_down(cols, block_size);
    for (unsigned int i=0; i < row_max; i+=block_size) {
        for (unsigned int j=0; j < col_max; j+= block_size) {
            transpose_scalar_block(&A[i*cols + j], &B[j*rows + i], rows, cols, 
                    block_size);
        }

        // handle the extra columns
        for (unsigned int k=i; k < i+block_size; ++k) {
            for (unsigned int j=col_max; j < cols; ++j) {
                B[j*rows + k] = A[k*cols + j];
            }
        }
    }

    // handle the extra rows
    for (unsigned int i=row_max; i < rows; ++i) {
        for (unsigned j=0; j < cols; ++j) {
            B[j*rows + i] = A[i*cols + j];
        }
    }
}

