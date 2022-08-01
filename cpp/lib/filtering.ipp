/** @file
 *  @brief   Filtering and Curve Fitting
 */

namespace upsp {

/********************************************************************
* TransPolyFitter
********************************************************************/

/*****************************************************************************/
template<typename T>
TransPolyFitter<T>::TransPolyFitter(unsigned int n_frames, 
        unsigned int degree, unsigned int n_pts) : A_(n_frames, degree+1), 
        coeffs_(degree+1), n_frames_(n_frames), n_pts_(n_pts),
        poly_(degree+1, n_pts), read_fits_(false) {

    // Fill the matrix frames with appropriate values for fitting and 
    // evaluating the polynomial
    for (unsigned int f=0; f < n_frames_; ++f) {
        for (unsigned int c=0; c < coeffs_; ++c) {
            A_(f,c) = std::pow( (float) f/n_frames_, c);
        }
    } 
}

/*****************************************************************************/
template<typename T>
TransPolyFitter<T>::TransPolyFitter(std::string filename, 
        unsigned int n_frames) : n_frames_(n_frames),
        read_fits_(true) {

    // load the poly fits
    read_coeffs(filename);

    // Fill the matrix frames with appropriate values for
    // evaluating the polynomial fits
    A_.resize(n_frames_, coeffs_);
    for (unsigned int f=0; f < n_frames_; ++f) {
        for (unsigned int c=0; c < coeffs_; ++c) {
            A_(f,c) = std::pow( (float) f/n_frames_, c);
        }
    } 
}

/*****************************************************************************/
template<typename T>
std::vector<T> TransPolyFitter<T>::eval_fit(T* data, unsigned int n_pts, unsigned int curr_pt) {

    // note: data is rowmajor, Eigen is default colmajor
    //       however, since the data is transposed, can just use
    //       the default colmajor

    // Only need to create polynomial fits if not read
    if (!read_fits_) {
        // Create map around data
        Eigen::Map<PolyMat> map_data(data, n_frames_, n_pts);
        
        // Solve for polynomial coefficients
        // A_ * poly_ = data
        // [fxc] * [cxn] = [fxn]
        //std::cout << "block = 0, " << curr_pt << ", " << coeffs_ << ", ";
        //std::cout << n_pts << std::endl;

        auto dec = Eigen::ColPivHouseholderQR<PolyMat>(A_);
        poly_.block(0, curr_pt, coeffs_, n_pts) = dec.solve(map_data);
    }

    // Evaluate the polynomial coefficients for each frame
    std::vector<T> out(n_frames_*n_pts);
    Eigen::Map<PolyMat> map_out(&out[0], n_frames_, n_pts);

    map_out = A_ * poly_.block(0, curr_pt, coeffs_, n_pts);

    return out;
}

/*****************************************************************************/
template<typename T>
void TransPolyFitter<T>::write_coeffs(std::string filename) const {

    // open file
    std::ofstream ofs(filename, std::ofstream::binary);
    if (!ofs.is_open()) {
        throw(std::invalid_argument("Unable to open '" + filename + "'"));
    }

    // Write out the header
    int check = 1;
    ofs.write((char*) &check, sizeof(check));

    int val_size = sizeof(T);
    ofs.write((char*)&val_size, sizeof(val_size));

    int rows = coeffs_;
    int cols = n_pts_;
    ofs.write((char*)&rows, sizeof(rows));
    ofs.write((char*)&cols, sizeof(cols));

    // Write data
    ofs.write((char*) poly_.data(), rows*cols*val_size);

    ofs.close();

}

/*****************************************************************************/
template<typename T>
void TransPolyFitter<T>::read_coeffs(std::string filename) {

    // open file
    std::ifstream ifs(filename, std::ifstream::binary);
    if (!ifs.is_open()) {
        throw(std::invalid_argument("Unable to open polynomial fit coefficients file"));
    }

    // check that T is the correct value of the matrix
    int check = 0;
    ifs.read((char*)&check, sizeof(check));
    if (check != 1) {
        throw(std::invalid_argument("Unable to parse polynomial fit coefficients file"));
    }

    int val_size = 0;
    ifs.read((char*)&val_size, sizeof(val_size));
    if (val_size != sizeof(T)) {
        throw(std::invalid_argument("TransPolyFitter value type is not consistent with file"));
    }

    // read header
    int rows = 0;
    int cols = 0;
    ifs.read((char*)&rows, sizeof(rows));
    ifs.read((char*)&cols, sizeof(cols));
    if ( (rows <= 0) || (cols <= 0)) {
        throw(std::invalid_argument("Invalid polynomial coefficient file"));
    }       

    coeffs_ = (unsigned int) rows;
    n_pts_ = (unsigned int) cols;

    // Define the size of matrix
    poly_.resize(coeffs_, n_pts_);

    // Load the matrix
    ifs.read((char*) poly_.data(), rows*cols*val_size);

    ifs.close();
}

/********************************************************************
* PolyFitter
********************************************************************/

/*****************************************************************************/
template<typename T>
PolyFitter<T>::PolyFitter() {}

/*****************************************************************************/
template<typename T>
template<typename InputIterator>
PolyFitter<T>::PolyFitter(InputIterator x_begin, unsigned int length, 
        unsigned int degree, unsigned int nodes, unsigned int incr/*=1*/) : 
        A_(length/incr, degree+1), length_(length), coeffs_(degree+1), 
        nodes_(nodes), incr_(incr) {

    // Fill the matrix
    for (unsigned int ind=0; ind < (length/incr_); ++ind, std::advance(x_begin,incr_)) {
        for (unsigned int i=0; i < coeffs_; ++i) {
            A_(ind, i) = std::pow(*x_begin,i);
        }
    }

    // Prepare for solving
    dec_ = Eigen::ColPivHouseholderQR<PolyMat>(A_);

    // Set up the matrix to hold the polynomials
    fits_ = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>(nodes, 
            coeffs_);
}

/*****************************************************************************/
template<typename T>
template<typename InputIterator>
PolyFitter<T>::PolyFitter(std::string filename, InputIterator x_begin, 
        unsigned int length, unsigned int incr/*=1*/) : 
        length_(length), incr_(incr) {

    // Load the polynomial fits
    read_coeffs(filename); // sets nodes_, coeffs_, and fits_

    // Fill the matrix
    A_.resize(length/incr, coeffs_);
    for (unsigned int ind=0; ind < (length/incr_); ++ind, std::advance(x_begin,incr_)) {
        for (unsigned int i=0; i < coeffs_; ++i) {
            A_(ind, i) = std::pow(*x_begin,i);
        }
    }
    // Prepare for solving
    dec_ = Eigen::ColPivHouseholderQR<PolyMat>(A_);

}

/*****************************************************************************/
template<typename T>
template<typename InputIterator>
T PolyFitter<T>::fit_data(InputIterator y_begin, unsigned int node) {

    // Create y Matrix
    Eigen::Matrix<T, Eigen::Dynamic, 1> y_mat(length_/incr_, 1);
    for (unsigned int i=0; i < (length_/incr_); ++i, std::advance(y_begin, incr_)) {
        y_mat(i) = *y_begin;
    }
    
    // Solve
    fits_.block(node,0,1,coeffs_) = (dec_.solve(y_mat)).transpose();

    // Compute R^2
    Eigen::Matrix<T, Eigen::Dynamic,1> y_eval(length_/incr_);
    y_eval = A_ * (fits_.block(node,0,1,coeffs_)).transpose();
    Eigen::Matrix<T, Eigen::Dynamic,1> err(length_/incr_);
    err = y_mat - y_eval;
    Eigen::Matrix<T, Eigen::Dynamic,1> f(length_/incr_);
    f = y_eval - y_mat.mean()*Eigen::Matrix<T, Eigen::Dynamic, 1>::Constant(
            length_/incr_, 1.0);

    T SSE = f.dot(f);
    T SSR = err.dot(err);
    T SST = SSE + SSR;

    return 1.0 - (SSR/SST);
}

/*****************************************************************************/
template<typename T>
void PolyFitter<T>::polyval(float x, std::vector<T>& out) {

    // fill vector with independent variable terms
    Eigen::Matrix<T, Eigen::Dynamic, 1> v1(coeffs_);
    for (unsigned int i=0; i < coeffs_; ++i) {
        v1(i) = std::pow(x,i);
    }

    // multiply with polynomial coefficients
    out.assign(nodes_, 0.0);
    Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, 1>> map_out(&out[0], nodes_);
    map_out = fits_ * v1;

}

/*****************************************************************************/
template<typename T>
void PolyFitter<T>::write_coeffs(std::string filename) const {

    // open file
    std::ofstream ofs(filename, std::ofstream::binary);
    if (!ofs.is_open()) {
        throw(std::invalid_argument("Unable to open file for writing polynomial fit coefficients"));
    }

    // Write out the header
    int check = 1;
    ofs.write((char*) &check, sizeof(check));

    int val_size = sizeof(T);
    ofs.write((char*)&val_size, sizeof(val_size));

    int rows = nodes_;
    int cols = coeffs_;
    ofs.write((char*)&rows, sizeof(rows));
    ofs.write((char*)&cols, sizeof(cols));

    // Write data
    ofs.write((char*) fits_.data(), rows*cols*val_size);

    ofs.close();

}

/*****************************************************************************/
template<typename T>
void PolyFitter<T>::read_coeffs(std::string filename) {

    // open file
    std::ifstream ifs(filename, std::ifstream::binary);
    if (!ifs.is_open()) {
        throw(std::invalid_argument("Unable to open polynomial fit coefficients file"));
    }

    // check that T is the correct value of the matrix
    int check = 0;
    ifs.read((char*)&check, sizeof(check));
    if (check != 1) {
        throw(std::invalid_argument("Unable to parse polynomial fit coefficients file"));
    }

    int val_size = 0;
    ifs.read((char*)&val_size, sizeof(val_size));
    if (val_size != sizeof(T)) {
        throw(std::invalid_argument("PolyFitter value type is not consistent with file"));
    }

    // read header
    int rows = 0;
    int cols = 0;
    ifs.read((char*)&rows, sizeof(rows));
    ifs.read((char*)&cols, sizeof(cols));
    if ( (rows <= 0) || (cols <= 0)) {
        throw(std::invalid_argument("Invalid polynomial coefficient file"));
    }       

    nodes_ = (unsigned int) rows;
    coeffs_ = (unsigned int) cols;

    // Define the size of matrix
    fits_.resize(nodes_, coeffs_);

    // Load the matrix
    ifs.read((char*) fits_.data(), rows*cols*val_size);

    ifs.close();
}

/*****************************************************************************/
template<typename T>
std::vector<T> PolyFitter<T>::get_coeffs(unsigned int node) {

    assert(node < nodes_);

    std::vector<T> n_coeffs(coeffs_);

    for (unsigned int i=0; i < coeffs_; ++i) {
        n_coeffs[i] = fits_(node, i);
    }

    return n_coeffs;
}

/********************************************************************
* Functions
********************************************************************/

/*****************************************************************************/
template<typename InputIterator, typename T>
std::vector<T> polyfit(InputIterator x_begin, InputIterator y_begin, 
        unsigned int length, unsigned int degree) {

    // determine the number of coefficients
    unsigned int coeffs = degree + 1;

    assert(length >= coeffs);

    // form the matrix
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> A(length, coeffs);
    for (unsigned int ind=0; ind < length; ++ind, ++x_begin) {
        for (unsigned int i=0; i < coeffs; ++i) {
            A(ind, i) = std::pow(*x_begin,i);
        }
    }

    // Initialize output vector
    std::vector<T> poly(coeffs, 0.0);

    // Create y Matrix
    Eigen::Matrix<T, Eigen::Dynamic, 1> y_mat(length, 1);
    for (unsigned int i=0; i < length; ++i, ++y_begin) {
        y_mat(i) = *y_begin;
    }

    // Solve
    Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, 1>> poly_map(&poly[0], coeffs);
    poly_map = A.colPivHouseholderQr().solve(y_mat);

    return poly;
}

/*****************************************************************************/
template<typename T>
std::vector<T> polyfit(const std::vector<T>& x, const std::vector<T>& y, 
        unsigned int degree) {
    assert( x.size() == y.size() );

    // determine the number of coefficients
    unsigned int coeffs = degree + 1;

    assert( x.size() >= coeffs);

    // form the matrix
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> A(x.size(), coeffs);
    unsigned int count = 0;
    for (unsigned int ind=0; ind < x.size(); ++ind) {
        for (unsigned int i=0; i < coeffs; ++i) {
            A(ind, i) = std::pow(x[ind],i);
        }
    }

    // Initialize output vector
    std::vector<T> poly(coeffs, 0.0);

    // Solve
    const Eigen::Map<const Eigen::Matrix<T, Eigen::Dynamic, 1>> y_map(&y[0], y.size());
    Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, 1>> poly_map(&poly[0], coeffs);
    poly_map = A.colPivHouseholderQr().solve(y_map);

    return poly;
}

/*****************************************************************************/
template<typename T>
T polyval(T x, const std::vector<T>& poly) {
    T y = 0.0;
    for (unsigned int i=0; i < poly.size(); ++i) {
        y += std::pow(x,i) * poly[i];
    }
    return y;
}

/*****************************************************************************/
template<typename T>
std::vector<T> polyval(const std::vector<T>& x, const std::vector<T>& poly) {
    std::vector<T> y(x.size(), 0.0);
    for (unsigned int j=0; j < x.size(); ++j) {
        for (unsigned int i=0; i < poly.size(); ++i) {
            y[j] += std::pow(x[j],i) * poly[i];
        }
    }
    return y;
}

} /* end namespace upsp */
